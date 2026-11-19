# Real-time design: scheduling, interrupts, watchdog

How the RoboCar firmware uses micro T-Kernel 3.0 on the RP2040: which tasks run at which priority and why, how each hardware source is serviced (interrupt, polling or FIFO stream), and how the watchdog recovers from a stall. Source of truth for every number: `app_program/rc_config.h`.

## 1. Scheduling

micro T-Kernel schedules by **fixed priority with preemption** (1 = highest; same priority runs in FIFO order). The application keeps that scheduler and assigns priorities **deadline-monotonically**: the shorter the time a task has to finish its work, the higher it runs. For the periodic tasks (deadline = period) this is the same as rate-monotonic. Dynamic-priority schemes (EDF, LLF) were not adopted: the kernel does not provide them, the CPU load is low, and half of the work is event-driven, with no natural deadline to schedule by.

| Pri | Task (file) | Trigger | Deadline | Notes |
|---|---|---|---|---|
| 4 | Defer (`rc_defer.c`) | flag from ISRs | < 1 ms | bottom halves: barcode ADC samples and edges (`drv_ir.c`), ultrasonic echo result (`drv_ultrasonic.c`) |
| 5 | FAST dispatcher (`rc_event.c`) | event flag (≤ 50 ms safety timeout) | < 10 ms | `sub_line` steering, `sub_nav`, `sub_terrain` run here |
| 6 | Sense (`rc_sense.c`) | `rc_period`, exactly 10 ms | 10 ms | line pair + IMU |
| 7 | Motion (`sub_motion.c`) | `rc_period`, exactly 20 ms | 20 ms | wheel PID, odometry, stall checks |
| 8 | Scan (`sub_scan.c`) | request semaphore, result flag, sleeps | 60 ms per ping | forward watch and step scan; mostly waiting on echoes |
| 9 | SLOW dispatcher (`rc_event.c`) | event flag | soft | logging-type handlers |
| 12 | Telemetry (`sub_telemetry.c`) | 250 ms + events | soft | lowest application priority |

The kernel's own USB console task (3) sits above the application. Its WiFi task (CYW43, `CYW43_TASK_PRIORITY` 4 in `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c`) has the same priority as Defer, so the two run in FIFO order.

**Priority inversion.** The FAST dispatcher calls `sub_motion_*`, which takes the motion mutex that the Motion task also holds every cycle. The mutex is `TA_INHERIT`: while FAST waits, Motion runs at FAST's priority, so Sense (between them) cannot hold FAST up.

**Two cores (SMP builds, incl. the MQTT track image).** No application task is pinned to a core (except the one-shot `usb_irq_prio_task`, §2), so Sense and Motion normally run at the same time; priority decides the order only when both cores are busy.

**Measuring for a response-time table.** A TEST build prints, at the end of the run:
- `[RISK1] task max: sense line … + imu … + bar … us, motion … us`: the worst-case run time of each periodic task;
- `[RISK1] isr cb max: enc L …, enc R …, echo … us`: the worst-case time inside each GPIO callback (NFR5 target 10 µs);
- `[RISK1] gpio_irq max=…`: the whole IO_BANK0 handler;
- `[RISK1] missed periods: sense …, motion …`: control periods each task missed (`rc_sense_overruns()`, `sub_motion_overruns()`).

## 2. Interrupts: which mechanism for which source

The rule: an **interrupt** for sporadic, time-critical events; **polling** for short, predictable waits and for level signals sampled on a schedule; a **FIFO (or DMA) stream** for a continuous signal.

| Source | Mechanism | Why |
|---|---|---|
| Wheel encoders (GP0, GP7) | GPIO edge IRQ (`rc_gpioirq.c`), timestamped; the ISR only updates count and period, and `sub_motion` reads `drv_encoder` every control period (no bottom half) | sporadic edges whose timing is the measurement |
| Ultrasonic echo (GP3) | GPIO edge IRQ, both edges, timestamped | the pulse width is the distance |
| Ultrasonic trigger pulse and 30 ms timeout | TIMER alarm IRQs 1 and 2 (`drv_ultrasonic.c`) | exact timing without a busy wait |
| Barcode AO (GP26, ADC0) | **ADC free-running at 8 kHz into its FIFO; FIFO IRQ at 4 readings; each 8 readings averaged to one 1 kHz value** (`drv_ir.c`, `RC_BAR_ADC_FIFO 1`); detector in the defer task | a continuous signal: sampled by the ADC's own clock, nothing waits for a conversion |
| Line sensors (GP16, GP6) | polled every 10 ms in Sense | level signals; a fixed rate is what the steering expects |
| IMU, LSM303D over I²C (400 kHz) | polled every 10 ms in Sense; the port's I²C driver queues all read commands at once (STOP on the last byte) and takes one RX interrupt per read | the sampling is periodic; a 6-byte read is one interrupt, not one per byte |
| USB console | ring buffer drained by the USB task (priority 3); full ring drops and counts | printing never blocks a task; on SMP `tm_printf` masks interrupts for the length of one call, so nothing prints per sample |
| MQTT | message buffer to the lwIP task, `TMO_POL` | a slow or lost link fails the send instead of blocking telemetry |
| Motors, scan servo | hardware PWM, no interrupt | the hardware holds the duty |
| Periodic wake-ups, nav timeouts | kernel cyclic and alarm handlers | exact periods (`rc_period.c`), one-shot timeouts (`sub_nav.c`) |

**Interrupt priorities.** Every interrupt whose handler calls kernel services (`tk_set_flg`, `tk_wup_tsk`, …) runs at NVIC level 3, the same level as PendSV, so none of them can preempt a task switch half-way through saving a context. The kernel port already does this for SysTick (`include/sys/sysdepend/cpu/rp2040/sysdef.h`, `INTPRI_SYSTICK`) and the inter-core IPI (`kernel/sysdepend/cpu/rp2040/smp_rp2040.c`), and its comments give the reason. The application sets the same level for its own interrupts:

| Interrupt | Set in | Level |
|---|---|---|
| IO_BANK0 (encoders, ultrasonic echo) | `rc_gpioirq.c` `INTPRI_IO_BANK0` | 3 |
| ADC FIFO (barcode) | `drv_ir.c` `INTPRI_ADC_FIFO` | 3 |
| TIMER alarms 1 and 2 (ultrasonic) | `drv_ultrasonic.c` `INTPRI_TIMER` | 3 |
| I2C0 (IMU) | `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` `DEVCNF_I2C0_INTPRI` | 3 |
| USBCTRL (console) | `app_main.c` `usb_irq_prio_task`, at boot | 3 |

The USB console enables its interrupt without setting a priority (it would stay at the reset level 0, above PendSV). The NVIC is per core and the console takes that interrupt on core 0, so `usb_irq_prio_task` is a short task pinned to core 0 that writes the level and exits.

A dual-core hang still occurs with WiFi under load; its cause is not yet known. The watchdog (§3) recovers it. A bug report for the kernel template's maintainer has been prepared.

**Sharing data between cores (SMP builds).** Masking interrupts (`DI`/`EI`) only stops interrupts on the local core; an ISR or a task on the other core can still touch the same data. Data shared with an ISR or with a task that may run on the other core is therefore guarded with a kernel spinlock (`ISpinLock`/`ISpinUnlock`; plain `DI`/`EI` in single-core builds):

- `rc_event.c`: the lane rings, subscribe/unsubscribe, and the dispatcher's copy of each subscriber's callback and context;
- `sub_telemetry.c`: the counters bumped by both the telemetry task and the SLOW dispatcher, and the event-frame sequence number;
- the port's I2C driver (`i2c_rp2040.c`);
- `drv_encoder.c` (`ENC_LOCK`): count, period and last-edge time, including `drv_encoder_distance_mm()`.

The barcode rings in `drv_ir.c` have one producer and one consumer each, so they need no lock: a compiler barrier (`RING_BARRIER`) makes sure the slot is written before the head index moves and read after it is seen. The ADC FIFO interrupt signals the defer task only when it produced a reading.

**Barcode stream in detail.** The ADC converts channel 0 every 6 000 cycles of its 48 MHz clock (8 kHz). The FIFO (8 deep) interrupts when half full (2 000 times a second, leaving 500 µs before a reading could be lost). The handler adds the readings to a running sum and, every 8 readings, puts their average (one value per ms) into a 256-entry ring and signals the defer task. There, `bar_samples_drain()` runs the peak/valley edge detector on each value, stamped from its position in the stream (1 ms apart).

**Sampling rate and aliasing.** The narrowest element is 3 mm, so a narrow bar plus a narrow space is a 6 mm period: a 5.5 Hz fundamental at the ~33 mm/s line speed and 50 Hz at 300 mm/s. The sensor spot is about one element wide, which blurs anything finer (an optical low-pass). Nyquist therefore needs at most ~100 Hz up to 300 mm/s. The detector times edges rather than rebuilding the waveform, so the useful rate is set by timing error (about one sample period): ±10 ms at 100 Hz (11 % of a narrow element at line speed, too much above ~100 mm/s) against ±1 ms at 1 kHz. AO has no analogue anti-alias filter, so the 8-reading average is the filter. It has nulls at every multiple of 1 kHz and cuts white noise by √8. The motor PWM (20 kHz) sampled at 8 kHz folds to 4 kHz, which is one of those nulls. With `RC_BAR_ADC_FIFO 0` the ADC is instead read once per 10 ms Sense tick through the device driver, which starts a 2 µs conversion and then sleeps on an interrupt for it; in FIFO mode the Sense task does not touch the barcode.

The detector's time constants are set in milliseconds, not samples (history ≥ 1 s, quiet timeout 300 ms, white-level creep 100 counts/s, glitch filter 15 ms), so both modes behave the same. The history has to be that long because a bar's falling edge is timed only once the space after it has shown its lowest point, up to the 300 ms quiet timeout later; with a shorter history the fall drops out and the last bar before a long space comes out too wide. Decoding at 1 kHz on the track is still to be checked.

## 3. Watchdog

`rc_wdog.c` arms the RP2040 hardware watchdog (`RC_WDOG_MS` 2 s) at the end of start-up. A cyclic handler (every `RC_WDOG_CHECK_MS` 250 ms) reloads it **only if the Sense task, the Motion task and the FAST dispatcher have all checked in** since its last visit. A task stuck in a driver, a deadlock or a stopped core therefore reboots the car; feeding the watchdog from a timer alone would hide those.

- **2 s:** normal check-in gaps are ≤ 50 ms; one I²C transfer can block Sense for up to the driver's 1 s timeout, so 2 s reboots only on a real stall.
- **Reset reason:** at boot `rc_wdog_boot()` reads `WATCHDOG_REASON`. After a watchdog reboot it prints `[wdog] boot: REBOOTED BY WATCHDOG, stopped: <tasks>` (the supervisor keeps the missing tasks in `SCRATCH0`, which survives the reset) and sets fault bit 7 `RC_FAULT_WDOG_RESET`, which the dashboard shows until START.
- **Safe restart:** the MQTT build waits in IDLE for START after any boot, so a reboot never sets the car moving.
- **Untouched:** the watchdog's tick generator, which also drives the 1 µs TIMER (`rc_time.c`). The watchdog pauses while a debugger halts the cores, and a watchdog reset resets everything except the oscillators (`PSM_WDSEL`).
- **Test:** build with `RC_WDOG_STALL_TEST 1`: Sense stops checking in 30 s after boot, and the car must reboot about 2 s later and report `stopped: Sense`.
- **Supervisor stopped:** if the cyclic handler itself stops running (a hung core or tick), the watchdog also expires; the boot line then reads `supervisor stopped (system hang)` instead of naming tasks. This is how the dual-core hang under WiFi load (§2) is recovered.
- **Limits:** the watchdog does not fix an IMU that returns a frozen reading (the CPU keeps running); `drv_imu.c` reports that as the IMU fault.
