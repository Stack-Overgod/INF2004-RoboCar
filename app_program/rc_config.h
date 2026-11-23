/*
 *  rc_config.h
 *
 *  Single place for every pin number and tuning constant in the robot.
 *  Nothing else in the tree hard-codes a GPIO.
 *
 *  Target: Raspberry Pi Pico W on a Cytron Robo Pico carrier, running
 *  micro T-Kernel 3.0 (mtk3smp-rp2040 port).
 *
 *  Several of these pins are fixed by the Robo Pico board and cannot be
 *  moved.
 */

#ifndef RC_CONFIG_H
#define RC_CONFIG_H

/* ------------------------------------------------------------------ *
 *  Fixed by the Robo Pico carrier board. Do not change.
 * ------------------------------------------------------------------ */

/* Left wheel on the MOTOR 2 terminal, right wheel on MOTOR 1. */
#define RC_PIN_MOTOR_L_A        (10U)   /* M2A, PWM slice 5 chan A */
#define RC_PIN_MOTOR_L_B        (11U)   /* M2B, PWM slice 5 chan B */
#define RC_PIN_MOTOR_R_A        (8U)    /* M1A, PWM slice 4 chan A */
#define RC_PIN_MOTOR_R_B        (9U)    /* M1B, PWM slice 4 chan B */

/* Servo header: GP12..GP15 are servo ports 1..4. The scan servo is on
 * port 4 (GP15, PWM slice 7 chan B). Ports 1-3 are free. */
#define RC_PIN_SERVO_SCAN       (15U)

/* ------------------------------------------------------------------ *
 *  Chosen by us. Move these if your wiring differs.
 *
 *  Each sensor below sits on one Robo Pico Grove socket, so each is one
 *  4-wire Grove cable: GND, 3V3, and the two GPIOs listed.
 * ------------------------------------------------------------------ */

/* Wheel encoders: two-channel (A/B) Hall-effect encoders built into the
 * back of each gear motor. Each motor has six wires: two motor-power wires
 * to its MOTOR terminal, and four encoder wires (VCC, GND, A, B) to one
 * Grove socket. Colour codes differ between makers, so wire by the labels
 * on the motor's encoder board, never by colour. VCC goes to the Grove
 * 3V3, which keeps A/B at the Pico's 3.3 V logic level.
 *
 * Channel A raises the edge interrupt; channel B is sampled at that
 * instant to get direction. If a wheel reads backwards, swap that
 * encoder's A and B wires rather than adding a sign in software.
 *
 * Left  = Grove 1 (GP0/GP1). Right = Grove 7 (GP7/GP28).
 *
 * GP0/GP1 are also UART0, the port's default console. This build MUST use
 * the USB console (make CONSOLE=usb_cdc): a UART build would drive GP0 as
 * TX and fight the encoder, so the #error below refuses it. */
#define RC_PIN_ENC_L_A          (0U)
#define RC_PIN_ENC_L_B          (1U)
#define RC_PIN_ENC_R_A          (7U)
#define RC_PIN_ENC_R_B          (28U)
#if defined(TM_CONSOLE_UART) && TM_CONSOLE_UART && (RC_PIN_ENC_L_A <= 1U)
#error "UART0 console (CONSOLE=uart or CRASH_REPORT=1) takes GP0/GP1, the left encoder"
#endif

/* HC-SR04 on Grove 2 (GP2/GP3).
 * ECHO is 5 V on this module and MUST go through a divider. */
#define RC_PIN_ULTRA_TRIG       (2U)
#define RC_PIN_ULTRA_ECHO       (3U)

/* IR line sensors, digital comparator output (DOUT) only.
 * Sensor 1 (left) = Grove 4 (GP16). Sensor 2 (right) = Grove 5 (GP6).
 *
 * Grove 5's second signal pin is GP26, which is the barcode sensor's
 * analogue output below. Connect ONLY the line sensor's DO wire on Grove 5;
 * leave its AO unconnected or it will short onto the barcode AOUT. */
#define RC_PIN_IR_LINE_L        (16U)
#define RC_PIN_IR_LINE_R        (6U)

/* IR barcode sensor on Grove 6 (GP26/GP27). Only AOUT (GP26, ADC0) is
 * read, as set by RC_BAR_ADC_FIFO below (needs config_device.h
 * DEVCNF_USE_ADC 1). DOUT (GP27) is wired but deliberately not configured
 * and has no IRQ, so its trim pot does not matter. */
#define RC_PIN_IR_BARCODE_A     (26U)
#define RC_ADC_CH_IR_BARCODE    (0U)

/* Barcode ADC edge detector (drv_ir.c), in ADC counts (0..4095).
 * PROM: how far the reading must come back from a bar's peak or a space's
 * valley to end that element. Narrow spaces between bars dip ~700-1700
 * counts at track speed; sensor noise is ~20.
 * MIN_BAR: a bar's peak must be this far above the white level (running
 * minimum), so mat texture is not read as bars.
 * BLACK_HIGH: 1 if AO reads higher over black (verified on the car). */
#define RC_BAR_ADC_PROM         (400U)
#define RC_BAR_ADC_MIN_BAR      (800U)
#define RC_BAR_ADC_BLACK_HIGH   (1)

/* How the barcode AO is sampled (drv_ir.c):
 * 1: the ADC converts continuously at RC_BAR_ADC_HZ x RC_BAR_ADC_OVERSAMPLE
 *    into its FIFO; the FIFO interrupt averages each OVERSAMPLE readings
 *    into one value at RC_BAR_ADC_HZ and hands it to the defer task, which
 *    runs the edge detector. Nothing waits for a conversion.
 * 0: one reading per Sense tick (RC_PERIOD_LINE_MS) through the "adca"
 *    device driver.
 * 1 kHz is >= 10x the Nyquist rate of the barcode signal up to 300 mm/s
 * (3 mm elements: 50 Hz fundamental) and times edges to ~1 ms. AO has no
 * analogue anti-alias filter: the averaging is the filter. Its nulls fall
 * on every multiple of RC_BAR_ADC_HZ, and the 20 kHz motor PWM, sampled at
 * 8 kHz, folds to 4 kHz, onto one of them. */
#define RC_BAR_ADC_FIFO         (1)
#define RC_BAR_ADC_HZ           (1000U)
#define RC_BAR_ADC_OVERSAMPLE   (8U)

/* IMU on Grove 3 (GP4/GP5). Those pins are I2C0 SDA/SCL in silicon, so the
 * IMU is on I2C unit 0. The stock driver maps I2C0 to GP8/GP9 (the right
 * motor), so its unit-0 pin table is patched to GP4/GP5. */
#define RC_I2C_UNIT_IMU         (0U)
#define RC_PIN_I2C0_SDA         (4U)
#define RC_PIN_I2C0_SCL         (5U)

/* This board carries an LSM303D (confirmed by an I2C bus scan: it ACKs at
 * 0x1D). The LSM303D is a SINGLE die with ONE address (0x1D, SA0 high) for
 * both the accelerometer and the magnetometer - unlike the older
 * LSM303DLHC, which used 0x19 (accel) and 0x1E (mag). */
#define RC_I2C_ADDR_ACCEL       (0x1DU)
#define RC_I2C_ADDR_MAG         (0x1DU)

/* ------------------------------------------------------------------ *
 *  Timing
 * ------------------------------------------------------------------ */

#define RC_TICK_PERIOD_MS       (10)    /* kernel tick = CNF_TIMER_PERIOD */

#define RC_PERIOD_MOTION_MS     (20)    /* PID update rate, 50 Hz */
#define RC_PERIOD_LINE_MS       (10)    /* line sampling, 100 Hz */
#define RC_PERIOD_IMU_MS        (10)    /* IMU sampling, 100 Hz */
#define RC_PERIOD_SCAN_MS       (60)    /* one ultrasonic ping per step */
#define RC_PERIOD_TELEM_MS      (250)   /* telemetry publish rate */

#if ((RC_PERIOD_MOTION_MS % RC_TICK_PERIOD_MS) != 0) \
 || ((RC_PERIOD_LINE_MS % RC_TICK_PERIOD_MS) != 0) \
 || ((RC_PERIOD_IMU_MS % RC_TICK_PERIOD_MS) != 0)
#error "every RC_PERIOD_* must be a whole number of kernel ticks"
#endif

/* ------------------------------------------------------------------ *
 *  Task priorities. Lower number wins in micro T-Kernel, which schedules
 *  by fixed priority with preemption. The priorities are deadline-
 *  monotonic: the shorter the time a task has to finish its work, the
 *  higher it runs.
 *    DEFER          ISR bottom halves (encoder, echo, barcode samples)
 *    DISPATCH_FAST  event handlers that must react before the next
 *                   10 ms sample (line steering, nav, terrain)
 *    SENSE          10 ms sampling of line and IMU
 *    MOTION         20 ms wheel PID
 *    SCAN           one ultrasonic ping per 60 ms step
 *    DISPATCH_SLOW  logging-type handlers, no hard deadline
 *    TELEMETRY      250 ms publish, no hard deadline
 *  sub_nav has no task of its own: its handlers run in the FAST
 *  dispatcher.
 * ------------------------------------------------------------------ */

#define RC_PRI_DEFER            (4)
#define RC_PRI_DISPATCH_FAST    (5)
#define RC_PRI_SENSE            (6)
#define RC_PRI_MOTION           (7)
#define RC_PRI_SCAN             (8)
#define RC_PRI_DISPATCH_SLOW    (9)
#define RC_PRI_TELEMETRY        (12)  /* lowest application priority */

#define RC_STACK_SZ             (4096)  /* SMP kernel paths are deep */

/* Watchdog (rc_wdog.c): reboot if Sense, Motion or the FAST dispatcher
 * stops checking in for this long. Normal gaps are <= 50 ms; one I2C
 * transfer can block Sense for up to 1 s (the driver's timeout), so 2 s
 * reboots only on a real stall. The supervisor looks every CHECK_MS. */
#define RC_WDOG_MS              (2000U)
#define RC_WDOG_CHECK_MS        (250U)
/* 1: the Sense task stops checking in 30 s after boot, so the watchdog
 * must reboot the car and report "stopped: Sense". Keep 0 in the repo. */
#define RC_WDOG_STALL_TEST      (0)
/* 1: the FAST dispatcher records the event and callback it is running in
 * the watchdog scratch registers (they survive a watchdog reset), and the
 * boot after a "stopped: FAST" reset prints them. Two register writes
 * per FAST callback. */
#define RC_WDOG_TRACE           (1)
/* 1: stand soak test for the watchdog: START goes straight to FOLLOWING (no
 * line-up scan, which needs the chassis to turn), the line is never
 * declared lost and the forward watch never starts avoidance, so line
 * steering and the Motion task run for as long as the car is left on the
 * stand. Keep 0 in the repo. */
#define RC_STAND_SOAK           (0)

/* Tasks that share the motion mutex (the FAST dispatcher, Motion, Scan)
 * all run on core 1, so a wait for the mutex and its release never cross
 * cores; the CYW43 WiFi task and the USB console task are pinned to
 * core 0 by the port. Single-core builds ignore it. */
#if defined(TK_SUPPORT_SMP) && TK_SUPPORT_SMP
#define RC_CTL_TSKATR           (TA_ASSPRC)
#define RC_CTL_ASSPRC           (TP_PRC2)
#else
#define RC_CTL_TSKATR           (0U)
#define RC_CTL_ASSPRC           (0U)
#endif

/* ------------------------------------------------------------------ *
 *  Mechanical constants. Measure these on your own car.
 * ------------------------------------------------------------------ */

/* Encoder ticks (channel-A rising edges) per WHEEL revolution. The Hall
 * encoder reads the motor shaft before the gearbox, so this is the
 * encoder's pulses per motor turn times the gear ratio - typically several
 * hundred, not the 20 of a slotted disc. Measure it: flash the motion
 * bench and, in phase 3 (motors off), turn one wheel exactly 10 turns by
 * hand in one direction. The count change divided by 10 is this number.
 *
 * Measured on the car, 10 hand turns per wheel: left 6350, right 6419
 * counts (635 / 642 per turn, within hand-counting error).
 * Cross-check: 35% duty gave ~900 counts/s = ~280 mm/s. */
#define RC_ENC_TICKS_PER_REV    (640U)
/* Effective rolling diameter: roll test, one wheel turn on paper = 211,
 * 208, 217 mm, mean 212 mm / pi = 67.5 mm. 68 rather than 67
 * because it keeps RC_ENC_UM_PER_TICK closest to 212000 / 640 = 331 um. */
#define RC_WHEEL_DIAM_MM        (68U)
#define RC_WHEEL_BASE_MM        (112U)  /* tread centre to tread centre, measured */

/* Distance per encoder edge, in micrometres, to stay in integer maths.
 * There is no FPU on the RP2040, so everything downstream is fixed point. */
#define RC_ENC_UM_PER_TICK      ((31416UL * (unsigned long)RC_WHEEL_DIAM_MM) \
                                 / (10UL * (unsigned long)RC_ENC_TICKS_PER_REV))

/* ------------------------------------------------------------------ *
 *  IMU and terrain (Buddy 4)
 * ------------------------------------------------------------------ */

/* FR5: front-to-rear axle distance L in h = L x sin(pitch): drive-wheel
 * axle to the caster's contact point. NOT RC_WHEEL_BASE_MM, which is the
 * left-right wheel spacing used for turns. PLACEHOLDER - measure. */
#define RC_AXLE_DIST_MM         (80U)

/* Lift / overturn watch (sub_terrain.c lift_step, lift_height) from the
 * accelerometer. Either condition stops the run (sub_nav -> STOPPED):
 *   raised: the vertical acceleration, integrated twice, says the car
 *           has risen RC_LIFT_UP_MM (a hand picking it up ~4-5 cm within
 *           ~0.6 s; slower lifts are not seen). A bump comes back down
 *           and cancels;
 *   tilted: more than 35 deg off level for RC_LIFT_TILT_MS (held at an
 *           angle, on its side); past 90 deg it is reported as
 *           overturned. A one-wheel 10 mm hump tilts the car ~5 deg. */
#define RC_LIFT_UP_MM           (20)
#define RC_LIFT_V_MM_S          (120)    /* and rising at least this fast */
#define RC_LIFT_TAN2_PERMILLE   (490)    /* tan^2(35 deg) x 1000 */
#define RC_LIFT_TILT_MS         (300U)

/* drv_imu_calibrate() sample count at boot. Each sample is followed by
 * tk_dly_tsk(5), one to two 10 ms kernel ticks, so ~1 s in all. */
#define RC_IMU_CAL_SAMPLES      (64U)

/* ------------------------------------------------------------------ *
 *  Ultrasonic
 * ------------------------------------------------------------------ */

#define RC_ULTRA_TIMEOUT_US     (30000UL)  /* ~5 m, give up after this */
#define RC_ULTRA_TRIG_US        (12UL)     /* datasheet asks for >= 10 us */
#define RC_ULTRA_MAX_MM         (4000U)
#define RC_ULTRA_MIN_MM         (20U)

/* ------------------------------------------------------------------ *
 *  Event bus sizing
 * ------------------------------------------------------------------ */

#define RC_EVENT_RING_SZ        (64U)   /* must be a power of two */
#define RC_EVENT_MAX_SUBS       (32U)   /* total subscriptions per lane */

/* Risk 1 (dispatcher WCET + ring drops) soak. When > 0 in a TEST=1 build,
 * the test run ends with an N-second soak: it starts the control path and
 * prints the event-bus report every 2 s while you apply worst-case load.
 * 0 (default) disables it - a plain TEST=1 run prints one report after the
 * subsystem tests and then sleeps. */
#ifndef RC_RISK1_SOAK_SECS
#define RC_RISK1_SOAK_SECS      0
#endif

/* ------------------------------------------------------------------ *
 *  Buddy 1 - communication, command and telemetry
 *
 *  The WiFi network, broker address and client id are set in
 *  config/wifi_secrets.h (git-ignored; copy wifi_secrets.example.h).
 * ------------------------------------------------------------------ */

/* Topic namespace. One base per car so several cars can share a broker:
 * car/+/state addresses every car, car/01/# just this one. Buddy 1 owns
 * this; sub_telemetry.c builds every publish topic from it. */
#define RC_TOPIC_BASE           "car/01"
/* The one topic we subscribe to, for inbound commands. */
#define RC_TOPIC_CMD            RC_TOPIC_BASE "/cmd"

/* Reconnect backoff for the telemetry task. On a dropped link it closes,
 * waits (doubling from MIN up to MAX), and reopens - never a busy loop. */
#define RC_NET_BACKOFF_MIN_MS   (500U)
#define RC_NET_BACKOFF_MAX_MS   (8000U)

#endif /* RC_CONFIG_H */
