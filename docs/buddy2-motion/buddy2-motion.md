# Buddy 2 — Motion Control (PID, Odometry, Movement)

> Your part of the team guide. The shared picture — the event bus (§1.2),
> the non-blocking rule (§1.3), start-up and the mission state machine
> (§2.1–2.2), the `core/` toolbox (§2.3) and the shared hardware (§3) — is
> in [`TEAM_GUIDE.md`](../../TEAM_GUIDE.md); every `§` below points there.
> Installing, building, flashing and testing is [`BUILD.md`](../../BUILD.md).
>
> The encoder gear motors have no datasheet in this repo — the labels on
> each motor's encoder board are the reference for its wires. The Robo Pico
> motor-driver details are in [`docs/Robo_Pico.pdf`](../Robo_Pico.pdf).

**Files:** `subsystems/sub_motion.c` / `.h`, `drivers/drv_motor.c` / `.h`,
`drivers/drv_encoder.c` / `.h`, and your test, `bench_motion()` in
`app/app_bench.c`

**Your brief, and where each part lives**

| Brief | In the code |
|---|---|
| Motor driver integration | `drv_motor.c` |
| Encoder integration, speed and distance estimation | `drv_encoder.c` |
| PID speed control | `pid_step()` in `sub_motion.c` |
| Straight-line motion correction | `sync_correction()` in `sub_motion.c` |
| Encoder-based turning | `sub_motion_turn_deg()` + `run_move()` |
| Motion calibration | the constants at the top of `sub_motion.c` and in `rc_config.h` |
| `moveForward(distance)` / `moveBackward(distance)` | `sub_motion_forward_mm()` / `sub_motion_backward_mm()` |
| `turnLeft(angle)` / `turnRight(angle)` | `sub_motion_turn_deg(-angle)` / `sub_motion_turn_deg(+angle)` |
| `stop()` | `sub_motion_stop()` |
| PID tuning report, motion accuracy evaluation | produced with the `motion` bench — see "How to get started" |

## Your hardware — wiring, pin by pin

**DC gear motor with built-in encoder + wheel (×2).** A black plastic
gearbox with the wheel on its output shaft, and a small bump at the back
of the motor with a white connector: that bump is the **encoder**, built
into the motor. Each motor has **six wires**: two power the motor and
screw into a Robo Pico motor terminal (**left wheel → MOTOR 2, right wheel
→ MOTOR 1**); the other four are the encoder's `VCC`, `GND`, `A`, `B` and
go to one Grove socket. The Robo Pico drives each motor with two PWM pins,
so "forward", "reverse" and "brake" are all done in `drv_motor.c` by
choosing which pin gets the duty.

**The encoder, two-channel (Hall effect).** A magnet on the motor shaft
spins past two magnetic sensors, giving two pulse outputs, `A` and `B`,
that are the same train of pulses shifted by a quarter of a step. Every
pulse on `A` is one "tick" → speed and distance in `drv_encoder.c`.
Because `B` is shifted, whether `B` is low or high at the instant `A`
rises tells you which way the wheel is turning — so direction is
*measured*, not guessed.

The encoder reads the **motor** shaft, before the gearbox, so one wheel
turn is the encoder's pulses per motor turn × the gear ratio — typically
**several hundred ticks**, not the handful a slotted disc would give. That
makes distances and turns very fine-grained, but the ticks-per-wheel-turn
number has to be **measured** (see "How to get started" below) before any
distance or speed is right.

**Find out which wire is which — by label, not colour.** Makers use
different colour codes for these six wires (on some motors black is a
motor wire, on others it's the encoder's GND), so read the labels printed
on the small board at the back of the motor (e.g. `M+ M− VCC GND A B`, or
`M1 M2 C1 C2 3V3 GND`) or the seller's page. No labels? With a multimeter,
the two motor wires read a few ohms between each other; the other four
don't. Getting this wrong can put battery voltage onto the Pico's 3.3 V
pins, so check before you power up.

**How to read a Grove socket.** Every Grove socket on the Robo Pico has
four pins, printed on the board in this order: `GND`, `3V3`, then the two
GPIO numbers. A standard Grove cable's wires are colour-coded — **black =
GND, red = 3V3, white = the first GPIO printed, yellow = the second** — but
don't trust colours blindly: hold the cable against the socket and read
which printed label each wire lands on. The motor's wires end in female
jumper (Dupont) plugs, so use a Grove-to-**male**-jumper cable: its four
pins push into the encoder's four plugs.

**Motors** — loosen the two screws on the terminal, put one motor wire
into each hole, tighten. The wires end in female jumper plugs, which a
screw terminal can't grip well: push a male header pin into each plug and
clamp the pin, or use bare wire. There is no "right way round": if a wheel
spins backwards in the `motion` bench, swap that motor's two wires.

| Motor | Robo Pico terminal | Pins behind it | In the code |
|---|---|---|---|
| Left wheel | **MOTOR 2** (the left of the two black terminals) | M2A = GP10, M2B = GP11 | `RC_PIN_MOTOR_L_A/B` |
| Right wheel | **MOTOR 1** (the right one, nearer the servo header) | M1A = GP8, M1B = GP9 | `RC_PIN_MOTOR_R_A/B` |

**Encoders** — each motor's four encoder wires go to one Grove socket.
The encoder's pins may be labelled `A`/`B`, `C1`/`C2` or `OUT A`/`OUT B`;
the first is A. Power it from the Grove `3V3`, never the battery: the
encoder's outputs follow its supply, and the Pico's pins only take 3.3 V.

| Encoder wire | Left encoder → **Grove 1** (left edge of the board) | Right encoder → **Grove 7** (right edge) | Grove cable colour |
|---|---|---|---|
| `VCC` | `3V3` | `3V3` | red |
| `GND` | `GND` | `GND` | black |
| `A` | `GP0` | `GP7` | white |
| `B` | `GP1` | `GP28` | yellow |

The colours in the last column are the **Grove cable's**, not the motor's.

Then check with the `motion` bench (`BUILD.md` §5.2): both speeds must read
**positive** when the car is driven forward. A negative one means that
encoder's A and B are swapped — swap the wires, don't change the code.

**Why Grove 1 is special.** GP0/GP1 are also the pins the kernel would use
for a wired console. `build/setup.sh` turns that off; if the left encoder
ever counts nothing while the right one works, that patch hasn't been
applied — run the setup task again.

**How your files connect to the rest of the car** (see §2.3 for what
each `core/` file is)

Three layers, bottom to top. `drv_motor` only knows how to push power to
a wheel; `drv_encoder` only knows how to count clicks; `sub_motion` is the
brain that reads the clicks, decides the power, and tells everyone else
what happened.

| Your file | Talks to | Through | In plain terms |
|---|---|---|---|
| `drv_motor.c` | `core/rc_pwm` | `rc_pwm_init_pin(pin, 20000)`, `rc_pwm_set_duty(pin, permille)` | "Set this wheel's dimmer to 40 %." Two pins per motor (GP8/9, GP10/11): drive A for forward, B for reverse, both high to brake. |
| `drv_encoder.c` | `core/rc_gpioirq` | `rc_gpioirq_attach(GP0/GP7, RC_EDGE_RISE, …)` | "Ring `encoder_isr` on every rising edge of channel A." Channel B (GP1/GP28) is a plain input the ISR reads for direction. |
| `drv_encoder.c` | `core/rc_time` | `rc_time_us()` inside the ISR | Stamps each click so the gap between clicks gives speed. |
| `drv_encoder.c` | `core/rc_defer` | `rc_defer_register(encoder_drain)`, `rc_defer_signal_i()` | The ISR only counts and rings the bell; `encoder_drain` does the rest in task context. |
| `drv_encoder.c` | `core/rc_event` | `rc_event_publish(RC_EVT_ENCODER_EDGE)` | Lets anyone (telemetry, debugging) watch raw clicks. |
| `sub_motion.c` | `drv_encoder.c` | `drv_encoder_speed_mm_s()`, `drv_encoder_count()`, `drv_encoder_distance_mm()`, `drv_encoder_reset()` | Each wheel's speed for the PID; click counts for "have I gone far enough yet?"; distances for telemetry. |
| `sub_motion.c` | `drv_motor.c` | `drv_motor_set_pair(left, right)`, `drv_motor_stop()` | The PID's output goes here every 20 ms. |
| `sub_motion.c` | `core/rc_event` | publishes `RC_EVT_ODOMETRY` and `RC_EVT_MOTION_DONE`; subscribes to `RC_EVT_MOTION_DONE` | Every 20 ms: "here's my speed/distance". When a move ends: "done" — and `sub_motion` listens to its own "done" to call the requester's callback. |
| `sub_motion.c` | the kernel | a task (`tk_cre_tsk`), a mutex (`tk_cre_mtx`, `tk_loc_mtx`, `tk_unl_mtx`) | Its own background task runs the control loop; the mutex stops two tasks changing the motion state at once. |
| `sub_motion.c` | `core/rc_config.h` | `RC_PERIOD_MOTION_MS`, `RC_PRI_MOTION`, `RC_STACK_SZ`, `RC_ENC_UM_PER_TICK`, `RC_WHEEL_BASE_MM` | Loop rate, task priority and stack, and the measured mechanics your maths depends on. |

Who calls *you*: `sub_line` (Buddy 3) calls `sub_motion_drive(base, steer)`
on every 5 ms line sample; `sub_nav` calls `sub_motion_turn_deg()` for
barcode turns, `sub_motion_turn_deg()` / `sub_motion_forward_mm()` for the
three legs of an obstacle bypass, `sub_motion_drive()` while recovering or
slowing for an obstacle, and `sub_motion_stop()` when the car must stop;
`app_main.c` sets the cruise speed with `sub_motion_set_speed(250)`. None of
them touch `drv_motor` directly — you own the wheels.

**What this module does**

This module is the car's legs and inner ear. It decides how much power to
send to the left and right wheel motors, and it listens to the two wheel
encoders that click every time the wheel turns a fixed amount — the same
idea as counting clicks on a bike's spoke card to know how fast and how
far you've gone — and, because each encoder has a second, offset channel,
can also tell which *way* the wheel is turning. That click-counting is
called **odometry**: turning "number of clicks" into "distance travelled"
and "current speed."

Because motors never spin at exactly the speed you ask for (battery sag,
friction, floor grip all get in the way), this module runs **PID
control** on each wheel — think of a car's cruise control, which watches
your actual speed, compares it to the speed you want, and constantly
nudges the throttle up or down to close the gap. On top of that it keeps
the car driving straight (if one wheel gets ahead, it's slowed down),
turns the car on the spot by driving the wheels in opposite directions,
eases into and out of every move so it stops where it should, and stops
the car on its own if a wheel isn't moving when it should be. Every other
module — line following, obstacle avoidance — just says "go forward
300 mm", "turn 90°" or "steer this much" and trusts this module to make
the wheels do it.

**Run your module** — bench `motion`. The full procedure, what good output
looks like and what each bad symptom means is in
[`BUILD.md`](../../BUILD.md) §5.2. Short version, in VS Code: **Terminal →
Run Task → RoboCar: build bench image…**, pick it from the list, put the
Pico in BOOTSEL, **…flash bench image…**, then open the Serial Monitor. It
runs three phases:

1. **Wheels off the ground:** both motors at 30 % power for 4 s, printing
   each wheel's click count, speed and direction. Checks the wiring.
2. **Car on the floor, 1 m clear:** forward 500 mm (printing the speed log
   for your PID report), backward 500 mm, turn right 90°, turn left 90°,
   U-turn — each printing whether it completed and how far the wheels
   went, with a 4 s pause after each to measure the car.
3. **Motors off:** keeps printing the click counts while you turn a wheel
   by hand. This is where you measure ticks per wheel turn.

**How to get started**

1. **Measure the hardware.** Set these in `core/rc_config.h`, then rebuild:
   - `RC_ENC_TICKS_PER_REV` — ticks per **wheel** turn. Prop the car up
     (wheels off the table for the whole run), flash the `motion` bench and
     wait for phase 3. Put a tape mark on the tyre, write down `cnt`, turn
     the wheel slowly **exactly 10 turns** in one direction, write down
     `cnt` again, and divide the difference by 10. Do both wheels; they
     should agree. The shipped `20` is a placeholder, and until it's right
     every distance and speed is off by the same large factor.
   - `RC_WHEEL_DIAM_MM` — the tyre's outside diameter. Already measured:
     64 mm.
   - `RC_WHEEL_BASE_MM` — centre-to-centre distance between the two
     tyres, needed for the turning maths. `110` is still a guess.
2. **Check the wiring** with phase 1 (wheels off the ground): both counts
   rise and both speeds are positive. `BUILD.md` §5.2 lists what each bad
   symptom means.
3. **Tune the PID.** Run phase 2 on the floor. During the forward move the
   bench prints `t_ms,tgt_l,spd_l,duty_l,tgt_r,spd_r,duty_r` every 40 ms:
   time since the move started, then for each wheel its target speed, its
   measured speed (both mm/s) and its duty (0–1000). Paste the lines into
   a spreadsheet and plot measured speed against target speed over time.
   Good: the measured speed catches the target within about 0.3 s and
   stays close, without overshooting. If not, change **one** constant at
   the top of `sub_motion.c` at a time, rebuild, and run again:

   | What you see | What to change |
   |---|---|
   | Measured speed stays below the target | raise `PID_KI` (e.g. 512 → 768) |
   | Measured speed overshoots, then wobbles | lower `PID_KP` (e.g. 256 → 192) |
   | The car is slow or jerky to get going | raise `FF_START_PERMILLE` (e.g. 150 → 250) |

   Keep the chart for every setting you try — that series is your **PID
   tuning report**.
4. **Calibrate distance.** Measure how far the car really went in phase 2
   with a tape, and compare it with the `wheels travelled … mm` the bench
   printed. If it is consistently off, set `RC_ENC_UM_PER_TICK` in
   `rc_config.h` to a number instead of the formula:
   *new value = old value × tape distance ÷ printed distance*, where the
   old value is `31416 × RC_WHEEL_DIAM_MM ÷ (10 × RC_ENC_TICKS_PER_REV)`.
   For example, old value 335, tape 490 mm, printed 500 mm →
   `#define RC_ENC_UM_PER_TICK (328UL)`.
5. **Calibrate turns.** Measure the real angle of the phase-2 turns (a
   protractor, or a taped line on the floor). Wheels slip when spinning in
   place, so the car usually turns a little less than asked. Set
   `TURN_SLIP_PERMILLE` in `sub_motion.c` to
   *1000 × angle asked ÷ average angle measured* — e.g. asked 90°, got
   80° → `1125`.
6. **Accuracy evaluation.** Run phase 2 ten times (press **RST** to restart
   the bench, put the car back at its start each time) and record the real
   forward distance and both turn angles each time. The average error is
   your accuracy; the spread between best and worst is your
   repeatability. That table is your **motion accuracy evaluation**.

**Code walkthrough**

This is a big module, so it's broken down file by file, and inside each
file, constant by constant and function by function — the same bar of
detail as "`#define STEER_KP (2)` — what it does, why, any links to other
code" that the rest of this walkthrough follows. Jargon is explained the
first time it comes up; skip ahead once you recognise a term.

## `drv_motor.h` / `drv_motor.c` — the "pure output" layer

This layer knows nothing about speed; it just turns electrical power on
and off in a direction. It never reads a sensor. Think of it as the gas
pedal and gearstick, with no speedometer attached — `sub_motion.c` below
is the part that actually watches the speedometer and decides how hard to
press.

**Jargon you need first:**
- **PWM (pulse-width modulation)** — the only way to get a variable
  "amount of power" out of a pin that can only be fully on or fully off.
  The pin is flipped on and off very fast (20,000 times a second here —
  `MOTOR_PWM_HZ`), and the *fraction* of each cycle spent "on" — the
  **duty cycle** — controls the average power delivered. It's the same
  trick as flicking a light switch faster than your eye can follow: flick
  it on 30% of the time and the bulb just looks dimmer, it doesn't
  visibly blink.
- **H-bridge** — the chip on the Robo Pico board that actually pushes
  current through the motor. It has four internal switches arranged so
  that, depending on which pair is turned on, current flows through the
  motor one way (forward) or the other (reverse). Robo Pico exposes that
  as *two* PWM pins per motor (`pin_a` and `pin_b`) instead of one PWM pin
  plus a separate direction wire — driving `pin_a` spins the motor one
  way, driving `pin_b` spins it the other way, and driving both at once
  would short the H-bridge, which the code goes out of its way to avoid.

**Constants:**
- `#define MOTOR_PWM_HZ (20000UL)` — the PWM switching frequency, 20 kHz.
  Chosen because it's above the top of human hearing (so the motor
  doesn't audibly whine at partial power, the way a slower PWM frequency
  would) and comfortably inside what the Robo Pico's output stage can
  switch cleanly. If your motors whine anyway, the comment right above it
  suggests dropping to 10 kHz.
- `#define DUTY_MAX (1000)` — the largest legal duty value in "permille"
  units (parts-per-thousand — this project's integer stand-in for a
  percentage with one extra digit of precision, used everywhere instead
  of a fractional 0.0–1.0 float, because the RP2040 has no FPU — no
  hardware floating-point unit, so real division of fractional numbers is
  slow/unsupported and the whole codebase avoids it — see `TEAM_GUIDE.md`
  §5, "No floating point anywhere"). `drv_motor_set()` clamps every
  incoming duty to ±`DUTY_MAX` so nothing downstream can ever ask for more
  than 100% power.

**The `motor_t` struct** — one instance per motor, kept in the private
`motors[2]` array indexed by `rc_side_t` (`RC_SIDE_LEFT`/`RC_SIDE_RIGHT`):
`pin_a`/`pin_b` are which two GPIO pins drive this motor (numbers come
from `rc_config.h`, the one file all pin numbers live in — see
`TEAM_GUIDE.md` §5); `last` is the most recently commanded signed duty,
kept purely so `drv_motor_get()` can report it later to telemetry.

**Functions:**
- `drv_motor_init(void)` — boot-time setup. Loops over both motors,
  configures PWM on both of their pins at `MOTOR_PWM_HZ`, and sets duty to
  zero on all four pins so the car can't lurch the instant power comes on.
  Call once, before anything else in this module.
- `drv_motor_set(rc_side_t side, int16_t duty_permille)` — the lowest-level
  "make a wheel spin" call in the whole project. Clamps the requested
  duty into ±1000, then — this is the important bit — always drops the
  *opposite* pin to zero before raising the driving pin, so the H-bridge
  is never told to energize both directions at once (see the H-bridge
  explanation above). Positive `duty_permille` drives forward through
  `pin_a`; negative drives reverse through `pin_b`.
- `drv_motor_set_pair(int16_t left_permille, int16_t right_permille)` —
  convenience wrapper that calls `drv_motor_set()` twice. `sub_motion.c`
  uses this almost exclusively, since nearly every control decision
  updates both wheels together.
- `drv_motor_stop(rc_motor_stop_t mode)` — stops both wheels one of two
  ways, chosen by the `rc_motor_stop_t` enum (an **enum**, short for
  "enumeration," is just a named list of integer constants — using
  `RC_MOTOR_COAST`/`RC_MOTOR_BRAKE` instead of raw `0`/`1` makes the call
  site read like English instead of a magic number). `RC_MOTOR_COAST` cuts
  power entirely so the wheel free-spins down under its own friction;
  `RC_MOTOR_BRAKE` drives *both* pins of the H-bridge high at once, which
  intentionally shorts the motor windings together — that's an
  electrical brake, not a physical one, and it stops the wheel much
  faster than coasting.
- `drv_motor_get(rc_side_t side)` — returns whatever duty was last
  commanded (not a live measurement — just "what we last told it to do").
  Telemetry and the motion bench's speed log read it. Wheel direction is
  *measured* by the encoder's B channel (next section), so nothing needs
  to guess it from this.

## `drv_encoder.h` / `drv_encoder.c` — turning wheel clicks into speed and distance

Each wheel has an encoder built into its motor with two outputs, `A` and
`B`. Every step of the motor pulses `A` — hundreds of times per wheel
turn. This is exactly the trick behind counting clicks on a bicycle's
spoke card as the wheel spins — count clicks per second and you know
speed; count total clicks and (knowing how far one click represents) you
know distance travelled. That whole idea — clicks in, distance/speed out
— is what this codebase calls **odometry**, and it's the numeric backbone
this entire module (and Buddy 4's terrain code, and telemetry) relies on.

`B` is the same pulse train shifted by a quarter of a step ("quadrature").
That shift is what gives you direction: at the exact instant `A` rises,
`B` is still low if the wheel turns one way and already high if it turns
the other. One extra pin read inside the interrupt, and direction is a
measurement instead of a guess.

**Jargon you need first:**
- **ISR (Interrupt Service Routine)** — a tiny function the processor
  jumps to *immediately* the instant a watched pin changes voltage,
  pausing whatever normal code was running, running the ISR, then
  resuming exactly where it left off. It exists so time-critical events
  (like "the wheel just clicked") get handled with microsecond precision
  instead of waiting for the next time some loop happens to check the pin.
  See TEAM_GUIDE.md §1.2 for the project-wide "golden rule" about what an
  ISR is and isn't allowed to do.
- **`volatile`** — a C keyword you'll see on every field of the `enc_t`
  struct below. It tells the compiler "this variable can change at any
  moment from outside the normal flow of this function (here: from an
  interrupt), so never assume a cached copy in a CPU register is still
  correct — always re-read it from memory." Without `volatile`, an
  optimizing compiler could quietly break this code by reusing a stale
  value.
- **Critical section (`DI`/`EI`)** — `DI(sts)` disables interrupts and
  saves the previous interrupt-enabled state into `sts`; `EI(sts)`
  restores it. Code between them can't be interrupted, which matters
  whenever it reads more than one `volatile` field that the ISR could
  update — without this, you could read a fresh timestamp paired with a
  stale period, a "torn read."
- **Debounce** — a signal wire can pick up a short electrical glitch
  (here, mostly noise from the motor's own PWM) that looks like an extra
  edge. "Debouncing" means ignoring any second edge that arrives
  suspiciously soon after the first, since it's almost certainly a
  glitch, not a real tick. The Hall-effect encoders switch cleanly, so
  the window only needs to be tiny.

**Constants:**
- `#define STALL_TIMEOUT_US (100000UL)` — if 100 ms pass with no new edge
  on a wheel, `drv_encoder_period_us()` reports `0` ("stopped") instead
  of letting the "time since last edge" number keep growing forever,
  which would otherwise make a stationary wheel look like it's moving at
  an ever-decreasing crawl rather than not moving at all. With hundreds of
  ticks per wheel turn, a moving wheel ticks far more often than that, so
  100 ms without one means the wheel is creeping at a few mm/s at most.
- `#define DEBOUNCE_US (50UL)` — the debounce window described above. It
  must stay well below the real gap between ticks at top speed: at 1000
  ticks per wheel turn and 200 wheel RPM that gap is 300 µs. (The earlier
  500 µs was chosen for a 20-slot optical disc and would throw away real
  ticks at speed.)

**The `enc_t` struct** — one instance per wheel, in the private `encs[2]`
array: `count` is the running total of accepted clicks since boot
(monotonic — only ever increases); `last_us` is the timestamp of the most
recent accepted edge; `period_us` is the microsecond gap between the two
most recent accepted edges — this is what speed gets computed from;
`base_count` is a snapshot of `count` taken at the last
`drv_encoder_reset()`, so "distance since reset" is just
`count - base_count`; `published` is the `count` value already turned
into an event, so the bottom-half worker (below) doesn't re-publish
unchanged data every cycle; `dir` is +1 or −1, the direction read from
channel B on the most recent edge; `pin_a` is the channel-A GPIO that
raises the interrupt and `pin_b` the channel-B GPIO the ISR samples.
Every field except the two pins is `volatile`, because the ISR writes
them and normal code reads them (see jargon above).

**`encoder_isr(pin, level, t_us, ctx)`** — the actual interrupt handler,
attached to rising edges only (`drv_encoder_init`, below, explains why
"rising only" rather than both edges). `ctx` arrives as a generic
`void *` — an untyped pointer — and is cast back to `enc_t *`; this is
the standard C pattern for giving one shared handler function its own
per-instance data (here, "which wheel is this edge for") without writing
two nearly-identical handler functions. Inside, it does the absolute
minimum: compute the gap since the last accepted edge (`delta`), bail out
if that gap is smaller than `DEBOUNCE_US` (a glitch, ignore it), otherwise
read channel B once with `gpio_get_val(e->pin_b)` to set `dir`, record
the new timestamp, store the new period, increment `count`, and call
`rc_defer_signal_i(defer_h)` to wake the bottom-half task. That single
GPIO read is a register read, which the interrupt rule allows. It does
**not** build or publish an event itself — see TEAM_GUIDE.md §1.2's rule
that an ISR may only record state and hand off, never do real work,
because the two encoder pins, the ultrasonic echo pin, and the barcode
pin all share the same interrupt hardware bank; time spent in one ISR is
time another pin's edge is waiting to be noticed, and for the barcode
reader specifically, edge *timing* is the actual data being measured, so
delay there directly corrupts a reading.

**`encoder_drain(ctx)`** — the "bottom half" (deferred work), which runs
later in normal task context rather than inside the interrupt, so it's
safe to do more here, including publishing on the event bus. For each
wheel, it copies `count`/`period_us` out inside a `DI`/`EI` critical
section (see jargon above), skips publishing if nothing changed since the
last drain (`count == published`), and otherwise publishes an
`RC_EVT_ENCODER_EDGE` event carrying the current count and period. Note
this deliberately *coalesces* rapid edges — if three clicks arrive before
the drain task next runs, one event reports the latest state rather than
three separate events replaying each click — because downstream code
cares about "current count and period," not "receiving one event per
physical click," and coalescing keeps a fast-spinning wheel from flooding
the event bus.

**`drv_encoder_init(void)`** — boot-time setup: registers `encoder_drain`
as the deferred worker, zeroes every field of both `enc_t` structs, sets
each channel-B pin up as a plain pulled-up input (no interrupt — the ISR
reads it), and attaches `encoder_isr` to each channel-A pin for **rising
edges only**. The comment in the code explains why not both edges:
counting both edges would double the resolution, but the encoder's
"mark" and "space" aren't the same width, so the timing between a rising
and the next falling edge isn't the same as between two risings — mixing
them would make the period measurement (and therefore speed) noisy and
wrong.

**`drv_encoder_dir(side)`** — returns the `dir` the ISR last stored: +1
forward, −1 reverse.

**`drv_encoder_count(side)`** — raw lifetime click count for one wheel,
no unit conversion, never resets. `sub_motion.c` writes this number down
at the start of every move and subtracts it later to get "clicks since
the move began" (`move_ticks()`, below).

**`drv_encoder_period_us(side)`** — returns the microsecond gap between
the two most recent accepted edges on one wheel, reading both fields
inside a critical section (see jargon above) so they can't be torn apart
by an interrupt mid-read. If more time has already passed since the last
edge than that gap, the wheel must be slowing down, so it returns the
longer time instead — that makes the speed fall smoothly while a wheel
stops, rather than freezing at its last value and fooling the PID. It
returns `0` before the first edge and once the wheel has been silent
longer than `STALL_TIMEOUT_US`.

**`drv_encoder_speed_mm_s(side)`** — the function the PID loop in
`sub_motion.c` reads every control cycle; this is the "actual speed" half
of the cruise-control comparison described under `sub_motion.c` below.
It converts the raw period into millimetres/second:
`speed = (RC_ENC_UM_PER_TICK * 1000UL) / period`. `RC_ENC_UM_PER_TICK`
(defined in `rc_config.h`, not owned by this module but central to it) is
how many micrometres of wheel travel one click represents, computed from
wheel circumference divided by ticks-per-wheel-revolution
(`(31416 * RC_WHEEL_DIAM_MM) / (10 * RC_ENC_TICKS_PER_REV)` — 31416 is
1000×π to three decimal places, again to stay in integers). Multiplying
by 1000 before dividing shifts the units from metres/second to
millimetres/second while the whole calculation stays in whole-number
(integer) arithmetic the entire way through — there's no FPU on this
chip, so a division can never be left half-finished as a fraction; it has
to land on a usable integer immediately. The sign comes from `dir`, the
direction channel B reported on the most recent edge — so a wheel that
is still coasting backwards after you command forward correctly reads
negative until it actually turns around. Which sense of B counts as
"forward" depends on how the encoder is mounted: if a wheel reads negative
while the car drives forward, swap that encoder's A and B wires rather
than negating in software. (`sub_motion.c` only uses the *size* of this
number — see `wheel_measure()` for why.)

**`drv_encoder_distance_mm(side)`** — `(count - base_count)` converted to
millimetres using the same `RC_ENC_UM_PER_TICK` constant: distance since
the last `drv_encoder_reset()`. `sub_motion.c` puts it in the
`RC_EVT_ODOMETRY` event for telemetry and Buddy 4's terrain code; its own
"have I gone far enough?" check counts ticks directly instead.

**`drv_encoder_reset(void)`** — snapshots the current lifetime count of
both wheels into `base_count`, inside a critical section. This does *not*
reset the lifetime `count` — it just moves the zero-point for
`drv_encoder_distance_mm()`. `start_move()` in `sub_motion.c` calls it at
the start of every move, so telemetry's distances restart from zero for
each move.

## `sub_motion.h` / `sub_motion.c` — the brain that ties it together

Where the previous two files only know "spin this hard" and "here's how
fast that wheel is currently going," this file decides *what* duty to
send, moment to moment, to make the car do what the rest of the team
asked for — "drive forward 300 mm," "turn 90°," "drive with this much
steering." The public API is non-blocking (see §1.3 up top): calling
`sub_motion_forward_mm(300, callback, ctx)` returns immediately with a
move ID while the car drives in the background on its own RTOS task; when
the move finishes, `RC_EVT_MOTION_DONE` is published on the event bus and
your callback is called.

### The big picture: one control cycle

Every 20 ms (`RC_PERIOD_MOTION_MS`) the background task, `motion_task()`,
wakes up and does this for a distance move or a turn:

```
 encoders ──► measured speed of each wheel ────────────────┐
                                                           │
 how far the move has got ──► speed profile                │
   (ramp up, cruise, slow down near the goal)              │
        │                                                  ▼
        └──► straight-line correction ──► target speed ──► PID (+ feedforward)
             (slow the wheel that's ahead)   per wheel      per wheel
                                                               │
                                          duty per wheel ◄─────┘
                                               │
                                               ▼
                                       drv_motor_set_pair()
```

Then it checks the safety rules (is a wheel stalled? is the move taking
far too long?) and, once the average of the two wheels has covered the
goal, brakes and reports the move as done.

### Jargon you need first

- **Task** — a function that runs "at the same time" as the rest of the
  program, scheduled by the RTOS (micro T-Kernel). `motion_task()` is
  this module's task: it sleeps for 20 ms, wakes, does one control cycle,
  and sleeps again. Other tasks — the event dispatchers that run
  `sub_nav`'s and `sub_line`'s code, the bench — call this module's
  public functions whenever they like.
- **Feedforward** — a *guess* at the right power before any error is
  measured. If you know a wheel needs roughly 68 % power to do 250 mm/s,
  you start there instead of starting from zero and waiting for the PID
  to find it. The PID then only corrects the difference between the
  guess and reality.
- **PID** — the correction: **P**roportional (react to how wrong the
  speed is *now*), **I**ntegral (react to how long it has *stayed* wrong
  — it builds up the extra push a constant problem like friction or a
  weak battery needs), **D**erivative (react to how fast the speed is
  *changing*, to damp overshoot). Walked through line by line under
  `pid_step()` below.
- **Fixed-point** — storing a fraction as a whole number that has been
  multiplied by an agreed scale. With a scale of 256, a gain of `1.0` is
  stored as `256` and `2.0` as `512`; the code divides by 256 at the end.
  Needed because the chip has no FPU (see the `DUTY_MAX` jargon above).
- **Integral windup / anti-windup** — if a wheel is blocked, its error
  never shrinks, so the integral keeps growing. The moment the wheel is
  freed, all that stored-up push is released as a jolt. *Anti-windup*
  means stopping the integral from growing while it can't help.
- **Low-pass filter** — smoothing a jumpy reading by blending each new
  value with the previous smoothed one, so single-sample spikes don't
  shake the motors.
- **Speed profile / ramp** — instead of jumping straight to full speed
  (wheel spin, a jolt) and stopping from full speed (overshooting the
  goal), a move speeds up gradually, cruises, then slows down near the
  end.
- **Mutex** — short for "mutual exclusion": a lock that only one task
  can hold at a time, like the single key to a shared room. The motion
  state is changed both by `motion_task()` and by whichever task calls
  the public functions; without the lock, a new request could land
  half-way through a control cycle and leave the state mixed up. Every
  piece of code that reads or changes the motion state takes the key
  first (`motion_lock()`) and gives it back straight after
  (`motion_unlock()`).
- **Callback** — a function you hand over now, to be called later when
  the thing you asked for is done (see `sub_motion.h`).

### Tuning constants

All at the top of `sub_motion.c`. Each is a starting value worked out from
the hardware's rough specs, not a measurement — see "How to get started"
for how to replace them.

- `#define DUTY_MAX (1000)` — the largest duty, in permille (100.0 %), the
  same limit `drv_motor.c` clamps to.
- `#define PID_SCALE (256)` — the fixed-point scale for the PID gains: a
  gain of 1.0 is stored as 256.
- `#define PID_KP (256)`, `PID_KI (512)`, `PID_KD (0)` — the three PID
  gains (1.0, 2.0 and 0, stored ×256). The speed error is in mm/s and the
  output is duty in permille, so `PID_KP` 1.0 means "50 mm/s too slow →
  +50 permille at once", and `PID_KI` 2.0 means "still 50 mm/s too slow →
  another +100 permille for every second it stays that way". `PID_KD`
  starts at 0 (so the controller begins as PI): the measured speed jitters
  from tick to tick and D amplifies jitter. Try a small value such as 8
  while tuning and see whether it helps.
- `#define FF_START_PERMILLE (150)` — feedforward: the duty at which a
  wheel on the floor just starts to turn. Below that, a motor only hums
  (friction, the motor's "deadband").
- `#define FF_TOP_SPEED_MM_S (400)` — feedforward: the wheel speed at
  100 % duty, car on the floor. Together with the previous constant it
  draws a straight line "speed → duty" (`feedforward()`, below).
- `#define MIN_SPEED_MM_S (60)` — every move starts at this speed and
  slows back down to it at the end. It's also the lowest cruise speed
  allowed, so a move can always finish.
- `#define ACCEL_MM_S_PER_CYCLE (20)` — how much faster the move may go
  each 20 ms cycle while ramping up: 20 mm/s per 20 ms = 1 m/s², gentle
  enough not to spin the wheels. From 60 to 250 mm/s takes about 0.2 s.
- `#define DECEL_ZONE_MM (60U)` — over the last 60 mm of a distance move,
  the speed slides down to `MIN_SPEED_MM_S`, so the brake at the goal
  stops the car close to it instead of coasting past.
- `#define TURN_SPEED_MM_S (150)` — how fast each wheel runs along its
  arc during a spin turn (slower than straight driving, for accuracy).
- `#define TURN_DECEL_ZONE_MM (30U)` — the same slow-down idea for turns,
  measured along each wheel's arc (a 90° turn is only ~86 mm of arc).
- `#define TURN_SLIP_PERMILLE (1000U)` — wheels slip when spinning in
  place, so a turn usually falls short of the geometry. This stretches
  every turn: 1000 = trust the geometry, 1100 = drive each wheel 10 %
  further. Calibrate it as in "How to get started" step 5.
- `#define SYNC_GAIN (4)` and `SYNC_MAX_MM_S (80)` — the straight-line
  correction: for every mm one wheel is ahead of the other, that wheel's
  target speed drops by 4 mm/s and the other's rises by 4 mm/s, capped at
  80 mm/s (and at half the current speed).
- `#define STALL_DUTY_PERMILLE (500)` and `STALL_CYCLES (30U)` — the stall
  guard: a wheel pushed at 50 % duty or more that reports no movement for
  30 cycles in a row (0.6 s) is stalled — blocked, or its encoder is
  unplugged — and the car stops instead of driving blind at full power.
- `#define MOVE_TIMEOUT_EXTRA_MS (2000U)` — the timeout guard: a move is
  stopped if it takes more than twice its expected time plus 2 s (e.g.
  the motors can't reach the speed).
- `#define LOCK_TMO_MS (20)` — the longest any caller waits for the mutex.
  `motion_task()` only holds it for a few tens of microseconds per cycle,
  so this is just a safety net.
- `#define CB_SLOTS (8U)` and `DONE_QUEUE (8U)` — room for callbacks
  waiting to be delivered and finished moves waiting to be announced. At
  most two are ever in flight (a move that just finished and the one that
  replaced it); the rest is spare.

### Types

- **`mode_t`** — what the car is doing right now:
  - `MODE_IDLE` — motors off, nothing to do.
  - `MODE_DISTANCE` — driving straight to a distance goal, forward or back.
  - `MODE_TURN` — spinning in place to an angle.
  - `MODE_CONTINUOUS` — open-loop drive for the line follower
    (`sub_motion_drive()`): duty in, no feedback.
  - `MODE_SPEED` — closed-loop drive (`sub_motion_drive_speed()`): speed
    in, PID holds it.

  The first three are "queued moves" in the sense of the API: they have a
  goal, a move ID and a callback. The two continuous modes just keep going
  until told otherwise.
- **`pid_t`** — one wheel's speed controller: the gains `kp`, `ki`, `kd`
  (×`PID_SCALE`); `i_term`, the integral's contribution so far (stored
  already multiplied by its gain, so the anti-windup limit is simply "one
  full duty"); and `prev_meas`, last cycle's measured speed, for the D
  term.
- **`wheel_t`** — everything the loop keeps about one wheel: its `pid`;
  `dir` (+1 forward, −1 reverse this cycle); `target` (the speed it's
  asked for, mm/s); `raw` (the measured speed) and `meas` (the smoothed
  speed the PID actually uses); `duty` (what was sent, 0–1000);
  `start_count` (the encoder count when the current move began); and
  `stall_cycles` (how many cycles in a row it has been pushed hard without
  moving). The two wheels live in `wheels[2]`, indexed by `rc_side_t`.
- **`cb_slot_t`** — a callback waiting for its move to finish: the move
  `id` (0 = empty slot), the function `cb`, and the caller's `ctx`.
- **`done_t`** — a finished move waiting to be announced: its `id`,
  whether it `completed`, and `travelled_mm`.

### State

All `static`, so private to this file, and all guarded by the mutex:

- `motion_mtxid`, `motion_tskid` — the kernel IDs of the mutex and task.
- `mode` — the current `mode_t`. It's also `volatile`, because
  `sub_motion_busy()` reads it without taking the lock.
- `wheels[2]` — the two `wheel_t`s.
- `cruise_mm_s` — the cruise speed for distance moves (default 250 mm/s),
  set by `sub_motion_set_speed()`.
- For the queued move in progress: `goal_ticks` (how many ticks the
  wheels must average), `decel_ticks` (when to start slowing down),
  `ramp_mm_s` (the speed limit while ramping up), `move_cycles` and
  `timeout_cycles` (the timeout guard), `move_sign` (+1 or −1 — forward /
  backward for a distance move, right / left for a turn), `move_id_cur`
  (0 = no move) and `move_id_next` (the next ID to hand out).
- For the continuous modes: `base_cmd` / `steer_cmd` (permille, open loop)
  and `base_spd` / `steer_spd` (mm/s, closed loop).
- `cb_slots[]`, `done_q[]` and `done_n` — the callback and announcement
  queues.
- `fault_msg` — a stall or timeout message, printed by `motion_task()`
  *after* it releases the lock (printing is slow and must not happen
  while holding it).

### Small helpers

- **`motion_lock()` / `motion_unlock()`** — take and give back the mutex
  "key" (`tk_loc_mtx` with a `LOCK_TMO_MS` timeout / `tk_unl_mtx`).
  `motion_lock()` returns false if the key didn't come back in time — it
  shouldn't happen, and callers treat it as "busy".
- **`queued_move_active()`** — true in `MODE_DISTANCE` or `MODE_TURN`.
- **`move_ticks(side)`** — ticks one wheel has turned since the current
  move began: today's `drv_encoder_count()` minus `start_count`. It counts
  every tick whichever way the wheel turns, which is what a move needs —
  the motors are driven the way the move asks, so the count is simply how
  far the wheel went.
- **`um_to_ticks(um)`** — micrometres to encoder ticks, rounded to the
  nearest tick. It uses 64-bit numbers because a long distance in
  micrometres doesn't fit in 32 bits; it runs once per move, so the cost
  doesn't matter.
- **`ticks_to_mm(ticks)`** — the reverse, for reporting `travelled_mm`.
- **`clamp_duty(v)`** — keeps a duty inside −1000..+1000.
- **`cruise_speed(m)`** — the speed a move cruises at: `TURN_SPEED_MM_S`
  for turns, `cruise_mm_s` for distance moves, never below
  `MIN_SPEED_MM_S` (a cruise speed of 0 would never arrive).

### The PID, with feedforward

**`pid_reset(p, meas)`** — clears the integral and sets `prev_meas` to the
wheel's current speed. Called at the start of every move and whenever a
wheel changes direction, so leftover memory from the last move can't leak
into the next one.

**`feedforward(v)`** — the guessed duty for `v` mm/s: 0 for no speed,
otherwise a straight line from `FF_START_PERMILLE` (just moving) up to
1000 at `FF_TOP_SPEED_MM_S`:
`FF_START_PERMILLE + v × (1000 − FF_START_PERMILLE) / FF_TOP_SPEED_MM_S`.
With the shipped values, 250 mm/s → 150 + 250 × 850 / 400 = **681**
permille.

**`pid_step(p, target, meas)`** — one control step for one wheel. Both
speeds are ≥ 0 (direction is applied later, when the duty is sent); it
returns the duty, 0–1000. Line by line:

1. **No speed wanted** (`target <= 0`): reset the controller and return 0
   — the wheel coasts.
2. **Error:** `err = target − meas`. Positive means "too slow, push
   harder".
3. **D term, on the measurement:**
   `d = −(meas − prev_meas) × 1000 / 20` — how fast the measured speed
   changed, in mm/s per second, negated so a wheel that's speeding up
   quickly is held back. It uses the measured speed rather than the error
   on purpose: at the start of a move the *target* jumps, and an error-
   based D would see that jump as a huge "change" and kick the motor
   ("derivative kick"). The measured speed doesn't jump, so there's no
   kick.
4. **Add everything up:**
   `out = (feedforward × 256 + kp × err + i_term + kd × d) / 256`.
   Everything except `i_term` is multiplied by its ×256 gain here, and the
   one division by `PID_SCALE` at the end undoes the scaling.
5. **Anti-windup:** the integral grows —
   `i_term += ki × err × 20 / 1000` (error × time) — *only* if the output
   can still act on it. If the duty is already at full power and the
   wheel is still too slow (or at zero and still too fast), growing the
   integral would only store up a jolt for later, so it's skipped. The
   integral is also capped at ±one full duty.
6. **Clamp to 0–1000.** It never commands the opposite direction to slow
   down; it eases off to 0 and lets friction (or the brake at the end of
   a move) do it.

A worked example with the shipped gains: the target is 250 mm/s and the
wheel measures 200. Feedforward gives 681; P adds 256 × 50 / 256 = **50**;
the integral grows by 512 × 50 × 20 / 1000 = 512, i.e. **+2 permille** per
cycle (+100 per second) for as long as the wheel stays 50 mm/s slow.

### The wheels

**`wheel_measure(w, speed)`** — takes this cycle's speed for one wheel.
`raw` is the size of the encoder speed; `meas`, the value the PID uses, is
halfway between `raw` and the previous `meas` — a simple low-pass filter
that smooths tick-to-tick jitter for about 20 ms of lag. Only the *size*
is used, because the direction comes from the motor command: a loose B
wire makes the encoder's direction wrong, and a PID that believed it
would think "going backwards" and drive the wheel to full power.

**`wheel_control(w, dir, target)`** — runs one wheel's PID towards
`target` mm/s in direction `dir`. If the direction changed since last
cycle it resets the PID first. It stores `target` and the resulting
`duty`, and updates the stall watch: if the duty is at least
`STALL_DUTY_PERMILLE` and the wheel reports zero speed, `stall_cycles`
goes up by one; otherwise it goes back to 0. Nothing is sent to the
motors yet.

**`wheels_apply()`** — sends both wheels' duty × direction to
`drv_motor_set_pair()`.

**`wheels_zero()`** — forgets both wheels' targets, duties and stall
counts and resets their PIDs; used whenever the motors are stopped.

**`stall_check()`** — returns a message naming the stalled wheel if
either wheel's `stall_cycles` reached `STALL_CYCLES`, otherwise `NULL`.

### Starting a move

**`start_move(m, goal_um, sign, cb, ctx)`** — the shared setup behind
`sub_motion_forward_mm()`, `sub_motion_backward_mm()` and
`sub_motion_turn_deg()`. Called with the lock held. In order:

1. If a move is already running, it's pre-empted: `finish_move(false)`
   ("that one didn't complete — a newer request took over").
2. Hands out a fresh move ID (skipping 0, which means "no move") and, if
   a callback was given, stores it in a free `cb_slots[]` entry under that
   ID (`cb_register()`).
3. Calls `drv_encoder_reset()` (telemetry's distances restart at 0),
   writes down each wheel's `start_count`, clears the stall counts and
   resets both PIDs.
4. Converts the goal to ticks (`goal_ticks`) and the slow-down zone to
   ticks (`decel_ticks`), starts the ramp at `MIN_SPEED_MM_S`, and
   computes the timeout: expected time in ms = micrometres ÷ (mm/s) —
   the units cancel to milliseconds — then ×2, plus
   `MOVE_TIMEOUT_EXTRA_MS`, converted to control cycles.
5. Records `move_sign` and the ID, and finally sets `mode` — from this
   moment `motion_task()` drives the move.

Worked numbers (assuming `RC_ENC_TICKS_PER_REV` comes out at 600, so one
tick = 31416 × 64 / 6000 = **335 µm**): `forward_mm(500)` → 500,000 µm →
**1,493 ticks**, slow down for the last 60 mm = **179 ticks**, timeout
500,000 / 250 × 2 + 2,000 = **6,000 ms** (300 cycles).

**`queue_move(...)`** — takes the lock, calls `start_move()`, gives the
lock back, and returns the new move ID (0 if the lock wasn't available).

### Every cycle

**`profile_speed(progress)`** — the speed the move should be at now,
given `progress` (ticks covered so far):

- start from the cruise speed;
- **ramp up:** raise `ramp_mm_s` by `ACCEL_MM_S_PER_CYCLE` and don't go
  faster than it;
- **slow down:** inside the last `decel_ticks`, slide linearly from the
  cruise speed down to `MIN_SPEED_MM_S` —
  `MIN + (cruise − MIN) × remaining / decel_ticks`;
- never below `MIN_SPEED_MM_S`.

**`sync_correction(t_l, t_r, v)`** — the straight-line correction. The
difference between the two wheels' tick counts *is* the car's heading
error: if the left wheel has gone further, the car has turned right. The
correction is `diff ticks × µm per tick / 1000 × SYNC_GAIN`, in mm/s,
capped at `SYNC_MAX_MM_S` and at half the current speed. It's positive
when the left wheel is ahead; `run_move()` then slows the left wheel and
speeds up the right by that amount. For example, with the left wheel
3 ticks (about 1 mm) ahead: 3 × 335 / 1000 × 4 ≈ **4 mm/s** of correction.
The multiplication is done in 64 bits because a blocked wheel can fall far
behind before the stall guard stops the move, and the product would then
overflow 32 bits.

**`run_move()`** — one cycle of a distance move or turn:

1. `progress` = the average of the two wheels' `move_ticks()`.
2. **Arrived?** If `progress ≥ goal_ticks`: `finish_move(true)`. Done.
3. **Too slow?** If this cycle is past `timeout_cycles`: set the message
   "move took too long" and `finish_move(false)`.
4. **Directions.** Distance move: both wheels go `move_sign`. Turn: the
   left wheel goes `move_sign` and the right the opposite — turning right
   (+1), the left wheel drives forward and the right one backward, so the
   car spins clockwise on the spot.
5. **Speeds.** `v = profile_speed(progress)`,
   `corr = sync_correction(...)`, then left target `v − corr`, right
   target `v + corr`, each through `wheel_control()`.
6. **Stalled?** If `stall_check()` found a wheel: set the message and
   `finish_move(false)`. Otherwise `wheels_apply()` sends the duties.

**`run_speed()`** — one cycle of `MODE_SPEED`: left target
`base_spd + steer_spd`, right `base_spd − steer_spd` (a negative target
means that wheel runs backwards), both through `wheel_control()`. If a
wheel stalls, the car brakes and goes idle with a message.

**`run_continuous()`** — one cycle of `MODE_CONTINUOUS`, the line
follower's open-loop drive: left duty `base_cmd + steer_cmd`, right duty
`base_cmd − steer_cmd`, clamped to ±1000 and sent straight to the motors.
Positive steer (`sub_line`'s "steer right") speeds up the left wheel and
slows the right, pivoting the car right. Nothing here reads the encoders,
so line following works even before they're wired or calibrated; the
price is that the speed sags as the battery drains.

### Finishing a move and delivering callbacks

**`finish_move(completed)`** — ends the queued move: works out
`travelled_mm` (the average of both wheels' ticks since the move began,
in mm — for a turn, how far each wheel ran along its arc), brakes both
motors, zeroes the wheels, sets `MODE_IDLE`, adds a `done_t` to
`done_q[]`, and calls `publish_done()`. `completed` is true only when the
goal was reached; false when the move was stopped, replaced by a newer
one, or stopped by the stall or timeout guard.

**`publish_done()`** — publishes every waiting `done_t` as an
`RC_EVT_MOTION_DONE` event. If the event bus's queue is full, the rest
stay in `done_q[]` and `motion_task()` tries again next cycle, so a
finished move is never silently lost — a lost one would leave `sub_nav`
waiting forever.

**`on_motion_done(evt, ctx)`** — this module's own subscriber to
`RC_EVT_MOTION_DONE`, on the fast lane. It finds the callback stored
under that move's ID, frees the slot, gives the lock back, and **then**
calls the callback. Why this detour through the event bus instead of
calling the callback straight from `finish_move()`?

- The callback runs in the **fast dispatcher task** — the same task as
  `sub_nav`'s other event handlers — so navigation code never runs in two
  tasks at once. That's what `TEAM_GUIDE.md` §1.3 promises: "my_callback
  gets called automatically (via the event bus)".
- The callback runs **without the lock held**, so it can safely start the
  next move straight away — `sub_nav`'s obstacle bypass does exactly that
  (turn → forward → turn back, each started from the previous one's
  callback). Called from inside `finish_move()`, it would try to take a
  lock that's already taken and the car would freeze.

### The control task

**`motion_task(stacd, exinf)`** — the background task created in
`sub_motion_init()`. Forever:

1. **Sleep** 20 ms with `tk_dly_tsk(RC_PERIOD_MOTION_MS)` — a real RTOS
   sleep; other tasks use the CPU meanwhile (§1.3's non-blocking rule at
   the task level).
2. **Odometry:** read both wheels' speed and distance and publish
   `RC_EVT_ODOMETRY` — every cycle, whatever the mode, so telemetry and
   Buddy 4's terrain code always have fresh numbers.
3. **Take the lock**, feed both speeds to `wheel_measure()`, and run the
   current mode: `run_move()`, `run_speed()` or `run_continuous()`
   (nothing when idle).
4. `publish_done()` again, to retry anything a full queue refused.
5. **Give the lock back**, then print the fault message if one was set
   (`[motion] left wheel not turning … - stopped` or
   `[motion] move took too long - stopped`).

### Public API (`sub_motion.h`)

- **`sub_motion_init()`** — boot-time setup, called once from
  `app_main.c`: sets both wheels' PID gains, creates the mutex (with
  `TA_INHERIT`: while a higher-priority task waits for the key,
  `motion_task` borrows that priority so the wait stays short),
  subscribes `on_motion_done`, and creates and starts `motion_task` at
  priority `RC_PRI_MOTION`.
- **`sub_motion_set_speed(mm_s)`** — cruise speed for distance moves.
  Takes effect at once, even on a move already running.
- **`sub_motion_forward_mm(mm, cb, ctx)`** — `moveForward(distance)`.
  Returns the move ID at once; the car drives in the background.
- **`sub_motion_backward_mm(mm, cb, ctx)`** — `moveBackward(distance)`:
  the same move with both wheels reversed.
- **`sub_motion_turn_deg(deg, cb, ctx)`** — `turnLeft(angle)` /
  `turnRight(angle)`: `deg < 0` turns left (anticlockwise), `deg > 0`
  turns right — the convention `sub_nav` uses. Spinning in place, each
  wheel runs along a circle whose diameter is the wheel base, so for
  `deg` degrees it covers `RC_WHEEL_BASE_MM × π × deg / 360` mm. In
  integers: `base × 31416 × deg / 3600` micrometres (31416 = π × 10⁴),
  then × `TURN_SLIP_PERMILLE` / 1000. Worked numbers: 90° with a 110 mm
  base → 86,394 µm → **258 ticks** per wheel (at 335 µm per tick).
- **`sub_motion_drive(base_permille, steer_permille)`** — the line
  follower's open-loop drive (`MODE_CONTINUOUS`). Returns `RC_ERR_BUSY`,
  changing nothing, while a queued move or turn is running: switching
  mode mid-move would drop that move without its callback ever firing.
- **`sub_motion_drive_speed(base_mm_s, steer_mm_s)`** — the closed-loop
  version (`MODE_SPEED`): the PID holds the left wheel at base + steer and
  the right at base − steer, so the speed stays the same whatever the
  battery level. Needs working, calibrated encoders. Same busy rule. The
  line follower doesn't use it yet — that's a decision for Buddy 3 and
  you.
- **`sub_motion_stop(brake)`** — `stop()`: cancels whatever is running (a
  queued move reports `completed = false`) and stops the motors —
  `brake = true` stops fast, `false` coasts. If the lock isn't available
  it stops the motors anyway: stopping must never be refused.
- **`sub_motion_busy()`** — true while a queued move is running; false in
  the continuous modes and when idle.
- **`sub_motion_target_mm_s(side)`** — the speed the controller is asking
  one wheel for right now (negative when reversing; 0 when idle or in
  open-loop drive). For logs such as the bench's step response only.

### Follow one move from start to finish

A barcode says "turn right", so `sub_nav` (running in the fast dispatcher
task) calls `sub_motion_turn_deg(90, cmd_done, NULL)`:

1. **The request** — `sub_motion_turn_deg()` works out the arc (86,394 µm)
   and calls `queue_move()`, which takes the lock and runs
   `start_move()`: if the line follower had the car in `MODE_CONTINUOUS`,
   that simply stops (no callback involved); `cmd_done` is stored under a
   new ID, say 7; `goal_ticks` = 258; `mode` = `MODE_TURN`. The lock is
   given back and the call returns 7. `sub_nav` carries on.
2. **Every 20 ms** — `motion_task()` wakes, measures both wheels and runs
   `run_move()`. The ramp lifts the target from 60 towards 150 mm/s; the
   left wheel is driven forward, the right one backward; if one wheel
   gets ahead, `sync_correction()` evens them out, so the car pivots
   about its centre.
3. **Near the end** — inside the last 30 mm of arc (90 ticks) the target
   slides down towards 60 mm/s.
4. **Arrival** — once the average reaches 258 ticks, `finish_move(true)`
   brakes, sets `MODE_IDLE` and publishes `RC_EVT_MOTION_DONE` for ID 7.
   Well under a second has passed.
5. **The callback** — the fast dispatcher delivers the event to
   `on_motion_done()`, which finds `cmd_done` under ID 7 and calls
   `cmd_done(7, true, 86, NULL)`. `sub_nav` goes back to following the
   line, and `sub_line` starts calling `sub_motion_drive()` again.

Had a wheel been blocked, step 4 would instead be the stall guard after
0.6 s: `finish_move(false)`, `[motion] … not turning … - stopped` on the
console, and `cmd_done(7, false, …)` — which `sub_nav` treats as "stop the
car".

## What's left for Buddy 2

The code for everything in the brief is written, and it passed a set of
simulated tests on a PC (straight moves with one motor weaker, turns,
stalls, timeouts, callbacks) — but **it has not run on the real car yet**.
What's left is the work that needs the car:

- **Wire it and check it** — phase 1 of the `motion` bench (wiring,
  directions).
- **Measure** `RC_ENC_TICKS_PER_REV` (still the placeholder `20`) and
  `RC_WHEEL_BASE_MM` (still a guess) — "How to get started" step 1.
- **Tune the PID** — `PID_KP`, `PID_KI`, `PID_KD`, `FF_START_PERMILLE`,
  `FF_TOP_SPEED_MM_S` are placeholders. The speed logs you record while
  tuning are your **PID tuning report**.
- **Calibrate** distance (`RC_ENC_UM_PER_TICK`) and turns
  (`TURN_SLIP_PERMILLE`) — steps 4 and 5.
- **Motion accuracy evaluation** — step 6.
- **Integration** — run the mission image (`BUILD.md` §5.10) with the
  team and check barcode turns and the obstacle bypass on the real track;
  agree with Buddy 3 whether line following should move to
  `sub_motion_drive_speed()` for a consistent speed.
