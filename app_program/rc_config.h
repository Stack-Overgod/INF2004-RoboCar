/*
 *  rc_config.h
 *
 *  Single place for every pin number and tuning constant in the robot.
 *  Nothing else in the tree hard-codes a GPIO.
 *
 *  Target: Raspberry Pi Pico W on a Cytron Robo Pico carrier, running
 *  micro T-Kernel 3.0 (mtk3smp-rp2040 port).
 *
 *  Read docs/HARDWARE.md before changing anything here. Several of these
 *  pins are fixed by the Robo Pico board and cannot be moved.
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
 *  4-wire Grove cable: GND, 3V3, and the two GPIOs listed. The only
 *  exception is the status LED, which hangs off the breakout header.
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
 * the USB console (make CONSOLE=usb_cdc) - a UART build would drive GP0 as
 * TX and fight the encoder. */
#define RC_PIN_ENC_L_A          (0U)
#define RC_PIN_ENC_L_B          (1U)
#define RC_PIN_ENC_R_A          (7U)
#define RC_PIN_ENC_R_B          (28U)

/* HC-SR04 on Grove 2 (GP2/GP3).
 * ECHO is 5 V on this module and MUST go through a divider. See HARDWARE.md. */
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

/* IR barcode sensor on Grove 6 (GP26/GP27). Both are inputs to the Pico:
 * AOUT on the ADC for calibration, DOUT on an edge interrupt for decoding. */
#define RC_PIN_IR_BARCODE_A     (26U)   /* ADC channel 0 */
#define RC_ADC_CH_IR_BARCODE    (0U)
#define RC_PIN_IR_BARCODE_D     (27U)

/* Liveness LED on the GPIO breakout header. The port defaults
 * BOARD_LED_PIN to GP16, which is line sensor 1 above, so override it in
 * sysdef.h to this pin. */
#define RC_PIN_STATUS_LED       (19U)

/* IMU on Grove 3 (GP4/GP5). Those pins are I2C0 SDA/SCL in silicon, so the
 * IMU is on I2C unit 0. The stock driver maps I2C0 to GP8/GP9 (the right
 * motor), so its unit-0 pin table is patched to GP4/GP5. See HARDWARE.md. */
#define RC_I2C_UNIT_IMU         (0U)
#define RC_PIN_I2C0_SDA         (4U)
#define RC_PIN_I2C0_SCL         (5U)

/* LSM303DLHC has two separate I2C addresses, one per die. */
#define RC_I2C_ADDR_ACCEL       (0x19U)
#define RC_I2C_ADDR_MAG         (0x1EU)

/* ------------------------------------------------------------------ *
 *  Timing
 * ------------------------------------------------------------------ */

#define RC_TICK_PERIOD_MS       (1)     /* kernel tick, matches the port */

#define RC_PERIOD_MOTION_MS     (20)    /* PID update rate, 50 Hz */
#define RC_PERIOD_LINE_MS       (5)     /* line sampling, 200 Hz */
#define RC_PERIOD_IMU_MS        (10)    /* IMU sampling, 100 Hz */
#define RC_PERIOD_SCAN_MS       (60)    /* one ultrasonic ping per step */
#define RC_PERIOD_TELEM_MS      (250)   /* telemetry publish rate */

/* ------------------------------------------------------------------ *
 *  Task priorities. Lower number wins in micro T-Kernel.
 * ------------------------------------------------------------------ */

#define RC_PRI_DEFER            (4)   /* bottom halves, above all */
#define RC_PRI_DISPATCH_FAST    (5)
#define RC_PRI_MOTION           (6)
#define RC_PRI_SENSE            (7)
#define RC_PRI_DISPATCH_SLOW    (9)
#define RC_PRI_TELEMETRY        (10)
/* Reserved. sub_nav has no task of its own today - its handlers run in
 * the FAST dispatcher - but keep the slot so a future nav task lands
 * between Sense and the SLOW dispatcher. */
#define RC_PRI_NAV              (8)

#define RC_STACK_SZ             (4096)  /* SMP kernel paths are deep */

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
 * TODO Buddy 2: 20 is a placeholder until that measurement is done. Until
 * then every distance and speed is wrong by the same large factor. */
#define RC_ENC_TICKS_PER_REV    (20U)
#define RC_WHEEL_DIAM_MM        (64U)   /* measured: tyre outside diameter */
#define RC_WHEEL_BASE_MM        (110U)  /* centre to centre of the wheels */

/* Distance per encoder edge, in micrometres, to stay in integer maths.
 * There is no FPU on the RP2040, so everything downstream is fixed point. */
#define RC_ENC_UM_PER_TICK      ((31416UL * (unsigned long)RC_WHEEL_DIAM_MM) \
                                 / (10UL * (unsigned long)RC_ENC_TICKS_PER_REV))

/* ------------------------------------------------------------------ *
 *  Ultrasonic
 * ------------------------------------------------------------------ */

#define RC_ULTRA_TIMEOUT_US     (30000UL)  /* ~5 m, give up after this */
#define RC_ULTRA_TRIG_US        (12UL)     /* datasheet asks for >= 10 us */
#define RC_ULTRA_MAX_MM         (4000U)
#define RC_ULTRA_MIN_MM         (20U)

/* ------------------------------------------------------------------ *
 *  Servo scan geometry
 * ------------------------------------------------------------------ */

#define RC_SCAN_COARSE_START    (30)
#define RC_SCAN_COARSE_END      (150)
#define RC_SCAN_COARSE_STEP     (30)
#define RC_SCAN_FINE_STEP       (6)
#define RC_SCAN_MAX_POINTS      (32U)

/* ------------------------------------------------------------------ *
 *  Event bus sizing
 * ------------------------------------------------------------------ */

#define RC_EVENT_RING_SZ        (64U)   /* must be a power of two */
#define RC_EVENT_MAX_SUBS       (32U)   /* total subscriptions per lane */

/* ------------------------------------------------------------------ *
 *  Buddy 1 - communication, command and telemetry
 *
 *  RC_NET_ENABLE is the master switch for the WiFi radio path. It is OFF
 *  by default so the graded mission image builds and links exactly as
 *  before, with telemetry going to the USB console. Turn it on ONLY in a
 *  build whose port links the Pico W WiFi profile (pico_cyw43_arch_lwip_
 *  threadsafe_background) and the lwIP MQTT app - see
 *  docs/buddy1-telemetry/MQTT.md. With it off, the UDP and MQTT sink
 *  accessors return NULL and pull in no network headers at all.
 * ------------------------------------------------------------------ */

#ifndef RC_NET_ENABLE
#define RC_NET_ENABLE           (0)
#endif

/* Topic namespace. One base per car so several cars can share a broker:
 * car/+/state addresses every car, car/01/# just this one. Buddy 1 owns
 * this; sub_telemetry.c builds every publish topic from it. */
#define RC_TOPIC_BASE           "car/01"
/* The one topic we subscribe to, for inbound commands. */
#define RC_TOPIC_CMD            RC_TOPIC_BASE "/cmd"

/* WiFi association. Fill SSID/pass for your demo hotspot. Country matters
 * for the CYW43 regulatory domain; keep it correct for legal TX power. */
#define RC_WIFI_SSID            "robocar-net"
#define RC_WIFI_PASS            "changeme123"
#define RC_WIFI_COUNTRY         "SG"
#define RC_WIFI_CONNECT_TMO_MS  (15000U)

/* MQTT broker (e.g. a laptop running mosquitto). An IPv4 literal avoids a
 * DNS round trip the port's lwIP profile may not have configured. */
#define RC_MQTT_BROKER_IP       "192.168.4.1"
#define RC_MQTT_BROKER_PORT     (1883U)
#define RC_MQTT_CLIENT_ID       "robocar-01"
#define RC_MQTT_KEEPALIVE_S     (10U)

/* UDP telemetry destination, used by the intermediate UDP sink (prove the
 * network path before bringing MQTT up). Point it at the listening host. */
#define RC_UDP_DEST_IP          "192.168.4.2"
#define RC_UDP_DEST_PORT        (5005U)

/* Reconnect backoff for the telemetry task. On a dropped link it closes,
 * waits (doubling from MIN up to MAX), and reopens - never a busy loop. */
#define RC_NET_BACKOFF_MIN_MS   (500U)
#define RC_NET_BACKOFF_MAX_MS   (8000U)

#endif /* RC_CONFIG_H */
