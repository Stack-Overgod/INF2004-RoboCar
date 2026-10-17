/*!
 * @file  subsystem3_test.c
 * @brief On-target tests for subsystem 3: IR sensor ON/off and Code 39.
 *
 * Built only with make TEST=1. app_main.c calls subsystem3_test() once,
 * after every subsystem has started. Each check prints one "[TEST]" line
 * (test_report.h); record the results in testing/subsystem3/TEST_LOG.md.
 *
 * Set-up: a Pico W (or the Robo Pico) with the three IR modules only, the
 * trim pots already set. No car: steering stays disabled. You move the
 * sensors by hand, 5-10 mm above the paper. Every manual step waits for
 * you to move a sensor, so there is time to open the serial monitor.
 *
 * Test 1, sensor ON/off (S3-U02..S3-U05): for each sensor in turn (L, R,
 * BC), hold it still over white paper until it reads off, then move it
 * across the black line and back, three times. Every change is printed:
 * ON = over black, off = over white. A sensor that reads ON over white
 * (reversed polarity) cannot pass.
 *
 * Test 2, Code 39 decode (S3-U06..S3-U12): part 1 feeds known bar widths
 * straight to the decoder, with no hardware. Part 2 arms the barcode
 * sensor; sweep it across the printed barcode and every decoded letter is
 * printed.
 *
 * NOTE: The decoder as it is today also reports the '*' start and stop
 * guards; after fix F1 (code-fixes.md) it reports the letter only. Guard
 * results are printed but never counted, so both versions pass.
 *
 * NOTE: Deviation from BARR-C 4.2.a / 4.3.c. By the template's design,
 * subsystem3_test() is declared in app.h (included through subsystem3.h),
 * so this file has no header of its own. The compiler still checks the
 * definition against that prototype.
 *
 * NOTE: A (void) cast on a parameter only tells the compiler it is unused
 * on purpose: the callback type fixes the parameter list. Results of
 * tm_printf() and tk_dly_tsk() are not checked: neither can fail in a way
 * this test could act on.
 */

#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "subsystem3.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_sense.h"
#include "../rc_time.h"
#include "../test_report.h"
#include "drv_ir.h"
#include "sub_barcode.h"

#if APP_TEST

/* ------------------------------------------------------------------ *
 *  Constants
 * ------------------------------------------------------------------ */

/* Test 1: the three sensors, in test order (index into sensors[]). */
#define N_SENSORS  (3U)
#define CH_LINE_L  (0U)
#define CH_LINE_R  (1U)
#define CH_BARCODE (2U)

/* Delays passed to tk_dly_tsk(), in milliseconds. */
#define POLL_MS        (10)   /* test 1 sensor poll period */
#define RATE_WINDOW_MS (1000) /* sample-rate measurement window */
#define SETTLE_MS      (100)  /* lets both event dispatchers drain */
#define CHAR_GAP_MS    (10)   /* pause between synthetic characters */
#define LIVE_POLL_MS   (20)   /* test 2 part 2 poll period */

/* Test 2 part 1: bar widths of the printed sample (narrow 3 mm, wide
 * 9 mm, so 3:1) at two hand speeds, plus a too-slow wide bar. */
#define NARROW_150_US (20000U)  /* 3 mm at 150 mm/s */
#define WIDE_150_US   (60000U)  /* 9 mm at 150 mm/s */
#define NARROW_350_US (8571U)   /* 3 mm at 350 mm/s */
#define WIDE_350_US   (25714U)  /* 9 mm at 350 mm/s */
#define TOO_WIDE_US   (250000U) /* over sub_barcode's 200 ms limit */
#define QUIET_US      (500000U) /* white margin around a code */

#define CHAR_ELEMENTS (9U)      /* 5 bars and 4 spaces per character */
#define GUARD_SYMBOL  ('*')     /* Code 39 start/stop guard */
#define GUARD_PATTERN (0x094UL) /* the guard's pattern, see ref_chars[] */
#define REF_A         (0U)      /* index of 'A' in ref_chars[] */
#define FRAME_WRONG   (0xFFUL)  /* run_frame(): wrong letter decoded */

/* Decoded results waiting to be printed. Must be a power of two. */
#define RES_RING_SZ   (16U)
#define RES_RING_MASK (RES_RING_SZ - 1U)

#define REF_COUNT      (sizeof(ref_chars) / sizeof(ref_chars[0]))
#define CMD_NAME_COUNT (sizeof(cmd_names) / sizeof(cmd_names[0]))

/* ------------------------------------------------------------------ *
 *  Data types
 * ------------------------------------------------------------------ */

typedef struct
{
	rc_ir_ch_t channel; /* drv_ir channel */
	UB const * p_name;  /* short name: L, R, BC */
	UB const * p_pin;   /* GPIO it is wired to */
} sensor_info_t;

typedef struct
{
	uint32_t crossings;   /* off -> ON -> off, after the sensor was ready */
	uint32_t flickers;    /* ON blips too short to be the line */
	bool     b_ready;     /* read off over white for ready_us */
	bool     b_changed;   /* the sensor changed at least once */
	bool     b_mostly_on; /* read ON most of the time it was not ready */
} sensor_result_t;

typedef struct
{
	sensor_result_t results[N_SENSORS];
	uint32_t        rate;        /* line samples in RATE_WINDOW_MS */
	bool            b_user_seen; /* someone moved a sensor */
} sensor_report_t;

typedef struct
{
	uint32_t start_us;          /* when the step began */
	uint32_t moved_us;          /* when this sensor first changed */
	uint32_t on_since_us;       /* when it last went ON */
	uint32_t off_since_us;      /* when it last went off */
	uint32_t on_total_us;       /* time spent ON before it was ready */
	uint32_t remind_us;         /* when the last reminder was printed */
	bool     b_on;              /* this sensor's current state */
	bool     b_moved;           /* this sensor has changed */
	bool     b_ready;           /* phase A done: off over white */
	bool     b_timing_on;       /* timing an ON pulse after ready */
	bool     b_told_other;      /* wrong-sensor hint already printed */
	bool     b_last[N_SENSORS]; /* every sensor's last state */
} step_state_t;

typedef struct
{
	bool     b_forwards_ok;  /* S3-U06 */
	bool     b_backwards_ok; /* S3-U07 */
	bool     b_others_ok;    /* S3-U08 */
	bool     b_too_slow_ok;  /* S3-U09 */
	uint32_t edges;          /* S3-U10 */
	uint32_t letters;        /* S3-U11 */
	bool     b_no_loss;      /* S3-U12 */
} barcode_report_t;

typedef struct
{
	uint32_t start_us;     /* when part 2 began */
	uint32_t last_edge_us; /* when an edge was last seen */
	uint32_t remind_us;    /* when the last reminder was printed */
	uint32_t edges_seen;   /* edge_count at the last poll */
	uint32_t sweep_edges;  /* edges in the sweep now under way */
	uint32_t letters;      /* data letters decoded so far */
	bool     b_any_edge;   /* at least one edge seen */
} live_state_t;

typedef struct
{
	char         symbol;
	rc_nav_cmd_t command;
	bool         b_reversed;
} decode_rec_t;

typedef struct
{
	char         symbol;
	uint32_t     pattern; /* bit 8 is the first element, 1 = wide */
	rc_nav_cmd_t command;
} ref_char_t;

typedef struct
{
	rc_nav_cmd_t command;
	UB const *   p_name;
} cmd_name_t;

/* ------------------------------------------------------------------ *
 *  Static data
 * ------------------------------------------------------------------ */

/* Settings, as variables so they can be printed with %lu. */
static uint32_t const crossings_needed   = 3U;
static uint32_t const ready_us           = 1000000U;  /* off, over white */
static uint32_t const min_on_us          = 20000U;    /* shorter = flicker */
static uint32_t const first_wait_us      = 90000000U; /* time to get ready */
static uint32_t const step_wait_us       = 30000000U; /* later steps */
static uint32_t const step_run_us        = 30000000U; /* after first move */
static uint32_t const remind_period_us   = 5000000U;
/* S3-U05: one line sample per RC_PERIOD_LINE_MS (NFR1: 10 ms, 100 per
 * second). The window allows -10% for a slow start and +5 for edges. */
static uint32_t const rate_min           =
	((1000U / RC_PERIOD_LINE_MS) * 9U) / 10U;
static uint32_t const rate_max           = (1000U / RC_PERIOD_LINE_MS) + 5U;
static uint32_t const live_wait_us       = 60000000U; /* first bar */
static uint32_t const live_wait_alone_us = 20000000U; /* nobody seen */
static uint32_t const live_idle_us       = 20000000U; /* after a sweep */
static uint32_t const live_max_us        = 120000000U;
static uint32_t const sweep_gap_us       = 300000U; /* ends a sweep */
static uint32_t const letters_wanted     = 3U;
static uint32_t const full_code_edges    = 30U; /* 15 bars in *X* */
static uint32_t const us_per_ms          = 1000U;
static uint32_t const us_per_s           = 1000000U;

/* Console messages. Kept as UB arrays because tm_printf() takes UB. */
static UB const msg_begin[] = "[S3] ===== subsystem 3 tests =====\n";
static UB const msg_end[]   = "[S3] ===== subsystem 3 tests done =====\n";
static UB const msg_sensor_title[] = "[S3] ----- TEST 1: SENSOR ON/off -----\n";
static UB const msg_rate[] =
	"[S3] sense thread: %lu line samples in 1 s (expect %lu..%lu)\n";
static UB const msg_subscribe_fail[] = "[S3] could not subscribe to %s\n";
static UB const msg_sensor_howto[] =
	"[S3] Hold each sensor 5-10 mm above white paper and move it\n"
	"[S3] across the black line and back. ON = black, off = white.\n";
static UB const msg_step[] =
	"[S3] Sensor %s (%s): first hold it still over WHITE paper.\n";
static UB const msg_wait_white[] =
	"[S3]   waiting for %s to read off over white: it reads %s now"
	" (%lu s left)\n";
static UB const msg_wait_cross[] =
	"[S3]   waiting for %s to cross the black line (%lu s left)\n";
static UB const msg_ready[] =
	"[S3]   %s reads off over white: ready. Now cross the black line\n"
	"[S3]   and come back to white, %lu times.\n";
static UB const msg_state_on[]  = "[S3]   %6lu ms  %s = ON   (black)\n";
static UB const msg_state_off[] = "[S3]   %6lu ms  %s = off  (white)\n";
static UB const msg_crossing[] =
	"[S3]   %6lu ms  %s = off  (white)  crossing %lu/%lu, ON %lu ms\n";
static UB const msg_flicker[] =
	"[S3]   %6lu ms  %s = off  (white)  flicker: ON only %lu ms\n";
static UB const msg_other[] = "[S3]   %s changed, but this step is for %s\n";
static UB const msg_step_done[] = "[S3]   %s: %lu crossings, %lu flickers\n";
static UB const msg_no_move[] =
	"[S3]   %s never changed: check its DO wire, 3V3, GND, trim pot\n";
static UB const msg_reversed[] =
	"[S3]   %s read ON over white paper: polarity reversed\n"
	"[S3]   (IR_ACTIVE_HIGH in drv_ir.c) or sensor held too high\n";
static UB const msg_not_ready[] =
	"[S3]   %s never stayed off for 1 s over white paper\n";
static UB const msg_flicker_hint[] =
	"[S3]   hint: flicker at the line edge - adjust the trim pot\n";
static UB const msg_skip_rest[] =
	"[S3] nobody moved a sensor: skipping the other sensors\n";
static UB const word_on[]  = "ON";
static UB const word_off[] = "off";
static UB const msg_barcode_title[] =
	"[S3] ----- TEST 2: CODE 39 DECODE -----\n";
static UB const msg_part1[] =
	"[S3] part 1: known bar widths fed to the decoder (no sensor)\n";
static UB const msg_part1_fail[] =
	"[S3] part 1: could not arm the decoder or send widths\n";
static UB const msg_feed[] = "[S3] feeding *%c* %s\n";
static UB const msg_feed_slow[] =
	"[S3] feeding *A* with 250 ms bars (too slow: expect nothing)\n";
static UB const msg_guard[]   = "[S3]     guard '*'  (read %s)\n";
static UB const msg_decoded[] = "[S3] >>> decoded '%c'  cmd=%s  (read %s)\n";
static UB const msg_part2[] =
	"[S3] part 2: sweep the barcode sensor (BC) across the printed\n"
	"[S3] barcode, bars crossing it, the whole code in 0.3-3 s. Start\n"
	"[S3] and end on white paper. Stops after %lu letters.\n";
static UB const msg_live_wait[] =
	"[S3]   waiting for the first bar (%lu s left)\n";
static UB const msg_sweep[] =
	"[S3]   sweep: %lu edges (a full code gives %lu)\n";
static UB const msg_live_sum[] =
	"[S3] part 2: %lu edges, %lu letters, %lu overruns\n";
static UB const msg_hint_no_edge[] =
	"[S3] hint: no edges - check the DO wire on GP27 and the trim pot\n";
static UB const msg_hint_no_letter[] =
	"[S3] hint: edges but no letter - sweep steadily, the whole code\n"
	"[S3]       in under 3 s, sensor 5-10 mm above the paper\n";

static UB const name_line_l[]    = "L";
static UB const name_line_r[]    = "R";
static UB const name_barcode[]   = "BC";
static UB const pin_line_l[]     = "GP16";
static UB const pin_line_r[]     = "GP6";
static UB const pin_barcode[]    = "GP27";
static UB const word_forwards[]  = "forwards";
static UB const word_backwards[] = "backwards";
static UB const word_samples[]   = "line samples";
static UB const word_decodes[]   = "decoded barcodes";
static UB const cmd_none[]       = "NONE";
static UB const cmd_left[]       = "TURN_LEFT";
static UB const cmd_right[]      = "TURN_RIGHT";
static UB const cmd_straight[]   = "GO_STRAIGHT";
static UB const cmd_u_turn[]     = "U_TURN";
static UB const cmd_stop[]       = "STOP";
static UB const cmd_unknown[]    = "?";

/* In test order; CH_LINE_L, CH_LINE_R and CH_BARCODE index this. */
static sensor_info_t const sensors[N_SENSORS] = {
	{ RC_IR_LINE_L, name_line_l, pin_line_l },
	{ RC_IR_LINE_R, name_line_r, pin_line_r },
	{ RC_IR_BARCODE, name_barcode, pin_barcode },
};

/*
 * Reference Code 39 patterns: bit 8 is the first element, 1 means wide.
 * Taken from the printed sample sheet and checked against the Code 39
 * standard (ISO/IEC 16388). Kept apart from sub_barcode.c's table on
 * purpose, so a wrong entry there shows up here.
 */
/* The formatter is switched off so each table keeps one row per line. */
/* clang-format off */
static ref_char_t const ref_chars[] = {
	{ 'A', 0x109U, RC_CMD_TURN_LEFT },   /* REF_A */
	{ 'B', 0x049U, RC_CMD_TURN_RIGHT },
	{ 'C', 0x148U, RC_CMD_GO_STRAIGHT },
	{ 'D', 0x019U, RC_CMD_U_TURN },
	{ 'Z', 0x0D0U, RC_CMD_NONE },
};
/* clang-format on */

/* clang-format off */
static cmd_name_t const cmd_names[] = {
	{ RC_CMD_NONE,        cmd_none },
	{ RC_CMD_TURN_LEFT,   cmd_left },
	{ RC_CMD_TURN_RIGHT,  cmd_right },
	{ RC_CMD_GO_STRAIGHT, cmd_straight },
	{ RC_CMD_U_TURN,      cmd_u_turn },
	{ RC_CMD_STOP,        cmd_stop },
};
/* clang-format on */

/*
 * Line-sample counter for S3-U05. The subscription is made once and never
 * removed: rc_event_unsubscribe() can race the dispatcher, so the callback
 * is gated by b_counting instead.
 */
static volatile bool     b_counting   = false;
static volatile uint32_t sample_count = 0U;
static int32_t           h_rate_sub   = -1;

/*
 * Decoded results, written by the slow-lane callback and read by this
 * test: one producer, one consumer. Gated by b_listening, for the same
 * reason as above.
 */
static volatile decode_rec_t res_ring[RES_RING_SZ];
static volatile uint32_t     res_head     = 0U;
static volatile uint32_t     res_tail     = 0U;
static volatile uint32_t     res_lost     = 0U;
static volatile bool         b_listening  = false;
static int32_t               h_decode_sub = -1;

/* Raw barcode edges seen by drv_ir's bottom half during part 2. */
static volatile uint32_t edge_count = 0U;

/* A synthetic width could not be published (event ring full). */
static bool b_publish_failed = false;

/* ------------------------------------------------------------------ *
 *  Private function prototypes
 * ------------------------------------------------------------------ */

static void     run_sensor_test (sensor_report_t * p_report);
static uint32_t measure_sample_rate (void);
static void     on_line_sample (rc_event_t const * p_evt, void * p_ctx);
static void     check_one_sensor (uint32_t index, uint32_t wait_us,
								  sensor_report_t * p_report);
static void     start_step (uint32_t index, step_state_t * p_step);
static void     handle_change (uint32_t index, bool b_now, uint32_t now_us,
							   step_state_t * p_step, sensor_result_t * p_res);
static void     count_crossing (uint32_t index, uint32_t now_us,
								step_state_t * p_step, sensor_result_t * p_res);
static void     check_ready (uint32_t index, uint32_t now_us,
							 step_state_t * p_step);
static void     check_other_sensors (uint32_t index, step_state_t * p_step,
									 sensor_report_t * p_report);
static void remind_if_due (uint32_t index, uint32_t wait_us, uint32_t now_us,
						   step_state_t * p_step);
static bool step_finished (step_state_t const *    p_step,
						   sensor_result_t const * p_res, uint32_t wait_us,
						   uint32_t now_us);
static void finish_step (uint32_t now_us, step_state_t const * p_step,
						 sensor_result_t * p_res);
static UB const * state_name (bool b_on);
static void       report_step (uint32_t index, sensor_result_t const * p_res);
static bool sensor_passed (sensor_report_t const * p_report, uint32_t index);
static bool rate_ok (uint32_t rate);

static void run_barcode_test (bool b_user_seen, barcode_report_t * p_report);
static void synthetic_checks (barcode_report_t * p_report);
static bool frame_ok (ref_char_t const * p_ref, bool b_backwards,
					  uint32_t narrow_us, uint32_t wide_us);
static uint32_t run_frame (ref_char_t const * p_ref, bool b_backwards,
						   uint32_t narrow_us, uint32_t wide_us);
static void send_char (uint32_t pattern, bool b_backwards, uint32_t narrow_us,
					   uint32_t wide_us);
static void send_width (uint32_t width_us);
static void live_checks (bool b_user_seen, barcode_report_t * p_report);
static bool live_poll (live_state_t * p_live, uint32_t wait_us);
static bool live_finished (live_state_t const * p_live, uint32_t wait_us,
						   uint32_t now_us);
static void report_live (barcode_report_t const * p_report, uint32_t overruns);
static void on_decoded (rc_event_t const * p_evt, void * p_ctx);
static void on_raw_edge (bool b_new_level, uint32_t width_us, void * p_ctx);
static bool take_result (decode_rec_t * p_out);
static void flush_results (void);
static uint32_t   drain_results (void);
static void       print_result (decode_rec_t const * p_rec);
static UB const * cmd_name (rc_nav_cmd_t command);
static UB const * direction_name (bool b_reversed);

/* ------------------------------------------------------------------ *
 *  Public functions
 * ------------------------------------------------------------------ */

/*!
 * @brief  Run subsystem 3's on-target tests: sensor ON/off, then Code 39.
 */
EXPORT void subsystem3_test (void)
{
	sensor_report_t  sensor_rep  = { 0 };
	barcode_report_t barcode_rep = { false };

	TEST_INIT();

	tm_printf(msg_begin);

	run_sensor_test(&sensor_rep);
	TEST_CHECK("S3-U02", sensor_passed(&sensor_rep, CH_LINE_L),
			   "line sensor L (GP16): off on white, 3 line crossings");
	TEST_CHECK("S3-U03", sensor_passed(&sensor_rep, CH_LINE_R),
			   "line sensor R (GP6): off on white, 3 line crossings");
	TEST_CHECK("S3-U04", sensor_passed(&sensor_rep, CH_BARCODE),
			   "barcode sensor DO (GP27): off on white, 3 crossings");
	TEST_CHECK("S3-U05", rate_ok(sensor_rep.rate),
			   "sense thread publishes line samples at ~100 Hz (NFR1)");

	run_barcode_test(sensor_rep.b_user_seen, &barcode_rep);
	TEST_CHECK("S3-U06", barcode_rep.b_forwards_ok,
			   "*A* forwards, 150 + 350 mm/s -> 'A' TURN_LEFT");
	TEST_CHECK("S3-U07", barcode_rep.b_backwards_ok,
			   "*A* backwards -> 'A' TURN_LEFT, reversed");
	TEST_CHECK("S3-U08", barcode_rep.b_others_ok,
			   "*B* *C* *D* *Z* both ways -> right letter");
	TEST_CHECK("S3-U09", barcode_rep.b_too_slow_ok,
			   "bar over 200 ms rejected, no letter");
	TEST_CHECK("S3-U10", (barcode_rep.edges > 0U),
			   "live: barcode sensor gave edges");
	TEST_CHECK("S3-U11", (barcode_rep.letters > 0U),
			   "live: at least one letter decoded");
	TEST_CHECK("S3-U12", barcode_rep.b_no_loss, "live: no edge or result lost");

	TEST_SUMMARY("subsystem3");
	tm_printf(msg_end);
}

/* ------------------------------------------------------------------ *
 *  Test 1: sensor ON/off
 * ------------------------------------------------------------------ */

/*!
 * @brief  Test 1: sample rate, then each sensor in turn.
 *
 * If nobody moves any sensor during the first step, the other sensors
 * are skipped, so an unattended team test run is not held up.
 *
 * @param  p_report  filled in with the results
 */
static void run_sensor_test (sensor_report_t * p_report)
{
	uint32_t index;
	uint32_t wait_us = first_wait_us;
	bool     b_skip  = false;

	tm_printf(msg_sensor_title);
	p_report->rate = measure_sample_rate();
	tm_printf(msg_rate, p_report->rate, rate_min, rate_max);
	tm_printf(msg_sensor_howto);

	for (index = 0U; index < N_SENSORS; index++)
	{
		if (!b_skip)
		{
			check_one_sensor(index, wait_us, p_report);
			wait_us = step_wait_us;
			b_skip  = !p_report->b_user_seen;
			if (b_skip)
			{
				tm_printf(msg_skip_rest);
			}
		}
	}
}

/*!
 * @brief  Count the line samples the sense thread publishes in
 *         RATE_WINDOW_MS (S3-U05).
 * @return samples counted, or 0 if the subscription failed.
 */
static uint32_t measure_sample_rate (void)
{
	uint32_t samples = 0U;

	if (h_rate_sub < 0)
	{
		h_rate_sub = rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW,
										on_line_sample, NULL);
	}

	if (h_rate_sub < 0)
	{
		tm_printf(msg_subscribe_fail, word_samples);
	}
	else
	{
		/* Where a short count comes from: few Sense passes (or
		   overruns) = the Sense task runs late; passes on time but
		   SLOW-lane drops = samples lost on the event bus. */
		uint32_t ticks0 = rc_sense_ticks();
		uint32_t over0  = rc_sense_overruns();
		uint32_t drop0  = rc_event_dropped(RC_LANE_SLOW);

		sample_count = 0U;
		b_counting   = true;
		tk_dly_tsk(RATE_WINDOW_MS);
		b_counting = false;
		samples    = sample_count;
		uint32_t line_us;
		uint32_t imu_us;

		rc_sense_max_us(&line_us, &imu_us);
		tm_printf((UB *)"[S3] in that window: %lu Sense passes, "
				  "%lu overruns, %lu SLOW-lane drops\n",
				  rc_sense_ticks() - ticks0,
				  rc_sense_overruns() - over0,
				  rc_event_dropped(RC_LANE_SLOW) - drop0);
		tm_printf((UB *)"[S3] longest Sense step so far: line %lu us, "
				  "IMU %lu us\n", line_us, imu_us);
	}

	return samples;
}

/*!
 * @brief  Slow-lane callback: counts line samples while b_counting.
 * @param  p_evt  the line sample (unused)
 * @param  p_ctx  context (unused)
 */
static void on_line_sample (rc_event_t const * p_evt, void * p_ctx)
{
	(void)p_evt;
	(void)p_ctx;

	if (b_counting)
	{
		sample_count++;
	}
}

/*!
 * @brief  One step of test 1, in two phases.
 *
 * Phase A: hold the sensor still over white paper until it reads off for
 * ready_us. A sensor with reversed polarity reads ON over white, so it
 * never gets past this phase. Phase B: cross the black line and come back
 * crossings_needed times; each off -> ON -> off counts.
 *
 * The step allows wait_us before this sensor first changes, then
 * step_run_us from that change. The other sensors are watched too, so
 * moving the wrong one gets a hint.
 *
 * @param  index     which sensor (index into sensors[])
 * @param  wait_us   time allowed before the first change
 * @param  p_report  results; b_user_seen is set on any change
 */
static void check_one_sensor (uint32_t index, uint32_t wait_us,
							  sensor_report_t * p_report)
{
	sensor_result_t * p_res  = &p_report->results[index];
	step_state_t      step   = { 0 };
	uint32_t          now_us = rc_time_us();
	bool              b_now;
	bool              b_done = false;

	start_step(index, &step);

	while (!b_done)
	{
		tk_dly_tsk(POLL_MS);
		now_us = rc_time_us();
		b_now  = drv_ir_on_line(sensors[index].channel);

		if (b_now != step.b_on)
		{
			handle_change(index, b_now, now_us, &step, p_res);
			p_report->b_user_seen = true;
		}
		else
		{
			check_other_sensors(index, &step, p_report);
			remind_if_due(index, wait_us, now_us, &step);
		}

		check_ready(index, now_us, &step);
		b_done = step_finished(&step, p_res, wait_us, now_us);
	}

	finish_step(now_us, &step, p_res);
	report_step(index, p_res);
}

/*!
 * @brief  Print the step's instructions and record every sensor's state.
 * @param  index   which sensor this step is for
 * @param  p_step  step state to set up
 */
static void start_step (uint32_t index, step_state_t * p_step)
{
	uint32_t other;

	tm_printf(msg_step, sensors[index].p_name, sensors[index].p_pin);

	p_step->start_us     = rc_time_us();
	p_step->remind_us    = p_step->start_us;
	p_step->on_since_us  = p_step->start_us;
	p_step->off_since_us = p_step->start_us;
	for (other = 0U; other < N_SENSORS; other++)
	{
		p_step->b_last[other] = drv_ir_on_line(sensors[other].channel);
	}
	p_step->b_on = p_step->b_last[index];
}

/*!
 * @brief  The step's sensor changed state: print it, and in phase B
 *         count the crossing.
 * @param  index   which sensor
 * @param  b_now   its new state (true = ON, black)
 * @param  now_us  time of the change
 * @param  p_step  step state
 * @param  p_res   the sensor's result
 */
static void handle_change (uint32_t index, bool b_now, uint32_t now_us,
						   step_state_t * p_step, sensor_result_t * p_res)
{
	uint32_t elapsed_ms = (now_us - p_step->start_us) / us_per_ms;

	p_step->b_on      = b_now;
	p_step->remind_us = now_us; /* no reminder while the sensor moves */
	if (!p_step->b_moved)
	{
		p_step->b_moved  = true;
		p_step->moved_us = now_us;
	}

	if (b_now)
	{
		p_step->on_since_us = now_us;
		p_step->b_timing_on = p_step->b_ready;
		tm_printf(msg_state_on, elapsed_ms, sensors[index].p_name);
	}
	else
	{
		p_step->off_since_us = now_us;
		if (!p_step->b_ready)
		{
			p_step->on_total_us += now_us - p_step->on_since_us;
		}

		if (p_step->b_timing_on)
		{
			count_crossing(index, now_us, p_step, p_res);
		}
		else
		{
			tm_printf(msg_state_off, elapsed_ms, sensors[index].p_name);
		}
	}
}

/*!
 * @brief  Phase B, ON -> off: count a crossing, or a flicker if the
 *         sensor was ON for less than min_on_us.
 * @param  index   which sensor
 * @param  now_us  time of the change
 * @param  p_step  step state
 * @param  p_res   the sensor's result
 */
static void count_crossing (uint32_t index, uint32_t now_us,
							step_state_t * p_step, sensor_result_t * p_res)
{
	UB const * p_name     = sensors[index].p_name;
	uint32_t   elapsed_ms = (now_us - p_step->start_us) / us_per_ms;
	uint32_t   on_us      = now_us - p_step->on_since_us;

	p_step->b_timing_on = false;
	if (on_us >= min_on_us)
	{
		p_res->crossings++;
		tm_printf(msg_crossing, elapsed_ms, p_name, p_res->crossings,
				  crossings_needed, on_us / us_per_ms);
	}
	else
	{
		p_res->flickers++;
		tm_printf(msg_flicker, elapsed_ms, p_name, on_us / us_per_ms);
	}
}

/*!
 * @brief  Phase A: the sensor is ready once it has read off (white) for
 *         ready_us without a break.
 * @param  index   which sensor
 * @param  now_us  current time
 * @param  p_step  step state
 */
static void check_ready (uint32_t index, uint32_t now_us, step_state_t * p_step)
{
	if ((!p_step->b_ready) && (!p_step->b_on)
		&& ((now_us - p_step->off_since_us) >= ready_us))
	{
		p_step->b_ready   = true;
		p_step->remind_us = now_us;
		tm_printf(msg_ready, sensors[index].p_name, crossings_needed);
	}
}

/*!
 * @brief  Watch the sensors this step is not for. Any change means
 *         someone is there; the first one also prints a hint.
 * @param  index     which sensor the step is for
 * @param  p_step    step state (holds every sensor's last state)
 * @param  p_report  b_user_seen is set on any change
 */
static void check_other_sensors (uint32_t index, step_state_t * p_step,
								 sensor_report_t * p_report)
{
	uint32_t other;
	bool     b_now;

	for (other = 0U; other < N_SENSORS; other++)
	{
		b_now = drv_ir_on_line(sensors[other].channel);
		if ((other != index) && (b_now != p_step->b_last[other]))
		{
			p_report->b_user_seen = true;
			if (!p_step->b_told_other)
			{
				tm_printf(msg_other, sensors[other].p_name,
						  sensors[index].p_name);
				p_step->b_told_other = true;
			}
		}
		p_step->b_last[other] = b_now;
	}
}

/*!
 * @brief  Every remind_period_us, say what the step is waiting for and
 *         how long is left.
 * @param  index    which sensor
 * @param  wait_us  time allowed before the first change
 * @param  now_us   current time
 * @param  p_step   step state
 */
static void remind_if_due (uint32_t index, uint32_t wait_us, uint32_t now_us,
						   step_state_t * p_step)
{
	uint32_t limit_us = wait_us;
	uint32_t used_us  = now_us - p_step->start_us;

	if (p_step->b_moved)
	{
		limit_us = step_run_us;
		used_us  = now_us - p_step->moved_us;
	}

	if ((used_us < limit_us)
		&& ((now_us - p_step->remind_us) >= remind_period_us))
	{
		if (p_step->b_ready)
		{
			tm_printf(msg_wait_cross, sensors[index].p_name,
					  (limit_us - used_us) / us_per_s);
		}
		else
		{
			tm_printf(msg_wait_white, sensors[index].p_name,
					  state_name(p_step->b_on),
					  (limit_us - used_us) / us_per_s);
		}
		p_step->remind_us = now_us;
	}
}

/*!
 * @brief  Decide whether the step is over.
 * @param  p_step   step state
 * @param  p_res    the sensor's result so far
 * @param  wait_us  time allowed before the first change
 * @param  now_us   current time
 * @return true when enough crossings were seen or time ran out.
 */
static bool step_finished (step_state_t const *    p_step,
						   sensor_result_t const * p_res, uint32_t wait_us,
						   uint32_t now_us)
{
	bool b_finished;

	if (p_res->crossings >= crossings_needed)
	{
		b_finished = true;
	}
	else if (p_step->b_moved)
	{
		b_finished = ((now_us - p_step->moved_us) >= step_run_us);
	}
	else
	{
		b_finished = ((now_us - p_step->start_us) >= wait_us);
	}

	return b_finished;
}

/*!
 * @brief  Copy the step's outcome into the sensor's result.
 *
 * b_mostly_on is set when the sensor never became ready and spent more
 * than half of the step reading ON: the sign of a reversed sensor.
 *
 * @param  now_us  time the step ended
 * @param  p_step  step state
 * @param  p_res   the sensor's result
 */
static void finish_step (uint32_t now_us, step_state_t const * p_step,
						 sensor_result_t * p_res)
{
	uint32_t on_total_us = p_step->on_total_us;
	uint32_t step_us     = now_us - p_step->start_us;

	if ((!p_step->b_ready) && p_step->b_on)
	{
		on_total_us += now_us - p_step->on_since_us;
	}

	p_res->b_ready     = p_step->b_ready;
	p_res->b_changed   = p_step->b_moved;
	p_res->b_mostly_on = ((!p_step->b_ready) && (on_total_us > (step_us / 2U)));
}

/*!
 * @brief  Print the result of one step, with a hint if it went wrong.
 * @param  index  which sensor
 * @param  p_res  its result
 */
static void report_step (uint32_t index, sensor_result_t const * p_res)
{
	UB const * p_name = sensors[index].p_name;

	if (p_res->b_mostly_on)
	{
		tm_printf(msg_reversed, p_name);
	}
	else if (!p_res->b_ready)
	{
		tm_printf(msg_not_ready, p_name);
	}
	else if (!p_res->b_changed)
	{
		tm_printf(msg_no_move, p_name);
	}
	else
	{
		tm_printf(msg_step_done, p_name, p_res->crossings, p_res->flickers);
		if (p_res->flickers > 0U)
		{
			tm_printf(msg_flicker_hint);
		}
	}
}

/*!
 * @brief  Whether one sensor passed test 1.
 * @param  p_report  test 1 results
 * @param  index     which sensor
 * @return true if it read off over white, then crossed the line
 *         crossings_needed times.
 */
static bool sensor_passed (sensor_report_t const * p_report, uint32_t index)
{
	return (p_report->results[index].b_ready
			&& (p_report->results[index].crossings >= crossings_needed));
}

/*!
 * @brief  Readable sensor state.
 * @param  b_on  true if the sensor reads ON (black)
 * @return "ON" or "off".
 */
static UB const * state_name (bool b_on)
{
	return (b_on ? word_on : word_off);
}

/*!
 * @brief  Whether the measured sample rate is about 100 Hz (S3-U05).
 * @param  rate  samples counted in RATE_WINDOW_MS
 * @return true if it is between rate_min and rate_max.
 */
static bool rate_ok (uint32_t rate)
{
	return ((rate >= rate_min) && (rate <= rate_max));
}

/* ------------------------------------------------------------------ *
 *  Test 2: Code 39 decode
 * ------------------------------------------------------------------ */

/*!
 * @brief  Test 2: part 1 (known widths), then part 2 (live sensor).
 * @param  b_user_seen  someone moved a sensor in test 1
 * @param  p_report     filled in with the results
 */
static void run_barcode_test (bool b_user_seen, barcode_report_t * p_report)
{
	tm_printf(msg_barcode_title);

	if (h_decode_sub < 0)
	{
		h_decode_sub = rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW,
										  on_decoded, NULL);
	}

	if (h_decode_sub < 0)
	{
		tm_printf(msg_subscribe_fail, word_decodes);
	}
	else
	{
		b_listening = true;
		synthetic_checks(p_report);
		live_checks(b_user_seen, p_report);
		b_listening = false;
	}
}

/*!
 * @brief  Part 1: feed known Code 39 widths straight to the decoder,
 *         with the barcode interrupt masked (S3-U06..S3-U09).
 * @param  p_report  filled in with the results
 */
static void synthetic_checks (barcode_report_t * p_report)
{
	ref_char_t const * p_letter_a = &ref_chars[REF_A];
	uint32_t           index;
	bool               b_all_ok = true;
	bool               b_setup_ok;

	tm_printf(msg_part1);
	b_publish_failed = false;

	/* Decoder on with a fresh window, then the real sensor masked, so only
	 * the widths sent below reach the decoder. */
	b_setup_ok = (RC_OK == sub_barcode_arm(true));
	b_setup_ok = ((RC_OK == drv_ir_barcode_enable(false)) && b_setup_ok);

	p_report->b_forwards_ok =
		frame_ok(p_letter_a, false, NARROW_150_US, WIDE_150_US);
	p_report->b_forwards_ok =
		(frame_ok(p_letter_a, false, NARROW_350_US, WIDE_350_US)
		 && p_report->b_forwards_ok);
	p_report->b_backwards_ok =
		frame_ok(p_letter_a, true, NARROW_150_US, WIDE_150_US);

	for (index = REF_A + 1U; index < REF_COUNT; index++)
	{
		b_all_ok =
			(frame_ok(&ref_chars[index], false, NARROW_150_US, WIDE_150_US)
			 && b_all_ok);
		b_all_ok =
			(frame_ok(&ref_chars[index], true, NARROW_150_US, WIDE_150_US)
			 && b_all_ok);
	}
	p_report->b_others_ok = b_all_ok;

	tm_printf(msg_feed_slow);
	p_report->b_too_slow_ok =
		(0U == run_frame(p_letter_a, false, NARROW_150_US, TOO_WIDE_US));

	b_setup_ok = ((RC_OK == sub_barcode_arm(false)) && b_setup_ok);
	if ((!b_setup_ok) || b_publish_failed)
	{
		tm_printf(msg_part1_fail);
		p_report->b_forwards_ok  = false;
		p_report->b_backwards_ok = false;
		p_report->b_others_ok    = false;
		p_report->b_too_slow_ok  = false;
	}
}

/*!
 * @brief  Feed one "*X*" and check exactly one correct X came out.
 * @param  p_ref        expected character
 * @param  b_backwards  read the code back to front
 * @param  narrow_us    narrow element width
 * @param  wide_us      wide element width
 * @return true if exactly one correct X was decoded.
 */
static bool frame_ok (ref_char_t const * p_ref, bool b_backwards,
					  uint32_t narrow_us, uint32_t wide_us)
{
	tm_printf(msg_feed, p_ref->symbol, direction_name(b_backwards));

	return (1U == run_frame(p_ref, b_backwards, narrow_us, wide_us));
}

/*!
 * @brief  Publish "*X*", with a narrow gap between characters and white
 *         margins either side, and print what the decoder reports.
 * @param  p_ref        expected character
 * @param  b_backwards  read the code back to front
 * @param  narrow_us    narrow element width
 * @param  wide_us      wide element width
 * @return number of data letters decoded, or FRAME_WRONG if any had the
 *         wrong letter, command or direction.
 */
static uint32_t run_frame (ref_char_t const * p_ref, bool b_backwards,
						   uint32_t narrow_us, uint32_t wide_us)
{
	decode_rec_t record;
	uint32_t     letters = 0U;
	bool         b_wrong = false;

	flush_results();

	send_width(QUIET_US);
	send_char(GUARD_PATTERN, b_backwards, narrow_us, wide_us);
	send_width(narrow_us);
	tk_dly_tsk(CHAR_GAP_MS);
	send_char(p_ref->pattern, b_backwards, narrow_us, wide_us);
	send_width(narrow_us);
	tk_dly_tsk(CHAR_GAP_MS);
	send_char(GUARD_PATTERN, b_backwards, narrow_us, wide_us);
	send_width(QUIET_US);
	tk_dly_tsk(SETTLE_MS);

	while (take_result(&record))
	{
		print_result(&record);
		if (GUARD_SYMBOL != record.symbol)
		{
			letters++;
			b_wrong = (b_wrong || (record.symbol != p_ref->symbol)
					   || (record.command != p_ref->command)
					   || (record.b_reversed != b_backwards));
		}
	}

	return (b_wrong ? FRAME_WRONG : letters);
}

/*!
 * @brief  Publish one character's 9 element widths.
 * @param  pattern      9-bit pattern, bit 8 first, 1 = wide
 * @param  b_backwards  send the elements last to first
 * @param  narrow_us    narrow element width
 * @param  wide_us      wide element width
 */
static void send_char (uint32_t pattern, bool b_backwards, uint32_t narrow_us,
					   uint32_t wide_us)
{
	uint32_t element;
	uint32_t bit;

	for (element = 0U; element < CHAR_ELEMENTS; element++)
	{
		bit = (b_backwards ? element : ((CHAR_ELEMENTS - 1U) - element));
		if (0U != ((pattern >> bit) & 1U))
		{
			send_width(wide_us);
		}
		else
		{
			send_width(narrow_us);
		}
	}
}

/*!
 * @brief  Publish one bar or space width, as drv_ir's bottom half would.
 * @param  width_us  width of the bar or space
 */
static void send_width (uint32_t width_us)
{
	rc_event_t edge_evt = { 0 };

	edge_evt.id                    = RC_EVT_BARCODE_EDGE;
	edge_evt.u.bar_edge.level_high = false;
	edge_evt.u.bar_edge.width_us   = width_us;

	if (RC_OK != rc_event_publish(&edge_evt))
	{
		b_publish_failed = true;
	}
}

/*!
 * @brief  Part 2: arm the real barcode sensor and print every letter
 *         decoded while you sweep it across the printed barcode
 *         (S3-U10..S3-U12).
 * @param  b_user_seen  someone moved a sensor in test 1; if not, the wait
 *                      for the first bar is shorter
 * @param  p_report     filled in with the results
 */
static void live_checks (bool b_user_seen, barcode_report_t * p_report)
{
	live_state_t live            = { 0 };
	uint32_t     wait_us         = live_wait_alone_us;
	uint32_t     overruns_before = drv_ir_barcode_overruns();
	uint32_t     drops_before    = rc_event_dropped(RC_LANE_FAST);
	uint32_t     overruns;
	bool         b_setup_ok;
	bool         b_done = false;

	if (b_user_seen)
	{
		wait_us = live_wait_us;
	}

	edge_count = 0U;
	res_lost   = 0U;
	flush_results();
	b_setup_ok = (RC_OK == drv_ir_on_barcode_edge(on_raw_edge, NULL));
	b_setup_ok = ((RC_OK == sub_barcode_arm(true)) && b_setup_ok);
	tm_printf(msg_part2, letters_wanted);

	live.start_us  = rc_time_us();
	live.remind_us = live.start_us;
	while (!b_done)
	{
		tk_dly_tsk(LIVE_POLL_MS);
		b_done = live_poll(&live, wait_us);
	}

	/* Disarm, let the bottom half and dispatchers go idle, then take the
	 * last results before removing the raw-edge callback. */
	b_setup_ok = ((RC_OK == sub_barcode_arm(false)) && b_setup_ok);
	tk_dly_tsk(SETTLE_MS);
	live.letters += drain_results();
	b_setup_ok = ((RC_OK == drv_ir_on_barcode_edge(NULL, NULL)) && b_setup_ok);
	if (live.sweep_edges > 0U)
	{
		tm_printf(msg_sweep, live.sweep_edges, full_code_edges);
	}

	overruns            = drv_ir_barcode_overruns() - overruns_before;
	p_report->edges     = edge_count;
	p_report->letters   = live.letters;
	p_report->b_no_loss = (b_setup_ok && (0U == overruns) && (0U == res_lost)
						   && (rc_event_dropped(RC_LANE_FAST) == drops_before));
	report_live(p_report, overruns);
}

/*!
 * @brief  One poll of part 2: print new results, track sweeps, remind.
 * @param  p_live   part 2 state
 * @param  wait_us  time allowed for the first bar
 * @return true when part 2 is over.
 */
static bool live_poll (live_state_t * p_live, uint32_t wait_us)
{
	uint32_t now_us = rc_time_us();
	uint32_t edges  = edge_count;

	p_live->letters += drain_results();

	if (edges != p_live->edges_seen)
	{
		p_live->sweep_edges += edges - p_live->edges_seen;
		p_live->edges_seen   = edges;
		p_live->last_edge_us = now_us;
		p_live->b_any_edge   = true;
	}
	else if ((p_live->sweep_edges > 0U)
			 && ((now_us - p_live->last_edge_us) >= sweep_gap_us))
	{
		tm_printf(msg_sweep, p_live->sweep_edges, full_code_edges);
		p_live->sweep_edges = 0U;
	}
	else if ((!p_live->b_any_edge)
			 && ((now_us - p_live->remind_us) >= remind_period_us)
			 && ((now_us - p_live->start_us) < wait_us))
	{
		tm_printf(msg_live_wait,
				  (wait_us - (now_us - p_live->start_us)) / us_per_s);
		p_live->remind_us = now_us;
	}
	else
	{
		/* Nothing to print on this poll. */
	}

	return live_finished(p_live, wait_us, now_us);
}

/*!
 * @brief  Decide whether part 2 is over.
 * @param  p_live   part 2 state
 * @param  wait_us  time allowed for the first bar
 * @param  now_us   current time
 * @return true after letters_wanted letters, after live_idle_us without
 *         an edge, after the first wait with no edge, or after live_max_us.
 */
static bool live_finished (live_state_t const * p_live, uint32_t wait_us,
						   uint32_t now_us)
{
	uint32_t ran_us = now_us - p_live->start_us;
	bool     b_finished;

	if ((p_live->letters >= letters_wanted) || (ran_us >= live_max_us))
	{
		b_finished = true;
	}
	else if (p_live->b_any_edge)
	{
		b_finished = ((now_us - p_live->last_edge_us) >= live_idle_us);
	}
	else
	{
		b_finished = (ran_us >= wait_us);
	}

	return b_finished;
}

/*!
 * @brief  Print the part 2 summary, with a hint if nothing was decoded.
 * @param  p_report  part 2 results
 * @param  overruns  edges drv_ir dropped during part 2
 */
static void report_live (barcode_report_t const * p_report, uint32_t overruns)
{
	tm_printf(msg_live_sum, p_report->edges, p_report->letters, overruns);

	if (0U == p_report->edges)
	{
		tm_printf(msg_hint_no_edge);
	}
	else if (0U == p_report->letters)
	{
		tm_printf(msg_hint_no_letter);
	}
	else
	{
		/* At least one letter was decoded: nothing to add. */
	}
}

/*!
 * @brief  Slow-lane callback for RC_EVT_BARCODE_DECODED: queue the result.
 * @param  p_evt  the decoded character
 * @param  p_ctx  context (unused)
 */
static void on_decoded (rc_event_t const * p_evt, void * p_ctx)
{
	uint32_t next;

	(void)p_ctx;

	if (b_listening)
	{
		next = (res_head + 1U) & RES_RING_MASK;
		if (next != res_tail)
		{
			res_ring[res_head].symbol     = p_evt->u.barcode.symbol;
			res_ring[res_head].command    = p_evt->u.barcode.command;
			res_ring[res_head].b_reversed = p_evt->u.barcode.reversed;
			res_head                      = next;
		}
		else
		{
			res_lost++;
		}
	}
}

/*!
 * @brief  drv_ir raw-edge callback (drv_ir's bottom half, task context):
 *         counts edges for part 2.
 * @param  b_new_level  level after the edge (unused)
 * @param  width_us     width of the bar or space (unused)
 * @param  p_ctx        context (unused)
 */
static void on_raw_edge (bool b_new_level, uint32_t width_us, void * p_ctx)
{
	(void)b_new_level;
	(void)width_us;
	(void)p_ctx;

	edge_count++;
}

/*!
 * @brief  Take the oldest queued result, if any.
 * @param  p_out  where to copy it
 * @return true if a result was taken.
 */
static bool take_result (decode_rec_t * p_out)
{
	bool b_got = false;

	if (res_tail != res_head)
	{
		p_out->symbol     = res_ring[res_tail].symbol;
		p_out->command    = res_ring[res_tail].command;
		p_out->b_reversed = res_ring[res_tail].b_reversed;
		res_tail          = (res_tail + 1U) & RES_RING_MASK;
		b_got             = true;
	}

	return b_got;
}

/*!
 * @brief  Throw away any queued results.
 */
static void flush_results (void)
{
	res_tail = res_head;
}

/*!
 * @brief  Print every queued result.
 * @return number of data letters among them ('*' guards not counted).
 */
static uint32_t drain_results (void)
{
	decode_rec_t record;
	uint32_t     letters = 0U;

	while (take_result(&record))
	{
		print_result(&record);
		if (GUARD_SYMBOL != record.symbol)
		{
			letters++;
		}
	}

	return letters;
}

/*!
 * @brief  Print one decoded result: a guard, or a letter and its command.
 * @param  p_rec  the result
 */
static void print_result (decode_rec_t const * p_rec)
{
	UB const * p_direction = direction_name(p_rec->b_reversed);

	if (GUARD_SYMBOL == p_rec->symbol)
	{
		tm_printf(msg_guard, p_direction);
	}
	else
	{
		tm_printf(msg_decoded, p_rec->symbol, cmd_name(p_rec->command),
				  p_direction);
	}
}

/*!
 * @brief  Readable name of a navigation command.
 * @param  command  the command
 * @return its name, or "?" if unknown.
 */
static UB const * cmd_name (rc_nav_cmd_t command)
{
	UB const * p_name = cmd_unknown;
	uint32_t   index;

	for (index = 0U; index < CMD_NAME_COUNT; index++)
	{
		if (command == cmd_names[index].command)
		{
			p_name = cmd_names[index].p_name;
		}
	}

	return p_name;
}

/*!
 * @brief  Readable reading direction.
 * @param  b_reversed  true if read back to front
 * @return "backwards" or "forwards".
 */
static UB const * direction_name (bool b_reversed)
{
	return (b_reversed ? word_backwards : word_forwards);
}

#endif /* APP_TEST */

/*** end of file ***/