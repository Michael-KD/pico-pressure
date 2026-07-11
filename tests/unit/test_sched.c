/*
 * test_sched.c — superframe scheduler unit tests (§4): segment math,
 * transmit gating, listen duty cycling, telemetry micro-slots, mode
 * switching, and GPS-uncertainty degradation.
 */
#include "mesh_test_helpers.h"
#include "test_util.h"

/* Small, round superframe for readable arithmetic:
 * | beacon 2s | cmd 3s | telem 4s | (bulk 0) | sleep 11s |  = 20 s */
#define SEC 1000000ull
#define PERIOD (20 * SEC)
#define B_WIN (2 * SEC)
#define C_WIN (3 * SEC)
#define T_WIN (4 * SEC)
#define ACT_END (B_WIN + C_WIN + T_WIN)
#define GUARD (100 * 1000ull)

static tx_capture_t cap;
static mesh_t m;

/* set_listen capture (the cap radio records transmits only). */
static bool listen_state = true;
static unsigned listen_calls;
static void cap_set_listen(void *ctx, bool enable, mesh_profile_t profile)
{
    (void)ctx;
    (void)profile;
    listen_state = enable;
    listen_calls++;
}

static mesh_sched_mode_t test_mode(void)
{
    mesh_sched_mode_t md = {
        .period_us = PERIOD,
        .beacon_win_us = B_WIN,
        .cmd_win_us = C_WIN,
        .telem_win_us = T_WIN,
        .bulk_win_us = 0,
        .telem_slot_us = (uint32_t)SEC,
    };
    return md;
}

static void init_sched(uint8_t id)
{
    memset(&cap, 0, sizeof(cap));
    listen_state = true;
    listen_calls = 0;
    mesh_cfg_t cfg = test_cfg(id, &cap);
    cfg.radio.set_listen = cap_set_listen;
    cfg.sched.enabled = true;
    cfg.sched.mode = test_mode();
    cfg.sched.guard_us = (uint32_t)GUARD;
    T_CHECK(mesh_init(&m, &cfg));
}

static void test_validation(void)
{
    mesh_cfg_t cfg = test_cfg(1, &cap);
    cfg.sched.enabled = true;
    cfg.sched.mode = test_mode();
    cfg.sched.mode.period_us = 8 * SEC; /* windows (9 s) overflow it */
    T_CHECK(!mesh_init(&m, &cfg));

    cfg.sched.mode = test_mode();
    cfg.sched.mode.beacon_win_us = 0; /* beaconless schedule is invalid */
    T_CHECK(!mesh_init(&m, &cfg));

    cfg.sched.mode = test_mode();
    cfg.sched.mode.telem_slot_us = (uint32_t)(5 * SEC); /* > window */
    T_CHECK(!mesh_init(&m, &cfg));

    cfg.sched.mode = test_mode();
    T_CHECK(mesh_init(&m, &cfg));
}

static void test_segments(void)
{
    init_sched(1);
    /* Phases within superframe 3 (arbitrary). */
    uint64_t sf0 = 3 * PERIOD;
    T_CHECK_EQ(mesh_sched_segment(&m, sf0), MESH_SEG_BEACON);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + B_WIN - 1), MESH_SEG_BEACON);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + B_WIN), MESH_SEG_CMD);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + B_WIN + C_WIN), MESH_SEG_TELEM);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + ACT_END), MESH_SEG_SLEEP);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + PERIOD - 1), MESH_SEG_SLEEP);
    T_CHECK_EQ(mesh_sched_segment(&m, sf0 + PERIOD), MESH_SEG_BEACON);
    T_CHECK_EQ(mesh_sched_superframe(&m, sf0), 3);
    T_CHECK_EQ(mesh_sched_superframe(&m, sf0 + PERIOD - 1), 3);
    T_CHECK_EQ(mesh_sched_superframe(&m, sf0 + PERIOD), 4);
}

static void test_gate(void)
{
    init_sched(1);
    uint64_t sf0 = 7 * PERIOD;

    /* Open window: pass through unchanged. */
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_BEACON, sf0 + SEC), sf0 + SEC);
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_CMD, sf0 + B_WIN), sf0 + B_WIN);

    /* Before the window within the same superframe: defer to its start. */
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_TELEM, sf0 + SEC),
               sf0 + B_WIN + C_WIN);

    /* After the window (sleep segment): defer to the next superframe. */
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_CMD, sf0 + ACT_END + SEC),
               sf0 + PERIOD + B_WIN);
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_BEACON, sf0 + ACT_END + SEC),
               sf0 + PERIOD);

    /* Zero-width bulk window never opens. */
    T_CHECK(mesh_sched_gate(&m, MESH_WIN_BULK, sf0 + SEC) > sf0 + SEC);

    /* ANY is never gated; a disabled scheduler gates nothing. */
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_ANY, sf0 + ACT_END + SEC),
               sf0 + ACT_END + SEC);
    mesh_cfg_t cfg = test_cfg(1, &cap);
    T_CHECK(mesh_init(&m, &cfg));
    T_CHECK_EQ(mesh_sched_gate(&m, MESH_WIN_CMD, 12345), 12345);
}

static void test_listen_duty_cycle(void)
{
    init_sched(1);
    uint64_t sf0 = 2 * PERIOD;

    /* Inside the active span: listening. */
    mesh_poll(&m, sf0 + SEC);
    T_CHECK(mesh_sched_listening(&m));
    T_CHECK(listen_state);

    /* Into the sleep segment (past the guard): radio off. */
    mesh_poll(&m, sf0 + ACT_END + GUARD + 1);
    T_CHECK(!mesh_sched_listening(&m));
    T_CHECK(!listen_state);

    /* Wakes a guard early for the next superframe. */
    mesh_poll(&m, sf0 + PERIOD - GUARD);
    T_CHECK(mesh_sched_listening(&m));
    T_CHECK(listen_state);

    /* Large GPS uncertainty swallows the sleep segment: stays on. */
    mesh_sched_clock_uncertainty(&m, (uint32_t)(6 * SEC));
    mesh_poll(&m, sf0 + PERIOD + ACT_END + GUARD + SEC);
    T_CHECK(mesh_sched_listening(&m));
    mesh_sched_clock_uncertainty(&m, 0);
    mesh_poll(&m, sf0 + PERIOD + ACT_END + GUARD + SEC);
    T_CHECK(!mesh_sched_listening(&m));

    /* mesh_poll returns a next-event time no later than the next
     * schedule edge (listen-on is the next edge here). */
    uint64_t next = mesh_poll(&m, sf0 + PERIOD + ACT_END + GUARD + SEC);
    T_CHECK(next <= sf0 + 2 * PERIOD - GUARD);
}

static void test_root_auto_beacon(void)
{
    init_sched(MESH_ID_ROOT);

    /* Poll inside beacon window: exactly one beacon per superframe,
     * even across repeated polls. */
    mesh_poll(&m, 0 * PERIOD + SEC / 2);
    mesh_poll(&m, 0 * PERIOD + SEC);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_BEACON, NULL), 1);

    mesh_poll(&m, 1 * PERIOD + SEC / 2);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_BEACON, NULL), 2);

    /* Polls outside the window do not beacon retroactively. */
    mesh_poll(&m, 2 * PERIOD + ACT_END + SEC);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_BEACON, NULL), 2);

    /* Field nodes never auto-beacon. */
    init_sched(5);
    mesh_poll(&m, 3 * PERIOD + SEC / 2);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_BEACON, NULL), 0);
}

static void test_command_deferred_to_window(void)
{
    init_sched(MESH_ID_ROOT);
    uint64_t sf0 = 4 * PERIOD;

    /* Command issued during the sleep segment must transmit at the next
     * command-window start, not immediately. */
    uint64_t t = sf0 + ACT_END + 2 * SEC;
    uint8_t pl[1] = { 'X' };
    uint16_t id = mesh_root_command(&m, MESH_ID_BROADCAST, pl, 1, 0, t);
    T_CHECK(id != 0);
    mesh_poll(&m, t);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 0);
    T_CHECK(m.stats.tx_deferred >= 1);

    /* Still queued through the next beacon window... */
    mesh_poll(&m, sf0 + PERIOD + SEC);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 0);
    /* ...and fires once the command window opens. */
    mesh_poll(&m, sf0 + PERIOD + B_WIN);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 1);
}

static void test_telem_micro_slots(void)
{
    /* Node 3's slot begins 2 slot-widths into the telemetry window. */
    init_sched(3);
    uint64_t sf0 = 6 * PERIOD;
    uint64_t due = mesh_sched_telem_due(&m, sf0 + SEC);
    uint64_t slot_start = sf0 + B_WIN + C_WIN + 2 * SEC;
    T_CHECK(due >= slot_start);
    T_CHECK(due < slot_start + SEC);

    /* Requested after its slot passed: lands in the next superframe. */
    due = mesh_sched_telem_due(&m, sf0 + ACT_END);
    T_CHECK(due >= slot_start + PERIOD);
    T_CHECK(due < slot_start + PERIOD + SEC);

    /* Node 1 owns the first slot. */
    init_sched(1);
    due = mesh_sched_telem_due(&m, sf0);
    T_CHECK(due >= sf0 + B_WIN + C_WIN);
    T_CHECK(due < sf0 + B_WIN + C_WIN + SEC);
}

static void test_mode_switch(void)
{
    init_sched(1);

    /* Switch to a 10 s period at an absolute future instant. */
    mesh_sched_mode_t fast = test_mode();
    fast.period_us = 10 * SEC;
    fast.telem_win_us = 1 * SEC;
    fast.telem_slot_us = 0;
    uint64_t at = 5 * PERIOD + 3 * SEC;
    T_CHECK(mesh_sched_set_mode(&m, &fast, at));
    /* Only one switch may be pending. */
    T_CHECK(!mesh_sched_set_mode(&m, &fast, at + SEC));

    /* Before `at`: old mode still in force. */
    mesh_poll(&m, at - SEC);
    T_CHECK_EQ(mesh_sched_superframe(&m, at - SEC), 5);

    /* After `at`: new mode, aligned so superframe 0 starts at `at`. */
    mesh_poll(&m, at + 1);
    T_CHECK_EQ(mesh_sched_segment(&m, at + SEC), MESH_SEG_BEACON);
    T_CHECK_EQ(mesh_sched_superframe(&m, at + SEC), 0);
    T_CHECK_EQ(mesh_sched_superframe(&m, at + 25 * SEC), 2);
    T_CHECK_EQ(mesh_sched_segment(&m, at + 10 * SEC + B_WIN), MESH_SEG_CMD);

    /* An invalid mode is rejected up front. */
    fast.beacon_win_us = 0;
    T_CHECK(!mesh_sched_set_mode(&m, &fast, at + 100 * SEC));
}

int main(void)
{
    T_RUN(test_validation);
    T_RUN(test_segments);
    T_RUN(test_gate);
    T_RUN(test_listen_duty_cycle);
    T_RUN(test_root_auto_beacon);
    T_RUN(test_command_deferred_to_window);
    T_RUN(test_telem_micro_slots);
    T_RUN(test_mode_switch);
    return t_summary();
}
