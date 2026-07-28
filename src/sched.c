/*
 * sched.c — GPS-time-aligned superframe scheduler (design doc §4).
 *
 * Everything here is pure arithmetic on the externally supplied clock:
 * the superframe layout is a function of (now - mode_start) mod period,
 * so a fleet sharing GPS time computes identical schedules with no
 * radio traffic. Alignment origin is absolute time 0 for the initial
 * mode, and the commanded switch time for every subsequent mode — both
 * are fleet-wide constants.
 *
 * Interactions with the rest of the library:
 *   - txqueue.c calls mesh_sched_gate() before firing an entry and
 *     defers it into its window;
 *   - telemetry.c places own-origin reports via mesh_sched_telem_due();
 *   - mesh_poll() calls mesh_sched_poll(), which owns set_listen()
 *     duty cycling and the root's automatic beacon origination.
 */
#include "mesh_internal.h"

mesh_sched_mode_t mesh_sched_mode_active(void)
{
    mesh_sched_mode_t md = {
        .period_us = MESH_SF_ACTIVE_PERIOD_US,
        .beacon_win_us = MESH_SF_BEACON_WIN_US,
        .cmd_win_us = MESH_SF_CMD_WIN_US,
        .telem_win_us = MESH_SF_TELEM_WIN_US,
        .bulk_win_us = 0,
        .telem_slot_us = MESH_SF_TELEM_SLOT_US,
    };
    return md;
}

mesh_sched_mode_t mesh_sched_mode_standby(void)
{
    mesh_sched_mode_t md = mesh_sched_mode_active();
    md.period_us = MESH_SF_STANDBY_PERIOD_US;
    return md;
}

/* Sum of the active windows = offset where the sleep segment starts. */
static uint64_t active_end(const mesh_sched_mode_t *md)
{
    return (uint64_t)md->beacon_win_us + md->cmd_win_us +
           md->telem_win_us + md->bulk_win_us;
}

bool mesh_sched_mode_valid(const mesh_sched_mode_t *md)
{
    return md->period_us > 0 && md->beacon_win_us > 0 &&
           active_end(md) <= md->period_us &&
           md->telem_slot_us <= md->telem_win_us;
}

/* Phase within the current superframe and that superframe's absolute
 * start time. now < mode_start cannot happen in normal operation (mode
 * switches apply only once their time arrives); clamp defensively. */
static uint64_t sched_phase(const mesh_t *m, uint64_t now, uint64_t *sf0)
{
    uint64_t rel = (now >= m->mode_start_us) ? now - m->mode_start_us : 0;
    uint64_t phase = rel % m->sched.mode.period_us;
    if (sf0)
        *sf0 = now - phase;
    return phase;
}

static void win_bounds(const mesh_sched_mode_t *md, uint8_t win,
                       uint64_t *start, uint64_t *end)
{
    uint64_t b = md->beacon_win_us;
    uint64_t c = md->cmd_win_us;
    uint64_t t = md->telem_win_us;
    switch (win) {
    case MESH_WIN_BEACON:
        *start = 0;
        *end = b;
        break;
    case MESH_WIN_CMD:
        *start = b;
        *end = b + c;
        break;
    case MESH_WIN_TELEM:
        *start = b + c;
        *end = b + c + t;
        break;
    default: /* MESH_WIN_BULK */
        *start = b + c + t;
        *end = b + c + t + md->bulk_win_us;
        break;
    }
}

uint64_t mesh_sched_gate(mesh_t *m, uint8_t win, uint64_t now)
{
    if (!m->sched.enabled || win == MESH_WIN_ANY)
        return now;

    uint64_t sf0, ws, we;
    uint64_t phase = sched_phase(m, now, &sf0);
    win_bounds(&m->sched.mode, win, &ws, &we);
    if (ws == we)
        return MESH_NO_EVENT; /* window absent in this mode: never opens */
    if (phase >= ws && phase < we)
        return now;
    if (phase < ws)
        return sf0 + ws;
    return sf0 + m->sched.mode.period_us + ws;
}

mesh_segment_t mesh_sched_segment(const mesh_t *m, uint64_t now)
{
    const mesh_sched_mode_t *md = &m->sched.mode;
    if (!m->sched.enabled || md->period_us == 0)
        return MESH_SEG_SLEEP;
    uint64_t phase = sched_phase(m, now, NULL);
    uint64_t edge = md->beacon_win_us;
    if (phase < edge)
        return MESH_SEG_BEACON;
    if (phase < (edge += md->cmd_win_us))
        return MESH_SEG_CMD;
    if (phase < (edge += md->telem_win_us))
        return MESH_SEG_TELEM;
    if (phase < (edge += md->bulk_win_us))
        return MESH_SEG_BULK;
    return MESH_SEG_SLEEP;
}

uint32_t mesh_sched_superframe(const mesh_t *m, uint64_t now)
{
    if (!m->sched.enabled || m->sched.mode.period_us == 0)
        return 0;
    uint64_t rel = (now >= m->mode_start_us) ? now - m->mode_start_us : 0;
    return (uint32_t)(rel / m->sched.mode.period_us);
}

bool mesh_sched_set_mode(mesh_t *m, const mesh_sched_mode_t *mode,
                         uint64_t at_us)
{
    if (!m || !m->initialized || !m->sched.enabled || !mode)
        return false;
    if (!mesh_sched_mode_valid(mode))
        return false;
    if (m->pending_mode_at != MESH_NO_EVENT)
        return false; /* one pending switch at a time */
    m->pending_mode = *mode;
    m->pending_mode_at = at_us;
    return true;
}

void mesh_sched_clock_uncertainty(mesh_t *m, uint32_t us)
{
    if (m)
        m->clock_unc_us = us;
}

bool mesh_sched_listening(const mesh_t *m)
{
    return m->listening;
}

uint64_t mesh_sched_telem_due(mesh_t *m, uint64_t now)
{
    const mesh_sched_mode_t *md = &m->sched.mode;
    if (!m->sched.enabled || md->telem_slot_us == 0)
        return now + mesh_rand_below(m, MESH_UP_JITTER_US);

    /* Slot offset inside the telemetry window, ordered by node ID
     * (§4). Wraps for fleets larger than the window holds — the
     * per-hop jitter and retries absorb the sharing. */
    uint64_t ws = (uint64_t)md->beacon_win_us + md->cmd_win_us;
    uint64_t slot_off = ((uint64_t)(m->node_id - 1) * md->telem_slot_us) %
                        md->telem_win_us;
    uint64_t target = ws + slot_off;

    uint64_t sf0;
    uint64_t phase = sched_phase(m, now, &sf0);
    uint64_t due = (phase <= target) ? sf0 + target
                                     : sf0 + md->period_us + target;

    /* Small jitter inside the slot so retransmissions of a lost first
     * attempt don't re-collide superframe after superframe. */
    uint32_t j = md->telem_slot_us / 4;
    if (j > MESH_UP_JITTER_US)
        j = MESH_UP_JITTER_US;
    return due + mesh_rand_below(m, j);
}

uint64_t mesh_sched_poll(mesh_t *m, uint64_t now)
{
    if (!m->sched.enabled)
        return MESH_NO_EVENT;

    /* Fleet-synchronized mode switch. */
    if (m->pending_mode_at != MESH_NO_EVENT && now >= m->pending_mode_at) {
        m->sched.mode = m->pending_mode;
        m->mode_start_us = m->pending_mode_at;
        m->pending_mode_at = MESH_NO_EVENT;
        m->have_beacon_sf = false;
    }

    const mesh_sched_mode_t *md = &m->sched.mode;
    uint64_t sf0;
    uint64_t phase = sched_phase(m, now, &sf0);
    uint64_t act_end = active_end(md);
    uint64_t lead = (uint64_t)m->sched.guard_us + m->clock_unc_us;

    /* Root: one beacon per superframe, originated inside the beacon
     * window. (If a poll never lands inside the window — the app
     * stalled — the superframe simply goes unbeaconed; the tree
     * tolerates missed epochs by design.) */
    if (m->is_root && phase < md->beacon_win_us) {
        uint32_t sf = mesh_sched_superframe(m, now);
        if (!m->have_beacon_sf || m->last_beacon_sf != sf) {
            m->have_beacon_sf = true;
            m->last_beacon_sf = sf;
            mesh_root_beacon(m, now);
        }
    }

    /* Radio duty cycle: listen from (superframe start - lead) through
     * (end of last active window + lead). A sleep segment shorter than
     * the two guard flanks is not worth sleeping for — and a large GPS
     * uncertainty degrades to always-listening (§4). */
    uint64_t sleep_len = md->period_us - act_end;
    bool want = true;
    if (sleep_len > 2 * lead)
        want = (phase < act_end + lead) ||
               (phase >= md->period_us - lead);
    if (want != m->listening) {
        m->listening = want;
        if (m->radio.set_listen)
            m->radio.set_listen(m->radio.ctx, want, MESH_PROFILE_CONTROL);
    }

    /* Next schedule edge we must be polled at: listen-off, listen-on,
     * the next superframe start (beacon origination), or a pending
     * mode switch — whichever comes first. */
    uint64_t next = sf0 + md->period_us; /* next superframe start */
    if (sleep_len > 2 * lead) {
        uint64_t off_edge = sf0 + act_end + lead;
        uint64_t on_edge = sf0 + md->period_us - lead;
        if (off_edge > now && off_edge < next)
            next = off_edge;
        if (on_edge > now && on_edge < next)
            next = on_edge;
    }
    if (m->pending_mode_at != MESH_NO_EVENT && m->pending_mode_at < next)
        next = m->pending_mode_at;
    return next;
}
