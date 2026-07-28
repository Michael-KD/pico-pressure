/* Unit tests: bulk retrieval first cut (§7.3) — CRC16, FETCH
 * validation, chunk streaming with single-in-flight discipline,
 * re-FETCH replacement, EOF short read, root-side delivery. */
#include "mesh_test_helpers.h"
#include "test_util.h"

static tx_capture_t cap;

/* Deterministic synthetic "recording" served by the node under test. */
static uint8_t src_byte(uint32_t off)
{
    return (uint8_t)(0xA5 ^ off ^ (off >> 7));
}

static struct {
    uint32_t limit;       /* bytes of data that exist */
    unsigned reads;
    uint8_t  last_session;
} g_src;

static uint8_t on_bulk_read(void *user, uint8_t session, uint32_t offset,
                            uint8_t *buf, uint8_t max)
{
    (void)user;
    g_src.reads++;
    g_src.last_session = session;
    uint8_t n = 0;
    while (n < max && offset + n < g_src.limit) {
        buf[n] = src_byte(offset + n);
        n++;
    }
    return n;
}

static struct {
    unsigned count;
    uint8_t  last_session;
    uint16_t last_idx;
    uint8_t  last_len;
    uint8_t  last_first;
} g_sink;

static void on_bulk_data(void *user, uint8_t session, uint16_t chunk_index,
                         const uint8_t *data, uint8_t len)
{
    (void)user;
    g_sink.count++;
    g_sink.last_session = session;
    g_sink.last_idx = chunk_index;
    g_sink.last_len = len;
    g_sink.last_first = len ? data[0] : 0;
}

/* Node 5, attached directly to the root, serving 1000 B of data. */
static void make_attached_node(mesh_t *m)
{
    mesh_cfg_t cfg = test_cfg(5, &cap);
    cfg.cb.on_bulk_read = on_bulk_read;
    T_CHECK(mesh_init(m, &cfg));
    memset(&g_src, 0, sizeof(g_src));
    g_src.limit = 1000;

    uint8_t buf[MESH_MAX_FRAME];
    uint8_t cost0[2] = { 0, 0 };
    size_t len = mesh_frame_build(TEST_NET, &mesh_mic_null, buf,
                                  MESH_FT_BEACON, MESH_ID_ROOT,
                                  MESH_ID_BROADCAST, 1, 0, cost0, 2);
    mesh_rx(m, buf, len, -60, 8, 1000000);
    T_CHECK_EQ(mesh_parent(m), MESH_ID_ROOT);
    drain(m, &cap);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static size_t make_fetch(uint8_t *buf, uint8_t dst, uint16_t seq,
                         uint8_t session, uint32_t base, uint32_t off,
                         uint32_t rlen, uint8_t chunk)
{
    uint8_t body[MESH_FETCH_PLEN];
    body[0] = session;
    put_le32(&body[1], base);
    put_le32(&body[5], off);
    put_le32(&body[9], rlen);
    body[13] = chunk;
    return mesh_frame_build(TEST_NET, &mesh_mic_null, buf,
                            MESH_FT_SESSION, MESH_ID_ROOT, dst, seq,
                            MESH_FLOOD_TTL, body, MESH_FETCH_PLEN);
}

/* Root hop-ACKs the node's DATA transfer (type + seq), freeing it. */
static void hop_ack_data(mesh_t *m, uint16_t seq, uint64_t t)
{
    uint8_t pl[3] = { MESH_FT_DATA, (uint8_t)(seq & 0xFF),
                      (uint8_t)(seq >> 8) };
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(TEST_NET, &mesh_mic_null, buf,
                                  MESH_FT_HOP_ACK, MESH_ID_ROOT, 5, 1, 0,
                                  pl, sizeof(pl));
    mesh_rx(m, buf, len, -60, 8, t);
}

/* Check one captured DATA frame: header fields, CRC, and content
 * against the synthetic source at `off`. */
static void check_chunk(const mesh_frame_t *f, uint8_t session,
                        uint16_t idx, uint32_t off, uint8_t dlen)
{
    T_CHECK_EQ(f->dst, MESH_ID_ROOT); /* one hop up: parent is root */
    T_CHECK_EQ(f->plen, MESH_BULK_DATA_HDR + dlen);
    T_CHECK_EQ(f->payload[0], session);
    T_CHECK_EQ(f->payload[1] | (f->payload[2] << 8), idx);
    uint16_t crc = (uint16_t)(f->payload[3] | (f->payload[4] << 8));
    T_CHECK_EQ(mesh_crc16(&f->payload[MESH_BULK_DATA_HDR], dlen), crc);
    bool match = true;
    for (uint8_t i = 0; i < dlen; i++)
        if (f->payload[MESH_BULK_DATA_HDR + i] != src_byte(off + i))
            match = false;
    T_CHECK(match);
}

static void test_crc16_vector(void)
{
    /* CRC-16/CCITT-FALSE known-answer: "123456789" -> 0x29B1. */
    T_CHECK_EQ(mesh_crc16((const uint8_t *)"123456789", 9), 0x29B1);
    T_CHECK_EQ(mesh_crc16(NULL, 0), 0xFFFF);
}

static void test_root_fetch_validation(void)
{
    mesh_t r;
    tx_capture_t rcap;
    memset(&rcap, 0, sizeof(rcap));
    mesh_cfg_t cfg = test_cfg(MESH_ID_ROOT, &rcap);
    T_CHECK(mesh_init(&r, &cfg));
    uint64_t t = 1000000;

    /* Rejected: bad target, bad chunk size, empty/misaligned ranges. */
    T_CHECK(!mesh_root_fetch(&r, MESH_ID_BROADCAST, 1, 0, 0, 100, 64, t));
    T_CHECK(!mesh_root_fetch(&r, MESH_ID_ROOT, 1, 0, 0, 100, 64, t));
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 0, 0, 100, 0, t));
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 0, 0, 100,
                             (uint8_t)(MESH_MAX_BULK_DATA + 1), t));
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 0, 0, 0, 64, t));
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 100, 0, 100, 64, t)); /* off<base */
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 0, 65, 100, 64, t)); /* misaligned */
    T_CHECK(!mesh_root_fetch(&r, 5, 1, 0, 0xFFFFFFF0u, 0x20, 8, t));

    /* Field nodes cannot originate FETCH. */
    mesh_t n;
    make_attached_node(&n);
    T_CHECK(!mesh_root_fetch(&n, 5, 1, 0, 0, 100, 64, 2000000));

    /* Accepted: a SESSION flood with the exact payload layout. */
    T_CHECK(mesh_root_fetch(&r, 5, 9, 4096, 4224, 1000, 64, t));
    mesh_poll(&r, t + 1);
    mesh_frame_t f;
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_SESSION, &f), 1);
    T_CHECK_EQ(f.src, MESH_ID_ROOT);
    T_CHECK_EQ(f.dst, 5);
    T_CHECK_EQ(f.ttl, MESH_FLOOD_TTL);
    T_CHECK_EQ(f.plen, MESH_FETCH_PLEN);
    T_CHECK_EQ(f.payload[0], 9);
    T_CHECK_EQ(f.payload[1] | (f.payload[2] << 8), 4096 & 0xFFFF);
    T_CHECK_EQ(f.payload[13], 64);
}

static void test_fetch_streams_chunks(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    /* 250 B in 100 B chunks: two full chunks and a 50 B tail. */
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_fetch(buf, 5, 500, 3, 0, 0, 250, 100);
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t);

    mesh_frame_t f;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 1);
    check_chunk(&f, 3, 0, 0, 100);
    uint16_t seq0 = f.seq;

    /* Single in flight: nothing more until the hop resolves. */
    mesh_poll(&m, t + 100000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, NULL), 1);
    T_CHECK_EQ(mesh_stats_get(&m)->bulk_chunks_sent, 1);

    /* HOP_ACK frees it; the pump emits chunk 1. */
    hop_ack_data(&m, seq0, t + 200000);
    mesh_poll(&m, t + 300000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 2);
    check_chunk(&f, 3, 1, 100, 100);

    hop_ack_data(&m, f.seq, t + 400000);
    mesh_poll(&m, t + 500000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 3);
    check_chunk(&f, 3, 2, 200, 50); /* tail chunk */

    hop_ack_data(&m, f.seq, t + 600000);
    mesh_poll(&m, t + 700000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, NULL), 3); /* done */
    T_CHECK(!m.bulk.active);
    T_CHECK_EQ(mesh_stats_get(&m)->bulk_chunks_sent, 3);
    T_CHECK_EQ(mesh_stats_get(&m)->bulk_fetches_rx, 1);
    T_CHECK_EQ(g_src.last_session, 3);

    /* The FETCH flood itself was also relayed onward. */
    mesh_poll(&m, t + 5000000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_SESSION, &f), 1);
    T_CHECK_EQ(f.ttl, MESH_FLOOD_TTL - 1);
}

static void test_refetch_replaces_session(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_fetch(buf, 5, 500, 3, 0, 0, 1000, 100);
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t);
    mesh_frame_t f;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 1);
    drain(&m, &cap); /* old chunk still unresolved... */

    /* ...when a re-FETCH of a later sub-range arrives (same session,
     * same base): the session restarts there, indices stay aligned. */
    len = make_fetch(buf, 5, 501, 3, 0, 600, 200, 100);
    mesh_rx(&m, buf, len, -60, 8, t + 1000000);
    mesh_poll(&m, t + 1000000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 1);
    check_chunk(&f, 3, 6, 600, 100); /* index 6 = 600/100 */
}

static void test_eof_short_read_closes_session(void)
{
    mesh_t m;
    make_attached_node(&m);
    g_src.limit = 130; /* less data than the requested 300 B */
    uint64_t t = 2000000;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_fetch(buf, 5, 500, 7, 0, 0, 300, 100);
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t);

    mesh_frame_t f;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 1);
    check_chunk(&f, 7, 0, 0, 100);
    hop_ack_data(&m, f.seq, t + 100000);
    mesh_poll(&m, t + 200000);

    /* 30 remaining bytes: short chunk marks EOF and ends the session. */
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, &f), 2);
    check_chunk(&f, 7, 1, 100, 30);
    T_CHECK(!m.bulk.active);
    hop_ack_data(&m, f.seq, t + 300000);
    mesh_poll(&m, t + 400000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_DATA, NULL), 2);
}

static void test_fetch_ignored_cases(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;
    uint8_t buf[MESH_MAX_FRAME];

    /* Broadcast FETCH: relayed but never executed. */
    size_t len = make_fetch(buf, MESH_ID_BROADCAST, 500, 1, 0, 0, 100, 50);
    mesh_rx(&m, buf, len, -60, 8, t);
    T_CHECK(!m.bulk.active);
    mesh_poll(&m, t + 5000000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_SESSION, NULL), 1);
    drain(&m, &cap);

    /* Misaligned req_offset: ignored. */
    len = make_fetch(buf, 5, 501, 1, 0, 33, 100, 50);
    mesh_rx(&m, buf, len, -60, 8, t + 6000000);
    T_CHECK(!m.bulk.active);

    /* Nodes without an on_bulk_read callback ignore FETCH entirely. */
    mesh_t plain;
    tx_capture_t pcap;
    memset(&pcap, 0, sizeof(pcap));
    mesh_cfg_t cfg = test_cfg(6, &pcap);
    T_CHECK(mesh_init(&plain, &cfg));
    len = make_fetch(buf, 6, 502, 1, 0, 0, 100, 50);
    mesh_rx(&plain, buf, len, -60, 8, t);
    T_CHECK(!plain.bulk.active);
    T_CHECK_EQ(mesh_stats_get(&plain)->bulk_fetches_rx, 0);
}

static void test_root_delivery_and_crc_gate(void)
{
    mesh_t r;
    tx_capture_t rcap;
    memset(&rcap, 0, sizeof(rcap));
    mesh_cfg_t cfg = test_cfg(MESH_ID_ROOT, &rcap);
    cfg.cb.on_bulk_data = on_bulk_data;
    T_CHECK(mesh_init(&r, &cfg));
    memset(&g_sink, 0, sizeof(g_sink));
    uint64_t t = 1000000;

    /* A valid chunk from node 5: delivered with fields intact. */
    uint8_t data[8] = { 11, 22, 33, 44, 55, 66, 77, 88 };
    uint16_t crc = mesh_crc16(data, sizeof(data));
    uint8_t pl[MESH_BULK_DATA_HDR + 8];
    pl[0] = 3;
    pl[1] = 4; /* chunk index 4 */
    pl[2] = 0;
    pl[3] = (uint8_t)(crc & 0xFF);
    pl[4] = (uint8_t)(crc >> 8);
    memcpy(&pl[MESH_BULK_DATA_HDR], data, sizeof(data));
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(TEST_NET, &mesh_mic_null, buf,
                                  MESH_FT_DATA, 5, MESH_ID_ROOT, 70, 7,
                                  pl, sizeof(pl));
    mesh_rx(&r, buf, len, -60, 8, t);
    T_CHECK_EQ(g_sink.count, 1);
    T_CHECK_EQ(g_sink.last_session, 3);
    T_CHECK_EQ(g_sink.last_idx, 4);
    T_CHECK_EQ(g_sink.last_len, 8);
    T_CHECK_EQ(g_sink.last_first, 11);

    /* The hop is ACKed back to the sender. */
    mesh_poll(&r, t + 1);
    mesh_frame_t hop;
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_HOP_ACK, &hop), 1);
    T_CHECK_EQ(hop.dst, 5);
    T_CHECK_EQ(hop.payload[0], MESH_FT_DATA);

    /* Duplicate delivery (lost HOP_ACK): re-ACKed, not re-delivered. */
    mesh_rx(&r, buf, len, -60, 8, t + 100000);
    mesh_poll(&r, t + 100001);
    T_CHECK_EQ(g_sink.count, 1);
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_HOP_ACK, NULL), 2);

    /* A chunk whose payload CRC lies (source-side corruption, §10.1):
     * dropped before the callback. */
    pl[3] ^= 0xFF;
    len = mesh_frame_build(TEST_NET, &mesh_mic_null, buf, MESH_FT_DATA,
                           5, MESH_ID_ROOT, 71, 7, pl, sizeof(pl));
    mesh_rx(&r, buf, len, -60, 8, t + 200000);
    T_CHECK_EQ(g_sink.count, 1);
    T_CHECK_EQ(mesh_stats_get(&r)->bulk_crc_drops, 1);
    T_CHECK_EQ(mesh_stats_get(&r)->bulk_chunks_root, 1);
}

int main(void)
{
    T_RUN(test_crc16_vector);
    T_RUN(test_root_fetch_validation);
    T_RUN(test_fetch_streams_chunks);
    T_RUN(test_refetch_replaces_session);
    T_RUN(test_eof_short_read_closes_session);
    T_RUN(test_fetch_ignored_cases);
    T_RUN(test_root_delivery_and_crc_gate);
    return t_summary();
}
