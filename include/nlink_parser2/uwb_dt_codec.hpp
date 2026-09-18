#ifndef NLINK_PARSER2_UWB_DT_CODEC_HPP
#define NLINK_PARSER2_UWB_DT_CODEC_HPP

/* Compact wire codec for LinkTrack data-transmission (DT) payloads.
 *
 * Header only, no dependencies beyond the standard library, so a SLAM node can
 * include it without linking against this package.
 *
 * Constraints this codec exists to work around, all measured on LinkTrack P-B
 * hardware in DR_MODE1:
 *
 *   - A single over-the-air transmission carries at most ~40 bytes. Larger
 *     writes are not fragmented and reassembled, they are mostly discarded.
 *   - The channel is a transparent byte pipe, NOT a message service. Messages
 *     written back to back arrive concatenated, and a single message can arrive
 *     split across chunks. Hence COBS framing plus a CRC.
 *   - The channel is lossy and occasionally duplicates. Hence the sequence
 *     number; there is no retransmission.
 *   - It IS fully binary transparent: 0x00, 0xFF and even the 0x55 0x09 frame
 *     header pattern pass through unaltered.
 *
 * Two usage rules that matter as much as the encoding:
 *
 *   1. Demultiplex by sender before decoding. Every NodeFrame6 node entry
 *      carries the sender's UID in nodes[i].id; bytes from different senders
 *      interleave and must never share a reassembly buffer. Reassembler below
 *      does this for you.
 *   2. Never write packets back to back. Stagger them by >=150 ms. The module's
 *      DT buffer holds roughly one slot and drops older bytes on overrun;
 *      staggering measured 29% delivery against 2% for the same packets sent
 *      as a burst.
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace uwb_dt
{

/* ------------------------------------------------------------------ types */

enum MsgType : uint8_t
{
    MSG_POSE     = 0x01,  /* 24 B: pose + per-axis sigmas */
    MSG_VEL      = 0x02,  /* 15 B: linear velocity + per-axis sigmas */
    MSG_SYNC_REQ = 0x03,  /*  8 B: clock sync request */
    MSG_SYNC_RSP = 0x04,  /* 10 B: clock sync response */
};

constexpr size_t POSE_SIZE     = 24;
constexpr size_t VEL_SIZE      = 15;
constexpr size_t SYNC_REQ_SIZE = 8;
constexpr size_t SYNC_RSP_SIZE = 10;

/* Largest payload that still fits a single transmission once COBS framing
 * (+1 overhead byte, +1 delimiter) is applied. */
constexpr size_t MAX_PAYLOAD = 38;

struct Pose
{
    uint8_t seq = 0;
    uint16_t t_ms = 0;            /* sender's own clock, wraps every 65.536 s */
    double x = 0, y = 0, z = 0;                 /* metres */
    double roll = 0, pitch = 0, yaw = 0;        /* radians */
    double sx = 0, sy = 0, sz = 0;              /* position sigmas, metres */
    double sroll = 0, spitch = 0, syaw = 0;     /* attitude sigmas, radians */
};

struct Vel
{
    uint8_t seq = 0;
    uint16_t t_ms = 0;
    double vx = 0, vy = 0, vz = 0;              /* m/s */
    double svx = 0, svy = 0, svz = 0;           /* velocity sigmas, m/s */
};

struct SyncReq
{
    uint8_t seq = 0;
    uint32_t t1_ms = 0;           /* requester's send time */
};

struct SyncRsp
{
    uint8_t seq = 0;              /* echoes the request's seq */
    uint32_t t2_ms = 0;           /* responder's receive time */
    uint16_t dt23_ms = 0;         /* responder's send time minus t2 */
};

/* ------------------------------------------------------------- quantisation
 *
 * Position resolves to 1 cm over +-327 m, attitude to 1e-4 rad, velocity to
 * 1 mm/s over +-32.7 m/s. All far finer than the +-10 cm the UWB ranging
 * itself achieves, so quantisation is not the limiting error term.
 *
 * Sigmas span orders of magnitude, so they use a logarithmic uint8: 0.1 mm to
 * 100 m in 256 steps is 5.6% per step. Out-of-range values saturate rather
 * than wrap, so a wildly wrong covariance degrades instead of aliasing to a
 * confident one.
 */

constexpr double SIGMA_MIN = 1e-4;
constexpr double SIGMA_MAX = 1e2;

inline uint8_t encodeSigma(double sigma)
{
    if (!(sigma > SIGMA_MIN))
    {
        return 0;
    }
    if (sigma >= SIGMA_MAX)
    {
        return 255;
    }
    const double r = std::log(sigma / SIGMA_MIN) / std::log(SIGMA_MAX / SIGMA_MIN);
    const long v = std::lround(r * 255.0);
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

inline double decodeSigma(uint8_t code)
{
    return SIGMA_MIN * std::pow(SIGMA_MAX / SIGMA_MIN, code / 255.0);
}

inline int16_t saturate16(double v)
{
    if (v > 32767.0)
    {
        return 32767;
    }
    if (v < -32768.0)
    {
        return -32768;
    }
    return static_cast<int16_t>(std::lround(v));
}

/* ---------------------------------------------------------------- checksum */

inline uint16_t crc16(const uint8_t * data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

/* ------------------------------------------------------------- little endian */

inline void put16(std::vector<uint8_t> & b, uint16_t v)
{
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>(v >> 8));
}

inline void puti16(std::vector<uint8_t> & b, int16_t v)
{
    put16(b, static_cast<uint16_t>(v));
}

inline void put32(std::vector<uint8_t> & b, uint32_t v)
{
    for (int i = 0; i < 4; ++i)
    {
        b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
}

inline uint16_t get16(const uint8_t * p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline int16_t geti16(const uint8_t * p)
{
    return static_cast<int16_t>(get16(p));
}

inline uint32_t get32(const uint8_t * p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/* ------------------------------------------------------------------- COBS
 *
 * Removes every 0x00 from the body so 0x00 delimits frames unambiguously.
 * Costs one byte per packet and resynchronises by itself after a loss, which
 * matters because this channel concatenates and splits arbitrarily.
 */

inline std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> & in)
{
    std::vector<uint8_t> out;
    out.reserve(in.size() + in.size() / 254 + 2);

    size_t code_pos = 0;
    uint8_t code = 1;
    out.push_back(0);

    for (uint8_t byte : in)
    {
        if (byte != 0)
        {
            out.push_back(byte);
            ++code;
            if (code != 0xFF)
            {
                continue;
            }
        }
        out[code_pos] = code;
        code_pos = out.size();
        out.push_back(0);
        code = 1;
    }

    out[code_pos] = code;
    out.push_back(0);  /* frame delimiter */
    return out;
}

/* Decodes one delimiter-free fragment. Returns empty on a malformed frame. */
inline std::vector<uint8_t> cobsDecode(const uint8_t * in, size_t len)
{
    std::vector<uint8_t> out;
    out.reserve(len);

    size_t i = 0;
    while (i < len)
    {
        const uint8_t code = in[i];
        if (code == 0)
        {
            return {};
        }
        if (i + code > len + 1)
        {
            return {};
        }
        for (uint8_t k = 1; k < code && i + k < len; ++k)
        {
            out.push_back(in[i + k]);
        }
        i += code;
        if (code != 0xFF && i < len)
        {
            out.push_back(0);
        }
    }
    return out;
}

/* ------------------------------------------------------------------ encode */

namespace detail
{
inline std::vector<uint8_t> finish(std::vector<uint8_t> & body)
{
    put16(body, crc16(body.data(), body.size()));
    return cobsEncode(body);
}
}  // namespace detail

/* Each encode* returns COBS-framed bytes ready to write to the DT topic. */

inline std::vector<uint8_t> encodePose(const Pose & p)
{
    std::vector<uint8_t> b;
    b.reserve(POSE_SIZE);
    b.push_back(MSG_POSE);
    b.push_back(p.seq);
    put16(b, p.t_ms);
    puti16(b, saturate16(p.x * 100.0));         /* cm */
    puti16(b, saturate16(p.y * 100.0));
    puti16(b, saturate16(p.z * 100.0));
    puti16(b, saturate16(p.roll * 1e4));        /* rad * 1e4 */
    puti16(b, saturate16(p.pitch * 1e4));
    puti16(b, saturate16(p.yaw * 1e4));
    b.push_back(encodeSigma(p.sx));
    b.push_back(encodeSigma(p.sy));
    b.push_back(encodeSigma(p.sz));
    b.push_back(encodeSigma(p.sroll));
    b.push_back(encodeSigma(p.spitch));
    b.push_back(encodeSigma(p.syaw));
    return detail::finish(b);
}

inline std::vector<uint8_t> encodeVel(const Vel & v)
{
    std::vector<uint8_t> b;
    b.reserve(VEL_SIZE);
    b.push_back(MSG_VEL);
    b.push_back(v.seq);
    put16(b, v.t_ms);
    puti16(b, saturate16(v.vx * 1000.0));       /* mm/s */
    puti16(b, saturate16(v.vy * 1000.0));
    puti16(b, saturate16(v.vz * 1000.0));
    b.push_back(encodeSigma(v.svx));
    b.push_back(encodeSigma(v.svy));
    b.push_back(encodeSigma(v.svz));
    return detail::finish(b);
}

inline std::vector<uint8_t> encodeSyncReq(const SyncReq & s)
{
    std::vector<uint8_t> b;
    b.reserve(SYNC_REQ_SIZE);
    b.push_back(MSG_SYNC_REQ);
    b.push_back(s.seq);
    put32(b, s.t1_ms);
    return detail::finish(b);
}

inline std::vector<uint8_t> encodeSyncRsp(const SyncRsp & s)
{
    std::vector<uint8_t> b;
    b.reserve(SYNC_RSP_SIZE);
    b.push_back(MSG_SYNC_RSP);
    b.push_back(s.seq);
    put32(b, s.t2_ms);
    put16(b, s.dt23_ms);
    return detail::finish(b);
}

/* ------------------------------------------------------------------ decode */

inline bool expectedSize(uint8_t type, size_t & out)
{
    switch (type)
    {
    case MSG_POSE:     out = POSE_SIZE;     return true;
    case MSG_VEL:      out = VEL_SIZE;      return true;
    case MSG_SYNC_REQ: out = SYNC_REQ_SIZE; return true;
    case MSG_SYNC_RSP: out = SYNC_RSP_SIZE; return true;
    default:                                return false;
    }
}

/* Validates type, length and CRC of an already COBS-decoded packet. */
inline bool validate(const std::vector<uint8_t> & p)
{
    if (p.size() < 4)
    {
        return false;
    }
    size_t want = 0;
    if (!expectedSize(p[0], want) || p.size() != want)
    {
        return false;
    }
    const uint16_t want_crc = get16(p.data() + p.size() - 2);
    return crc16(p.data(), p.size() - 2) == want_crc;
}

inline Pose decodePose(const std::vector<uint8_t> & p)
{
    Pose o;
    o.seq = p[1];
    o.t_ms = get16(&p[2]);
    o.x = geti16(&p[4]) / 100.0;
    o.y = geti16(&p[6]) / 100.0;
    o.z = geti16(&p[8]) / 100.0;
    o.roll = geti16(&p[10]) / 1e4;
    o.pitch = geti16(&p[12]) / 1e4;
    o.yaw = geti16(&p[14]) / 1e4;
    o.sx = decodeSigma(p[16]);
    o.sy = decodeSigma(p[17]);
    o.sz = decodeSigma(p[18]);
    o.sroll = decodeSigma(p[19]);
    o.spitch = decodeSigma(p[20]);
    o.syaw = decodeSigma(p[21]);
    return o;
}

inline Vel decodeVel(const std::vector<uint8_t> & p)
{
    Vel o;
    o.seq = p[1];
    o.t_ms = get16(&p[2]);
    o.vx = geti16(&p[4]) / 1000.0;
    o.vy = geti16(&p[6]) / 1000.0;
    o.vz = geti16(&p[8]) / 1000.0;
    o.svx = decodeSigma(p[10]);
    o.svy = decodeSigma(p[11]);
    o.svz = decodeSigma(p[12]);
    return o;
}

inline SyncReq decodeSyncReq(const std::vector<uint8_t> & p)
{
    SyncReq o;
    o.seq = p[1];
    o.t1_ms = get32(&p[2]);
    return o;
}

inline SyncRsp decodeSyncRsp(const std::vector<uint8_t> & p)
{
    SyncRsp o;
    o.seq = p[1];
    o.t2_ms = get32(&p[2]);
    o.dt23_ms = get16(&p[6]);
    return o;
}

/* ------------------------------------------------------------ reassembly
 *
 * One buffer per sender UID. Feeding bytes from two senders into a shared
 * buffer shreds both streams: measured 2% delivery shared versus 29% per
 * sender, which looks exactly like radio loss and is not.
 */

class Reassembler
{
public:
    /* Guards against a sender that never emits a delimiter. */
    static constexpr size_t MAX_BUFFER = 512;

    /* Feeds one NodeFrame6 node entry. on_packet(sender_id, packet) fires for
     * every frame that passes type, length and CRC validation. */
    template <typename F>
    void feed(uint32_t sender_id, const uint8_t * data, size_t len, F && on_packet)
    {
        std::vector<uint8_t> & buf = buffers_[sender_id];

        for (size_t i = 0; i < len; ++i)
        {
            const uint8_t byte = data[i];
            if (byte == 0)
            {
                if (!buf.empty())
                {
                    std::vector<uint8_t> pkt = cobsDecode(buf.data(), buf.size());
                    if (validate(pkt))
                    {
                        on_packet(sender_id, pkt);
                    }
                    buf.clear();
                }
                continue;
            }

            buf.push_back(byte);
            if (buf.size() > MAX_BUFFER)
            {
                buf.clear();  /* desynchronised; wait for the next delimiter */
            }
        }
    }

    void reset(uint32_t sender_id)
    {
        buffers_.erase(sender_id);
    }

    void clear()
    {
        buffers_.clear();
    }

private:
    std::unordered_map<uint32_t, std::vector<uint8_t>> buffers_;
};

/* --------------------------------------------------------------- duplicates
 *
 * The channel occasionally delivers a packet twice. Tracks the last sequence
 * number per (sender, type) and reports whether a packet is new.
 */

class DuplicateFilter
{
public:
    bool isNew(uint32_t sender_id, uint8_t type, uint8_t seq)
    {
        const uint64_t key = (static_cast<uint64_t>(sender_id) << 8) | type;
        auto it = last_.find(key);
        if (it != last_.end() && it->second == seq)
        {
            return false;
        }
        last_[key] = seq;
        return true;
    }

private:
    std::unordered_map<uint64_t, uint8_t> last_;
};

}  // namespace uwb_dt

#endif  /* NLINK_PARSER2_UWB_DT_CODEC_HPP */
