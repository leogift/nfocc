/**
 * @file hanqtech_loader.cpp
 * @brief hanqtech 标定/ego 的极简 JSON 解析。
 *
 * ## 为什么不引第三方 JSON 库
 * 核心库要求 `-fno-exceptions -fno-rtti`，而主流 JSON 库
 * (nlohmann/json, RapidJSON DOM) 都依赖异常或 RTTI。
 * 这些 JSON 的结构是固定的、扁平的，手写一个足够。
 *
 * ## 解析策略
 * 不做通用解析器，只做**按 key 抠值**：
 *   找 "key"，跳过冒号，读值（数字 / 字符串 / [数组] / {对象}）。
 * 对这份数据足够，且出错时能明确报错而不是静默给 0。
 */
#include "nfocc/hanqtech_loader.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

/** 在 [p, end) 中找 "key"，返回冒号之后的位置；找不到返回 nullptr */
const char *find_key(const char *p, const char *end, const char *key) {
    const size_t klen = std::strlen(key);
    for (const char *q = p; q + klen + 2 < end; ++q) {
        if (*q != '"') continue;
        if (std::strncmp(q + 1, key, klen) != 0) continue;
        if (q[1 + klen] != '"') continue;
        /* 找到 key，跳冒号 */
        const char *r = q + 2 + klen;
        while (r < end && (*r == ' ' || *r == '\t' || *r == '\n' || *r == '\r')) ++r;
        if (r < end && *r == ':') {
            ++r;
            while (r < end && (*r == ' ' || *r == '\t' || *r == '\n' || *r == '\r')) ++r;
            return r;
        }
    }
    return nullptr;
}

/** 读一个浮点值；失败置 ok=0 */
float read_float(const char *p, const char *end, int *ok) {
    *ok = 0;
    if (!p || p >= end) return 0.0f;
    if (*p == '"') ++p;                       /* 数值也可能被引号包着 */
    char buf[64];
    size_t n = 0;
    while (p + n < end && n < sizeof(buf) - 1) {
        const char ch = p[n];
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' ||
            ch == '.' || ch == 'e' || ch == 'E') {
            buf[n] = ch; ++n;
        } else {
            break;
        }
    }
    if (n == 0) return 0.0f;
    buf[n] = '\0';
    char *endp = nullptr;
    const double v = std::strtod(buf, &endp);
    if (endp == buf) return 0.0f;
    *ok = 1;
    return (float)v;
}

/** 读一个字符串（去掉引号），写入 out */
bool read_string(const char *p, const char *end, char *out, size_t cap) {
    if (!p || p >= end || *p != '"') return false;
    ++p;
    size_t n = 0;
    while (p < end && *p != '"' && n + 1 < cap) out[n++] = *p++;
    out[n] = '\0';
    return true;
}

/** 读 [a, b, c, ...] 到 out，返回读到的个数 */
int read_array(const char *p, const char *end, float *out, int cap) {
    if (!p || p >= end || *p != '[') return 0;
    ++p;
    int n = 0;
    while (p < end && n < cap) {
        while (p < end && (*p == ' ' || *p == ',' || *p == '\n' ||
                           *p == '\r' || *p == '\t')) ++p;
        if (p >= end || *p == ']') break;
        int ok = 0;
        const float v = read_float(p, end, &ok);
        if (!ok) break;
        out[n++] = v;
        /* 跳到下一个分隔符 */
        while (p < end && *p != ',' && *p != ']') ++p;
    }
    return n;
}

/** 读取整个文件到堆缓冲（C 风格，不用 std::string 的异常） */
char *slurp(const char *path, long *out_len) {
    std::FILE *f = std::fopen(path, "rb");
    if (!f) return nullptr;
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0) { std::fclose(f); return nullptr; }
    char *buf = (char *)std::malloc((size_t)n + 1);
    if (!buf) { std::fclose(f); return nullptr; }
    const size_t got = std::fread(buf, 1, (size_t)n, f);
    std::fclose(f);
    buf[got] = '\0';
    if (out_len) *out_len = (long)got;
    return buf;
}

/** 四元数 (xyzw) -> 旋转矩阵，行优先。数值稳定版，不求范数。 */
void quat_to_mat(const float *q, float *R) {
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float n = std::sqrt(x*x + y*y + z*z + w*w);
    if (n < 1e-12f) {
        std::memset(R, 0, 9 * sizeof(float));
        R[0] = R[4] = R[8] = 1.0f;
        return;
    }
    const float s = 2.0f / (n * n);
    const float xx = x*x*s, yy = y*y*s, zz = z*z*s;
    const float xy = x*y*s, xz = x*z*s, yz = y*z*s;
    const float wx = w*x*s, wy = w*y*s, wz = w*z*s;
    R[0] = 1.0f - (yy + zz); R[1] = xy - wz;         R[2] = xz + wy;
    R[3] = xy + wz;          R[4] = 1.0f - (xx + zz); R[5] = yz - wx;
    R[6] = xz - wy;          R[7] = yz + wx;          R[8] = 1.0f - (xx + yy);
}

}  // namespace

extern "C" {

NFStatus nf_ws_load_calib(const char *json_path, nf_u32 cam_id,
                          NFWSCalib *out) {
    if (!json_path || !out) return NF_E_PARAM_NULL;
    if (cam_id >= NF_WS_MAX_CAMS) return NF_E_PARAM_RANGE;

    long len = 0;
    char *buf = slurp(json_path, &len);
    if (!buf) return NF_E_IO;
    const char *p = buf;
    const char *end = buf + len;

    /* ---- intrinsic ---- */
    const char *pi = find_key(p, end, "intrinsic");
    if (!pi) { std::free(buf); return NF_E_PARAM_RANGE; }

    int ok = 0;
    const float w   = read_float(find_key(pi, end, "width"),  end, &ok);
    if (!ok) { std::free(buf); return NF_E_PARAM_RANGE; }
    const float h   = read_float(find_key(pi, end, "height"), end, &ok);
    if (!ok) { std::free(buf); return NF_E_PARAM_RANGE; }
    const float cxo = read_float(find_key(pi, end, "cx_offset"), end, &ok);
    if (!ok) { std::free(buf); return NF_E_PARAM_RANGE; }
    const float cyo = read_float(find_key(pi, end, "cy_offset"), end, &ok);
    if (!ok) { std::free(buf); return NF_E_PARAM_RANGE; }
    const float k1 = read_float(find_key(pi, end, "k1"), end, &ok);
    if (!ok) { std::free(buf); return NF_E_PARAM_RANGE; }
    const float k2 = read_float(find_key(pi, end, "k2"), end, &ok);
    const float k3 = read_float(find_key(pi, end, "k3"), end, &ok);
    const float k4 = read_float(find_key(pi, end, "k4"), end, &ok);
    const float ar = read_float(find_key(pi, end, "aspect_ratio"), end, &ok);

    /* ---- extrinsic ---- */
    const char *pe = find_key(p, end, "extrinsic");
    if (!pe) { std::free(buf); return NF_E_PARAM_RANGE; }
    float q[4] = { 0, 0, 0, 1 };
    const int nq = read_array(find_key(pe, end, "quaternion"), end, q, 4);
    float tr[3] = { 0, 0, 0 };
    const int nt = read_array(find_key(pe, end, "translation"), end, tr, 3);

    char nm[8] = { 0 };
    read_string(find_key(p, end, "name"), end, nm, sizeof(nm));

    std::free(buf);

    if (nq != 4 || nt != 3) return NF_E_PARAM_RANGE;

    NFCameraModel *c = &out->cam[cam_id];
    std::memset(c, 0, sizeof(*c));
    c->version = NF_CAMERA_MODEL_VERSION;
    c->kind    = NF_CAM_MODEL_RADIAL_POLY;
    c->cam_id  = (nf_u8)cam_id;
    c->k1 = k1; c->k2 = k2; c->k3 = k3; c->k4 = k4;
    /* hanqtech 的 u = ρ·X/χ + cx_offset + w/2 − 0.5，
     * 故**主点像素坐标** = cx_offset + w/2 − 0.5。 */
    c->cx = cxo + w * 0.5f - 0.5f;
    c->cy = cyo + h * 0.5f - 0.5f;
    /* aspect_ratio 若不是 1，v 方向被拉伸；本项目按 1 处理并在下方断言。
     * 实测 hanqtech 四路均为 1.0。 */
    if (std::fabs(ar - 1.0f) > 1e-6f) return NF_E_PARAM_RANGE;
    /* 有效半径: 取对角，留 45% 余量覆盖四角外缘（实测四角 theta ≈112°）。
     * 该值仅用于 LUT 的 D2 判据，宁松勿紧，真正的边界由 max_theta 管。 */
    c->r_valid = std::sqrt(w * w + h * h) * 0.5f * 1.45f;
    /* 角度上限: 四次多项式在 112° 附近单调，取 120° 容差 */
    c->max_theta = 2.0943951f;      /* 120° in rad */

    NFPose *ps = &out->pose[cam_id];
    std::memset(ps, 0, sizeof(*ps));
    ps->version = NF_POSE_VERSION;
    ps->cam_id  = (nf_u8)cam_id;
    quat_to_mat(q, ps->R);
    ps->t[0] = tr[0] * 1000.0f;     /* m -> mm */
    ps->t[1] = tr[1] * 1000.0f;
    ps->t[2] = tr[2] * 1000.0f;

    out->image_w = (nf_u32)w;
    out->image_h = (nf_u32)h;
    std::snprintf(out->name[cam_id], sizeof(out->name[0]), "%s", nm);
    if (cam_id + 1 > out->n_cams) out->n_cams = cam_id + 1;
    return NF_OK;
}

NFStatus nf_ws_load_ego(const char *json_path, NFWSEgo *out) {
    if (!json_path || !out) return NF_E_PARAM_NULL;
    long len = 0;
    char *buf = slurp(json_path, &len);
    if (!buf) return NF_E_IO;
    const char *p = buf, *end = buf + len;

    std::memset(out, 0, sizeof(*out));
    int ok = 0;
    float v;

    v = read_float(find_key(p, end, "timestamp"), end, &ok);
    out->timestamp_us = ok ? (nf_u64)v : 0;
    v = read_float(find_key(p, end, "ego_speed"), end, &ok);
    out->speed_mps = ok ? v : 0.0f;
    v = read_float(find_key(p, end, "ego_yawRate"), end, &ok);
    out->yaw_rate = ok ? v : 0.0f;
    v = read_float(find_key(p, end, "ego_steering"), end, &ok);
    out->steering = ok ? v : 0.0f;
    v = read_float(find_key(p, end, "ego_acceleration"), end, &ok);
    out->acceleration = ok ? v : 0.0f;
    v = read_float(find_key(p, end, "ego_distance"), end, &ok);
    out->distance_m = ok ? v : 0.0f;
    read_string(find_key(p, end, "ego_gear"), end, out->gear, sizeof(out->gear));

    std::free(buf);
    return NF_OK;
}

NFStatus nf_ws_load_frame(const char *calib_dir, nf_u32 frame_id,
                          NFWSCalib *out) {
    if (!calib_dir || !out) return NF_E_PARAM_NULL;
    std::memset(out, 0, sizeof(*out));
    out->frame_id = frame_id;

    /* 顺序由 hanqtech 的命名固定: FV, MVL, MVR, RV
     * 映射到项目 cam_id: 0=FV 1=RV 2=ML 3=MR
     *
     * **序号不是同一个**: 数据集给每路相机独立递增的序号。
     *   00164_FV  00165_MVL  00166_MVR  00167_RV
     *   00168_FV  00169_MVL  ...
     * 即 frame_id 传 FV 的序号, 后面三路是 +1/+2/+3。
     * 实测教训: 一开始四路都用 frame_id, 只有 FV 加载成功。 */
    static const char *kNames[4] = { "FV", "MVL", "MVR", "RV" };
    static const nf_u32 kCamId[4] = { 0, 2, 3, 1 };

    NFStatus last = NF_OK;
    for (int i = 0; i < 4; ++i) {
        char path[512];
        std::snprintf(path, sizeof(path), "%s/%05u_%s.json",
                      calib_dir, frame_id + (nf_u32)i, kNames[i]);
        const NFStatus st = nf_ws_load_calib(path, kCamId[i], out);
        if (st != NF_OK) last = st;
    }
    /* 四路都加载到才算成功 */
    if (out->n_cams < NF_WS_MAX_CAMS) return last != NF_OK ? last : NF_E_PARAM_RANGE;
    return NF_OK;
}

} /* extern "C" */
