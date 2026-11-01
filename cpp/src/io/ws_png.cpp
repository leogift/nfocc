/**
 * @file ws_png.cpp
 * @brief 极简 PNG 解码：只支持 8-bit 灰度/RGB/RGBA、非隔行。
 *
 * ## 为什么不引 libpng
 * 数据集固定是 8-bit RGB、非隔行，libpng 会多一个第三方依赖、
 * 多一层 ABI 风险（本项目在 macOS 上已因 OpenCV 5.0 的 C++17 ABI
 * 吃过一次 SIGTRAP）。这里只需要 IDAT 的 zlib 解压 + 除过滤。
 *
 * ## 依赖
 * zlib 是系统自带的（macOS/Linux 都有），用 `-lz` 链接。
 */
#include "nfocc/util/input_source.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <zlib.h>

namespace nf {
namespace {

inline uint32_t be32(const unsigned char *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/** Paeth 预测器（PNG 规范 9.4） */
inline int paeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

}  // namespace

bool png_read_gray(const char *path, Png *out) {
    std::FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (sz < 8) { std::fclose(f); return false; }
    std::vector<unsigned char> buf((size_t)sz);
    if (std::fread(buf.data(), 1, (size_t)sz, f) != (size_t)sz) {
        std::fclose(f); return false;
    }
    std::fclose(f);

    static const unsigned char kSig[8] = { 137, 80, 78, 71, 13, 10, 26, 10 };
    if (std::memcmp(buf.data(), kSig, 8) != 0) return false;

    int w = 0, h = 0, bitdepth = 0, colortype = 0, interlace = 0;
    std::vector<unsigned char> idat;
    size_t p = 8;
    while (p + 8 <= buf.size()) {
        const uint32_t len = be32(&buf[p]);
        const char *type = (const char *)&buf[p + 4];
        const size_t dstart = p + 8;
        if (dstart + len > buf.size()) break;

        if (std::memcmp(type, "IHDR", 4) == 0 && len >= 13) {
            w = (int)be32(&buf[dstart]);
            h = (int)be32(&buf[dstart + 4]);
            bitdepth  = buf[dstart + 8];
            colortype = buf[dstart + 9];
            interlace = buf[dstart + 12];
        } else if (std::memcmp(type, "IDAT", 4) == 0) {
            idat.insert(idat.end(), &buf[dstart], &buf[dstart + len]);
        } else if (std::memcmp(type, "IEND", 4) == 0) {
            break;
        }
        p = dstart + len + 4;      /* 跳过 CRC */
    }

    if (w <= 0 || h <= 0 || bitdepth != 8 || interlace != 0) return false;
    int ch = 0;
    if (colortype == 0) ch = 1;        /* 灰度 */
    else if (colortype == 2) ch = 3;   /* RGB */
    else if (colortype == 4) ch = 2;   /* 灰度+alpha */
    else if (colortype == 6) ch = 4;   /* RGBA */
    else return false;
    if (idat.empty()) return false;

    /* zlib 解压 */
    const size_t stride = (size_t)w * ch;
    const size_t raw_sz = (stride + 1) * (size_t)h;
    std::vector<unsigned char> raw(raw_sz);
    uLongf dest = (uLongf)raw_sz;
    const int zr = uncompress(raw.data(), &dest, idat.data(),
                              (uLong)idat.size());
    if (zr != Z_OK || dest != raw_sz) return false;

    /* 除过滤 */
    std::vector<unsigned char> img(stride * (size_t)h);
    const unsigned char *rp = raw.data();
    for (int y = 0; y < h; ++y) {
        const int filter = *rp++;
        unsigned char *cur = &img[(size_t)y * stride];
        const unsigned char *prev = (y > 0) ? &img[(size_t)(y - 1) * stride]
                                            : nullptr;
        for (size_t x = 0; x < stride; ++x) {
            const int a = (x >= (size_t)ch) ? cur[x - ch] : 0;
            const int b = prev ? prev[x] : 0;
            const int c = (prev && x >= (size_t)ch) ? prev[x - ch] : 0;
            int v = rp[x];
            switch (filter) {
                case 0: break;
                case 1: v += a; break;
                case 2: v += b; break;
                case 3: v += (a + b) / 2; break;
                case 4: v += paeth(a, b, c); break;
                default: return false;
            }
            cur[x] = (unsigned char)(v & 0xFF);
        }
        rp += stride;
    }

    /* 转灰度 */
    std::vector<unsigned char> gray((size_t)w * (size_t)h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const unsigned char *q = &img[((size_t)y * w + x) * ch];
            int g;
            if (ch == 1 || ch == 2) {
                g = q[0];
            } else {
                /* **Y 通道 (BT.601)**，不是"加权亮度"。
                 * Y = 0.299R + 0.587G + 0.114B
                 * 定点: (77R + 150G + 29B) >> 8，系数和为 256。
                 *
                 * SGBM 要的是与传感器 Y 通道一致的灰度。用平均或别的权重
                 * 会改变噪声特性与对比度分布，影响匹配稳定性。
                 * 本项目原图若已带 YUV，应直接取 Y，不要重算。 */
                g = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            }
            gray[(size_t)y * w + x] = (unsigned char)(g > 255 ? 255 : g);
        }
    }

    out->w = w; out->h = h; out->channels = 1;
    out->data.swap(gray);
    return true;
}

}  // namespace nf
