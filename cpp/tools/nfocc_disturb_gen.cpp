/**
 * @file nfocc_disturb_gen.cpp
 * @brief 外参扰动数据生成器：模拟急刹急转。
 *
 * ## 为什么需要它
 * D4 双基线、在线自校准都需要验证"外参有偏差时能不能被纠正"。
 * 真车数据拿不到，但**扰动是已知的**——可以人为注入 ±7.5° 的 yaw/pitch
 * 偏差，再看算法能否估回来。这比"看起来合理"强得多。
 *
 * ## 扰动模型
 * 急刹 -> 车头下俯（pitch 为正，绕车体 Y 轴）
 * 急转 -> 车身侧倾（roll，绕车体 X 轴）+ 航向偏转（yaw，绕 Z 轴）
 *
 * 每帧的扰动 = 基准扰动 + 随机抖动，幅值由 --max-deg 控制（默认 7.5°）。
 *
 * ## 输出格式
 * 每帧打印：
 *   frame=<id> v=<km/h> yaw=<deg> pitch=<deg> roll=<deg> b_true=<mm>
 * 供下游脚本解析，作为"真值外参"与"标定外参"对拍。
 *
 * 用法:
 *   nfocc_disturb_gen --frames 50 --v0 5 --v1 15 --max-deg 7.5 --seed 42
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

/** 简单可复现的 LCG，避免依赖 rand() 的实现差异 */
struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed ? seed : 1ULL) {}
    unsigned long long next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return s >> 33;
    }
    /** [0, 1) */
    double uni() { return (double)(next() & 0xFFFFFF) / 16777216.0; }
    /** [-1, 1) */
    double sym() { return uni() * 2.0 - 1.0; }
};

}  // namespace

int main(int argc, char **argv) {
    int   frames   = 50;
    double v0      = 5.0;      /* km/h */
    double v1      = 15.0;
    double max_deg = 7.5;
    unsigned long long seed = 42;
    double hz = 5.0;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--v0") && i + 1 < argc) v0 = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--v1") && i + 1 < argc) v1 = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--max-deg") && i + 1 < argc) max_deg = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = (unsigned long long)std::atoll(argv[++i]);
        else if (!std::strcmp(argv[i], "--hz") && i + 1 < argc) hz = std::atof(argv[++i]);
    }

    Rng rng(seed);
    const double dt = 1.0 / hz;

    std::printf("# NFOCC 外参扰动序列\n");
    std::printf("# 急刹 = pitch+, 急转 = roll+, 幅度上限 %.1f 度\n", max_deg);
    std::printf("# frame v_kmh b_true_mm yaw_deg pitch_deg roll_deg\n");

    for (int f = 0; f < frames; ++f) {
        /* 车速在 v0..v1 之间来回扫（模拟起步-加速-减速） */
        const double t = (double)f / (double)(frames > 1 ? frames - 1 : 1);
        const double v = v0 + (v1 - v0) * (0.5 - 0.5 * std::cos(t * 6.28318530718));

        /* 每 10 帧来一次"急刹"或"急转"事件 */
        const int phase = f % 20;
        double pitch_base = 0.0, roll_base = 0.0, yaw_base = 0.0;
        if (phase >= 5 && phase < 8) {
            pitch_base = max_deg * 0.8;          /* 急刹：车头下俯 */
        } else if (phase >= 12 && phase < 16) {
            roll_base = max_deg * 0.7;           /* 急转：车身侧倾 */
            yaw_base  = max_deg * 0.5;
        }

        /* 随机抖动，保证不出 ±max_deg */
        double pitch = pitch_base + max_deg * 0.2 * rng.sym();
        double roll  = roll_base  + max_deg * 0.2 * rng.sym();
        double yaw   = yaw_base   + max_deg * 0.3 * rng.sym();

        /* 硬限幅 */
        if (pitch >  max_deg) pitch =  max_deg;
        if (pitch < -max_deg) pitch = -max_deg;
        if (roll  >  max_deg) roll  =  max_deg;
        if (roll  < -max_deg) roll  = -max_deg;
        if (yaw   >  max_deg) yaw   =  max_deg;
        if (yaw   < -max_deg) yaw   = -max_deg;

        /* 真实基线：车速 × dt，但俯仰会让有效基线缩短 cos(pitch) */
        const double b_nom = (v / 3.6) * dt * 1000.0;
        const double b_eff = b_nom * std::cos(pitch * 3.14159265358979 / 180.0);

        std::printf("%d %.3f %.2f %.4f %.4f %.4f\n",
                    f, v, b_eff, yaw, pitch, roll);
    }
    return 0;
}
