/**
 * @file nf_test.h
 * @brief 轻量测试框架。不引入 gtest —— Aarch64 交叉编译时少一个依赖。
 *
 * 用法:
 *   NF_TEST(case_name) {
 *       NF_CHECK(expr);
 *       NF_CHECK_EQ_INT(a, b);
 *       NF_CHECK_NEAR(a, b, 1e-6);
 *   }
 *   int main() { return nf_test_run_all(); }
 */
#ifndef NF_TEST_H
#define NF_TEST_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace nftest {

struct Case {
    const char *name;
    void (*fn)();
};

inline std::vector<Case> &registry() {
    static std::vector<Case> r;
    return r;
}

struct Failure {
    std::string what;
};

inline int &fail_count() { static int n = 0; return n; }
inline int &check_count() { static int n = 0; return n; }
inline const char *&current_case() { static const char *c = ""; return c; }

struct Registrar {
    Registrar(const char *name, void (*fn)()) {
        Case c; c.name = name; c.fn = fn;
        registry().push_back(c);
    }
};

inline void report_fail(const char *file, int line, const std::string &msg) {
    ++fail_count();
    std::printf("    FAIL [%s] %s:%d  %s\n", current_case(), file, line, msg.c_str());
}

inline int run_all() {
    int failed_cases = 0;
    /* NF_TEST_FILTER=<substr> 只跑名字包含该子串的用例。长测试（depth_accuracy）
     * 单跑一个用例时用，避免为了验证一个 case 重跑全部。 */
    const char *filter = std::getenv("NF_TEST_FILTER");
    std::printf("跑 %d 个用例\n", (int)registry().size());
    for (size_t i = 0; i < registry().size(); ++i) {
        if (filter && std::strstr(registry()[i].name, filter) == nullptr)
            continue;
        current_case() = registry()[i].name;
        const int before = fail_count();
        std::printf("  [%2d] %-40s", (int)i + 1, registry()[i].name);
        std::fflush(stdout);
        registry()[i].fn();
        if (fail_count() == before) std::printf("PASS\n");
        else { std::printf("FAIL\n"); ++failed_cases; }
    }
    std::printf("\n检查 %d 次, 失败 %d 次, 用例失败 %d/%d\n",
                check_count(), fail_count(), failed_cases, (int)registry().size());
    return failed_cases == 0 ? 0 : 1;
}

} // namespace nftest

#define NF_TEST(name)                                                    \
    static void nf_test_fn_##name();                                     \
    static nftest::Registrar nf_test_reg_##name(#name, nf_test_fn_##name); \
    static void nf_test_fn_##name()

#define NF_CHECK(expr)                                                   \
    do {                                                                 \
        ++nftest::check_count();                                         \
        if (!(expr)) nftest::report_fail(__FILE__, __LINE__, #expr);     \
    } while (0)

#define NF_CHECK_MSG(expr, msg)                                          \
    do {                                                                 \
        ++nftest::check_count();                                         \
        if (!(expr)) nftest::report_fail(__FILE__, __LINE__, (msg));     \
    } while (0)

#define NF_CHECK_EQ_INT(a, b)                                            \
    do {                                                                 \
        ++nftest::check_count();                                         \
        const long long _va = (long long)(a), _vb = (long long)(b);      \
        if (_va != _vb) {                                                \
            char _buf[256];                                              \
            std::snprintf(_buf, sizeof(_buf), "%s=%lld != %s=%lld",      \
                          #a, _va, #b, _vb);                             \
            nftest::report_fail(__FILE__, __LINE__, _buf);               \
        }                                                                \
    } while (0)

#define NF_CHECK_NEAR(a, b, tol)                                         \
    do {                                                                 \
        ++nftest::check_count();                                         \
        const double _va = (double)(a), _vb = (double)(b);               \
        if (!(std::fabs(_va - _vb) <= (double)(tol))) {                  \
            char _buf[256];                                              \
            std::snprintf(_buf, sizeof(_buf), "%s=%.9g vs %s=%.9g (tol %.1e)", \
                          #a, _va, #b, _vb, (double)(tol));              \
            nftest::report_fail(__FILE__, __LINE__, _buf);               \
        }                                                                \
    } while (0)

/** 期望某表达式返回指定 NFStatus */
#define NF_CHECK_STATUS(call, expect)                                    \
    do {                                                                 \
        ++nftest::check_count();                                         \
        const NFStatus _st = (call);                                     \
        if (_st != (expect)) {                                           \
            char _buf[256];                                              \
            std::snprintf(_buf, sizeof(_buf), "%s 返回 %s, 期望 %s",     \
                          #call, nf_status_str(_st), nf_status_str(expect)); \
            nftest::report_fail(__FILE__, __LINE__, _buf);               \
        }                                                                \
    } while (0)

#endif /* NF_TEST_H */
