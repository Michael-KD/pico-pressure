/*
 * test_util.h — minimal assert-macro test harness (no framework
 * dependency). A test binary runs a series of functions with T_RUN and
 * returns nonzero from t_summary() if any T_CHECK failed, which is what
 * CTest keys on.
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <inttypes.h>
#include <stdio.h>

static int t_failures;
static int t_checks;

#define T_CHECK(cond)                                                    \
    do {                                                                 \
        t_checks++;                                                      \
        if (!(cond)) {                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            t_failures++;                                                \
        }                                                                \
    } while (0)

#define T_CHECK_EQ(a, b)                                                 \
    do {                                                                 \
        t_checks++;                                                      \
        long long t_va = (long long)(a);                                 \
        long long t_vb = (long long)(b);                                 \
        if (t_va != t_vb) {                                              \
            printf("  FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__,  \
                   __LINE__, #a, #b, t_va, t_vb);                        \
            t_failures++;                                                \
        }                                                                \
    } while (0)

#define T_RUN(fn)                                                        \
    do {                                                                 \
        printf("== %s\n", #fn);                                          \
        fn();                                                            \
    } while (0)

static inline int t_summary(void)
{
    printf("%s: %d checks, %d failures\n",
           t_failures ? "FAILED" : "ok", t_checks, t_failures);
    return t_failures ? 1 : 0;
}

#endif /* TEST_UTIL_H */
