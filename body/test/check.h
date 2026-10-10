// Minimal test harness: CHECK records a failure and keeps going, so one run
// reports every broken case. main() returns the failure count.
#ifndef KILO_TEST_CHECK_H
#define KILO_TEST_CHECK_H
#include <cstdio>

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_fail++;                                                         \
      if (g_fail <= 20) std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
    }                                                                   \
  } while (0)

#define CHECKF(cond, ...)                                               \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_fail++;                                                         \
      if (g_fail <= 20) {                                               \
        std::printf("%s:%d: CHECK(%s) failed: ", __FILE__, __LINE__, #cond); \
        std::printf(__VA_ARGS__);                                       \
        std::printf("\n");                                              \
      }                                                                 \
    }                                                                   \
  } while (0)

static int report(const char* name) {
  std::printf("%-12s %6d checks, %d failed\n", name, g_checks, g_fail);
  return g_fail ? 1 : 0;
}

#endif
