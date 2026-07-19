// 나이브 커널 베이스라인 벤치마크.
// 사용법: bench_naive [출력.csv]  (기본: results/naive_baseline.csv 위치는 실행 디렉토리 기준)

#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "bench.hpp"
#include "naive/conv2d.hpp"
#include "naive/matmul.hpp"

#ifndef MI_BUILD_FLAGS
#define MI_BUILD_FLAGS "unknown"
#endif

namespace {

constexpr int kWarmup = 10;
constexpr int kIters = 100;  // 측정 규칙: >=100회

std::vector<float> random_vec(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::vector<float> v(n);
    for (auto& x : v) x = dist(rng);
    return v;
}

mini_infer::bench::Result bench_matmul(int m, int k, int n) {
    const auto a = random_vec(static_cast<size_t>(m) * k, 1);
    const auto b = random_vec(static_cast<size_t>(k) * n, 2);
    std::vector<float> c(static_cast<size_t>(m) * n);

    char params[64];
    std::snprintf(params, sizeof(params), "M=%d K=%d N=%d", m, k, n);
    const double flops = 2.0 * m * k * n;

    return mini_infer::bench::run("matmul_naive", params, flops, kWarmup, kIters,
                                  [&]() -> float {
                                      mini_infer::naive::matmul(a.data(), b.data(),
                                                                c.data(), m, k, n);
                                      return c[0] + c[c.size() - 1];
                                  });
}

mini_infer::bench::Result bench_conv2d(mini_infer::naive::Conv2DShape s) {
    const auto in = random_vec(static_cast<size_t>(s.cin) * s.h * s.w, 3);
    const auto wgt =
        random_vec(static_cast<size_t>(s.cout) * s.cin * s.kh * s.kw, 4);
    const auto bias = random_vec(static_cast<size_t>(s.cout), 5);
    std::vector<float> out(static_cast<size_t>(s.cout) * s.out_h() * s.out_w());

    char params[96];
    std::snprintf(params, sizeof(params),
                  "Cin=%d H=%d W=%d Cout=%d K=%dx%d s=%d p=%d", s.cin, s.h, s.w,
                  s.cout, s.kh, s.kw, s.stride, s.pad);

    return mini_infer::bench::run("conv2d_naive", params, s.flops(), kWarmup,
                                  kIters, [&]() -> float {
                                      mini_infer::naive::conv2d(
                                          in.data(), wgt.data(), bias.data(),
                                          out.data(), s);
                                      return out[0] + out[out.size() - 1];
                                  });
}

}  // namespace

int main(int argc, char** argv) {
    const char* csv_path = argc > 1 ? argv[1] : "naive_baseline.csv";

    std::vector<mini_infer::bench::Result> results;
    results.push_back(bench_matmul(64, 64, 64));
    results.push_back(bench_matmul(128, 128, 128));
    results.push_back(bench_matmul(256, 256, 256));
    results.push_back(bench_matmul(1, 256, 256));  // 추론에서 흔한 matvec 형태

    results.push_back(bench_conv2d({3, 64, 64, 16, 3, 3, 1, 1}));
    results.push_back(bench_conv2d({16, 32, 32, 32, 3, 3, 1, 1}));
    results.push_back(bench_conv2d({32, 16, 16, 64, 3, 3, 2, 1}));

    std::printf("build flags: %s | warmup=%d iters=%d\n\n", MI_BUILD_FLAGS,
                kWarmup, kIters);
    std::printf("%-14s %-36s %10s %10s %10s %8s\n", "kernel", "params",
                "median(ms)", "p99(ms)", "min(ms)", "GFLOP/s");
    for (const auto& r : results) {
        std::printf("%-14s %-36s %10.3f %10.3f %10.3f %8.2f\n", r.name.c_str(),
                    r.params.c_str(), r.median_ns / 1e6, r.p99_ns / 1e6,
                    r.min_ns / 1e6, r.gflops_median());
    }

    FILE* f = std::fopen(csv_path, "w");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", csv_path);
        return 1;
    }
    std::fprintf(f,
                 "kernel,params,build_flags,warmup,iters,min_ms,mean_ms,"
                 "median_ms,p99_ms,gflops_median\n");
    for (const auto& r : results) {
        std::fprintf(f, "%s,\"%s\",\"%s\",%d,%d,%.6f,%.6f,%.6f,%.6f,%.3f\n",
                     r.name.c_str(), r.params.c_str(), MI_BUILD_FLAGS, r.warmup,
                     r.iters, r.min_ns / 1e6, r.mean_ns / 1e6, r.median_ns / 1e6,
                     r.p99_ns / 1e6, r.gflops_median());
    }
    std::fclose(f);
    std::printf("\ncsv written: %s\n", csv_path);
    return 0;
}
