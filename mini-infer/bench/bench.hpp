#pragma once
// 벤치마크 하네스 (측정 규칙: 워밍업 제외, >=100회 반복, 중앙값/p99 기록)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace mini_infer::bench {

// 컴파일러가 결과 계산을 통째로 제거하지 못하게 하는 싱크.
// 벤치 본문은 결과에 의존하는 float 하나를 반환해야 한다.
inline volatile float g_sink = 0.0f;

struct Result {
    std::string name;
    std::string params;
    double flops = 0.0;  // 1회 실행의 연산량
    int warmup = 0;
    int iters = 0;
    double min_ns = 0.0;
    double mean_ns = 0.0;
    double median_ns = 0.0;
    double p99_ns = 0.0;

    double gflops_median() const { return flops / median_ns; }  // GFLOP/s
};

// body: 커널 1회 실행 후 결과에 의존하는 float(체크섬 등)를 반환하는 callable
template <class F>
Result run(const std::string& name, const std::string& params, double flops,
           int warmup, int iters, F&& body) {
    using clock = std::chrono::steady_clock;

    for (int i = 0; i < warmup; ++i) {
        g_sink = g_sink + body();
    }

    std::vector<double> ns(iters);
    for (int i = 0; i < iters; ++i) {
        const auto t0 = clock::now();
        const float r = body();
        const auto t1 = clock::now();
        g_sink = g_sink + r;
        ns[i] = std::chrono::duration<double, std::nano>(t1 - t0).count();
    }

    std::vector<double> sorted = ns;
    std::sort(sorted.begin(), sorted.end());

    Result res;
    res.name = name;
    res.params = params;
    res.flops = flops;
    res.warmup = warmup;
    res.iters = iters;
    res.min_ns = sorted.front();
    double sum = 0.0;
    for (double v : ns) sum += v;
    res.mean_ns = sum / iters;
    res.median_ns = (iters % 2 == 0)
                        ? 0.5 * (sorted[iters / 2 - 1] + sorted[iters / 2])
                        : sorted[iters / 2];
    const auto p99_idx = static_cast<size_t>(std::ceil(0.99 * iters)) - 1;
    res.p99_ns = sorted[std::min(p99_idx, sorted.size() - 1)];
    return res;
}

}  // namespace mini_infer::bench
