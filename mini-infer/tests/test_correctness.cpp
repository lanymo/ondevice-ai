// NumPy 레퍼런스(tests/data/*.bin)와 나이브 커널 출력을 비교한다.
// 사용법: test_correctness <data_dir>   (실패 시 종료 코드 1)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "naive/conv2d.hpp"
#include "naive/matmul.hpp"

namespace fs = std::filesystem;

namespace {

constexpr float kTol = 1e-3f;  // |diff| <= kTol + kTol * |expected|

std::vector<int32_t> read_i32(std::ifstream& f, size_t n) {
    std::vector<int32_t> v(n);
    f.read(reinterpret_cast<char*>(v.data()), n * sizeof(int32_t));
    return v;
}

std::vector<float> read_f32(std::ifstream& f, size_t n) {
    std::vector<float> v(n);
    f.read(reinterpret_cast<char*>(v.data()), n * sizeof(float));
    return v;
}

// 반환: 최대 절대 오차. 허용치 초과 원소가 있으면 fail을 true로.
float compare(const std::vector<float>& got, const std::vector<float>& exp,
              bool& fail) {
    float max_diff = 0.0f;
    for (size_t i = 0; i < exp.size(); ++i) {
        const float diff = std::fabs(got[i] - exp[i]);
        max_diff = std::max(max_diff, diff);
        if (diff > kTol + kTol * std::fabs(exp[i])) fail = true;
    }
    return max_diff;
}

bool run_matmul_case(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    const auto dims = read_i32(f, 3);
    const int m = dims[0], k = dims[1], n = dims[2];
    const auto a = read_f32(f, static_cast<size_t>(m) * k);
    const auto b = read_f32(f, static_cast<size_t>(k) * n);
    const auto expected = read_f32(f, static_cast<size_t>(m) * n);

    std::vector<float> c(expected.size());
    mini_infer::naive::matmul(a.data(), b.data(), c.data(), m, k, n);

    bool fail = false;
    const float max_diff = compare(c, expected, fail);
    std::printf("%-24s M=%-4d K=%-4d N=%-4d max_diff=%.2e  %s\n",
                path.filename().c_str(), m, k, n, max_diff,
                fail ? "FAIL" : "PASS");
    return !fail;
}

bool run_conv2d_case(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    const auto d = read_i32(f, 8);
    const mini_infer::naive::Conv2DShape s{d[0], d[1], d[2], d[3],
                                           d[4], d[5], d[6], d[7]};
    const auto in = read_f32(f, static_cast<size_t>(s.cin) * s.h * s.w);
    const auto wgt =
        read_f32(f, static_cast<size_t>(s.cout) * s.cin * s.kh * s.kw);
    const auto bias = read_f32(f, static_cast<size_t>(s.cout));
    const auto expected =
        read_f32(f, static_cast<size_t>(s.cout) * s.out_h() * s.out_w());

    std::vector<float> out(expected.size());
    mini_infer::naive::conv2d(in.data(), wgt.data(), bias.data(), out.data(), s);

    bool fail = false;
    const float max_diff = compare(out, expected, fail);
    std::printf("%-24s cin=%-3d %dx%-3d cout=%-3d k=%dx%d s=%d p=%d "
                "max_diff=%.2e  %s\n",
                path.filename().c_str(), s.cin, s.h, s.w, s.cout, s.kh, s.kw,
                s.stride, s.pad, max_diff, fail ? "FAIL" : "PASS");
    return !fail;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <data_dir>\n", argv[0]);
        return 2;
    }
    const fs::path dir(argv[1]);
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr,
                     "no data dir: %s\n(run tests/gen_reference.py first)\n",
                     dir.c_str());
        return 2;
    }

    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() == ".bin") files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());

    int passed = 0, failed = 0, skipped = 0;
    for (const auto& p : files) {
        const std::string name = p.filename().string();
        bool ok;
        if (name.rfind("matmul_", 0) == 0)      ok = run_matmul_case(p);
        else if (name.rfind("conv2d_", 0) == 0) ok = run_conv2d_case(p);
        else { ++skipped; continue; }
        ok ? ++passed : ++failed;
    }

    std::printf("\n%d passed, %d failed, %d skipped\n", passed, failed, skipped);
    if (passed == 0 && failed == 0) {
        std::fprintf(stderr, "no test cases found in %s\n", dir.c_str());
        return 2;
    }
    return failed == 0 ? 0 : 1;
}
