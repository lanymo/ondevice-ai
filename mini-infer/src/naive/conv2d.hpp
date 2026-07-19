#pragma once

namespace mini_infer::naive {

// 단일 배치 Conv2D. 레이아웃:
//   input  [cin, h, w]           (CHW)
//   weight [cout, cin, kh, kw]
//   bias   [cout]                (nullptr 허용 = bias 없음)
//   output [cout, out_h, out_w]
struct Conv2DShape {
    int cin, h, w;
    int cout, kh, kw;
    int stride, pad;

    int out_h() const { return (h + 2 * pad - kh) / stride + 1; }
    int out_w() const { return (w + 2 * pad - kw) / stride + 1; }
    // 곱셈+덧셈을 각각 1 FLOP으로 센 총 연산량
    double flops() const {
        return 2.0 * cout * out_h() * out_w() * cin * kh * kw;
    }
};

void conv2d(const float* in, const float* wgt, const float* bias, float* out,
            const Conv2DShape& s);

}  // namespace mini_infer::naive
