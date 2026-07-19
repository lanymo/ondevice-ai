#include "naive/conv2d.hpp"

namespace mini_infer::naive {

void conv2d(const float* in, const float* wgt, const float* bias, float* out,
            const Conv2DShape& s) {
    const int oh = s.out_h();
    const int ow = s.out_w();

    for (int oc = 0; oc < s.cout; ++oc) {
        for (int oy = 0; oy < oh; ++oy) {
            for (int ox = 0; ox < ow; ++ox) {
                float acc = bias ? bias[oc] : 0.0f;
                for (int ic = 0; ic < s.cin; ++ic) {
                    for (int ky = 0; ky < s.kh; ++ky) {
                        const int iy = oy * s.stride + ky - s.pad;
                        if (iy < 0 || iy >= s.h) continue;
                        for (int kx = 0; kx < s.kw; ++kx) {
                            const int ix = ox * s.stride + kx - s.pad;
                            if (ix < 0 || ix >= s.w) continue;
                            acc += in[(ic * s.h + iy) * s.w + ix] *
                                   wgt[((oc * s.cin + ic) * s.kh + ky) * s.kw + kx];
                        }
                    }
                }
                out[(oc * oh + oy) * ow + ox] = acc;
            }
        }
    }
}

}  // namespace mini_infer::naive
