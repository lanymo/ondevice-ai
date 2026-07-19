#include "naive/matmul.hpp"

namespace mini_infer::naive {

void matmul(const float* a, const float* b, float* c, int m, int k, int n) {
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            float acc = 0.0f;
            for (int p = 0; p < k; ++p) {
                acc += a[i * k + p] * b[p * n + j];
            }
            c[i * n + j] = acc;
        }
    }
}

}  // namespace mini_infer::naive
