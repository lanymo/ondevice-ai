#pragma once

namespace mini_infer::naive {

// C[M,N] = A[M,K] * B[K,N]. 모두 row-major, C는 덮어쓴다.
void matmul(const float* a, const float* b, float* c, int m, int k, int n);

}  // namespace mini_infer::naive
