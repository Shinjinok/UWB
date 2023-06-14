// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     backends/cpp/templates/function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_mag_y_innov_var_and_h
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix24_24
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov_var: Scalar
 *     H: Matrix24_1
 */
template <typename Scalar>
void ComputeMagYInnovVarAndH(const matrix::Matrix<Scalar, 24, 1>& state,
                             const matrix::Matrix<Scalar, 24, 24>& P, const Scalar R,
                             const Scalar epsilon, Scalar* const innov_var = nullptr,
                             matrix::Matrix<Scalar, 24, 1>* const H = nullptr) {
  // Total ops: 160

  // Input arrays

  // Intermediate terms (12)
  const Scalar _tmp0 = 2 * state(16, 0);
  const Scalar _tmp1 = 4 * state(17, 0);
  const Scalar _tmp2 = 2 * state(18, 0);
  const Scalar _tmp3 = _tmp0 * state(2, 0) - _tmp1 * state(1, 0) + _tmp2 * state(0, 0);
  const Scalar _tmp4 = -_tmp0 * state(0, 0) - _tmp1 * state(3, 0) + _tmp2 * state(2, 0);
  const Scalar _tmp5 = -_tmp0 * state(3, 0) + _tmp2 * state(1, 0);
  const Scalar _tmp6 = _tmp0 * state(1, 0) + _tmp2 * state(3, 0);
  const Scalar _tmp7 =
      -2 * std::pow(state(1, 0), Scalar(2)) - 2 * std::pow(state(3, 0), Scalar(2)) + 1;
  const Scalar _tmp8 = 2 * state(3, 0);
  const Scalar _tmp9 = 2 * state(1, 0);
  const Scalar _tmp10 = _tmp8 * state(2, 0) + _tmp9 * state(0, 0);
  const Scalar _tmp11 = -_tmp8 * state(0, 0) + _tmp9 * state(2, 0);

  // Output terms (2)
  if (innov_var != nullptr) {
    Scalar& _innov_var = (*innov_var);

    _innov_var =
        P(0, 20) * _tmp5 + P(1, 20) * _tmp3 + P(16, 20) * _tmp11 + P(17, 20) * _tmp7 +
        P(18, 20) * _tmp10 + P(2, 20) * _tmp6 + P(20, 20) + P(3, 20) * _tmp4 + R +
        _tmp10 * (P(0, 18) * _tmp5 + P(1, 18) * _tmp3 + P(16, 18) * _tmp11 + P(17, 18) * _tmp7 +
                  P(18, 18) * _tmp10 + P(2, 18) * _tmp6 + P(20, 18) + P(3, 18) * _tmp4) +
        _tmp11 * (P(0, 16) * _tmp5 + P(1, 16) * _tmp3 + P(16, 16) * _tmp11 + P(17, 16) * _tmp7 +
                  P(18, 16) * _tmp10 + P(2, 16) * _tmp6 + P(20, 16) + P(3, 16) * _tmp4) +
        _tmp3 * (P(0, 1) * _tmp5 + P(1, 1) * _tmp3 + P(16, 1) * _tmp11 + P(17, 1) * _tmp7 +
                 P(18, 1) * _tmp10 + P(2, 1) * _tmp6 + P(20, 1) + P(3, 1) * _tmp4) +
        _tmp4 * (P(0, 3) * _tmp5 + P(1, 3) * _tmp3 + P(16, 3) * _tmp11 + P(17, 3) * _tmp7 +
                 P(18, 3) * _tmp10 + P(2, 3) * _tmp6 + P(20, 3) + P(3, 3) * _tmp4) +
        _tmp5 * (P(0, 0) * _tmp5 + P(1, 0) * _tmp3 + P(16, 0) * _tmp11 + P(17, 0) * _tmp7 +
                 P(18, 0) * _tmp10 + P(2, 0) * _tmp6 + P(20, 0) + P(3, 0) * _tmp4) +
        _tmp6 * (P(0, 2) * _tmp5 + P(1, 2) * _tmp3 + P(16, 2) * _tmp11 + P(17, 2) * _tmp7 +
                 P(18, 2) * _tmp10 + P(2, 2) * _tmp6 + P(20, 2) + P(3, 2) * _tmp4) +
        _tmp7 * (P(0, 17) * _tmp5 + P(1, 17) * _tmp3 + P(16, 17) * _tmp11 + P(17, 17) * _tmp7 +
                 P(18, 17) * _tmp10 + P(2, 17) * _tmp6 + P(20, 17) + P(3, 17) * _tmp4);
  }

  if (H != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _h = (*H);

    _h.setZero();

    _h(0, 0) = _tmp5;
    _h(1, 0) = _tmp3;
    _h(2, 0) = _tmp6;
    _h(3, 0) = _tmp4;
    _h(16, 0) = _tmp11;
    _h(17, 0) = _tmp7;
    _h(18, 0) = _tmp10;
    _h(20, 0) = 1;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
