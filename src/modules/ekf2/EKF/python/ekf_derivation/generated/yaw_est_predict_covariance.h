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
 * Symbolic function: yaw_est_predict_covariance
 *
 * Args:
 *     state: Matrix31
 *     P: Matrix33
 *     d_vel: Matrix21
 *     d_vel_var: Scalar
 *     d_ang_var: Scalar
 *
 * Outputs:
 *     P_new: Matrix33
 */
template <typename Scalar>
void YawEstPredictCovariance(const matrix::Matrix<Scalar, 3, 1>& state,
                             const matrix::Matrix<Scalar, 3, 3>& P,
                             const matrix::Matrix<Scalar, 2, 1>& d_vel, const Scalar d_vel_var,
                             const Scalar d_ang_var,
                             matrix::Matrix<Scalar, 3, 3>* const P_new = nullptr) {
  // Total ops: 33

  // Input arrays

  // Intermediate terms (7)
  const Scalar _tmp0 = std::cos(state(2, 0));
  const Scalar _tmp1 = std::sin(state(2, 0));
  const Scalar _tmp2 = -_tmp0 * d_vel(1, 0) - _tmp1 * d_vel(0, 0);
  const Scalar _tmp3 = P(0, 2) + P(2, 2) * _tmp2;
  const Scalar _tmp4 =
      std::pow(_tmp0, Scalar(2)) * d_vel_var + std::pow(_tmp1, Scalar(2)) * d_vel_var;
  const Scalar _tmp5 = _tmp0 * d_vel(0, 0) - _tmp1 * d_vel(1, 0);
  const Scalar _tmp6 = P(1, 2) + P(2, 2) * _tmp5;

  // Output terms (1)
  if (P_new != nullptr) {
    matrix::Matrix<Scalar, 3, 3>& _p_new = (*P_new);

    _p_new(0, 0) = P(0, 0) + P(2, 0) * _tmp2 + _tmp2 * _tmp3 + _tmp4;
    _p_new(1, 0) = 0;
    _p_new(2, 0) = 0;
    _p_new(0, 1) = P(0, 1) + P(2, 1) * _tmp2 + _tmp3 * _tmp5;
    _p_new(1, 1) = P(1, 1) + P(2, 1) * _tmp5 + _tmp4 + _tmp5 * _tmp6;
    _p_new(2, 1) = 0;
    _p_new(0, 2) = _tmp3;
    _p_new(1, 2) = _tmp6;
    _p_new(2, 2) = P(2, 2) + d_ang_var;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
