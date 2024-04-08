// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_gravity_xyz_innov_var_and_hx
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix23_23
 *     R: Scalar
 *
 * Outputs:
 *     innov_var: Matrix31
 *     Hx: Matrix23_1
 */
template <typename Scalar>
void ComputeGravityXyzInnovVarAndHx(const matrix::Matrix<Scalar, 24, 1>& state,
                                    const matrix::Matrix<Scalar, 23, 23>& P, const Scalar R,
                                    matrix::Matrix<Scalar, 3, 1>* const innov_var = nullptr,
                                    matrix::Matrix<Scalar, 23, 1>* const Hx = nullptr) {
  // Total ops: 53

  // Input arrays

  // Intermediate terms (13)
  const Scalar _tmp0 = 2 * state(0, 0);
  const Scalar _tmp1 = -_tmp0 * state(3, 0);
  const Scalar _tmp2 = 2 * state(2, 0);
  const Scalar _tmp3 = _tmp2 * state(1, 0);
  const Scalar _tmp4 = _tmp1 - _tmp3;
  const Scalar _tmp5 = std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp6 = std::pow(state(0, 0), Scalar(2));
  const Scalar _tmp7 = std::pow(state(1, 0), Scalar(2)) - std::pow(state(2, 0), Scalar(2));
  const Scalar _tmp8 = -_tmp5 + _tmp6 + _tmp7;
  const Scalar _tmp9 = _tmp1 + _tmp3;
  const Scalar _tmp10 = _tmp5 - _tmp6 + _tmp7;
  const Scalar _tmp11 = _tmp0 * state(1, 0) - _tmp2 * state(3, 0);
  const Scalar _tmp12 = _tmp2 * state(0, 0) + 2 * state(1, 0) * state(3, 0);

  // Output terms (2)
  if (innov_var != nullptr) {
    matrix::Matrix<Scalar, 3, 1>& _innov_var = (*innov_var);

    _innov_var(0, 0) = R + _tmp4 * (P(0, 0) * _tmp4 + P(1, 0) * _tmp8) +
                       _tmp8 * (P(0, 1) * _tmp4 + P(1, 1) * _tmp8);
    _innov_var(1, 0) = R + _tmp10 * (P(0, 0) * _tmp10 + P(1, 0) * _tmp9) +
                       _tmp9 * (P(0, 1) * _tmp10 + P(1, 1) * _tmp9);
    _innov_var(2, 0) = R + _tmp11 * (P(0, 0) * _tmp11 + P(1, 0) * _tmp12) +
                       _tmp12 * (P(0, 1) * _tmp11 + P(1, 1) * _tmp12);
  }

  if (Hx != nullptr) {
    matrix::Matrix<Scalar, 23, 1>& _hx = (*Hx);

    _hx.setZero();

    _hx(0, 0) = _tmp4;
    _hx(1, 0) = _tmp8;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
