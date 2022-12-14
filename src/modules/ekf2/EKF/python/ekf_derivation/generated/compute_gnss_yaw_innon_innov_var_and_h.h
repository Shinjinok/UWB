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
 * Symbolic function: compute_gnss_yaw_innon_innov_var_and_h
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix24_24
 *     antenna_yaw_offset: Scalar
 *     meas: Scalar
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov: Scalar
 *     innov_var: Scalar
 *     H: Matrix24_1
 */
template <typename Scalar>
void ComputeGnssYawInnonInnovVarAndH(const matrix::Matrix<Scalar, 24, 1>& state,
                                     const matrix::Matrix<Scalar, 24, 24>& P,
                                     const Scalar antenna_yaw_offset, const Scalar meas,
                                     const Scalar R, const Scalar epsilon,
                                     Scalar* const innov = nullptr,
                                     Scalar* const innov_var = nullptr,
                                     matrix::Matrix<Scalar, 24, 1>* const H = nullptr) {
  // Total ops: 106

  // Input arrays

  // Intermediate terms (28)
  const Scalar _tmp0 = std::pow(state(2, 0), Scalar(2));
  const Scalar _tmp1 = std::pow(state(1, 0), Scalar(2));
  const Scalar _tmp2 = std::pow(state(0, 0), Scalar(2)) - std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp3 = std::sin(antenna_yaw_offset);
  const Scalar _tmp4 = state(0, 0) * state(3, 0);
  const Scalar _tmp5 = state(1, 0) * state(2, 0);
  const Scalar _tmp6 = std::cos(antenna_yaw_offset);
  const Scalar _tmp7 = _tmp3 * (_tmp0 - _tmp1 + _tmp2) + 2 * _tmp6 * (_tmp4 + _tmp5);
  const Scalar _tmp8 = 2 * _tmp3 * (-_tmp4 + _tmp5) + _tmp6 * (-_tmp0 + _tmp1 + _tmp2);
  const Scalar _tmp9 = _tmp8 + epsilon * ((((_tmp8) > 0) - ((_tmp8) < 0)) + Scalar(0.5));
  const Scalar _tmp10 = 2 * state(3, 0);
  const Scalar _tmp11 = 2 * state(0, 0);
  const Scalar _tmp12 = -_tmp10 * _tmp3 + _tmp11 * _tmp6;
  const Scalar _tmp13 = Scalar(1.0) / (_tmp9);
  const Scalar _tmp14 = _tmp10 * _tmp6;
  const Scalar _tmp15 = _tmp11 * _tmp3;
  const Scalar _tmp16 = std::pow(_tmp9, Scalar(2));
  const Scalar _tmp17 = _tmp7 / _tmp16;
  const Scalar _tmp18 = _tmp16 / (_tmp16 + std::pow(_tmp7, Scalar(2)));
  const Scalar _tmp19 = _tmp18 * (_tmp12 * _tmp13 - _tmp17 * (-_tmp14 - _tmp15));
  const Scalar _tmp20 = 2 * state(1, 0);
  const Scalar _tmp21 = 2 * state(2, 0);
  const Scalar _tmp22 = _tmp20 * _tmp6 + _tmp21 * _tmp3;
  const Scalar _tmp23 = _tmp20 * _tmp3;
  const Scalar _tmp24 = _tmp21 * _tmp6;
  const Scalar _tmp25 = _tmp18 * (_tmp13 * (-_tmp23 + _tmp24) - _tmp17 * _tmp22);
  const Scalar _tmp26 = _tmp18 * (-_tmp12 * _tmp17 + _tmp13 * (_tmp14 + _tmp15));
  const Scalar _tmp27 = _tmp18 * (_tmp13 * _tmp22 - _tmp17 * (_tmp23 - _tmp24));

  // Output terms (3)
  if (innov != nullptr) {
    Scalar& _innov = (*innov);

    _innov = -meas + std::atan2(_tmp7, _tmp9);
  }

  if (innov_var != nullptr) {
    Scalar& _innov_var = (*innov_var);

    _innov_var =
        R + _tmp19 * (P(0, 3) * _tmp26 + P(1, 3) * _tmp25 + P(2, 3) * _tmp27 + P(3, 3) * _tmp19) +
        _tmp25 * (P(0, 1) * _tmp26 + P(1, 1) * _tmp25 + P(2, 1) * _tmp27 + P(3, 1) * _tmp19) +
        _tmp26 * (P(0, 0) * _tmp26 + P(1, 0) * _tmp25 + P(2, 0) * _tmp27 + P(3, 0) * _tmp19) +
        _tmp27 * (P(0, 2) * _tmp26 + P(1, 2) * _tmp25 + P(2, 2) * _tmp27 + P(3, 2) * _tmp19);
  }

  if (H != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _H = (*H);

    _H.setZero();

    _H(0, 0) = _tmp26;
    _H(1, 0) = _tmp25;
    _H(2, 0) = _tmp27;
    _H(3, 0) = _tmp19;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
