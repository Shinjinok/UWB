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
 * Symbolic function: compute_mag_innov_innov_var_and_hx
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix24_24
 *     meas: Matrix31
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov: Matrix31
 *     innov_var: Matrix31
 *     Hx: Matrix24_1
 */
template <typename Scalar>
void ComputeMagInnovInnovVarAndHx(const matrix::Matrix<Scalar, 24, 1>& state,
                                  const matrix::Matrix<Scalar, 24, 24>& P,
                                  const matrix::Matrix<Scalar, 3, 1>& meas, const Scalar R,
                                  const Scalar epsilon,
                                  matrix::Matrix<Scalar, 3, 1>* const innov = nullptr,
                                  matrix::Matrix<Scalar, 3, 1>* const innov_var = nullptr,
                                  matrix::Matrix<Scalar, 24, 1>* const Hx = nullptr) {
  // Total ops: 470

  // Input arrays

  // Intermediate terms (48)
  const Scalar _tmp0 = std::pow(state(2, 0), Scalar(2));
  const Scalar _tmp1 = -_tmp0;
  const Scalar _tmp2 = std::pow(state(1, 0), Scalar(2));
  const Scalar _tmp3 = std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp4 = std::pow(state(0, 0), Scalar(2));
  const Scalar _tmp5 = -_tmp3 + _tmp4;
  const Scalar _tmp6 = _tmp1 + _tmp2 + _tmp5;
  const Scalar _tmp7 = state(0, 0) * state(3, 0);
  const Scalar _tmp8 = state(1, 0) * state(2, 0);
  const Scalar _tmp9 = 2 * state(17, 0);
  const Scalar _tmp10 = state(0, 0) * state(2, 0);
  const Scalar _tmp11 = state(1, 0) * state(3, 0);
  const Scalar _tmp12 = 2 * state(18, 0);
  const Scalar _tmp13 = -_tmp2;
  const Scalar _tmp14 = _tmp0 + _tmp13 + _tmp5;
  const Scalar _tmp15 = state(2, 0) * state(3, 0);
  const Scalar _tmp16 = state(0, 0) * state(1, 0);
  const Scalar _tmp17 = 2 * state(16, 0);
  const Scalar _tmp18 = _tmp1 + _tmp13 + _tmp3 + _tmp4;
  const Scalar _tmp19 = _tmp9 * state(3, 0);
  const Scalar _tmp20 = _tmp12 * state(2, 0);
  const Scalar _tmp21 = 2 * state(0, 0);
  const Scalar _tmp22 = _tmp21 * state(16, 0);
  const Scalar _tmp23 = _tmp19 - _tmp20 + _tmp22;
  const Scalar _tmp24 = _tmp12 * state(3, 0) + _tmp17 * state(1, 0) + _tmp9 * state(2, 0);
  const Scalar _tmp25 = _tmp17 * state(3, 0);
  const Scalar _tmp26 = _tmp12 * state(1, 0);
  const Scalar _tmp27 = _tmp21 * state(17, 0);
  const Scalar _tmp28 = -_tmp25 + _tmp26 + _tmp27;
  const Scalar _tmp29 = _tmp17 * state(2, 0);
  const Scalar _tmp30 = _tmp9 * state(1, 0);
  const Scalar _tmp31 = _tmp12 * state(0, 0);
  const Scalar _tmp32 = -_tmp29 + _tmp30 - _tmp31;
  const Scalar _tmp33 = 2 * _tmp7;
  const Scalar _tmp34 = 2 * _tmp8;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = 2 * _tmp10;
  const Scalar _tmp37 = 2 * _tmp11;
  const Scalar _tmp38 = -_tmp36 + _tmp37;
  const Scalar _tmp39 = _tmp29 - _tmp30 + _tmp31;
  const Scalar _tmp40 = -_tmp19 + _tmp20 - _tmp22;
  const Scalar _tmp41 = -_tmp33 + _tmp34;
  const Scalar _tmp42 = 2 * _tmp15;
  const Scalar _tmp43 = 2 * _tmp16;
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = _tmp25 - _tmp26 - _tmp27;
  const Scalar _tmp46 = _tmp36 + _tmp37;
  const Scalar _tmp47 = _tmp42 - _tmp43;

  // Output terms (3)
  if (innov != nullptr) {
    matrix::Matrix<Scalar, 3, 1>& _innov = (*innov);

    _innov(0, 0) = _tmp12 * (-_tmp10 + _tmp11) + _tmp6 * state(16, 0) + _tmp9 * (_tmp7 + _tmp8) -
                   meas(0, 0) + state(19, 0);
    _innov(1, 0) = _tmp12 * (_tmp15 + _tmp16) + _tmp14 * state(17, 0) + _tmp17 * (-_tmp7 + _tmp8) -
                   meas(1, 0) + state(20, 0);
    _innov(2, 0) = _tmp17 * (_tmp10 + _tmp11) + _tmp18 * state(18, 0) + _tmp9 * (_tmp15 - _tmp16) -
                   meas(2, 0) + state(21, 0);
  }

  if (innov_var != nullptr) {
    matrix::Matrix<Scalar, 3, 1>& _innov_var = (*innov_var);

    _innov_var(0, 0) =
        P(0, 19) * _tmp23 + P(1, 19) * _tmp24 + P(16, 19) * _tmp6 + P(17, 19) * _tmp35 +
        P(18, 19) * _tmp38 + P(19, 19) + P(2, 19) * _tmp32 + P(3, 19) * _tmp28 + R +
        _tmp23 * (P(0, 0) * _tmp23 + P(1, 0) * _tmp24 + P(16, 0) * _tmp6 + P(17, 0) * _tmp35 +
                  P(18, 0) * _tmp38 + P(19, 0) + P(2, 0) * _tmp32 + P(3, 0) * _tmp28) +
        _tmp24 * (P(0, 1) * _tmp23 + P(1, 1) * _tmp24 + P(16, 1) * _tmp6 + P(17, 1) * _tmp35 +
                  P(18, 1) * _tmp38 + P(19, 1) + P(2, 1) * _tmp32 + P(3, 1) * _tmp28) +
        _tmp28 * (P(0, 3) * _tmp23 + P(1, 3) * _tmp24 + P(16, 3) * _tmp6 + P(17, 3) * _tmp35 +
                  P(18, 3) * _tmp38 + P(19, 3) + P(2, 3) * _tmp32 + P(3, 3) * _tmp28) +
        _tmp32 * (P(0, 2) * _tmp23 + P(1, 2) * _tmp24 + P(16, 2) * _tmp6 + P(17, 2) * _tmp35 +
                  P(18, 2) * _tmp38 + P(19, 2) + P(2, 2) * _tmp32 + P(3, 2) * _tmp28) +
        _tmp35 * (P(0, 17) * _tmp23 + P(1, 17) * _tmp24 + P(16, 17) * _tmp6 + P(17, 17) * _tmp35 +
                  P(18, 17) * _tmp38 + P(19, 17) + P(2, 17) * _tmp32 + P(3, 17) * _tmp28) +
        _tmp38 * (P(0, 18) * _tmp23 + P(1, 18) * _tmp24 + P(16, 18) * _tmp6 + P(17, 18) * _tmp35 +
                  P(18, 18) * _tmp38 + P(19, 18) + P(2, 18) * _tmp32 + P(3, 18) * _tmp28) +
        _tmp6 * (P(0, 16) * _tmp23 + P(1, 16) * _tmp24 + P(16, 16) * _tmp6 + P(17, 16) * _tmp35 +
                 P(18, 16) * _tmp38 + P(19, 16) + P(2, 16) * _tmp32 + P(3, 16) * _tmp28);
    _innov_var(1, 0) =
        P(0, 20) * _tmp28 + P(1, 20) * _tmp39 + P(16, 20) * _tmp41 + P(17, 20) * _tmp14 +
        P(18, 20) * _tmp44 + P(2, 20) * _tmp24 + P(20, 20) + P(3, 20) * _tmp40 + R +
        _tmp14 * (P(0, 17) * _tmp28 + P(1, 17) * _tmp39 + P(16, 17) * _tmp41 + P(17, 17) * _tmp14 +
                  P(18, 17) * _tmp44 + P(2, 17) * _tmp24 + P(20, 17) + P(3, 17) * _tmp40) +
        _tmp24 * (P(0, 2) * _tmp28 + P(1, 2) * _tmp39 + P(16, 2) * _tmp41 + P(17, 2) * _tmp14 +
                  P(18, 2) * _tmp44 + P(2, 2) * _tmp24 + P(20, 2) + P(3, 2) * _tmp40) +
        _tmp28 * (P(0, 0) * _tmp28 + P(1, 0) * _tmp39 + P(16, 0) * _tmp41 + P(17, 0) * _tmp14 +
                  P(18, 0) * _tmp44 + P(2, 0) * _tmp24 + P(20, 0) + P(3, 0) * _tmp40) +
        _tmp39 * (P(0, 1) * _tmp28 + P(1, 1) * _tmp39 + P(16, 1) * _tmp41 + P(17, 1) * _tmp14 +
                  P(18, 1) * _tmp44 + P(2, 1) * _tmp24 + P(20, 1) + P(3, 1) * _tmp40) +
        _tmp40 * (P(0, 3) * _tmp28 + P(1, 3) * _tmp39 + P(16, 3) * _tmp41 + P(17, 3) * _tmp14 +
                  P(18, 3) * _tmp44 + P(2, 3) * _tmp24 + P(20, 3) + P(3, 3) * _tmp40) +
        _tmp41 * (P(0, 16) * _tmp28 + P(1, 16) * _tmp39 + P(16, 16) * _tmp41 + P(17, 16) * _tmp14 +
                  P(18, 16) * _tmp44 + P(2, 16) * _tmp24 + P(20, 16) + P(3, 16) * _tmp40) +
        _tmp44 * (P(0, 18) * _tmp28 + P(1, 18) * _tmp39 + P(16, 18) * _tmp41 + P(17, 18) * _tmp14 +
                  P(18, 18) * _tmp44 + P(2, 18) * _tmp24 + P(20, 18) + P(3, 18) * _tmp40);
    _innov_var(2, 0) =
        P(0, 21) * _tmp39 + P(1, 21) * _tmp45 + P(16, 21) * _tmp46 + P(17, 21) * _tmp47 +
        P(18, 21) * _tmp18 + P(2, 21) * _tmp23 + P(21, 21) + P(3, 21) * _tmp24 + R +
        _tmp18 * (P(0, 18) * _tmp39 + P(1, 18) * _tmp45 + P(16, 18) * _tmp46 + P(17, 18) * _tmp47 +
                  P(18, 18) * _tmp18 + P(2, 18) * _tmp23 + P(21, 18) + P(3, 18) * _tmp24) +
        _tmp23 * (P(0, 2) * _tmp39 + P(1, 2) * _tmp45 + P(16, 2) * _tmp46 + P(17, 2) * _tmp47 +
                  P(18, 2) * _tmp18 + P(2, 2) * _tmp23 + P(21, 2) + P(3, 2) * _tmp24) +
        _tmp24 * (P(0, 3) * _tmp39 + P(1, 3) * _tmp45 + P(16, 3) * _tmp46 + P(17, 3) * _tmp47 +
                  P(18, 3) * _tmp18 + P(2, 3) * _tmp23 + P(21, 3) + P(3, 3) * _tmp24) +
        _tmp39 * (P(0, 0) * _tmp39 + P(1, 0) * _tmp45 + P(16, 0) * _tmp46 + P(17, 0) * _tmp47 +
                  P(18, 0) * _tmp18 + P(2, 0) * _tmp23 + P(21, 0) + P(3, 0) * _tmp24) +
        _tmp45 * (P(0, 1) * _tmp39 + P(1, 1) * _tmp45 + P(16, 1) * _tmp46 + P(17, 1) * _tmp47 +
                  P(18, 1) * _tmp18 + P(2, 1) * _tmp23 + P(21, 1) + P(3, 1) * _tmp24) +
        _tmp46 * (P(0, 16) * _tmp39 + P(1, 16) * _tmp45 + P(16, 16) * _tmp46 + P(17, 16) * _tmp47 +
                  P(18, 16) * _tmp18 + P(2, 16) * _tmp23 + P(21, 16) + P(3, 16) * _tmp24) +
        _tmp47 * (P(0, 17) * _tmp39 + P(1, 17) * _tmp45 + P(16, 17) * _tmp46 + P(17, 17) * _tmp47 +
                  P(18, 17) * _tmp18 + P(2, 17) * _tmp23 + P(21, 17) + P(3, 17) * _tmp24);
  }

  if (Hx != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _hx = (*Hx);

    _hx.setZero();

    _hx(0, 0) = _tmp23;
    _hx(1, 0) = _tmp24;
    _hx(2, 0) = _tmp32;
    _hx(3, 0) = _tmp28;
    _hx(16, 0) = _tmp6;
    _hx(17, 0) = _tmp35;
    _hx(18, 0) = _tmp38;
    _hx(19, 0) = 1;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
