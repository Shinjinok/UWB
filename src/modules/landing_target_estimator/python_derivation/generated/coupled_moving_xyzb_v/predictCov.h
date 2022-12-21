// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym
{

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: predictCov
 *
 * Args:
 *     dt: Scalar
 *     input_var: Matrix33
 *     bias_var: Matrix33
 *     acc_var: Matrix33
 *     covariance: Matrix15_15
 *
 * Outputs:
 *     cov_updated: Matrix15_15
 */
template <typename Scalar>
void Predictcov(const Scalar dt, const matrix::Matrix<Scalar, 3, 3> &input_var,
		const matrix::Matrix<Scalar, 3, 3> &bias_var,
		const matrix::Matrix<Scalar, 3, 3> &acc_var,
		const matrix::Matrix<Scalar, 15, 15> &covariance,
		matrix::Matrix<Scalar, 15, 15> *const cov_updated = nullptr)
{
	// Total ops: 750

	// Input arrays

	// Intermediate terms (130)
	const Scalar _tmp0 = std::pow(dt, Scalar(2));
	const Scalar _tmp1 = Scalar(0.5) * _tmp0;
	const Scalar _tmp2 = Scalar(0.25) * std::pow(dt, Scalar(4));
	const Scalar _tmp3 = covariance(3, 12) * dt;
	const Scalar _tmp4 =
		_tmp1 * covariance(9, 12) - _tmp3 + covariance(0, 12) + covariance(12, 12) * dt;
	const Scalar _tmp5 = -covariance(3, 3) * dt;
	const Scalar _tmp6 = _tmp1 * covariance(9, 3) + _tmp5 + covariance(0, 3) + covariance(12, 3) * dt;
	const Scalar _tmp7 = _tmp1 * covariance(9, 9);
	const Scalar _tmp8 = covariance(3, 9) * dt;
	const Scalar _tmp9 = _tmp7 - _tmp8 + covariance(0, 9) + covariance(12, 9) * dt;
	const Scalar _tmp10 = covariance(4, 9) * dt;
	const Scalar _tmp11 = _tmp1 * covariance(10, 9);
	const Scalar _tmp12 = -_tmp10 + _tmp11 + covariance(1, 9) + covariance(13, 9) * dt;
	const Scalar _tmp13 = -covariance(4, 3) * dt;
	const Scalar _tmp14 =
		_tmp1 * covariance(10, 3) + _tmp13 + covariance(1, 3) + covariance(13, 3) * dt;
	const Scalar _tmp15 = covariance(4, 12) * dt;
	const Scalar _tmp16 =
		_tmp1 * covariance(10, 12) - _tmp15 + covariance(1, 12) + covariance(13, 12) * dt;
	const Scalar _tmp17 = -covariance(5, 3) * dt;
	const Scalar _tmp18 =
		_tmp1 * covariance(11, 3) + _tmp17 + covariance(14, 3) * dt + covariance(2, 3);
	const Scalar _tmp19 = covariance(5, 9) * dt;
	const Scalar _tmp20 = _tmp1 * covariance(11, 9);
	const Scalar _tmp21 = -_tmp19 + _tmp20 + covariance(14, 9) * dt + covariance(2, 9);
	const Scalar _tmp22 = covariance(5, 12) * dt;
	const Scalar _tmp23 =
		_tmp1 * covariance(11, 12) - _tmp22 + covariance(14, 12) * dt + covariance(2, 12);
	const Scalar _tmp24 = Scalar(0.5) * [&]() {
		const Scalar base = dt;
		return base * base * base;
	}();
	const Scalar _tmp25 = -_tmp24 * input_var(0, 0);
	const Scalar _tmp26 = -_tmp24 * input_var(1, 0);
	const Scalar _tmp27 = -_tmp24 * input_var(2, 0);
	const Scalar _tmp28 = covariance(9, 3) * dt;
	const Scalar _tmp29 = covariance(9, 12) * dt;
	const Scalar _tmp30 = covariance(10, 12) * dt;
	const Scalar _tmp31 = covariance(10, 3) * dt;
	const Scalar _tmp32 = covariance(11, 12) * dt;
	const Scalar _tmp33 = covariance(11, 3) * dt;
	const Scalar _tmp34 = covariance(9, 9) * dt;
	const Scalar _tmp35 = _tmp34 + covariance(12, 9);
	const Scalar _tmp36 = _tmp28 + covariance(12, 3);
	const Scalar _tmp37 = _tmp29 + covariance(12, 12);
	const Scalar _tmp38 = _tmp31 + covariance(13, 3);
	const Scalar _tmp39 = covariance(10, 9) * dt;
	const Scalar _tmp40 = _tmp39 + covariance(13, 9);
	const Scalar _tmp41 = _tmp30 + covariance(13, 12);
	const Scalar _tmp42 = _tmp33 + covariance(14, 3);
	const Scalar _tmp43 = covariance(11, 9) * dt;
	const Scalar _tmp44 = _tmp43 + covariance(14, 9);
	const Scalar _tmp45 = _tmp32 + covariance(14, 12);
	const Scalar _tmp46 = _tmp1 * covariance(9, 10);
	const Scalar _tmp47 = covariance(3, 10) * dt;
	const Scalar _tmp48 = _tmp46 - _tmp47 + covariance(0, 10) + covariance(12, 10) * dt;
	const Scalar _tmp49 = covariance(3, 13) * dt;
	const Scalar _tmp50 =
		_tmp1 * covariance(9, 13) - _tmp49 + covariance(0, 13) + covariance(12, 13) * dt;
	const Scalar _tmp51 = -covariance(3, 4) * dt;
	const Scalar _tmp52 =
		_tmp1 * covariance(9, 4) + _tmp51 + covariance(0, 4) + covariance(12, 4) * dt;
	const Scalar _tmp53 = -covariance(4, 4) * dt;
	const Scalar _tmp54 =
		_tmp1 * covariance(10, 4) + _tmp53 + covariance(1, 4) + covariance(13, 4) * dt;
	const Scalar _tmp55 = covariance(4, 13) * dt;
	const Scalar _tmp56 =
		_tmp1 * covariance(10, 13) - _tmp55 + covariance(1, 13) + covariance(13, 13) * dt;
	const Scalar _tmp57 = _tmp1 * covariance(10, 10);
	const Scalar _tmp58 = covariance(4, 10) * dt;
	const Scalar _tmp59 = _tmp57 - _tmp58 + covariance(1, 10) + covariance(13, 10) * dt;
	const Scalar _tmp60 = -covariance(5, 4) * dt;
	const Scalar _tmp61 =
		_tmp1 * covariance(11, 4) + _tmp60 + covariance(14, 4) * dt + covariance(2, 4);
	const Scalar _tmp62 = covariance(5, 13) * dt;
	const Scalar _tmp63 =
		_tmp1 * covariance(11, 13) - _tmp62 + covariance(14, 13) * dt + covariance(2, 13);
	const Scalar _tmp64 = _tmp1 * covariance(11, 10);
	const Scalar _tmp65 = covariance(5, 10) * dt;
	const Scalar _tmp66 = _tmp64 - _tmp65 + covariance(14, 10) * dt + covariance(2, 10);
	const Scalar _tmp67 = -_tmp24 * input_var(0, 1);
	const Scalar _tmp68 = -_tmp24 * input_var(1, 1);
	const Scalar _tmp69 = -_tmp24 * input_var(2, 1);
	const Scalar _tmp70 = covariance(9, 4) * dt;
	const Scalar _tmp71 = covariance(9, 13) * dt;
	const Scalar _tmp72 = covariance(10, 13) * dt;
	const Scalar _tmp73 = covariance(10, 4) * dt;
	const Scalar _tmp74 = covariance(11, 13) * dt;
	const Scalar _tmp75 = covariance(11, 4) * dt;
	const Scalar _tmp76 = _tmp70 + covariance(12, 4);
	const Scalar _tmp77 = _tmp71 + covariance(12, 13);
	const Scalar _tmp78 = covariance(9, 10) * dt;
	const Scalar _tmp79 = _tmp78 + covariance(12, 10);
	const Scalar _tmp80 = _tmp73 + covariance(13, 4);
	const Scalar _tmp81 = _tmp72 + covariance(13, 13);
	const Scalar _tmp82 = covariance(10, 10) * dt;
	const Scalar _tmp83 = _tmp82 + covariance(13, 10);
	const Scalar _tmp84 = _tmp75 + covariance(14, 4);
	const Scalar _tmp85 = covariance(11, 10) * dt;
	const Scalar _tmp86 = _tmp85 + covariance(14, 10);
	const Scalar _tmp87 = _tmp74 + covariance(14, 13);
	const Scalar _tmp88 = covariance(3, 14) * dt;
	const Scalar _tmp89 =
		_tmp1 * covariance(9, 14) - _tmp88 + covariance(0, 14) + covariance(12, 14) * dt;
	const Scalar _tmp90 = _tmp1 * covariance(9, 11);
	const Scalar _tmp91 = covariance(3, 11) * dt;
	const Scalar _tmp92 = _tmp90 - _tmp91 + covariance(0, 11) + covariance(12, 11) * dt;
	const Scalar _tmp93 = -covariance(3, 5) * dt;
	const Scalar _tmp94 =
		_tmp1 * covariance(9, 5) + _tmp93 + covariance(0, 5) + covariance(12, 5) * dt;
	const Scalar _tmp95 = -covariance(4, 5) * dt;
	const Scalar _tmp96 =
		_tmp1 * covariance(10, 5) + _tmp95 + covariance(1, 5) + covariance(13, 5) * dt;
	const Scalar _tmp97 = covariance(4, 14) * dt;
	const Scalar _tmp98 =
		_tmp1 * covariance(10, 14) - _tmp97 + covariance(1, 14) + covariance(13, 14) * dt;
	const Scalar _tmp99 = _tmp1 * covariance(10, 11);
	const Scalar _tmp100 = covariance(4, 11) * dt;
	const Scalar _tmp101 = -_tmp100 + _tmp99 + covariance(1, 11) + covariance(13, 11) * dt;
	const Scalar _tmp102 = -covariance(5, 5) * dt;
	const Scalar _tmp103 =
		_tmp1 * covariance(11, 5) + _tmp102 + covariance(14, 5) * dt + covariance(2, 5);
	const Scalar _tmp104 = covariance(5, 14) * dt;
	const Scalar _tmp105 =
		_tmp1 * covariance(11, 14) - _tmp104 + covariance(14, 14) * dt + covariance(2, 14);
	const Scalar _tmp106 = _tmp1 * covariance(11, 11);
	const Scalar _tmp107 = covariance(5, 11) * dt;
	const Scalar _tmp108 = _tmp106 - _tmp107 + covariance(14, 11) * dt + covariance(2, 11);
	const Scalar _tmp109 = -_tmp24 * input_var(0, 2);
	const Scalar _tmp110 = -_tmp24 * input_var(1, 2);
	const Scalar _tmp111 = -_tmp24 * input_var(2, 2);
	const Scalar _tmp112 = covariance(9, 5) * dt;
	const Scalar _tmp113 = covariance(9, 14) * dt;
	const Scalar _tmp114 = covariance(10, 14) * dt;
	const Scalar _tmp115 = covariance(10, 5) * dt;
	const Scalar _tmp116 = covariance(11, 14) * dt;
	const Scalar _tmp117 = covariance(11, 5) * dt;
	const Scalar _tmp118 = _tmp112 + covariance(12, 5);
	const Scalar _tmp119 = _tmp113 + covariance(12, 14);
	const Scalar _tmp120 = covariance(9, 11) * dt;
	const Scalar _tmp121 = _tmp120 + covariance(12, 11);
	const Scalar _tmp122 = _tmp115 + covariance(13, 5);
	const Scalar _tmp123 = covariance(10, 11) * dt;
	const Scalar _tmp124 = _tmp123 + covariance(13, 11);
	const Scalar _tmp125 = _tmp114 + covariance(13, 14);
	const Scalar _tmp126 = _tmp117 + covariance(14, 5);
	const Scalar _tmp127 = covariance(11, 11) * dt;
	const Scalar _tmp128 = _tmp127 + covariance(14, 11);
	const Scalar _tmp129 = _tmp116 + covariance(14, 14);

	// Output terms (1)
	if (cov_updated != nullptr) {
		matrix::Matrix<Scalar, 15, 15> &_cov_updated = (*cov_updated);

		_cov_updated(0, 0) = _tmp1 * _tmp9 + _tmp1 * covariance(9, 0) + _tmp2 * input_var(0, 0) +
				     _tmp4 * dt - _tmp6 * dt + covariance(0, 0) + covariance(12, 0) * dt -
				     covariance(3, 0) * dt;
		_cov_updated(1, 0) = _tmp1 * _tmp12 + _tmp1 * covariance(10, 0) - _tmp14 * dt + _tmp16 * dt +
				     _tmp2 * input_var(1, 0) + covariance(1, 0) + covariance(13, 0) * dt -
				     covariance(4, 0) * dt;
		_cov_updated(2, 0) = _tmp1 * _tmp21 + _tmp1 * covariance(11, 0) - _tmp18 * dt +
				     _tmp2 * input_var(2, 0) + _tmp23 * dt + covariance(14, 0) * dt +
				     covariance(2, 0) - covariance(5, 0) * dt;
		_cov_updated(3, 0) = _tmp1 * covariance(3, 9) + _tmp25 + _tmp3 + _tmp5 + covariance(3, 0);
		_cov_updated(4, 0) = _tmp1 * covariance(4, 9) + _tmp13 + _tmp15 + _tmp26 + covariance(4, 0);
		_cov_updated(5, 0) = _tmp1 * covariance(5, 9) + _tmp17 + _tmp22 + _tmp27 + covariance(5, 0);
		_cov_updated(6, 0) = _tmp1 * covariance(6, 9) + covariance(6, 0) + covariance(6, 12) * dt -
				     covariance(6, 3) * dt;
		_cov_updated(7, 0) = _tmp1 * covariance(7, 9) + covariance(7, 0) + covariance(7, 12) * dt -
				     covariance(7, 3) * dt;
		_cov_updated(8, 0) = _tmp1 * covariance(8, 9) + covariance(8, 0) + covariance(8, 12) * dt -
				     covariance(8, 3) * dt;
		_cov_updated(9, 0) = -_tmp28 + _tmp29 + _tmp7 + covariance(9, 0);
		_cov_updated(10, 0) = _tmp11 + _tmp30 - _tmp31 + covariance(10, 0);
		_cov_updated(11, 0) = _tmp20 + _tmp32 - _tmp33 + covariance(11, 0);
		_cov_updated(12, 0) =
			_tmp1 * _tmp35 - _tmp36 * dt + _tmp37 * dt + covariance(12, 0) + covariance(9, 0) * dt;
		_cov_updated(13, 0) =
			_tmp1 * _tmp40 - _tmp38 * dt + _tmp41 * dt + covariance(10, 0) * dt + covariance(13, 0);
		_cov_updated(14, 0) =
			_tmp1 * _tmp44 - _tmp42 * dt + _tmp45 * dt + covariance(11, 0) * dt + covariance(14, 0);
		_cov_updated(0, 1) = _tmp1 * _tmp48 + _tmp1 * covariance(9, 1) + _tmp2 * input_var(0, 1) +
				     _tmp50 * dt - _tmp52 * dt + covariance(0, 1) + covariance(12, 1) * dt -
				     covariance(3, 1) * dt;
		_cov_updated(1, 1) = _tmp1 * _tmp59 + _tmp1 * covariance(10, 1) + _tmp2 * input_var(1, 1) -
				     _tmp54 * dt + _tmp56 * dt + covariance(1, 1) + covariance(13, 1) * dt -
				     covariance(4, 1) * dt;
		_cov_updated(2, 1) = _tmp1 * _tmp66 + _tmp1 * covariance(11, 1) + _tmp2 * input_var(2, 1) -
				     _tmp61 * dt + _tmp63 * dt + covariance(14, 1) * dt + covariance(2, 1) -
				     covariance(5, 1) * dt;
		_cov_updated(3, 1) = _tmp1 * covariance(3, 10) + _tmp49 + _tmp51 + _tmp67 + covariance(3, 1);
		_cov_updated(4, 1) = _tmp1 * covariance(4, 10) + _tmp53 + _tmp55 + _tmp68 + covariance(4, 1);
		_cov_updated(5, 1) = _tmp1 * covariance(5, 10) + _tmp60 + _tmp62 + _tmp69 + covariance(5, 1);
		_cov_updated(6, 1) = _tmp1 * covariance(6, 10) + covariance(6, 1) + covariance(6, 13) * dt -
				     covariance(6, 4) * dt;
		_cov_updated(7, 1) = _tmp1 * covariance(7, 10) + covariance(7, 1) + covariance(7, 13) * dt -
				     covariance(7, 4) * dt;
		_cov_updated(8, 1) = _tmp1 * covariance(8, 10) + covariance(8, 1) + covariance(8, 13) * dt -
				     covariance(8, 4) * dt;
		_cov_updated(9, 1) = _tmp46 - _tmp70 + _tmp71 + covariance(9, 1);
		_cov_updated(10, 1) = _tmp57 + _tmp72 - _tmp73 + covariance(10, 1);
		_cov_updated(11, 1) = _tmp64 + _tmp74 - _tmp75 + covariance(11, 1);
		_cov_updated(12, 1) =
			_tmp1 * _tmp79 - _tmp76 * dt + _tmp77 * dt + covariance(12, 1) + covariance(9, 1) * dt;
		_cov_updated(13, 1) =
			_tmp1 * _tmp83 - _tmp80 * dt + _tmp81 * dt + covariance(10, 1) * dt + covariance(13, 1);
		_cov_updated(14, 1) =
			_tmp1 * _tmp86 - _tmp84 * dt + _tmp87 * dt + covariance(11, 1) * dt + covariance(14, 1);
		_cov_updated(0, 2) = _tmp1 * _tmp92 + _tmp1 * covariance(9, 2) + _tmp2 * input_var(0, 2) +
				     _tmp89 * dt - _tmp94 * dt + covariance(0, 2) + covariance(12, 2) * dt -
				     covariance(3, 2) * dt;
		_cov_updated(1, 2) = _tmp1 * _tmp101 + _tmp1 * covariance(10, 2) + _tmp2 * input_var(1, 2) -
				     _tmp96 * dt + _tmp98 * dt + covariance(1, 2) + covariance(13, 2) * dt -
				     covariance(4, 2) * dt;
		_cov_updated(2, 2) = _tmp1 * _tmp108 + _tmp1 * covariance(11, 2) - _tmp103 * dt + _tmp105 * dt +
				     _tmp2 * input_var(2, 2) + bias_var(0, 0) + covariance(14, 2) * dt +
				     covariance(2, 2) - covariance(5, 2) * dt;
		_cov_updated(3, 2) = _tmp1 * covariance(3, 11) + _tmp109 + _tmp88 + _tmp93 + covariance(3, 2);
		_cov_updated(4, 2) = _tmp1 * covariance(4, 11) + _tmp110 + _tmp95 + _tmp97 + covariance(4, 2);
		_cov_updated(5, 2) = _tmp1 * covariance(5, 11) + _tmp102 + _tmp104 + _tmp111 + covariance(5, 2);
		_cov_updated(6, 2) = _tmp1 * covariance(6, 11) + covariance(6, 14) * dt + covariance(6, 2) -
				     covariance(6, 5) * dt;
		_cov_updated(7, 2) = _tmp1 * covariance(7, 11) + covariance(7, 14) * dt + covariance(7, 2) -
				     covariance(7, 5) * dt;
		_cov_updated(8, 2) = _tmp1 * covariance(8, 11) + covariance(8, 14) * dt + covariance(8, 2) -
				     covariance(8, 5) * dt;
		_cov_updated(9, 2) = -_tmp112 + _tmp113 + _tmp90 + covariance(9, 2);
		_cov_updated(10, 2) = _tmp114 - _tmp115 + _tmp99 + covariance(10, 2);
		_cov_updated(11, 2) = _tmp106 + _tmp116 - _tmp117 + covariance(11, 2);
		_cov_updated(12, 2) =
			_tmp1 * _tmp121 - _tmp118 * dt + _tmp119 * dt + covariance(12, 2) + covariance(9, 2) * dt;
		_cov_updated(13, 2) =
			_tmp1 * _tmp124 - _tmp122 * dt + _tmp125 * dt + covariance(10, 2) * dt + covariance(13, 2);
		_cov_updated(14, 2) =
			_tmp1 * _tmp128 - _tmp126 * dt + _tmp129 * dt + covariance(11, 2) * dt + covariance(14, 2);
		_cov_updated(0, 3) = _tmp25 + _tmp6;
		_cov_updated(1, 3) = _tmp14 + _tmp26;
		_cov_updated(2, 3) = _tmp18 + _tmp27;
		_cov_updated(3, 3) = _tmp0 * input_var(0, 0) + covariance(3, 3);
		_cov_updated(4, 3) = _tmp0 * input_var(1, 0) + covariance(4, 3);
		_cov_updated(5, 3) = _tmp0 * input_var(2, 0) + covariance(5, 3);
		_cov_updated(6, 3) = covariance(6, 3);
		_cov_updated(7, 3) = covariance(7, 3);
		_cov_updated(8, 3) = covariance(8, 3);
		_cov_updated(9, 3) = covariance(9, 3);
		_cov_updated(10, 3) = covariance(10, 3);
		_cov_updated(11, 3) = covariance(11, 3);
		_cov_updated(12, 3) = _tmp36;
		_cov_updated(13, 3) = _tmp38;
		_cov_updated(14, 3) = _tmp42;
		_cov_updated(0, 4) = _tmp52 + _tmp67;
		_cov_updated(1, 4) = _tmp54 + _tmp68;
		_cov_updated(2, 4) = _tmp61 + _tmp69;
		_cov_updated(3, 4) = _tmp0 * input_var(0, 1) + covariance(3, 4);
		_cov_updated(4, 4) = _tmp0 * input_var(1, 1) + acc_var(0, 0) + covariance(4, 4);
		_cov_updated(5, 4) = _tmp0 * input_var(2, 1) + covariance(5, 4);
		_cov_updated(6, 4) = covariance(6, 4);
		_cov_updated(7, 4) = covariance(7, 4);
		_cov_updated(8, 4) = covariance(8, 4);
		_cov_updated(9, 4) = covariance(9, 4);
		_cov_updated(10, 4) = covariance(10, 4);
		_cov_updated(11, 4) = covariance(11, 4);
		_cov_updated(12, 4) = _tmp76;
		_cov_updated(13, 4) = _tmp80;
		_cov_updated(14, 4) = _tmp84;
		_cov_updated(0, 5) = _tmp109 + _tmp94;
		_cov_updated(1, 5) = _tmp110 + _tmp96;
		_cov_updated(2, 5) = _tmp103 + _tmp111;
		_cov_updated(3, 5) = _tmp0 * input_var(0, 2) + covariance(3, 5);
		_cov_updated(4, 5) = _tmp0 * input_var(1, 2) + covariance(4, 5);
		_cov_updated(5, 5) = _tmp0 * input_var(2, 2) + covariance(5, 5);
		_cov_updated(6, 5) = covariance(6, 5);
		_cov_updated(7, 5) = covariance(7, 5);
		_cov_updated(8, 5) = covariance(8, 5);
		_cov_updated(9, 5) = covariance(9, 5);
		_cov_updated(10, 5) = covariance(10, 5);
		_cov_updated(11, 5) = covariance(11, 5);
		_cov_updated(12, 5) = _tmp118;
		_cov_updated(13, 5) = _tmp122;
		_cov_updated(14, 5) = _tmp126;
		_cov_updated(0, 6) = _tmp1 * covariance(9, 6) + covariance(0, 6) + covariance(12, 6) * dt -
				     covariance(3, 6) * dt;
		_cov_updated(1, 6) = _tmp1 * covariance(10, 6) + covariance(1, 6) + covariance(13, 6) * dt -
				     covariance(4, 6) * dt;
		_cov_updated(2, 6) = _tmp1 * covariance(11, 6) + covariance(14, 6) * dt + covariance(2, 6) -
				     covariance(5, 6) * dt;
		_cov_updated(3, 6) = covariance(3, 6);
		_cov_updated(4, 6) = covariance(4, 6);
		_cov_updated(5, 6) = covariance(5, 6);
		_cov_updated(6, 6) = covariance(6, 6);
		_cov_updated(7, 6) = covariance(7, 6);
		_cov_updated(8, 6) = covariance(8, 6);
		_cov_updated(9, 6) = covariance(9, 6);
		_cov_updated(10, 6) = covariance(10, 6);
		_cov_updated(11, 6) = covariance(11, 6);
		_cov_updated(12, 6) = covariance(12, 6) + covariance(9, 6) * dt;
		_cov_updated(13, 6) = covariance(10, 6) * dt + covariance(13, 6);
		_cov_updated(14, 6) = covariance(11, 6) * dt + covariance(14, 6);
		_cov_updated(0, 7) = _tmp1 * covariance(9, 7) + covariance(0, 7) + covariance(12, 7) * dt -
				     covariance(3, 7) * dt;
		_cov_updated(1, 7) = _tmp1 * covariance(10, 7) + covariance(1, 7) + covariance(13, 7) * dt -
				     covariance(4, 7) * dt;
		_cov_updated(2, 7) = _tmp1 * covariance(11, 7) + covariance(14, 7) * dt + covariance(2, 7) -
				     covariance(5, 7) * dt;
		_cov_updated(3, 7) = covariance(3, 7);
		_cov_updated(4, 7) = covariance(4, 7);
		_cov_updated(5, 7) = covariance(5, 7);
		_cov_updated(6, 7) = covariance(6, 7);
		_cov_updated(7, 7) = bias_var(1, 1) + covariance(7, 7);
		_cov_updated(8, 7) = covariance(8, 7);
		_cov_updated(9, 7) = covariance(9, 7);
		_cov_updated(10, 7) = covariance(10, 7);
		_cov_updated(11, 7) = covariance(11, 7);
		_cov_updated(12, 7) = covariance(12, 7) + covariance(9, 7) * dt;
		_cov_updated(13, 7) = covariance(10, 7) * dt + covariance(13, 7);
		_cov_updated(14, 7) = covariance(11, 7) * dt + covariance(14, 7);
		_cov_updated(0, 8) = _tmp1 * covariance(9, 8) + covariance(0, 8) + covariance(12, 8) * dt -
				     covariance(3, 8) * dt;
		_cov_updated(1, 8) = _tmp1 * covariance(10, 8) + covariance(1, 8) + covariance(13, 8) * dt -
				     covariance(4, 8) * dt;
		_cov_updated(2, 8) = _tmp1 * covariance(11, 8) + covariance(14, 8) * dt + covariance(2, 8) -
				     covariance(5, 8) * dt;
		_cov_updated(3, 8) = covariance(3, 8);
		_cov_updated(4, 8) = covariance(4, 8);
		_cov_updated(5, 8) = covariance(5, 8);
		_cov_updated(6, 8) = covariance(6, 8);
		_cov_updated(7, 8) = covariance(7, 8);
		_cov_updated(8, 8) = covariance(8, 8);
		_cov_updated(9, 8) = covariance(9, 8);
		_cov_updated(10, 8) = covariance(10, 8);
		_cov_updated(11, 8) = covariance(11, 8);
		_cov_updated(12, 8) = covariance(12, 8) + covariance(9, 8) * dt;
		_cov_updated(13, 8) = covariance(10, 8) * dt + covariance(13, 8);
		_cov_updated(14, 8) = covariance(11, 8) * dt + covariance(14, 8);
		_cov_updated(0, 9) = _tmp9;
		_cov_updated(1, 9) = _tmp12;
		_cov_updated(2, 9) = _tmp21;
		_cov_updated(3, 9) = covariance(3, 9);
		_cov_updated(4, 9) = covariance(4, 9);
		_cov_updated(5, 9) = covariance(5, 9);
		_cov_updated(6, 9) = covariance(6, 9);
		_cov_updated(7, 9) = covariance(7, 9);
		_cov_updated(8, 9) = covariance(8, 9);
		_cov_updated(9, 9) = acc_var(1, 1) + covariance(9, 9);
		_cov_updated(10, 9) = covariance(10, 9);
		_cov_updated(11, 9) = covariance(11, 9);
		_cov_updated(12, 9) = _tmp35;
		_cov_updated(13, 9) = _tmp40;
		_cov_updated(14, 9) = _tmp44;
		_cov_updated(0, 10) = _tmp48;
		_cov_updated(1, 10) = _tmp59;
		_cov_updated(2, 10) = _tmp66;
		_cov_updated(3, 10) = covariance(3, 10);
		_cov_updated(4, 10) = covariance(4, 10);
		_cov_updated(5, 10) = covariance(5, 10);
		_cov_updated(6, 10) = covariance(6, 10);
		_cov_updated(7, 10) = covariance(7, 10);
		_cov_updated(8, 10) = covariance(8, 10);
		_cov_updated(9, 10) = covariance(9, 10);
		_cov_updated(10, 10) = covariance(10, 10);
		_cov_updated(11, 10) = covariance(11, 10);
		_cov_updated(12, 10) = _tmp79;
		_cov_updated(13, 10) = _tmp83;
		_cov_updated(14, 10) = _tmp86;
		_cov_updated(0, 11) = _tmp92;
		_cov_updated(1, 11) = _tmp101;
		_cov_updated(2, 11) = _tmp108;
		_cov_updated(3, 11) = covariance(3, 11);
		_cov_updated(4, 11) = covariance(4, 11);
		_cov_updated(5, 11) = covariance(5, 11);
		_cov_updated(6, 11) = covariance(6, 11);
		_cov_updated(7, 11) = covariance(7, 11);
		_cov_updated(8, 11) = covariance(8, 11);
		_cov_updated(9, 11) = covariance(9, 11);
		_cov_updated(10, 11) = covariance(10, 11);
		_cov_updated(11, 11) = covariance(11, 11);
		_cov_updated(12, 11) = _tmp121;
		_cov_updated(13, 11) = _tmp124;
		_cov_updated(14, 11) = _tmp128;
		_cov_updated(0, 12) = _tmp4 + _tmp9 * dt;
		_cov_updated(1, 12) = _tmp12 * dt + _tmp16;
		_cov_updated(2, 12) = _tmp21 * dt + _tmp23;
		_cov_updated(3, 12) = _tmp8 + covariance(3, 12);
		_cov_updated(4, 12) = _tmp10 + covariance(4, 12);
		_cov_updated(5, 12) = _tmp19 + covariance(5, 12);
		_cov_updated(6, 12) = covariance(6, 12) + covariance(6, 9) * dt;
		_cov_updated(7, 12) = covariance(7, 12) + covariance(7, 9) * dt;
		_cov_updated(8, 12) = covariance(8, 12) + covariance(8, 9) * dt;
		_cov_updated(9, 12) = _tmp34 + covariance(9, 12);
		_cov_updated(10, 12) = _tmp39 + covariance(10, 12);
		_cov_updated(11, 12) = _tmp43 + covariance(11, 12);
		_cov_updated(12, 12) = _tmp35 * dt + _tmp37 + bias_var(2, 2);
		_cov_updated(13, 12) = _tmp40 * dt + _tmp41;
		_cov_updated(14, 12) = _tmp44 * dt + _tmp45;
		_cov_updated(0, 13) = _tmp48 * dt + _tmp50;
		_cov_updated(1, 13) = _tmp56 + _tmp59 * dt;
		_cov_updated(2, 13) = _tmp63 + _tmp66 * dt;
		_cov_updated(3, 13) = _tmp47 + covariance(3, 13);
		_cov_updated(4, 13) = _tmp58 + covariance(4, 13);
		_cov_updated(5, 13) = _tmp65 + covariance(5, 13);
		_cov_updated(6, 13) = covariance(6, 10) * dt + covariance(6, 13);
		_cov_updated(7, 13) = covariance(7, 10) * dt + covariance(7, 13);
		_cov_updated(8, 13) = covariance(8, 10) * dt + covariance(8, 13);
		_cov_updated(9, 13) = _tmp78 + covariance(9, 13);
		_cov_updated(10, 13) = _tmp82 + covariance(10, 13);
		_cov_updated(11, 13) = _tmp85 + covariance(11, 13);
		_cov_updated(12, 13) = _tmp77 + _tmp79 * dt;
		_cov_updated(13, 13) = _tmp81 + _tmp83 * dt;
		_cov_updated(14, 13) = _tmp86 * dt + _tmp87;
		_cov_updated(0, 14) = _tmp89 + _tmp92 * dt;
		_cov_updated(1, 14) = _tmp101 * dt + _tmp98;
		_cov_updated(2, 14) = _tmp105 + _tmp108 * dt;
		_cov_updated(3, 14) = _tmp91 + covariance(3, 14);
		_cov_updated(4, 14) = _tmp100 + covariance(4, 14);
		_cov_updated(5, 14) = _tmp107 + covariance(5, 14);
		_cov_updated(6, 14) = covariance(6, 11) * dt + covariance(6, 14);
		_cov_updated(7, 14) = covariance(7, 11) * dt + covariance(7, 14);
		_cov_updated(8, 14) = covariance(8, 11) * dt + covariance(8, 14);
		_cov_updated(9, 14) = _tmp120 + covariance(9, 14);
		_cov_updated(10, 14) = _tmp123 + covariance(10, 14);
		_cov_updated(11, 14) = _tmp127 + covariance(11, 14);
		_cov_updated(12, 14) = _tmp119 + _tmp121 * dt;
		_cov_updated(13, 14) = _tmp124 * dt + _tmp125;
		_cov_updated(14, 14) = _tmp128 * dt + _tmp129 + acc_var(2, 2);
	}
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
