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
 * Symbolic function: computeInnovCov
 *
 * Args:
 *     meas_unc: Scalar
 *     covariance: Matrix99
 *     meas_matrix: Matrix19
 *
 * Outputs:
 *     innov_cov_updated: Scalar
 */
template <typename Scalar>
void Computeinnovcov(const Scalar meas_unc, const matrix::Matrix<Scalar, 9, 9> &covariance,
		     const matrix::Matrix<Scalar, 1, 9> &meas_matrix,
		     Scalar *const innov_cov_updated = nullptr)
{
	// Total ops: 171

	// Input arrays

	// Intermediate terms (0)

	// Output terms (1)
	if (innov_cov_updated != nullptr) {
		Scalar &_innov_cov_updated = (*innov_cov_updated);

		_innov_cov_updated =
			meas_matrix(0, 0) *
			(covariance(0, 0) * meas_matrix(0, 0) + covariance(1, 0) * meas_matrix(0, 1) +
			 covariance(2, 0) * meas_matrix(0, 2) + covariance(3, 0) * meas_matrix(0, 3) +
			 covariance(4, 0) * meas_matrix(0, 4) + covariance(5, 0) * meas_matrix(0, 5) +
			 covariance(6, 0) * meas_matrix(0, 6) + covariance(7, 0) * meas_matrix(0, 7) +
			 covariance(8, 0) * meas_matrix(0, 8)) +
			meas_matrix(0, 1) *
			(covariance(0, 1) * meas_matrix(0, 0) + covariance(1, 1) * meas_matrix(0, 1) +
			 covariance(2, 1) * meas_matrix(0, 2) + covariance(3, 1) * meas_matrix(0, 3) +
			 covariance(4, 1) * meas_matrix(0, 4) + covariance(5, 1) * meas_matrix(0, 5) +
			 covariance(6, 1) * meas_matrix(0, 6) + covariance(7, 1) * meas_matrix(0, 7) +
			 covariance(8, 1) * meas_matrix(0, 8)) +
			meas_matrix(0, 2) *
			(covariance(0, 2) * meas_matrix(0, 0) + covariance(1, 2) * meas_matrix(0, 1) +
			 covariance(2, 2) * meas_matrix(0, 2) + covariance(3, 2) * meas_matrix(0, 3) +
			 covariance(4, 2) * meas_matrix(0, 4) + covariance(5, 2) * meas_matrix(0, 5) +
			 covariance(6, 2) * meas_matrix(0, 6) + covariance(7, 2) * meas_matrix(0, 7) +
			 covariance(8, 2) * meas_matrix(0, 8)) +
			meas_matrix(0, 3) *
			(covariance(0, 3) * meas_matrix(0, 0) + covariance(1, 3) * meas_matrix(0, 1) +
			 covariance(2, 3) * meas_matrix(0, 2) + covariance(3, 3) * meas_matrix(0, 3) +
			 covariance(4, 3) * meas_matrix(0, 4) + covariance(5, 3) * meas_matrix(0, 5) +
			 covariance(6, 3) * meas_matrix(0, 6) + covariance(7, 3) * meas_matrix(0, 7) +
			 covariance(8, 3) * meas_matrix(0, 8)) +
			meas_matrix(0, 4) *
			(covariance(0, 4) * meas_matrix(0, 0) + covariance(1, 4) * meas_matrix(0, 1) +
			 covariance(2, 4) * meas_matrix(0, 2) + covariance(3, 4) * meas_matrix(0, 3) +
			 covariance(4, 4) * meas_matrix(0, 4) + covariance(5, 4) * meas_matrix(0, 5) +
			 covariance(6, 4) * meas_matrix(0, 6) + covariance(7, 4) * meas_matrix(0, 7) +
			 covariance(8, 4) * meas_matrix(0, 8)) +
			meas_matrix(0, 5) *
			(covariance(0, 5) * meas_matrix(0, 0) + covariance(1, 5) * meas_matrix(0, 1) +
			 covariance(2, 5) * meas_matrix(0, 2) + covariance(3, 5) * meas_matrix(0, 3) +
			 covariance(4, 5) * meas_matrix(0, 4) + covariance(5, 5) * meas_matrix(0, 5) +
			 covariance(6, 5) * meas_matrix(0, 6) + covariance(7, 5) * meas_matrix(0, 7) +
			 covariance(8, 5) * meas_matrix(0, 8)) +
			meas_matrix(0, 6) *
			(covariance(0, 6) * meas_matrix(0, 0) + covariance(1, 6) * meas_matrix(0, 1) +
			 covariance(2, 6) * meas_matrix(0, 2) + covariance(3, 6) * meas_matrix(0, 3) +
			 covariance(4, 6) * meas_matrix(0, 4) + covariance(5, 6) * meas_matrix(0, 5) +
			 covariance(6, 6) * meas_matrix(0, 6) + covariance(7, 6) * meas_matrix(0, 7) +
			 covariance(8, 6) * meas_matrix(0, 8)) +
			meas_matrix(0, 7) *
			(covariance(0, 7) * meas_matrix(0, 0) + covariance(1, 7) * meas_matrix(0, 1) +
			 covariance(2, 7) * meas_matrix(0, 2) + covariance(3, 7) * meas_matrix(0, 3) +
			 covariance(4, 7) * meas_matrix(0, 4) + covariance(5, 7) * meas_matrix(0, 5) +
			 covariance(6, 7) * meas_matrix(0, 6) + covariance(7, 7) * meas_matrix(0, 7) +
			 covariance(8, 7) * meas_matrix(0, 8)) +
			meas_matrix(0, 8) *
			(covariance(0, 8) * meas_matrix(0, 0) + covariance(1, 8) * meas_matrix(0, 1) +
			 covariance(2, 8) * meas_matrix(0, 2) + covariance(3, 8) * meas_matrix(0, 3) +
			 covariance(4, 8) * meas_matrix(0, 4) + covariance(5, 8) * meas_matrix(0, 5) +
			 covariance(6, 8) * meas_matrix(0, 6) + covariance(7, 8) * meas_matrix(0, 7) +
			 covariance(8, 8) * meas_matrix(0, 8)) +
			meas_unc;
	}
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
