/**
 *   @file: inertial_filter.h
 *
 *   Copyright (C) 2013-2015 Anton Babushkin. All rights reserved.
 *   @author Anton Babushkin <rk3dov@gmail.com>
 */

#include <cmath>
#include <stdbool.h>
#include <eigen/px4_eigen.h>
#include <drivers/drv_hrt.h>

using namespace Eigen;

class InertialFilter
{
public:
	InertialFilter();
	~InertialFilter() {};

	void inertial_filter_predict(float dt, Vector2f &x, float acc);

	void inertial_filter_correct(float e, float dt, Vector2f &x, int i, float w);
};
