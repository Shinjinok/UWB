/**
 * Wind estimator wind process noise.
 *
 * @min 0
 * @max 1
 * @unit m/s/s
 * @group Wind Estimator
 */
PARAM_DEFINE_FLOAT(WEST_W_P_NOISE, 0.1f);

/**
 * Wind estimator true airspeed scale process noise.
 *
 * @min 0
 * @max 0.1
 * @group Wind Estimator
 */
PARAM_DEFINE_FLOAT(WEST_SC_P_NOISE, 0.0001);

/**
 * Wind estimator true airspeed measurement noise.
 *
 * @min 0
 * @max 4
 * @unit m/s
 * @group Wind Estimator
 */
PARAM_DEFINE_FLOAT(WEST_TAS_NOISE, 1.4);

/**
 * Wind estimator sideslip measurement noise.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @group Wind Estimator
 */
PARAM_DEFINE_FLOAT(WEST_BETA_NOISE, 0.3);
