/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * Rotation from Fake GPS coordiation to earth NE .
 * @min -180.0
 * @max +180.0
 * @unit deg
 * @decimal 1
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_ROT, 0.f);

/**
 * Latitude offset for fake GPS
 * @min -90.0
 * @max +90.0
 * @unit deg
 * @decimal 7
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_LAT, 37.3014533f);
/**
 * Longitude offset for fake GPS
 * @min -180.0
 * @max +180.0
 * @unit deg
 * @decimal 7
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_LON, 126.8401891f);
/**
 * Height offset for fake GPS
 * @min -1000.0
 * @max +1000.0
 * @unit m
 * @decimal 3
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_HGT, 50.f);

/**
 * Kalman gain of position for fake GPS
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_KP, 0.59f);//0.8
/**
 * Kalman gain of velocity for fake GPS
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_KV, 0.9f);//0.4
/**
 * Sampling Time for fake GPS[sec]
 * @min 0.1
 * @max 1.0
 * @decimal 3
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_DT, 0.1f);
/**
 * Heading calculattion with two uwb tag of not
 *
 * If this is set to 1, calculattion using one uwb tag
 * *
 * If this is set to 2, calculattion using two uwb tag
 *
 * @min 1
 * @max 2
 * @value 1 one uwb tag
 * @value 2 two uwb tag
 * @group FAKE_GPS
 */
PARAM_DEFINE_INT32(FAKE_GPS_TAG, 2);

/**
 * Heading offset angle for fake GPS coordiation to earth NE .
 * @min -180.0
 * @max +180.0
 * @unit deg
 * @decimal 1
 * @group FAKE_GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_OFS, -78.f);//two tag atan(5/94) = 3 degree

/**
 * GPS1 used for USS
 * @min 0
 * @max 1
 * @value 0 GPS1 used GPS1
 * @value 1 GPS1 used sonar for altitude
 * @group FAKE_GPS
 */
PARAM_DEFINE_INT32(FAKE_GPS_USS, 0);
/**
 * UWB available max iteration number
 * @min 1
 * @max 100
 * @group FAKE_GPS
 */
PARAM_DEFINE_INT32(FAKE_GPS_ITR, 50);
/**
 * Fake gps enable
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(FAKE_GPS_EN, 0);
/**
 * Fake gps UGV mode
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(FAKE_GPS_UGV, 0);
/**
 * Fake gps NED vel mode
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(FAKE_GPS_NED, 1);
