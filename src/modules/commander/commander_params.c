/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file commander_params.c
 *
 * Parameters defined by the sensors task.
 * 由传感器任务定义的参数
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Julian Oes <julian@px4.io>
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Roll trim  横滚调整
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * 调整值是系统在直线以及水平飞行时需要的执行器控制输出值。
 * 可以通过使用'RC调整'手动操作飞机直线或水平飞行进行校正。
 * 并使用地面站复制
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);

/**
 * Pitch trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);

/**
 * Yaw trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * Datalink loss time threshold   数据链丢失时间阈值
 *
 * After this amount of seconds without datalink the data link lost mode triggers
 * 经过一段没有数据链的时间(s)后，数据链路丢失模式会被触发
 *
 * @group Commander
 * @unit s
 * @min 5
 * @max 300
 * @decimal 1
 * @increment 0.5
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_T, 10);

/**
 * Datalink regain time threshold
 *
 * After a data link loss: after this this amount of seconds with a healthy datalink the 'datalink loss'
 * flag is set back to false
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 3
 * @decimal 1
 * @increment 0.5
 */
PARAM_DEFINE_INT32(COM_DL_REG_T, 0);

/**
 * Engine Failure Throttle Threshold 引擎故障油门阈值
 *
 * Engine failure triggers only above this throttle value
 * 仅当油门值超过这个阈值时油门故障才会被触发
 *
 * @group Commander
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_EF_THROT, 0.5f);

/**
 * Engine Failure Current/Throttle Threshold
 * 电机故障 电流/油门 阈值
 *
 * Engine failure triggers only below this current value
 * 仅当低于此电流值时触发电机故障
 *
 * @group Commander
 * @min 0.0
 * @max 50.0
 * @unit A/%
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_EF_C2T, 5.0f);

/**
 * Engine Failure Time Threshold
 *
 * Engine failure triggers only if the throttle threshold and the
 * current to throttle threshold are violated for this time
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_EF_TIME, 10.0f);

/**
 * RC loss time threshold
 *
 * After this amount of seconds without RC connection the rc lost flag is set to true
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 35
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);

/**
 * Home set horizontal threshold
 * 起飞点设置水平阈值
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 * 如果估计位置精度低于此阈值那么起飞点位置将置位
 *
 * @group Commander
 * @unit m
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.5
 */
PARAM_DEFINE_FLOAT(COM_HOME_H_T, 5.0f);

/**
 * Home set vertical threshold
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 *
 * @group Commander
 * @unit m
 * @min 5
 * @max 25
 * @decimal 2
 * @increment 0.5
 */
PARAM_DEFINE_FLOAT(COM_HOME_V_T, 10.0f);

/**
 * Autosaving of params
 *
 * If not equal to zero the commander will automatically save parameters to persistent storage once changed.
 * Default is on, as the interoperability with currently deployed GCS solutions depends on parameters
 * being sticky. Developers can default it to off.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_AUTOS_PAR, 1);

/**
 * RC control input mode
 *
 * The default value of 0 requires a valid RC transmitter setup.
 * Setting this to 1 allows joystick control and disables RC input handling and the associated checks. A value of
 * 2 will generate RC control data from manual input received via MAVLink instead
 * of directly forwarding the manual input data.
 *
 * @group Commander
 * @min 0
 * @max 2
 * @value 0 RC Transmitter
 * @value 1 Joystick/No RC Checks
 * @value 2 Virtual RC by Joystick
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);

/**
 * RC input arm/disarm command duration
 * 遥控器输入解锁/上锁命令持续时间
 *
 * The default value of 1000 requires the stick to be held in the arm or disarm position for 1 second.
 * 默认值为1000，需要摇杆保持在解锁或者上锁的位置1秒
 *
 * @group Commander
 * @min 100
 * @max 1500
 */
PARAM_DEFINE_INT32(COM_RC_ARM_HYST, 1000);

/**
 * Time-out for auto disarm after landing
 *
 * A non-zero, positive value specifies the time-out period in seconds after which the vehicle will be
 * automatically disarmed in case a landing situation has been detected during this period.
 * A value of zero means that automatic disarming is disabled.
 *
 * @group Commander
 * @min 0
 * @max 20
 * @unit s
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_DISARM_LAND, 0);

/**
 * Allow arming without GPS
 *
 * The default allows to arm the vehicle without GPS signal.
 *
 * @group Commander
 * @min 0
 * @max 1
 * @value 0 Don't allow arming without GPS
 * @value 1 Allow arming without GPS
 */
PARAM_DEFINE_INT32(COM_ARM_WO_GPS, 1);

/**
 * Battery failsafe mode
 * 电池故障保护模式
 * 
 * Action the system takes on low battery. Defaults to off
 * 系统在低电压模式下采取的措施。默认关闭，即不采取措施，非低电压状态
 *
 * @group Commander
 * @value 0 Warning  警告 -> 0
 * @value 1 Return to Land 返航 -> 1
 * @value 2 Land at current position  降落 -> 2
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);

/**
 * Time-out to wait when offboard connection is lost before triggering offboard lost action.
 * See COM_OBL_ACT and COM_OBL_RC_ACT to configure action.
 *
 * @group Commander
 * @unit second
 * @min 0
 * @max 60
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 0.0f);

/**
 * Set offboard loss failsafe mode 设置外部控制模式丢失故障保护模式
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 * 外部控制模式故障保护只会在一个超时后进入
 * 这个时间由COM_OF_LOSS_T设置，单位 s
 *
 * @value 0 Land at current position
 * @value 1 Loiter
 * @value 2 Return to Land
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(COM_OBL_ACT, 0);

/**
 * Set offboard loss failsafe mode when RC is available
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value 0 Position control
 * @value 1 Altitude control
 * @value 2 Manual
 * @value 3 Return to Land
 * @value 4 Land at current position
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(COM_OBL_RC_ACT, 0);

/**
 * First flightmode slot (1000-1160)  飞行模式安排
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 如果主切换通道在此范围内，选择的飞行模式将会被应用
 *
 * @value -1 Unassigned      -1 -> 未指定
 * @value 0 Manual			  0 -> 手动
 * @value 1 Altitude		  1 -> 定高
 * @value 2 Position          2 -> 定点
 * @value 3 Mission           3 -> 任务
 * @value 4 Hold              4 -> 悬停
 * @value 10 Takeoff         10 -> 起飞
 * @value 11 Land            11 -> 降落
 * @value 5 Return            5 -> 返航
 * @value 6 Acro              6 -> 特技
 * @value 7 Offboard          7 -> 外部
 * @value 8 Stabilized        8 -> 自稳
 * @value 9 Rattitude         9 -> 角速度
 * @value 12 Follow Me       12 -> 跟踪
 */
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);

/**
 * Second flightmode slot (1160-1320)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Rattitude
 * @value 12 Follow Me
 */
PARAM_DEFINE_INT32(COM_FLTMODE2, -1);

/**
 * Third flightmode slot (1320-1480)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Rattitude
 * @value 12 Follow Me
 */
PARAM_DEFINE_INT32(COM_FLTMODE3, -1);

/**
 * Fourth flightmode slot (1480-1640)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Rattitude
 * @value 12 Follow Me
 */
PARAM_DEFINE_INT32(COM_FLTMODE4, -1);

/**
 * Fifth flightmode slot (1640-1800)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Rattitude
 * @value 12 Follow Me
 */
PARAM_DEFINE_INT32(COM_FLTMODE5, -1);

/**
 * Sixth flightmode slot (1800-2000)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Rattitude
 * @value 12 Follow Me
 */
PARAM_DEFINE_INT32(COM_FLTMODE6, -1);
