
/home/sjo/uwb/boards/px4/fmu-v5/default.px4board

CONFIG_EXAMPLES_FAKE_GPS=y
CONFIG_MODULES_ROVER_ACKERMANN=y
CONFIG_MODULES_ROVER_DIFFERENTIAL=y

parameter ekf2_gps_ctrl = 15 check dual antenna heading

Tag head tele2
Tag tail uart2&i2c

mode ch 5
arm /disarm ch6
