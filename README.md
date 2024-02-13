Description :
=============
First cement, sand and water are fed in the mixture vessel. Reserviour cement sensor
(RSV_CMT_SENSOR) is used to indicate level of cement in a cement reserviour. Sand sensor(RSV_SAND_SENSOR) is used to indicate level of sand in a sand reserviour. Water
sensor(RSV_WATER_SENSOR) is used to indicate level of water in a water reserviour. 

STAGE 1:
If reserviour cement sensor's value is above specific thresold level, then run cement flow control motor. Cement quantity Sensor(VSL_CMT_SENSOR) is used to monitor quantity of cement flown in mixture vessel. After specific required quantity of cement is fed in mixture vessel, stop run cement flow control motor. If Cement quantity Sensor in mixture vessel does not reach the specific required quantity of cement in the mixture vessel within a specific time duration from time of start of running the cement flow control motor, the stop the cement flow control motor and sound alarm (ALARM_CMT_QTY)by flashing led on for cement indicator as there may be
problem of cement leavel or cement related sensors is not proper functioning. If reserviour cement sensor's value is less than specific thresold level, then alarm sound(ALARM_CMT_LVL) by flashing led on.

STAGE 2:
If required quantity of cement is fed in the mixture vessel, and if reserviour sand sensor's value is above specific thresold level, then run sand flow control motor. Sand quantity Sensor (VSL_SAND_SENSOR) is used to monitor quantity of sand flown in mixture vessel. After specific required quantity of sand is fed in mixture vessel, stop run sand flow control motor. If sand quantity Sensor in mixture vessel does not reach the specific required quantity of sand in the mixture vessel
within a specific time duration from time of start of running the sand flow control motor, the stop the sand flow control motor and sound alarm (ALARM_SAND_QTY)by flashing led on for sand indicator as there may be problem of sand level or sand related sensors is not proper functioning. If reserviour sand sensor's value is less than specific thresold level, then alarm sound (ALARM_SAND_LVL) by flashing led on.

STAGE 3:
If required quantity of sand and cement are fed in the mixture vessel, and if reserviour water sensor's value is above specific thresold level, then run water pump. water quantity Sensor (VSL_WATER_SENSOR) is used to monitor quantity of water flown in mixture vessel. After specific required quantity of water is fed in mixture vessel, stop run water flow control motor. If water quantity Sensor in mixture vessel does not reach the specific required quantity of water in the mixture vessel within a specific time duration from time of start of running the water flow control motor, the stop the water flow control motor and sound alarm (ALARM_WATER_QTY) by flashing led on for water indicator as there may be problem of water level or sand related sensors is not proper functioning. If reserviour water sensor's value is less than specific thresold level, then alarm sound(ALARM_WATER_LVL) by flashing led on.

STAGE 4:
After required quantity of sand, cement and water are fed in the mixture vessel. Start run mixture vessel rotoring control motor (mixture of cement, sand and water) in foward direction for 3 minutes and then run mixture rotoring control motor in reverse dirction for 3 minutes. Repeat this process of forward and then reverse run of mixture vessel rotoring control motor for another 2 times. Then stop mixture rotoring control motor and flash led on (MIXED_OK)to indicate the mixture of cement, sand and water are stired well and complete. After a RESTART_SW is pressed to indicate that stirred well mixture of cement, sand and
water are taken out of mixture vessel and ready to receive fresh intake of cement, sand and water in
the mixture vessel. Repeat the process.

CAUTION:
========
Schematics and simulation is done by Proteus CAD. NOT EXPERIMENTED IN REAL TIME ENVIRONMENT.

Purpose :
=========
In all my respective repositories, I just shared my works that I worked as the learning path and practiced, with designed, developed, implemented, simulated and tested, including some projects, assignments, documentations and all other related files and some programming that might not being implement, not being completed, lacks some features or have some bugs. Purpose of all my repositories, if used, can be used for EDUCATIONAL PURPOSE ONLY. It can be used as the open source and freeware. Kindly read the LICENSE.txt for license, terms and conditions about the use of source codes, binaries, documentation and all other files, located in all my repositories. 

