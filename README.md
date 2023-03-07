Description :
=============
I,  K.M. Arun Kumar alias Arunkumar Murugeswaran, just shared my works, which I worked as learning path and practiced Embedded C programming using PIC16F887 (Microchip's 8 bit microcontroller in PIC16 family) such as First cement, sand and water are fed in the mixture vessel. Reserviour cement sensor
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

Projects, using PIC16F887, are included with Design, development, implemented, simulated and tested, by using a simulator are as follows :
===========================================================================================================================================
 1: smart milk vending machine. 
 2: Simple Digital Single phase Electric Motor Controller controlled by Electric current.
 3: Simple Digital Single phase Electric Motor Controller controlled by Electric Voltage.
 4: Soil moisture based Water spraying system for nursery farms.
 5: Traffic density congestion control based traffic signal controller.
 6: Construction automation for mixing of cement, sand and water in the construction mixture equipment. 
 7: In LCD, running text is displayed either to left or to right with specified number of gaps between consecutive same text display.

Purpose :
=========
In all my respective repositories, I just shared my works that I worked as the learning path and practiced, with designed, developed, implemented, simulated and tested, including some projects, assignments, documentations and all other related files and some programming that might not being implement, not being completed, lacks some features or have some bugs. Purpose of all my repositories, if used, can be used for LEARNING AND EDUCATIONAL PURPOSE ONLY. It can be used as the open source and freeware. Kindly read the LICENSE.txt for license, terms and conditions about the use of source codes, binaries, documentation and all other files, located in all my repositories. 

My Thanks and Tribute :
========================
I thank to my family, Friends, Teachers, People behind the toolchains and references that I used, all those who directly or indirectly supported me and/or helped me and/or my family, Nature and God. My tribute to my family, Friends, Teachers, People behind the toolchains and references that I used, Nature, Jimmy Dog, God and all those, who directly or indirectly help and/or support me and/or my family.

Toolchains that I used for PIC16F887 Application design and development are as follows :
=========================================================================================
1: IDE and compiler for PIC16F887                                           - Microchip's MPLAB X IDE (v4.01) with MPLAB XC8 compiler(v1.45) or
                                                                              Microchip's MPLAB 8.6 IDE with Hi-Tech C Compiler.  
2: CAD and simulator for PIC16F887                                          - Proteus 8.0 Professional and/or Proteus 8.3 Professional SP2.
3: PIC16F887 development board                                              - www.alselectro.com 
4: Flash Programmer for PIC16F887                                           – PicKit3.
5: LCD                                                                      - JHD162A.
6: Desktop Computer Architecture and OS for PIC16F887 development           - Intel X64 & Windows 7 (32 bit).
7: Code editor                                                              - Notepad++.
8: Documentation                                                            - Microsoft Office 2007 (alternative LibreOffice) and Text Editor.

Some reference that I refered for PIC16F887  Application design and development, are as follows :
==================================================================================================
1: Schaum's Outline of Programming with C, 2nd Edition - Book authored by Byron Gottfried.
2: Understanding and Using C Pointers: Core Techniques for Memory Management - Book authored by Richard M. Reese. 
3: Embedded C - Book authored by Michael J. Pont.
4: PIC16F887 product datasheet.
5: Hitachi HD44780U - LCD product data sheet.

Note :
======
Kindly read in the source codes, if mentioned, about the Program Description or Purpose, Known Bugs, Caution and Notes and documentations. Some applications are also tested in a PIC16F887 development board.

My Repositories Web Link :
==========================
https://github.com/k-m-arun-kumar-5

