/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "macros.h"

#define BOARD_UNKNOWN -1

////
// RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560//坡道1.3/1.4-ATmega1280、ATmega2560
////

#define BOARD_RAMPS_OLD               1000  // MEGA/RAMPS up to 1.2//超高速/高速上升至1.2

#define BOARD_RAMPS_13_EFB            1010  // RAMPS 1.3 (Power outputs: Hotend, Fan, Bed)//坡道1.3（电源输出：热端、风扇、床）
#define BOARD_RAMPS_13_EEB            1011  // RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Bed)//斜坡1.3（电源输出：Hotend0、Hotend1、床）
#define BOARD_RAMPS_13_EFF            1012  // RAMPS 1.3 (Power outputs: Hotend, Fan0, Fan1)//斜坡1.3（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_13_EEF            1013  // RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Fan)//斜坡1.3（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_13_SF             1014  // RAMPS 1.3 (Power outputs: Spindle, Controller Fan)//斜坡1.3（电源输出：主轴、控制器风扇）

#define BOARD_RAMPS_14_EFB            1020  // RAMPS 1.4 (Power outputs: Hotend, Fan, Bed)//坡道1.4（电源输出：热端、风扇、床）
#define BOARD_RAMPS_14_EEB            1021  // RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Bed)//斜坡1.4（电源输出：Hotend0、Hotend1、床）
#define BOARD_RAMPS_14_EFF            1022  // RAMPS 1.4 (Power outputs: Hotend, Fan0, Fan1)//斜坡1.4（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_14_EEF            1023  // RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Fan)//斜坡1.4（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_14_SF             1024  // RAMPS 1.4 (Power outputs: Spindle, Controller Fan)//斜坡1.4（电源输出：主轴、控制器风扇）

#define BOARD_RAMPS_PLUS_EFB          1030  // RAMPS Plus 3DYMY (Power outputs: Hotend, Fan, Bed)//斜坡加3DYMY（电源输出：热端、风扇、床）
#define BOARD_RAMPS_PLUS_EEB          1031  // RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Bed)//斜坡加3DYMY（电源输出：Hotend0、Hotend1、Bed）
#define BOARD_RAMPS_PLUS_EFF          1032  // RAMPS Plus 3DYMY (Power outputs: Hotend, Fan0, Fan1)//斜坡加3DYMY（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_PLUS_EEF          1033  // RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Fan)//斜坡加3DYMY（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_PLUS_SF           1034  // RAMPS Plus 3DYMY (Power outputs: Spindle, Controller Fan)//斜坡加3DYMY（电源输出：主轴、控制器风扇）

////
// RAMPS Derivatives - ATmega1280, ATmega2560//斜坡衍生产品-ATmega1280、ATmega2560
////

#define BOARD_3DRAG                   1100  // 3Drag Controller//3Drag控制器
#define BOARD_K8200                   1101  // Velleman K8200 Controller (derived from 3Drag Controller)//Velleman K8200控制器（源自3Drag控制器）
#define BOARD_K8400                   1102  // Velleman K8400 Controller (derived from 3Drag Controller)//Velleman K8400控制器（源自3Drag控制器）
#define BOARD_K8600                   1103  // Velleman K8600 Controller (Vertex Nano)//Velleman K8600控制器（顶点纳米）
#define BOARD_K8800                   1104  // Velleman K8800 Controller (Vertex Delta)//Velleman K8800控制器（顶点三角形）
#define BOARD_BAM_DICE                1105  // 2PrintBeta BAM&DICE with STK drivers//2PrintBeta BAM&DICE与STK驱动程序
#define BOARD_BAM_DICE_DUE            1106  // 2PrintBeta BAM&DICE Due with STK drivers//STK驱动程序的2PrintBeta BAM和DICE到期
#define BOARD_MKS_BASE                1107  // MKS BASE v1.0//MKS BASE v1.0
#define BOARD_MKS_BASE_14             1108  // MKS BASE v1.4 with Allegro A4982 stepper drivers//带有Allegro A4982步进驱动程序的MKS BASE v1.4
#define BOARD_MKS_BASE_15             1109  // MKS BASE v1.5 with Allegro A4982 stepper drivers//带有Allegro A4982步进驱动程序的MKS BASE v1.5
#define BOARD_MKS_BASE_16             1110  // MKS BASE v1.6 with Allegro A4982 stepper drivers//带有Allegro A4982步进驱动程序的MKS BASE v1.6
#define BOARD_MKS_BASE_HEROIC         1111  // MKS BASE 1.0 with Heroic HR4982 stepper drivers//MKS BASE 1.0，配备英勇的HR4982步进驱动程序
#define BOARD_MKS_GEN_13              1112  // MKS GEN v1.3 or 1.4//MKS GEN v1.3或1.4
#define BOARD_MKS_GEN_L               1113  // MKS GEN L//MKS GEN L
#define BOARD_KFB_2                   1114  // BigTreeTech or BIQU KFB2.0//BigTreeTech或BIQU KFB2.0
#define BOARD_ZRIB_V20                1115  // zrib V2.0 (Chinese RAMPS replica)//zrib V2.0（中国坡道复制品）
#define BOARD_ZRIB_V52                1116  // zrib V5.2 (Chinese RAMPS replica)//zrib V5.2（中文坡道副本）
#define BOARD_FELIX2                  1117  // Felix 2.0+ Electronics Board (RAMPS like)//Felix 2.0+电子板（类似坡道）
#define BOARD_RIGIDBOARD              1118  // Invent-A-Part RigidBoard//发明一部分刚体
#define BOARD_RIGIDBOARD_V2           1119  // Invent-A-Part RigidBoard V2//发明一个零件刚性板V2
#define BOARD_SAINSMART_2IN1          1120  // Sainsmart 2-in-1 board//赛因斯马特二合一板
#define BOARD_ULTIMAKER               1121  // Ultimaker//终极制造者
#define BOARD_ULTIMAKER_OLD           1122  // Ultimaker (Older electronics. Pre 1.5.4. This is rare)//Ultimaker（较旧的电子产品。1.5.4之前。这很少见）
#define BOARD_AZTEEG_X3               1123  // Azteeg X3//阿兹提格X3
#define BOARD_AZTEEG_X3_PRO           1124  // Azteeg X3 Pro//Azteeg X3专业版
#define BOARD_ULTIMAIN_2              1125  // Ultimainboard 2.x (Uses TEMP_SENSOR 20)//Ultimainboard 2.x（使用温度传感器20）
#define BOARD_RUMBA                   1126  // Rumba//伦巴
#define BOARD_RUMBA_RAISE3D           1127  // Raise3D N series Rumba derivative//Raise3D N系列伦巴导数
#define BOARD_RL200                   1128  // Rapide Lite 200 (v1, low-cost RUMBA clone with drv)//Rapide Lite 200（v1，带drv的低成本伦巴克隆）
#define BOARD_FORMBOT_TREX2PLUS       1129  // Formbot T-Rex 2 Plus//Formbot T-Rex 2 Plus
#define BOARD_FORMBOT_TREX3           1130  // Formbot T-Rex 3//Formbot T-Rex 3
#define BOARD_FORMBOT_RAPTOR          1131  // Formbot Raptor//Formbot猛禽
#define BOARD_FORMBOT_RAPTOR2         1132  // Formbot Raptor 2//Formbot猛禽2
#define BOARD_BQ_ZUM_MEGA_3D          1133  // bq ZUM Mega 3D//bq ZUM Mega 3D
#define BOARD_MAKEBOARD_MINI          1134  // MakeBoard Mini v2.1.2 by MicroMake//MicroMake的MakeBoard Mini v2.1.2版
#define BOARD_TRIGORILLA_13           1135  // TriGorilla Anycubic version 1.3-based on RAMPS EFB//TriGorilla Anycublic 1.3版-基于RAMPS EFB
#define BOARD_TRIGORILLA_14           1136  //   ... Ver 1.4//   ... 1.4版
#define BOARD_TRIGORILLA_14_11        1137  //   ... Rev 1.1 (new servo pin order)//   ... 版本1.1（新伺服销订单）
#define BOARD_RAMPS_ENDER_4           1138  // Creality: Ender-4, CR-8//信度：Ender-4，CR-8
#define BOARD_RAMPS_CREALITY          1139  // Creality: CR10S, CR20, CR-X//公司名称：CR10S、CR20、CR-X
#define BOARD_DAGOMA_F5               1140  // Dagoma F5//达戈马F5
#define BOARD_FYSETC_F6_13            1141  // FYSETC F6 1.3//FYSETC F6 1.3
#define BOARD_FYSETC_F6_14            1142  // FYSETC F6 1.4//FYSETC F6 1.4
#define BOARD_DUPLICATOR_I3_PLUS      1143  // Wanhao Duplicator i3 Plus//万豪i3plus复印机
#define BOARD_VORON                   1144  // VORON Design//沃隆设计
#define BOARD_TRONXY_V3_1_0           1145  // Tronxy TRONXY-V3-1.0//Tronxy Tronxy-V3-1.0
#define BOARD_Z_BOLT_X_SERIES         1146  // Z-Bolt X Series//Z型螺栓X系列
#define BOARD_TT_OSCAR                1147  // TT OSCAR//TT奥斯卡
#define BOARD_OVERLORD                1148  // Overlord/Overlord Pro//霸王/霸王职业
#define BOARD_HJC2560C_REV1           1149  // ADIMLab Gantry v1//ADIMLab龙门v1
#define BOARD_HJC2560C_REV2           1150  // ADIMLab Gantry v2//ADIMLab龙门v2
#define BOARD_TANGO                   1151  // BIQU Tango V1//琵琶探戈V1
#define BOARD_MKS_GEN_L_V2            1152  // MKS GEN L V2//MKS GEN L V2
#define BOARD_MKS_GEN_L_V21           1153  // MKS GEN L V2.1//MKS GEN L V2.1
#define BOARD_COPYMASTER_3D           1154  // Copymaster 3D//复印机3D
#define BOARD_ORTUR_4                 1155  // Ortur 4//奥图尔4
#define BOARD_TENLOG_D3_HERO          1156  // Tenlog D3 Hero IDEX printer//Tenlog D3 Hero IDEX打印机
#define BOARD_RAMPS_S_12_EEFB         1157  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Fan, Bed)//Sakul.cz的斜坡1.2（电源输出：Hotend0、Hotend1、风扇、床）
#define BOARD_RAMPS_S_12_EEEB         1158  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Hotend2, Bed)//Sakul.cz提供的斜坡1.2（电源输出：Hotend0、Hotend1、Hotend2、Bed）
#define BOARD_RAMPS_S_12_EFFB         1159  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend, Fan0, Fan1, Bed)//Sakul.cz的斜坡1.2（电源输出：热端、风扇0、风扇1、床）
#define BOARD_LONGER3D_LK1_PRO        1160  // Longer LK1 PRO / Alfawise U20 Pro (PRO version)//更长的LK1 PRO/Alfawise U20 PRO（专业版）
#define BOARD_LONGER3D_LKx_PRO        1161  // Longer LKx PRO / Alfawise Uxx Pro (PRO version)//更长的LKx PRO/Alfawise Uxx PRO（专业版）

////
// RAMBo and derivatives//兰博及其衍生物
////

#define BOARD_RAMBO                   1200  // Rambo//兰博
#define BOARD_MINIRAMBO               1201  // Mini-Rambo//迷你兰博
#define BOARD_MINIRAMBO_10A           1202  // Mini-Rambo 1.0a//迷你兰博1.0a
#define BOARD_EINSY_RAMBO             1203  // Einsy Rambo//艾恩西·兰博
#define BOARD_EINSY_RETRO             1204  // Einsy Retro//艾恩西复古酒店
#define BOARD_SCOOVO_X9H              1205  // abee Scoovo X9H//abee Scoovo X9H
#define BOARD_RAMBO_THINKERV2         1206  // ThinkerV2//ThinkerV2

////
// Other ATmega1280, ATmega2560//其他ATmega1280、ATmega2560
////

#define BOARD_CNCONTROLS_11           1300  // Cartesio CN Controls V11//Cartesio CN控制V11
#define BOARD_CNCONTROLS_12           1301  // Cartesio CN Controls V12//Cartesio CN控制V12
#define BOARD_CNCONTROLS_15           1302  // Cartesio CN Controls V15//Cartesio CN控制V15
#define BOARD_CHEAPTRONIC             1303  // Cheaptronic v1.0//Cheaptronic v1.0
#define BOARD_CHEAPTRONIC_V2          1304  // Cheaptronic v2.0//Cheaptronic v2.0
#define BOARD_MIGHTYBOARD_REVE        1305  // Makerbot Mightyboard Revision E//Makerbot可能会出现板修订版E
#define BOARD_MEGATRONICS             1306  // Megatronics//威震电子学
#define BOARD_MEGATRONICS_2           1307  // Megatronics v2.0//Megatronics v2.0
#define BOARD_MEGATRONICS_3           1308  // Megatronics v3.0//Megatronics v3.0
#define BOARD_MEGATRONICS_31          1309  // Megatronics v3.1//Megatronics v3.1
#define BOARD_MEGATRONICS_32          1310  // Megatronics v3.2//Megatronics v3.2
#define BOARD_ELEFU_3                 1311  // Elefu Ra Board (v3)//Elefu Ra板（v3）
#define BOARD_LEAPFROG                1312  // Leapfrog//蛙跳
#define BOARD_MEGACONTROLLER          1313  // Mega controller//超级控制器
#define BOARD_GT2560_REV_A            1314  // Geeetech GT2560 Rev A//Geeetech GT2560版本A
#define BOARD_GT2560_REV_A_PLUS       1315  // Geeetech GT2560 Rev A+ (with auto level probe)//Geeetech GT2560版本A+（带自动液位探头）
#define BOARD_GT2560_REV_B            1316  // Geeetech GT2560 Rev B//Geeetech GT2560版本B
#define BOARD_GT2560_V3               1317  // Geeetech GT2560 Rev B for A10(M/D)//适用于A10（米/天）的Geeetech GT2560版本B
#define BOARD_GT2560_V4               1318  // Geeetech GT2560 Rev B for A10(M/D)//适用于A10（米/天）的Geeetech GT2560版本B
#define BOARD_GT2560_V3_MC2           1319  // Geeetech GT2560 Rev B for Mecreator2//Geeetech GT2560第B版，适用于Mecreator2
#define BOARD_GT2560_V3_A20           1320  // Geeetech GT2560 Rev B for A20(M/D)//适用于A20的Geeetech GT2560版本B（米/天）
#define BOARD_EINSTART_S              1321  // Einstart retrofit//Einstart改造
#define BOARD_WANHAO_ONEPLUS          1322  // Wanhao 0ne+ i3 Mini//万豪0ne+i3迷你
#define BOARD_LEAPFROG_XEED2015       1323  // Leapfrog Xeed 2015//跃进Xeed 2015
#define BOARD_PICA_REVB               1324  // PICA Shield (original version)//皮卡盾（原版）
#define BOARD_PICA                    1325  // PICA Shield (rev C or later)//PICA防护罩（C版或更高版本）
#define BOARD_INTAMSYS40              1326  // Intamsys 4.0 (Funmat HT)//Intamsys 4.0（Funmat HT）
#define BOARD_MALYAN_M180             1327  // Malyan M180 Mainboard Version 2 (no display function, direct gcode only)//Malyan M180主板版本2（无显示功能，仅直接gcode）

////
// ATmega1281, ATmega2561//ATmega1281、ATmega2561
////

#define BOARD_MINITRONICS             1400  // Minitronics v1.0/1.1//Minitronics v1.0/1.1
#define BOARD_SILVER_GATE             1401  // Silvergate v1.0//Silvergate v1.0

////
// Sanguinololu and Derivatives - ATmega644P, ATmega1284P//Sanguinololu及其衍生物-ATmega644P、ATmega1284P
////

#define BOARD_SANGUINOLOLU_11         1500  // Sanguinololu < 1.2//血色<1.2
#define BOARD_SANGUINOLOLU_12         1501  // Sanguinololu 1.2 and above//Sanguinololu 1.2及以上
#define BOARD_MELZI                   1502  // Melzi//梅尔齐
#define BOARD_MELZI_V2                1503  // Melzi V2//梅尔齐V2
#define BOARD_MELZI_MAKR3D            1504  // Melzi with ATmega1284 (MaKr3d version)//带ATmega1284的Melzi（MaKr3d版本）
#define BOARD_MELZI_CREALITY          1505  // Melzi Creality3D (for CR-10 etc)//Melzi Creality3D（适用于CR-10等）
#define BOARD_MELZI_MALYAN            1506  // Melzi Malyan M150//梅尔齐·马尔扬M150
#define BOARD_MELZI_TRONXY            1507  // Tronxy X5S//Tronxy X5S
#define BOARD_STB_11                  1508  // STB V1.1//机顶盒V1.1
#define BOARD_AZTEEG_X1               1509  // Azteeg X1//阿兹特格X1
#define BOARD_ANET_10                 1510  // Anet 1.0 (Melzi clone)//Anet 1.0（Melzi克隆）
#define BOARD_ZMIB_V2                 1511  // ZoneStar ZMIB V2//ZoneStar ZMIB V2

////
// Other ATmega644P, ATmega644, ATmega1284P//其他ATmega644P、ATmega644、ATmega1284P
////

#define BOARD_GEN3_MONOLITHIC         1600  // Gen3 Monolithic Electronics//第三代单片电子学
#define BOARD_GEN3_PLUS               1601  // Gen3+//第3代+
#define BOARD_GEN6                    1602  // Gen6//第6代
#define BOARD_GEN6_DELUXE             1603  // Gen6 deluxe//Gen6豪华版
#define BOARD_GEN7_CUSTOM             1604  // Gen7 custom (Alfons3 Version) https://github.com/Alfons3/Generation_7_Electronics//Gen7定制版（阿方斯3版）https://github.com/Alfons3/Generation_7_Electronics
#define BOARD_GEN7_12                 1605  // Gen7 v1.1, v1.2//第7代1.1、1.2版
#define BOARD_GEN7_13                 1606  // Gen7 v1.3//第7代1.3版
#define BOARD_GEN7_14                 1607  // Gen7 v1.4//第7代1.4版
#define BOARD_OMCA_A                  1608  // Alpha OMCA//阿尔法OMCA
#define BOARD_OMCA                    1609  // Final OMCA//最后OMCA
#define BOARD_SETHI                   1610  // Sethi 3D_1//Sethi 3D_1

////
// Teensyduino - AT90USB1286, AT90USB1286P//蒂恩西杜伊诺-AT90USB1286，AT90USB1286P
////

#define BOARD_TEENSYLU                1700  // Teensylu//天西卢
#define BOARD_PRINTRBOARD             1701  // Printrboard (AT90USB1286)//打印板（AT90USB1286）
#define BOARD_PRINTRBOARD_REVF        1702  // Printrboard Revision F (AT90USB1286)//打印板版本F（AT90USB1286）
#define BOARD_BRAINWAVE               1703  // Brainwave (AT90USB646)//脑波（AT90USB646）
#define BOARD_BRAINWAVE_PRO           1704  // Brainwave Pro (AT90USB1286)//Brainwave Pro（AT90USB1286）
#define BOARD_SAV_MKI                 1705  // SAV Mk-I (AT90USB1286)//SAV Mk-I（AT90USB1286）
#define BOARD_TEENSY2                 1706  // Teensy++2.0 (AT90USB1286)//Teensy++2.0（AT90USB1286）
#define BOARD_5DPRINT                 1707  // 5DPrint D8 Driver Board//5D打印D8驱动板

////
// LPC1768 ARM Cortex M3//LPC1768臂皮质M3
////

#define BOARD_RAMPS_14_RE_ARM_EFB     2000  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend, Fan, Bed)//使用斜坡1.4重新武装（电源输出：热端、风扇、床）
#define BOARD_RAMPS_14_RE_ARM_EEB     2001  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Bed)//使用斜坡1.4重新武装（电源输出：Hotend0、Hotend1、Bed）
#define BOARD_RAMPS_14_RE_ARM_EFF     2002  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend, Fan0, Fan1)//用斜坡1.4重新武装（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_14_RE_ARM_EEF     2003  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Fan)//用斜坡1.4重新武装（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_14_RE_ARM_SF      2004  // Re-ARM with RAMPS 1.4 (Power outputs: Spindle, Controller Fan)//用斜坡1.4重新武装（电源输出：主轴、控制器风扇）
#define BOARD_MKS_SBASE               2005  // MKS-Sbase (Power outputs: Hotend0, Hotend1, Bed, Fan)//MKS Sbase（电源输出：Hotend0、Hotend1、床、风扇）
#define BOARD_AZSMZ_MINI              2006  // AZSMZ Mini//AZSMZ迷你型
#define BOARD_BIQU_BQ111_A4           2007  // BIQU BQ111-A4 (Power outputs: Hotend, Fan, Bed)//BIQU BQ111-A4（电源输出：热端、风扇、床）
#define BOARD_SELENA_COMPACT          2008  // Selena Compact (Power outputs: Hotend0, Hotend1, Bed0, Bed1, Fan0, Fan1)//赛琳娜紧凑型（电源输出：Hotend0、Hotend1、Bed0、Bed1、Fan0、Fan1）
#define BOARD_BIQU_B300_V1_0          2009  // BIQU B300_V1.0 (Power outputs: Hotend0, Fan, Bed, SPI Driver)//BIQU B300_V1.0（电源输出：Hotend0、风扇、床、SPI驱动器）
#define BOARD_MKS_SGEN_L              2010  // MKS-SGen-L (Power outputs: Hotend0, Hotend1, Bed, Fan)//MKS-SGen-L（电源输出：Hotend0、Hotend1、床、风扇）
#define BOARD_GMARSH_X6_REV1          2011  // GMARSH X6, revision 1 prototype//GMARSH X6，第1版原型
#define BOARD_BTT_SKR_V1_1            2012  // BigTreeTech SKR v1.1 (Power outputs: Hotend0, Hotend1, Fan, Bed)//BigTreeTech SKR v1.1（电源输出：Hotend0、Hotend1、风扇、床）
#define BOARD_BTT_SKR_V1_3            2013  // BigTreeTech SKR v1.3 (Power outputs: Hotend0, Hotend1, Fan, Bed)//BigTreeTech SKR v1.3（电源输出：Hotend0、Hotend1、风扇、床）
#define BOARD_BTT_SKR_V1_4            2014  // BigTreeTech SKR v1.4 (Power outputs: Hotend0, Hotend1, Fan, Bed)//BigTreeTech SKR v1.4（电源输出：Hotend0、Hotend1、风扇、床）

////
// LPC1769 ARM Cortex M3//LPC1769臂皮质M3
////

#define BOARD_MKS_SGEN                2500  // MKS-SGen (Power outputs: Hotend0, Hotend1, Bed, Fan)//MKS SGen（电源输出：Hotend0、Hotend1、床、风扇）
#define BOARD_AZTEEG_X5_GT            2501  // Azteeg X5 GT (Power outputs: Hotend0, Hotend1, Bed, Fan)//Azteeg X5 GT（电源输出：热端0、热端1、床、风扇）
#define BOARD_AZTEEG_X5_MINI          2502  // Azteeg X5 Mini (Power outputs: Hotend0, Bed, Fan)//Azteeg X5迷你型（电源输出：Hotend0、床、风扇）
#define BOARD_AZTEEG_X5_MINI_WIFI     2503  // Azteeg X5 Mini Wifi (Power outputs: Hotend0, Bed, Fan)//Azteeg X5迷你Wifi（电源输出：Hotend0、床、风扇）
#define BOARD_COHESION3D_REMIX        2504  // Cohesion3D ReMix//内聚三维混音
#define BOARD_COHESION3D_MINI         2505  // Cohesion3D Mini//Cohension3D迷你版
#define BOARD_SMOOTHIEBOARD           2506  // Smoothieboard//刨花板
#define BOARD_TH3D_EZBOARD            2507  // TH3D EZBoard v1.0//TH3D EZBoard v1.0
#define BOARD_BTT_SKR_V1_4_TURBO      2508  // BigTreeTech SKR v1.4 TURBO (Power outputs: Hotend0, Hotend1, Fan, Bed)//BigTreeTech SKR v1.4 TURBO（功率输出：Hotend0、Hotend1、风扇、床）
#define BOARD_MKS_SGEN_L_V2           2509  // MKS SGEN_L V2 (Power outputs: Hotend0, Hotend1, Bed, Fan)//MKS SGEN_L V2（电源输出：热端0、热端1、床、风扇）
#define BOARD_BTT_SKR_E3_TURBO        2510  // BigTreeTech SKR E3 Turbo (Power outputs: Hotend0, Hotend1, Bed, Fan0, Fan1)//BigTreeTech SKR E3 Turbo（电源输出：Hotend0、Hotend1、Bed、Fan0、Fan1）
#define BOARD_FLY_CDY                 2511  // FLY_CDY (Power outputs: Hotend0, Hotend1, Hotend2, Bed, Fan0, Fan1, Fan2)//FLY_CDY（电源输出：Hotend0、Hotend1、Hotend2、床、风扇0、风扇1、风扇2）

////
// SAM3X8E ARM Cortex M3//SAM3X8E手臂皮质M3
////

#define BOARD_DUE3DOM                 3000  // DUE3DOM for Arduino DUE//Arduino的DUE3DOM到期
#define BOARD_DUE3DOM_MINI            3001  // DUE3DOM MINI for Arduino DUE//DUE3DOM MINI用于Arduino到期
#define BOARD_RADDS                   3002  // RADDS//拉德
#define BOARD_RAMPS_FD_V1             3003  // RAMPS-FD v1//RAMPS-FD v1
#define BOARD_RAMPS_FD_V2             3004  // RAMPS-FD v2//RAMPS-FD v2
#define BOARD_RAMPS_SMART_EFB         3005  // RAMPS-SMART (Power outputs: Hotend, Fan, Bed)//RAMPS-SMART（电源输出：热端、风扇、床）
#define BOARD_RAMPS_SMART_EEB         3006  // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Bed)//RAMPS-SMART（电源输出：Hotend0、Hotend1、Bed）
#define BOARD_RAMPS_SMART_EFF         3007  // RAMPS-SMART (Power outputs: Hotend, Fan0, Fan1)//RAMPS-SMART（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_SMART_EEF         3008  // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Fan)//RAMPS-SMART（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_SMART_SF          3009  // RAMPS-SMART (Power outputs: Spindle, Controller Fan)//RAMPS-SMART（电源输出：主轴、控制器风扇）
#define BOARD_RAMPS_DUO_EFB           3010  // RAMPS Duo (Power outputs: Hotend, Fan, Bed)//RAMPS Duo（电源输出：热端、风扇、床）
#define BOARD_RAMPS_DUO_EEB           3011  // RAMPS Duo (Power outputs: Hotend0, Hotend1, Bed)//RAMPS Duo（电源输出：Hotend0、Hotend1、Bed）
#define BOARD_RAMPS_DUO_EFF           3012  // RAMPS Duo (Power outputs: Hotend, Fan0, Fan1)//RAMPS Duo（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS_DUO_EEF           3013  // RAMPS Duo (Power outputs: Hotend0, Hotend1, Fan)//RAMPS Duo（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS_DUO_SF            3014  // RAMPS Duo (Power outputs: Spindle, Controller Fan)//RAMPS Duo（电源输出：主轴、控制器风扇）
#define BOARD_RAMPS4DUE_EFB           3015  // RAMPS4DUE (Power outputs: Hotend, Fan, Bed)//RAMPS4DUE（电源输出：热端、风扇、床）
#define BOARD_RAMPS4DUE_EEB           3016  // RAMPS4DUE (Power outputs: Hotend0, Hotend1, Bed)//RAMPS4DUE（电源输出：Hotend0、Hotend1、Bed）
#define BOARD_RAMPS4DUE_EFF           3017  // RAMPS4DUE (Power outputs: Hotend, Fan0, Fan1)//RAMPS4DUE（电源输出：热端、风扇0、风扇1）
#define BOARD_RAMPS4DUE_EEF           3018  // RAMPS4DUE (Power outputs: Hotend0, Hotend1, Fan)//RAMPS4DUE（电源输出：Hotend0、Hotend1、风扇）
#define BOARD_RAMPS4DUE_SF            3019  // RAMPS4DUE (Power outputs: Spindle, Controller Fan)//RAMPS4DUE（电源输出：主轴、控制器风扇）
#define BOARD_RURAMPS4D_11            3020  // RuRAMPS4Duo v1.1 (Power outputs: Hotend0, Hotend1, Hotend2, Fan0, Fan1, Bed)//RuRAMPS4Duo v1.1（电源输出：Hotend0、Hotend1、Hotend2、Fan0、Fan1、Bed）
#define BOARD_RURAMPS4D_13            3021  // RuRAMPS4Duo v1.3 (Power outputs: Hotend0, Hotend1, Hotend2, Fan0, Fan1, Bed)//RuRAMPS4Duo v1.3（电源输出：Hotend0、Hotend1、Hotend2、Fan0、Fan1、Bed）
#define BOARD_ULTRATRONICS_PRO        3022  // ReprapWorld Ultratronics Pro V1.0//ReprapWorld Ultratronics Pro 1.0版
#define BOARD_ARCHIM1                 3023  // UltiMachine Archim1 (with DRV8825 drivers)//Ultimage Archim1（带DRV8825驱动程序）
#define BOARD_ARCHIM2                 3024  // UltiMachine Archim2 (with TMC2130 drivers)//多机Archim2（带TMC2130驱动器）
#define BOARD_ALLIGATOR               3025  // Alligator Board R2//鳄鱼板R2
#define BOARD_CNCONTROLS_15D          3026  // Cartesio CN Controls V15 on DUE//Cartesio CN在到期时控制V15
#define BOARD_KRATOS32                3027  // K.3D Kratos32 (Arduino Due Shield)//K.3D克瑞托斯32（阿杜伊诺因盾）

////
// SAM3X8C ARM Cortex M3//SAM3X8C臂皮质M3
////

#define BOARD_PRINTRBOARD_G2          3100  // PRINTRBOARD G2//印刷电路板G2
#define BOARD_ADSK                    3101  // Arduino DUE Shield Kit (ADSK)//Arduino到期屏蔽套件（ADSK）

////
// STM32 ARM Cortex-M3//STM32臂皮质-M3
////

#define BOARD_MALYAN_M200_V2          4000  // STM32F070CB controller//STM32F070CB控制器
#define BOARD_MALYAN_M300             4001  // STM32F070-based delta//基于STM32F070的增量
#define BOARD_STM32F103RE             4002  // STM32F103RE Libmaple-based STM32F1 controller//基于STM32F103RE Libmaple的STM32F1控制器
#define BOARD_MALYAN_M200             4003  // STM32C8T6 Libmaple-based STM32F1 controller//基于STM32C8T6 Libmaple的STM32F1控制器
#define BOARD_STM3R_MINI              4004  // STM32F103RE Libmaple-based STM32F1 controller//基于STM32F103RE Libmaple的STM32F1控制器
#define BOARD_GTM32_PRO_VB            4005  // STM32F103VET6 controller//STM32F103VET6控制器
#define BOARD_GTM32_MINI              4006  // STM32F103VET6 controller//STM32F103VET6控制器
#define BOARD_GTM32_MINI_A30          4007  // STM32F103VET6 controller//STM32F103VET6控制器
#define BOARD_GTM32_REV_B             4008  // STM32F103VET6 controller//STM32F103VET6控制器
#define BOARD_MORPHEUS                4009  // STM32F103C8 / STM32F103CB  Libmaple-based STM32F1 controller//基于STM32F103C8/STM32F103CB Libmaple的STM32F1控制器
#define BOARD_CHITU3D                 4010  // Chitu3D (STM32F103RET6)//Chitu3D（STM32F103RET6）
#define BOARD_MKS_ROBIN               4011  // MKS Robin (STM32F103ZET6)//MKS Robin（STM32F103ZET6）
#define BOARD_MKS_ROBIN_MINI          4012  // MKS Robin Mini (STM32F103VET6)//MKS罗宾迷你（STM32F103VET6）
#define BOARD_MKS_ROBIN_NANO          4013  // MKS Robin Nano (STM32F103VET6)//MKS Robin Nano（STM32F103VET6）
#define BOARD_MKS_ROBIN_NANO_V2       4014  // MKS Robin Nano V2 (STM32F103VET6)//MKS Robin Nano V2（STM32F103VET6）
#define BOARD_MKS_ROBIN_LITE          4015  // MKS Robin Lite/Lite2 (STM32F103RCT6)//MKS Robin Lite/Lite2（STM32F103RCT6）
#define BOARD_MKS_ROBIN_LITE3         4016  // MKS Robin Lite3 (STM32F103RCT6)//MKS Robin Lite3（STM32F103RCT6）
#define BOARD_MKS_ROBIN_PRO           4017  // MKS Robin Pro (STM32F103ZET6)//MKS Robin Pro（STM32F103ZET6）
#define BOARD_MKS_ROBIN_E3            4018  // MKS Robin E3 (STM32F103RCT6)//MKS Robin E3（STM32F103RCT6）
#define BOARD_MKS_ROBIN_E3_V1_1       4019  // MKS Robin E3 V1.1 (STM32F103RCT6)//MKS Robin E3 V1.1（STM32F103RCT6）
#define BOARD_MKS_ROBIN_E3D           4020  // MKS Robin E3D (STM32F103RCT6)//MKS Robin E3D（STM32F103RCT6）
#define BOARD_MKS_ROBIN_E3D_V1_1      4021  // MKS Robin E3D V1.1 (STM32F103RCT6)//MKS Robin E3D V1.1（STM32F103RCT6）
#define BOARD_MKS_ROBIN_E3P           4022  // MKS Robin E3p (STM32F103VET6)//MKS Robin E3p（STM32F103VET6）
#define BOARD_BTT_SKR_MINI_V1_1       4023  // BigTreeTech SKR Mini v1.1 (STM32F103RC)//BigTreeTech SKR Mini v1.1（STM32F103RC）
#define BOARD_BTT_SKR_MINI_E3_V1_0    4024  // BigTreeTech SKR Mini E3 (STM32F103RC)//BigTreeTech SKR迷你E3（STM32F103RC）
#define BOARD_BTT_SKR_MINI_E3_V1_2    4025  // BigTreeTech SKR Mini E3 V1.2 (STM32F103RC)//BigTreeTech SKR Mini E3 V1.2（STM32F103RC）
#define BOARD_BTT_SKR_MINI_E3_V2_0    4026  // BigTreeTech SKR Mini E3 V2.0 (STM32F103RC / STM32F103RE)//BigTreeTech SKR迷你E3 V2.0（STM32F103RC/STM32F103RE）
#define BOARD_BTT_SKR_MINI_MZ_V1_0    4027  // BigTreeTech SKR Mini MZ V1.0 (STM32F103RC)//BigTreeTech SKR迷你MZ V1.0（STM32F103RC）
#define BOARD_BTT_SKR_E3_DIP          4028  // BigTreeTech SKR E3 DIP V1.0 (STM32F103RC / STM32F103RE)//BigTreeTech SKR E3 DIP V1.0（STM32F103RC/STM32F103RE）
#define BOARD_BTT_SKR_CR6             4029  // BigTreeTech SKR CR6 v1.0 (STM32F103RE)//BigTreeTech SKR CR6 v1.0（STM32F103RE）
#define BOARD_JGAURORA_A5S_A1         4030  // JGAurora A5S A1 (STM32F103ZET6)//JGAurora A5S A1（STM32F103ZET6）
#define BOARD_FYSETC_AIO_II           4031  // FYSETC AIO_II//FYSETC AIO_II
#define BOARD_FYSETC_CHEETAH          4032  // FYSETC Cheetah//FYSETC猎豹
#define BOARD_FYSETC_CHEETAH_V12      4033  // FYSETC Cheetah V1.2//FYSETC猎豹V1.2
#define BOARD_LONGER3D_LK             4034  // Alfawise U20/U20+/U30 (Longer3D LK1/2) / STM32F103VET6//Alfawise U20/U20+/U30（长3D LK1/2）/STM32F103VET6
#define BOARD_CCROBOT_MEEB_3DP        4035  // ccrobot-online.com MEEB_3DP (STM32F103RC)//ccrobot-online.com MEEB_3DP（STM32F103RC）
#define BOARD_CHITU3D_V5              4036  // Chitu3D TronXY X5SA V5 Board//Chitu3D TronXY X5SA V5板
#define BOARD_CHITU3D_V6              4037  // Chitu3D TronXY X5SA V6 Board//Chitu3D TronXY X5SA V6板
#define BOARD_CREALITY_V4             4038  // Creality v4.x (STM32F103RE)//Creality v4.x（STM32F103RE）
#define BOARD_CREALITY_V427           4039  // Creality v4.2.7 (STM32F103RE)//Creality v4.2.7（STM32F103RE）
#define BOARD_CREALITY_V4210          4040  // Creality v4.2.10 (STM32F103RE) as found in the CR-30//CR-30中的Creality v4.2.10（STM32F103RE）
#define BOARD_CREALITY_V431           4041  // Creality v4.3.1 (STM32F103RE)//Creality v4.3.1（STM32F103RE）
#define BOARD_CREALITY_V452           4042  // Creality v4.5.2 (STM32F103RE)//Creality v4.5.2（STM32F103RE）
#define BOARD_CREALITY_V453           4043  // Creality v4.5.3 (STM32F103RE)//Creality v4.5.3（STM32F103RE）
#define BOARD_TRIGORILLA_PRO          4044  // Trigorilla Pro (STM32F103ZET6)//Trigorilla Pro（STM32F103ZET6）
#define BOARD_FLY_MINI                4045  // FLY MINI (STM32F103RCT6)//小型飞行（STM32F103RCT6）
#define BOARD_FLSUN_HISPEED           4046  // FLSUN HiSpeedV1 (STM32F103VET6)//FLSUN HiSpeedV1（STM32F103VET6）
#define BOARD_BEAST                   4047  // STM32F103RET6 Libmaple-based controller//基于STM32F103RET6 Libmaple的控制器
#define BOARD_MINGDA_MPX_ARM_MINI     4048  // STM32F103ZET6 Mingda MD-16//STM32F103ZET6明达MD-16
#define BOARD_GTM32_PRO_VD            4049  // STM32F103VET6 controller//STM32F103VET6控制器

////
// ARM Cortex-M4F//臂皮质-M4F
////

#define BOARD_TEENSY31_32             4100  // Teensy3.1 and Teensy3.2//第3.1节和第3.2节
#define BOARD_TEENSY35_36             4101  // Teensy3.5 and Teensy3.6//第3.5节和第3.6节

////
// STM32 ARM Cortex-M4F//STM32臂皮质-M4F
////

#define BOARD_ARMED                   4200  // Arm'ed STM32F4-based controller//基于Arm的STM32F4控制器
#define BOARD_RUMBA32_V1_0            4201  // RUMBA32 STM32F446VET6 based controller from Aus3D//Aus3D基于RUMBA32 STM32F446VET6的控制器
#define BOARD_RUMBA32_V1_1            4202  // RUMBA32 STM32F446VET6 based controller from Aus3D//Aus3D基于RUMBA32 STM32F446VET6的控制器
#define BOARD_RUMBA32_MKS             4203  // RUMBA32 STM32F446VET6 based controller from Makerbase//来自Makerbase的基于RUMBA32 STM32F446VET6的控制器
#define BOARD_BLACK_STM32F407VE       4204  // BLACK_STM32F407VE//黑色STM32F407VE
#define BOARD_BLACK_STM32F407ZE       4205  // BLACK_STM32F407ZE//黑色STM32F407ZE
#define BOARD_STEVAL_3DP001V1         4206  // STEVAL-3DP001V1 3D PRINTER BOARD//STEVAL-3DP001V1三维打印板
#define BOARD_BTT_SKR_PRO_V1_1        4207  // BigTreeTech SKR Pro v1.1 (STM32F407ZGT6)//BigTreeTech SKR Pro v1.1（STM32F407ZGT6）
#define BOARD_BTT_SKR_PRO_V1_2        4208  // BigTreeTech SKR Pro v1.2 (STM32F407ZGT6)//BigTreeTech SKR Pro v1.2（STM32F407ZGT6）
#define BOARD_BTT_BTT002_V1_0         4209  // BigTreeTech BTT002 v1.0 (STM32F407VGT6)//BigTreeTech BTT002 v1.0（STM32F407VGT6）
#define BOARD_BTT_E3_RRF              4210  // BigTreeTech E3 RRF (STM32F407VGT6)//BigTreeTech E3 RRF（STM32F407VGT6）
#define BOARD_BTT_SKR_V2_0_REV_A      4211  // BigTreeTech SKR v2.0 Rev A (STM32F407VGT6)//BigTreeTech SKR v2.0版本A（STM32F407VGT6）
#define BOARD_BTT_SKR_V2_0_REV_B      4212  // BigTreeTech SKR v2.0 Rev B (STM32F407VGT6)//BigTreeTech SKR v2.0版本B（STM32F407VGT6）
#define BOARD_BTT_GTR_V1_0            4213  // BigTreeTech GTR v1.0 (STM32F407IGT)//BigTreeTech全球技术法规v1.0（STM32F407IGT）
#define BOARD_BTT_OCTOPUS_V1_0        4214  // BigTreeTech Octopus v1.0 (STM32F446ZET6)//BigTreeTech章鱼v1.0（STM32F446ZET6）
#define BOARD_BTT_OCTOPUS_V1_1        4215  // BigTreeTech Octopus v1.1 (STM32F446ZET6)//BigTreeTech章鱼v1.1（STM32F446ZET6）
#define BOARD_LERDGE_K                4216  // Lerdge K (STM32F407ZG)//勒奇K（STM32F407ZG）
#define BOARD_LERDGE_S                4217  // Lerdge S (STM32F407VE)//勒奇S（STM32F407VE）
#define BOARD_LERDGE_X                4218  // Lerdge X (STM32F407VE)//Lerdge X（STM32F407VE）
#define BOARD_VAKE403D                4219  // VAkE 403D (STM32F446VET6)//VAkE 403D（STM32F4466）
#define BOARD_FYSETC_S6               4220  // FYSETC S6 (STM32F446VET6)//FYSETC S6（STM32F4466）
#define BOARD_FYSETC_S6_V2_0          4221  // FYSETC S6 v2.0 (STM32F446VET6)//FYSETC S6 v2.0（STM32F4466）
#define BOARD_FYSETC_SPIDER           4222  // FYSETC Spider (STM32F446VET6)//FYSETC蜘蛛（STM32F446VET6）
#define BOARD_FLYF407ZG               4223  // FLYF407ZG (STM32F407ZG)//FLYF407ZG（STM32F407ZG）
#define BOARD_MKS_ROBIN2              4224  // MKS_ROBIN2 (STM32F407ZE)//MKS_ROBIN2（STM32F407ZE）
#define BOARD_MKS_ROBIN_PRO_V2        4225  // MKS Robin Pro V2 (STM32F407VE)//MKS Robin Pro V2（STM32F407VE）
#define BOARD_MKS_ROBIN_NANO_V3       4226  // MKS Robin Nano V3 (STM32F407VG)//MKS Robin Nano V3（STM32F407VG）
#define BOARD_ANET_ET4                4227  // ANET ET4 V1.x (STM32F407VGT6)//ANET ET4 V1.x（STM32F407VGT6）
#define BOARD_ANET_ET4P               4228  // ANET ET4P V1.x (STM32F407VGT6)//ANET ET4P V1.x（STM32F407VGT6）
#define BOARD_FYSETC_CHEETAH_V20      4229  // FYSETC Cheetah V2.0//FYSETC猎豹V2.0


////
// ARM Cortex M7//臂皮质M7
////

#define BOARD_REMRAM_V1               5000  // RemRam v1//RemRam v1
#define BOARD_TEENSY41                5001  // Teensy 4.1//小4.1
#define BOARD_T41U5XBB                5002  // T41U5XBB Teensy 4.1 breakout board//T41U5XBB Teensy 4.1分接板
#define BOARD_NUCLEO_F767ZI           5003  // ST NUCLEO-F767ZI Dev Board//ST核仁-F767ZI开发板
#define BOARD_BTT_SKR_SE_BX           5004  // BigTreeTech SKR SE BX (STM32H743II)//BigTreeTech SKR SE BX（STM32H743II）

////
// Espressif ESP32 WiFi//ESPRESIF ESP32无线网络
////

#define BOARD_ESPRESSIF_ESP32         6000  // Generic ESP32//通用ESP32
#define BOARD_MRR_ESPA                6001  // MRR ESPA based on ESP32 (native pins only)//基于ESP32的MRR ESPA（仅限本机管脚）
#define BOARD_MRR_ESPE                6002  // MRR ESPE based on ESP32 (with I2S stepper stream)//基于ESP32的MRR ESPE（带I2S步进流）
#define BOARD_E4D_BOX                 6003  // E4d@BOX// E4d@BOX
#define BOARD_FYSETC_E4               6004  // FYSETC E4//FYSETC E4
#define BOARD_ESP32CONTROLLERR3       6005

////
// SAMD51 ARM Cortex M4//SAMD51手臂皮质M4
////

#define BOARD_AGCM4_RAMPS_144         6100  // RAMPS 1.4.4//坡道1.4.4

////
// Custom board//定制板
////

#define BOARD_CUSTOM                  9998  // Custom pins definition for development and/or rare boards//开发板和/或稀有板的自定义管脚定义

////
// Simulations//模拟
////

#define BOARD_LINUX_RAMPS             9999

#define _MB_1(B)  (defined(BOARD_##B) && MOTHERBOARD==BOARD_##B)
#define MB(V...)  DO(MB,||,V)
