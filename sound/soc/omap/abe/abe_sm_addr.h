/*
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2010-2011 Texas Instruments Incorporated,
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2010-2011 Texas Instruments Incorporated,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _ABE_SM_ADDR_H_
#define _ABE_SM_ADDR_H_
#define init_SM_ADDR                                        0
#define init_SM_ADDR_END                                    320
#define init_SM_sizeof                                      321
#define S_Data0_ADDR                                        321
#define S_Data0_ADDR_END                                    321
#define S_Data0_sizeof                                      1
#define S_Temp_ADDR                                         322
#define S_Temp_ADDR_END                                     322
#define S_Temp_sizeof                                       1
#define S_PhoenixOffset_ADDR                                323
#define S_PhoenixOffset_ADDR_END                            323
#define S_PhoenixOffset_sizeof                              1
#define S_GTarget1_ADDR                                     324
#define S_GTarget1_ADDR_END                                 330
#define S_GTarget1_sizeof                                   7
#define S_Gtarget_DL1_ADDR                                  331
#define S_Gtarget_DL1_ADDR_END                              332
#define S_Gtarget_DL1_sizeof                                2
#define S_Gtarget_DL2_ADDR                                  333
#define S_Gtarget_DL2_ADDR_END                              334
#define S_Gtarget_DL2_sizeof                                2
#define S_Gtarget_Echo_ADDR                                 335
#define S_Gtarget_Echo_ADDR_END                             335
#define S_Gtarget_Echo_sizeof                               1
#define S_Gtarget_SDT_ADDR                                  336
#define S_Gtarget_SDT_ADDR_END                              336
#define S_Gtarget_SDT_sizeof                                1
#define S_Gtarget_VxRec_ADDR                                337
#define S_Gtarget_VxRec_ADDR_END                            338
#define S_Gtarget_VxRec_sizeof                              2
#define S_Gtarget_UL_ADDR                                   339
#define S_Gtarget_UL_ADDR_END                               340
#define S_Gtarget_UL_sizeof                                 2
#define S_Gtarget_BTUL_ADDR                                 341
#define S_Gtarget_BTUL_ADDR_END                             341
#define S_Gtarget_BTUL_sizeof                               1
#define S_GCurrent_ADDR                                     342
#define S_GCurrent_ADDR_END                                 359
#define S_GCurrent_sizeof                                   18
#define S_GAIN_ONE_ADDR                                     360
#define S_GAIN_ONE_ADDR_END                                 360
#define S_GAIN_ONE_sizeof                                   1
#define S_Tones_ADDR                                        361
#define S_Tones_ADDR_END                                    372
#define S_Tones_sizeof                                      12
#define S_VX_DL_ADDR                                        373
#define S_VX_DL_ADDR_END                                    384
#define S_VX_DL_sizeof                                      12
#define S_MM_UL2_ADDR                                       385
#define S_MM_UL2_ADDR_END                                   396
#define S_MM_UL2_sizeof                                     12
#define S_MM_DL_ADDR                                        397
#define S_MM_DL_ADDR_END                                    408
#define S_MM_DL_sizeof                                      12
#define S_DL1_M_Out_ADDR                                    409
#define S_DL1_M_Out_ADDR_END                                420
#define S_DL1_M_Out_sizeof                                  12
#define S_DL2_M_Out_ADDR                                    421
#define S_DL2_M_Out_ADDR_END                                432
#define S_DL2_M_Out_sizeof                                  12
#define S_Echo_M_Out_ADDR                                   433
#define S_Echo_M_Out_ADDR_END                               444
#define S_Echo_M_Out_sizeof                                 12
#define S_SDT_M_Out_ADDR                                    445
#define S_SDT_M_Out_ADDR_END                                456
#define S_SDT_M_Out_sizeof                                  12
#define S_VX_UL_ADDR                                        457
#define S_VX_UL_ADDR_END                                    468
#define S_VX_UL_sizeof                                      12
#define S_VX_UL_M_ADDR                                      469
#define S_VX_UL_M_ADDR_END                                  480
#define S_VX_UL_M_sizeof                                    12
#define S_BT_DL_ADDR                                        481
#define S_BT_DL_ADDR_END                                    492
#define S_BT_DL_sizeof                                      12
#define S_BT_UL_ADDR                                        493
#define S_BT_UL_ADDR_END                                    504
#define S_BT_UL_sizeof                                      12
#define S_BT_DL_8k_ADDR                                     505
#define S_BT_DL_8k_ADDR_END                                 507
#define S_BT_DL_8k_sizeof                                   3
#define S_BT_DL_16k_ADDR                                    508
#define S_BT_DL_16k_ADDR_END                                512
#define S_BT_DL_16k_sizeof                                  5
#define S_BT_UL_8k_ADDR                                     513
#define S_BT_UL_8k_ADDR_END                                 514
#define S_BT_UL_8k_sizeof                                   2
#define S_BT_UL_16k_ADDR                                    515
#define S_BT_UL_16k_ADDR_END                                518
#define S_BT_UL_16k_sizeof                                  4
#define S_SDT_F_ADDR                                        519
#define S_SDT_F_ADDR_END                                    530
#define S_SDT_F_sizeof                                      12
#define S_SDT_F_data_ADDR                                   531
#define S_SDT_F_data_ADDR_END                               539
#define S_SDT_F_data_sizeof                                 9
#define S_MM_DL_OSR_ADDR                                    540
#define S_MM_DL_OSR_ADDR_END                                563
#define S_MM_DL_OSR_sizeof                                  24
#define S_24_zeros_ADDR                                     564
#define S_24_zeros_ADDR_END                                 587
#define S_24_zeros_sizeof                                   24
#define S_DMIC1_ADDR                                        588
#define S_DMIC1_ADDR_END                                    599
#define S_DMIC1_sizeof                                      12
#define S_DMIC2_ADDR                                        600
#define S_DMIC2_ADDR_END                                    611
#define S_DMIC2_sizeof                                      12
#define S_DMIC3_ADDR                                        612
#define S_DMIC3_ADDR_END                                    623
#define S_DMIC3_sizeof                                      12
#define S_AMIC_ADDR                                         624
#define S_AMIC_ADDR_END                                     635
#define S_AMIC_sizeof                                       12
#define S_DMIC1_L_ADDR                                      636
#define S_DMIC1_L_ADDR_END                                  647
#define S_DMIC1_L_sizeof                                    12
#define S_DMIC1_R_ADDR                                      648
#define S_DMIC1_R_ADDR_END                                  659
#define S_DMIC1_R_sizeof                                    12
#define S_DMIC2_L_ADDR                                      660
#define S_DMIC2_L_ADDR_END                                  671
#define S_DMIC2_L_sizeof                                    12
#define S_DMIC2_R_ADDR                                      672
#define S_DMIC2_R_ADDR_END                                  683
#define S_DMIC2_R_sizeof                                    12
#define S_DMIC3_L_ADDR                                      684
#define S_DMIC3_L_ADDR_END                                  695
#define S_DMIC3_L_sizeof                                    12
#define S_DMIC3_R_ADDR                                      696
#define S_DMIC3_R_ADDR_END                                  707
#define S_DMIC3_R_sizeof                                    12
#define S_BT_UL_L_ADDR                                      708
#define S_BT_UL_L_ADDR_END                                  719
#define S_BT_UL_L_sizeof                                    12
#define S_BT_UL_R_ADDR                                      720
#define S_BT_UL_R_ADDR_END                                  731
#define S_BT_UL_R_sizeof                                    12
#define S_AMIC_L_ADDR                                       732
#define S_AMIC_L_ADDR_END                                   743
#define S_AMIC_L_sizeof                                     12
#define S_AMIC_R_ADDR                                       744
#define S_AMIC_R_ADDR_END                                   755
#define S_AMIC_R_sizeof                                     12
#define S_EchoRef_L_ADDR                                    756
#define S_EchoRef_L_ADDR_END                                767
#define S_EchoRef_L_sizeof                                  12
#define S_EchoRef_R_ADDR                                    768
#define S_EchoRef_R_ADDR_END                                779
#define S_EchoRef_R_sizeof                                  12
#define S_MM_DL_L_ADDR                                      780
#define S_MM_DL_L_ADDR_END                                  791
#define S_MM_DL_L_sizeof                                    12
#define S_MM_DL_R_ADDR                                      792
#define S_MM_DL_R_ADDR_END                                  803
#define S_MM_DL_R_sizeof                                    12
#define S_MM_UL_ADDR                                        804
#define S_MM_UL_ADDR_END                                    923
#define S_MM_UL_sizeof                                      120
#define S_AMIC_96k_ADDR                                     924
#define S_AMIC_96k_ADDR_END                                 947
#define S_AMIC_96k_sizeof                                   24
#define S_DMIC0_96k_ADDR                                    948
#define S_DMIC0_96k_ADDR_END                                971
#define S_DMIC0_96k_sizeof                                  24
#define S_DMIC1_96k_ADDR                                    972
#define S_DMIC1_96k_ADDR_END                                995
#define S_DMIC1_96k_sizeof                                  24
#define S_DMIC2_96k_ADDR                                    996
#define S_DMIC2_96k_ADDR_END                                1019
#define S_DMIC2_96k_sizeof                                  24
#define S_UL_VX_UL_48_8K_ADDR                               1020
#define S_UL_VX_UL_48_8K_ADDR_END                           1031
#define S_UL_VX_UL_48_8K_sizeof                             12
#define S_UL_VX_UL_48_16K_ADDR                              1032
#define S_UL_VX_UL_48_16K_ADDR_END                          1043
#define S_UL_VX_UL_48_16K_sizeof                            12
#define S_UL_MIC_48K_ADDR                                   1044
#define S_UL_MIC_48K_ADDR_END                               1055
#define S_UL_MIC_48K_sizeof                                 12
#define S_Voice_8k_UL_ADDR                                  1056
#define S_Voice_8k_UL_ADDR_END                              1058
#define S_Voice_8k_UL_sizeof                                3
#define S_Voice_8k_DL_ADDR                                  1059
#define S_Voice_8k_DL_ADDR_END                              1060
#define S_Voice_8k_DL_sizeof                                2
#define S_McPDM_Out1_ADDR                                   1061
#define S_McPDM_Out1_ADDR_END                               1084
#define S_McPDM_Out1_sizeof                                 24
#define S_McPDM_Out2_ADDR                                   1085
#define S_McPDM_Out2_ADDR_END                               1108
#define S_McPDM_Out2_sizeof                                 24
#define S_McPDM_Out3_ADDR                                   1109
#define S_McPDM_Out3_ADDR_END                               1132
#define S_McPDM_Out3_sizeof                                 24
#define S_Voice_16k_UL_ADDR                                 1133
#define S_Voice_16k_UL_ADDR_END                             1137
#define S_Voice_16k_UL_sizeof                               5
#define S_Voice_16k_DL_ADDR                                 1138
#define S_Voice_16k_DL_ADDR_END                             1141
#define S_Voice_16k_DL_sizeof                               4
#define S_XinASRC_DL_VX_ADDR                                1142
#define S_XinASRC_DL_VX_ADDR_END                            1181
#define S_XinASRC_DL_VX_sizeof                              40
#define S_XinASRC_UL_VX_ADDR                                1182
#define S_XinASRC_UL_VX_ADDR_END                            1221
#define S_XinASRC_UL_VX_sizeof                              40
#define S_XinASRC_MM_EXT_IN_ADDR                            1222
#define S_XinASRC_MM_EXT_IN_ADDR_END                        1261
#define S_XinASRC_MM_EXT_IN_sizeof                          40
#define S_VX_REC_ADDR                                       1262
#define S_VX_REC_ADDR_END                                   1273
#define S_VX_REC_sizeof                                     12
#define S_VX_REC_L_ADDR                                     1274
#define S_VX_REC_L_ADDR_END                                 1285
#define S_VX_REC_L_sizeof                                   12
#define S_VX_REC_R_ADDR                                     1286
#define S_VX_REC_R_ADDR_END                                 1297
#define S_VX_REC_R_sizeof                                   12
#define S_DL2_M_L_ADDR                                      1298
#define S_DL2_M_L_ADDR_END                                  1309
#define S_DL2_M_L_sizeof                                    12
#define S_DL2_M_R_ADDR                                      1310
#define S_DL2_M_R_ADDR_END                                  1321
#define S_DL2_M_R_sizeof                                    12
#define S_DL2_M_LR_EQ_data_ADDR                             1322
#define S_DL2_M_LR_EQ_data_ADDR_END                         1346
#define S_DL2_M_LR_EQ_data_sizeof                           25
#define S_DL1_M_EQ_data_ADDR                                1347
#define S_DL1_M_EQ_data_ADDR_END                            1371
#define S_DL1_M_EQ_data_sizeof                              25
#define S_EARP_48_96_LP_data_ADDR                           1372
#define S_EARP_48_96_LP_data_ADDR_END                       1386
#define S_EARP_48_96_LP_data_sizeof                         15
#define S_IHF_48_96_LP_data_ADDR                            1387
#define S_IHF_48_96_LP_data_ADDR_END                        1401
#define S_IHF_48_96_LP_data_sizeof                          15
#define S_VX_UL_8_TEMP_ADDR                                 1402
#define S_VX_UL_8_TEMP_ADDR_END                             1403
#define S_VX_UL_8_TEMP_sizeof                               2
#define S_VX_UL_16_TEMP_ADDR                                1404
#define S_VX_UL_16_TEMP_ADDR_END                            1407
#define S_VX_UL_16_TEMP_sizeof                              4
#define S_VX_DL_8_48_LP_data_ADDR                           1408
#define S_VX_DL_8_48_LP_data_ADDR_END                       1420
#define S_VX_DL_8_48_LP_data_sizeof                         13
#define S_VX_DL_8_48_HP_data_ADDR                           1421
#define S_VX_DL_8_48_HP_data_ADDR_END                       1427
#define S_VX_DL_8_48_HP_data_sizeof                         7
#define S_VX_DL_16_48_LP_data_ADDR                          1428
#define S_VX_DL_16_48_LP_data_ADDR_END                      1440
#define S_VX_DL_16_48_LP_data_sizeof                        13
#define S_VX_DL_16_48_HP_data_ADDR                          1441
#define S_VX_DL_16_48_HP_data_ADDR_END                      1445
#define S_VX_DL_16_48_HP_data_sizeof                        5
#define S_VX_UL_48_8_LP_data_ADDR                           1446
#define S_VX_UL_48_8_LP_data_ADDR_END                       1458
#define S_VX_UL_48_8_LP_data_sizeof                         13
#define S_VX_UL_48_8_HP_data_ADDR                           1459
#define S_VX_UL_48_8_HP_data_ADDR_END                       1465
#define S_VX_UL_48_8_HP_data_sizeof                         7
#define S_VX_UL_48_16_LP_data_ADDR                          1466
#define S_VX_UL_48_16_LP_data_ADDR_END                      1478
#define S_VX_UL_48_16_LP_data_sizeof                        13
#define S_VX_UL_48_16_HP_data_ADDR                          1479
#define S_VX_UL_48_16_HP_data_ADDR_END                      1483
#define S_VX_UL_48_16_HP_data_sizeof                        5
#define S_BT_UL_8_48_LP_data_ADDR                           1484
#define S_BT_UL_8_48_LP_data_ADDR_END                       1496
#define S_BT_UL_8_48_LP_data_sizeof                         13
#define S_BT_UL_8_48_HP_data_ADDR                           1497
#define S_BT_UL_8_48_HP_data_ADDR_END                       1503
#define S_BT_UL_8_48_HP_data_sizeof                         7
#define S_BT_UL_16_48_LP_data_ADDR                          1504
#define S_BT_UL_16_48_LP_data_ADDR_END                      1516
#define S_BT_UL_16_48_LP_data_sizeof                        13
#define S_BT_UL_16_48_HP_data_ADDR                          1517
#define S_BT_UL_16_48_HP_data_ADDR_END                      1521
#define S_BT_UL_16_48_HP_data_sizeof                        5
#define S_BT_DL_48_8_LP_data_ADDR                           1522
#define S_BT_DL_48_8_LP_data_ADDR_END                       1534
#define S_BT_DL_48_8_LP_data_sizeof                         13
#define S_BT_DL_48_8_HP_data_ADDR                           1535
#define S_BT_DL_48_8_HP_data_ADDR_END                       1541
#define S_BT_DL_48_8_HP_data_sizeof                         7
#define S_BT_DL_48_16_LP_data_ADDR                          1542
#define S_BT_DL_48_16_LP_data_ADDR_END                      1554
#define S_BT_DL_48_16_LP_data_sizeof                        13
#define S_BT_DL_48_16_HP_data_ADDR                          1555
#define S_BT_DL_48_16_HP_data_ADDR_END                      1559
#define S_BT_DL_48_16_HP_data_sizeof                        5
#define S_ECHO_REF_48_8_LP_data_ADDR                        1560
#define S_ECHO_REF_48_8_LP_data_ADDR_END                    1572
#define S_ECHO_REF_48_8_LP_data_sizeof                      13
#define S_ECHO_REF_48_8_HP_data_ADDR                        1573
#define S_ECHO_REF_48_8_HP_data_ADDR_END                    1579
#define S_ECHO_REF_48_8_HP_data_sizeof                      7
#define S_ECHO_REF_48_16_LP_data_ADDR                       1580
#define S_ECHO_REF_48_16_LP_data_ADDR_END                   1592
#define S_ECHO_REF_48_16_LP_data_sizeof                     13
#define S_ECHO_REF_48_16_HP_data_ADDR                       1593
#define S_ECHO_REF_48_16_HP_data_ADDR_END                   1597
#define S_ECHO_REF_48_16_HP_data_sizeof                     5
#define S_APS_IIRmem1_ADDR                                  1598
#define S_APS_IIRmem1_ADDR_END                              1606
#define S_APS_IIRmem1_sizeof                                9
#define S_APS_M_IIRmem2_ADDR                                1607
#define S_APS_M_IIRmem2_ADDR_END                            1609
#define S_APS_M_IIRmem2_sizeof                              3
#define S_APS_C_IIRmem2_ADDR                                1610
#define S_APS_C_IIRmem2_ADDR_END                            1612
#define S_APS_C_IIRmem2_sizeof                              3
#define S_APS_DL1_OutSamples_ADDR                           1613
#define S_APS_DL1_OutSamples_ADDR_END                       1614
#define S_APS_DL1_OutSamples_sizeof                         2
#define S_APS_DL1_COIL_OutSamples_ADDR                      1615
#define S_APS_DL1_COIL_OutSamples_ADDR_END                  1616
#define S_APS_DL1_COIL_OutSamples_sizeof                    2
#define S_APS_DL2_L_OutSamples_ADDR                         1617
#define S_APS_DL2_L_OutSamples_ADDR_END                     1618
#define S_APS_DL2_L_OutSamples_sizeof                       2
#define S_APS_DL2_L_COIL_OutSamples_ADDR                    1619
#define S_APS_DL2_L_COIL_OutSamples_ADDR_END                1620
#define S_APS_DL2_L_COIL_OutSamples_sizeof                  2
#define S_APS_DL2_R_OutSamples_ADDR                         1621
#define S_APS_DL2_R_OutSamples_ADDR_END                     1622
#define S_APS_DL2_R_OutSamples_sizeof                       2
#define S_APS_DL2_R_COIL_OutSamples_ADDR                    1623
#define S_APS_DL2_R_COIL_OutSamples_ADDR_END                1624
#define S_APS_DL2_R_COIL_OutSamples_sizeof                  2
#define S_XinASRC_ECHO_REF_ADDR                             1625
#define S_XinASRC_ECHO_REF_ADDR_END                         1664
#define S_XinASRC_ECHO_REF_sizeof                           40
#define S_ECHO_REF_16K_ADDR                                 1665
#define S_ECHO_REF_16K_ADDR_END                             1669
#define S_ECHO_REF_16K_sizeof                               5
#define S_ECHO_REF_8K_ADDR                                  1670
#define S_ECHO_REF_8K_ADDR_END                              1672
#define S_ECHO_REF_8K_sizeof                                3
#define S_DL1_EQ_ADDR                                       1673
#define S_DL1_EQ_ADDR_END                                   1684
#define S_DL1_EQ_sizeof                                     12
#define S_DL2_EQ_ADDR                                       1685
#define S_DL2_EQ_ADDR_END                                   1696
#define S_DL2_EQ_sizeof                                     12
#define S_DL1_GAIN_out_ADDR                                 1697
#define S_DL1_GAIN_out_ADDR_END                             1708
#define S_DL1_GAIN_out_sizeof                               12
#define S_DL2_GAIN_out_ADDR                                 1709
#define S_DL2_GAIN_out_ADDR_END                             1720
#define S_DL2_GAIN_out_sizeof                               12
#define S_APS_DL2_L_IIRmem1_ADDR                            1721
#define S_APS_DL2_L_IIRmem1_ADDR_END                        1729
#define S_APS_DL2_L_IIRmem1_sizeof                          9
#define S_APS_DL2_R_IIRmem1_ADDR                            1730
#define S_APS_DL2_R_IIRmem1_ADDR_END                        1738
#define S_APS_DL2_R_IIRmem1_sizeof                          9
#define S_APS_DL2_L_M_IIRmem2_ADDR                          1739
#define S_APS_DL2_L_M_IIRmem2_ADDR_END                      1741
#define S_APS_DL2_L_M_IIRmem2_sizeof                        3
#define S_APS_DL2_R_M_IIRmem2_ADDR                          1742
#define S_APS_DL2_R_M_IIRmem2_ADDR_END                      1744
#define S_APS_DL2_R_M_IIRmem2_sizeof                        3
#define S_APS_DL2_L_C_IIRmem2_ADDR                          1745
#define S_APS_DL2_L_C_IIRmem2_ADDR_END                      1747
#define S_APS_DL2_L_C_IIRmem2_sizeof                        3
#define S_APS_DL2_R_C_IIRmem2_ADDR                          1748
#define S_APS_DL2_R_C_IIRmem2_ADDR_END                      1750
#define S_APS_DL2_R_C_IIRmem2_sizeof                        3
#define S_DL1_APS_ADDR                                      1751
#define S_DL1_APS_ADDR_END                                  1762
#define S_DL1_APS_sizeof                                    12
#define S_DL2_L_APS_ADDR                                    1763
#define S_DL2_L_APS_ADDR_END                                1774
#define S_DL2_L_APS_sizeof                                  12
#define S_DL2_R_APS_ADDR                                    1775
#define S_DL2_R_APS_ADDR_END                                1786
#define S_DL2_R_APS_sizeof                                  12
#define S_APS_DL1_EQ_data_ADDR                              1787
#define S_APS_DL1_EQ_data_ADDR_END                          1795
#define S_APS_DL1_EQ_data_sizeof                            9
#define S_APS_DL2_EQ_data_ADDR                              1796
#define S_APS_DL2_EQ_data_ADDR_END                          1804
#define S_APS_DL2_EQ_data_sizeof                            9
#define S_DC_DCvalue_ADDR                                   1805
#define S_DC_DCvalue_ADDR_END                               1805
#define S_DC_DCvalue_sizeof                                 1
#define S_VIBRA_ADDR                                        1806
#define S_VIBRA_ADDR_END                                    1811
#define S_VIBRA_sizeof                                      6
#define S_Vibra2_in_ADDR                                    1812
#define S_Vibra2_in_ADDR_END                                1817
#define S_Vibra2_in_sizeof                                  6
#define S_Vibra2_addr_ADDR                                  1818
#define S_Vibra2_addr_ADDR_END                              1818
#define S_Vibra2_addr_sizeof                                1
#define S_VibraCtrl_forRightSM_ADDR                         1819
#define S_VibraCtrl_forRightSM_ADDR_END                     1842
#define S_VibraCtrl_forRightSM_sizeof                       24
#define S_Rnoise_mem_ADDR                                   1843
#define S_Rnoise_mem_ADDR_END                               1843
#define S_Rnoise_mem_sizeof                                 1
#define S_Ctrl_ADDR                                         1844
#define S_Ctrl_ADDR_END                                     1861
#define S_Ctrl_sizeof                                       18
#define S_Vibra1_in_ADDR                                    1862
#define S_Vibra1_in_ADDR_END                                1867
#define S_Vibra1_in_sizeof                                  6
#define S_Vibra1_temp_ADDR                                  1868
#define S_Vibra1_temp_ADDR_END                              1891
#define S_Vibra1_temp_sizeof                                24
#define S_VibraCtrl_forLeftSM_ADDR                          1892
#define S_VibraCtrl_forLeftSM_ADDR_END                      1915
#define S_VibraCtrl_forLeftSM_sizeof                        24
#define S_Vibra1_mem_ADDR                                   1916
#define S_Vibra1_mem_ADDR_END                               1926
#define S_Vibra1_mem_sizeof                                 11
#define S_VibraCtrl_Stereo_ADDR                             1927
#define S_VibraCtrl_Stereo_ADDR_END                         1950
#define S_VibraCtrl_Stereo_sizeof                           24
#define S_AMIC_96_48_data_ADDR                              1951
#define S_AMIC_96_48_data_ADDR_END                          1969
#define S_AMIC_96_48_data_sizeof                            19
#define S_DMIC0_96_48_data_ADDR                             1970
#define S_DMIC0_96_48_data_ADDR_END                         1988
#define S_DMIC0_96_48_data_sizeof                           19
#define S_DMIC1_96_48_data_ADDR                             1989
#define S_DMIC1_96_48_data_ADDR_END                         2007
#define S_DMIC1_96_48_data_sizeof                           19
#define S_DMIC2_96_48_data_ADDR                             2008
#define S_DMIC2_96_48_data_ADDR_END                         2026
#define S_DMIC2_96_48_data_sizeof                           19
#define S_DBG_8K_PATTERN_ADDR                               2027
#define S_DBG_8K_PATTERN_ADDR_END                           2028
#define S_DBG_8K_PATTERN_sizeof                             2
#define S_DBG_16K_PATTERN_ADDR                              2029
#define S_DBG_16K_PATTERN_ADDR_END                          2032
#define S_DBG_16K_PATTERN_sizeof                            4
#define S_DBG_24K_PATTERN_ADDR                              2033
#define S_DBG_24K_PATTERN_ADDR_END                          2038
#define S_DBG_24K_PATTERN_sizeof                            6
#define S_DBG_48K_PATTERN_ADDR                              2039
#define S_DBG_48K_PATTERN_ADDR_END                          2050
#define S_DBG_48K_PATTERN_sizeof                            12
#define S_DBG_96K_PATTERN_ADDR                              2051
#define S_DBG_96K_PATTERN_ADDR_END                          2074
#define S_DBG_96K_PATTERN_sizeof                            24
#define S_MM_EXT_IN_ADDR                                    2075
#define S_MM_EXT_IN_ADDR_END                                2086
#define S_MM_EXT_IN_sizeof                                  12
#define S_MM_EXT_IN_L_ADDR                                  2087
#define S_MM_EXT_IN_L_ADDR_END                              2098
#define S_MM_EXT_IN_L_sizeof                                12
#define S_MM_EXT_IN_R_ADDR                                  2099
#define S_MM_EXT_IN_R_ADDR_END                              2110
#define S_MM_EXT_IN_R_sizeof                                12
#define S_MIC4_ADDR                                         2111
#define S_MIC4_ADDR_END                                     2122
#define S_MIC4_sizeof                                       12
#define S_MIC4_L_ADDR                                       2123
#define S_MIC4_L_ADDR_END                                   2134
#define S_MIC4_L_sizeof                                     12
#define S_MIC4_R_ADDR                                       2135
#define S_MIC4_R_ADDR_END                                   2146
#define S_MIC4_R_sizeof                                     12
#define S_HW_TEST_ADDR                                      2147
#define S_HW_TEST_ADDR_END                                  2147
#define S_HW_TEST_sizeof                                    1
#define S_XinASRC_BT_UL_ADDR                                2148
#define S_XinASRC_BT_UL_ADDR_END                            2187
#define S_XinASRC_BT_UL_sizeof                              40
#define S_XinASRC_BT_DL_ADDR                                2188
#define S_XinASRC_BT_DL_ADDR_END                            2227
#define S_XinASRC_BT_DL_sizeof                              40
#define S_BT_DL_8k_TEMP_ADDR                                2228
#define S_BT_DL_8k_TEMP_ADDR_END                            2229
#define S_BT_DL_8k_TEMP_sizeof                              2
#define S_BT_DL_16k_TEMP_ADDR                               2230
#define S_BT_DL_16k_TEMP_ADDR_END                           2233
#define S_BT_DL_16k_TEMP_sizeof                             4
#define S_VX_DL_8_48_OSR_LP_data_ADDR                       2234
#define S_VX_DL_8_48_OSR_LP_data_ADDR_END                   2261
#define S_VX_DL_8_48_OSR_LP_data_sizeof                     28
#define S_sat4_ADDR                                         2262
#define S_sat4_ADDR_END                                     2262
#define S_sat4_sizeof                                       1
#define S_BT_DL_48_8_LP_NEW_data_ADDR                       2263
#define S_BT_DL_48_8_LP_NEW_data_ADDR_END                   2279
#define S_BT_DL_48_8_LP_NEW_data_sizeof                     17
#define S_BT_DL_8_48_OSR_LP_data_ADDR                       2280
#define S_BT_DL_8_48_OSR_LP_data_ADDR_END                   2400
#define S_BT_DL_8_48_OSR_LP_data_sizeof                     121
#endif /* _ABESM_ADDR_H_ */
