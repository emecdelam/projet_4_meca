#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Mon Mar 24 20:25:47 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 28
#
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S18 = sin(q[18])
    C18 = cos(q[18])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    ROlp2_25 = S22*S23
    ROlp2_35 = -C22*S23
    ROlp2_85 = -S22*C23
    ROlp2_95 = C22*C23
    ROlp2_16 = C23*C24
    ROlp2_26 = ROlp2_25*C24+C22*S24
    ROlp2_36 = ROlp2_35*C24+S22*S24
    ROlp2_46 = -C23*S24
    ROlp2_56 = -ROlp2_25*S24+C22*C24
    ROlp2_66 = -ROlp2_35*S24+S22*C24
    ROlp2_17 = ROlp2_16*C25-S23*S25
    ROlp2_27 = ROlp2_26*C25-ROlp2_85*S25
    ROlp2_37 = ROlp2_36*C25-ROlp2_95*S25
    POlp2_12 = q[20]+s.dpt[1,13]
    RLlp2_17 = ROlp2_46*s.dpt[2,32]+s.dpt[3,32]*S23
    RLlp2_27 = ROlp2_56*s.dpt[2,32]+ROlp2_85*s.dpt[3,32]
    RLlp2_37 = ROlp2_66*s.dpt[2,32]+ROlp2_95*s.dpt[3,32]
    POlp2_17 = POlp2_12+RLlp2_17
    POlp2_27 = RLlp2_27+q[21]
    POlp2_37 = RLlp2_37+q[19]
    JTlp2_17_5 = -RLlp2_27*S22+RLlp2_37*C22
    JTlp2_27_5 = RLlp2_17*S22
    JTlp2_37_5 = -RLlp2_17*C22
    JTlp2_17_6 = -RLlp2_27*ROlp2_95+RLlp2_37*ROlp2_85
    JTlp2_27_6 = RLlp2_17*ROlp2_95-RLlp2_37*S23
    JTlp2_37_6 = -RLlp2_17*ROlp2_85+RLlp2_27*S23
    RLlp2_18 = ROlp2_17*s.dpt[1,40]
    RLlp2_28 = ROlp2_27*s.dpt[1,40]
    RLlp2_38 = ROlp2_37*s.dpt[1,40]
    POlp2_18 = POlp2_17+RLlp2_18
    POlp2_28 = POlp2_27+RLlp2_28
    POlp2_38 = POlp2_37+RLlp2_38
    JTlp2_28_4 = -RLlp2_37-RLlp2_38
    JTlp2_38_4 = RLlp2_27+RLlp2_28
    JTlp2_18_5 = JTlp2_17_5-RLlp2_28*S22+RLlp2_38*C22
    JTlp2_28_5 = JTlp2_27_5+RLlp2_18*S22
    JTlp2_38_5 = JTlp2_37_5-RLlp2_18*C22
    JTlp2_18_6 = JTlp2_17_6-RLlp2_28*ROlp2_95+RLlp2_38*ROlp2_85
    JTlp2_28_6 = JTlp2_27_6+RLlp2_18*ROlp2_95-RLlp2_38*S23
    JTlp2_38_6 = JTlp2_37_6-RLlp2_18*ROlp2_85+RLlp2_28*S23
    JTlp2_18_7 = -RLlp2_28*ROlp2_66+RLlp2_38*ROlp2_56
    JTlp2_28_7 = RLlp2_18*ROlp2_66-RLlp2_38*ROlp2_46
    JTlp2_38_7 = -RLlp2_18*ROlp2_56+RLlp2_28*ROlp2_46
    h_1 = -POlp2_18+s.dpt[1,9]
    h_2 = -POlp2_28+s.dpt[2,9]
    h_3 = -POlp2_38+s.dpt[3,9]
    ROlp3_25 = S22*S23
    ROlp3_35 = -C22*S23
    ROlp3_85 = -S22*C23
    ROlp3_95 = C22*C23
    ROlp3_16 = C23*C24
    ROlp3_26 = ROlp3_25*C24+C22*S24
    ROlp3_36 = ROlp3_35*C24+S22*S24
    ROlp3_46 = -C23*S24
    ROlp3_56 = -ROlp3_25*S24+C22*C24
    ROlp3_66 = -ROlp3_35*S24+S22*C24
    ROlp3_17 = ROlp3_16*C26-S23*S26
    ROlp3_27 = ROlp3_26*C26-ROlp3_85*S26
    ROlp3_37 = ROlp3_36*C26-ROlp3_95*S26
    POlp3_12 = q[20]+s.dpt[1,13]
    RLlp3_17 = ROlp3_46*s.dpt[2,35]+s.dpt[3,35]*S23
    RLlp3_27 = ROlp3_56*s.dpt[2,35]+ROlp3_85*s.dpt[3,35]
    RLlp3_37 = ROlp3_66*s.dpt[2,35]+ROlp3_95*s.dpt[3,35]
    POlp3_17 = POlp3_12+RLlp3_17
    POlp3_27 = RLlp3_27+q[21]
    POlp3_37 = RLlp3_37+q[19]
    JTlp3_17_5 = -RLlp3_27*S22+RLlp3_37*C22
    JTlp3_27_5 = RLlp3_17*S22
    JTlp3_37_5 = -RLlp3_17*C22
    JTlp3_17_6 = -RLlp3_27*ROlp3_95+RLlp3_37*ROlp3_85
    JTlp3_27_6 = RLlp3_17*ROlp3_95-RLlp3_37*S23
    JTlp3_37_6 = -RLlp3_17*ROlp3_85+RLlp3_27*S23
    RLlp3_18 = ROlp3_17*s.dpt[1,41]
    RLlp3_28 = ROlp3_27*s.dpt[1,41]
    RLlp3_38 = ROlp3_37*s.dpt[1,41]
    POlp3_18 = POlp3_17+RLlp3_18
    POlp3_28 = POlp3_27+RLlp3_28
    POlp3_38 = POlp3_37+RLlp3_38
    JTlp3_28_4 = -RLlp3_37-RLlp3_38
    JTlp3_38_4 = RLlp3_27+RLlp3_28
    JTlp3_18_5 = JTlp3_17_5-RLlp3_28*S22+RLlp3_38*C22
    JTlp3_28_5 = JTlp3_27_5+RLlp3_18*S22
    JTlp3_38_5 = JTlp3_37_5-RLlp3_18*C22
    JTlp3_18_6 = JTlp3_17_6-RLlp3_28*ROlp3_95+RLlp3_38*ROlp3_85
    JTlp3_28_6 = JTlp3_27_6+RLlp3_18*ROlp3_95-RLlp3_38*S23
    JTlp3_38_6 = JTlp3_37_6-RLlp3_18*ROlp3_85+RLlp3_28*S23
    JTlp3_18_7 = -RLlp3_28*ROlp3_66+RLlp3_38*ROlp3_56
    JTlp3_28_7 = RLlp3_18*ROlp3_66-RLlp3_38*ROlp3_46
    JTlp3_38_7 = -RLlp3_18*ROlp3_56+RLlp3_28*ROlp3_46
    h_4 = POlp3_18-s.dpt[1,10]
    h_5 = POlp3_28-s.dpt[2,10]
    h_6 = POlp3_38-s.dpt[3,10]
    RLlp5_22 = s.dpt[2,22]*C12
    RLlp5_32 = s.dpt[2,22]*S12
    POlp5_22 = RLlp5_22+s.dpt[2,2]
    POlp5_32 = RLlp5_32+s.dpt[3,2]
    ROlp6_82 = -C7*S8-S7*C8
    ROlp6_92 = C7*C8-S7*S8
    RLlp6_22 = s.dpt[2,16]*C7
    RLlp6_32 = s.dpt[2,16]*S7
    POlp6_22 = RLlp6_22+s.dpt[2,1]
    RLlp6_24 = ROlp6_82*s.dpt[3,18]
    RLlp6_34 = ROlp6_92*s.dpt[3,18]
    POlp6_24 = POlp6_22+RLlp6_24
    POlp6_34 = RLlp6_32+RLlp6_34
    JTlp6_24_1 = -RLlp6_32-RLlp6_34
    JTlp6_34_1 = RLlp6_22+RLlp6_24
    JTlp6_14_3 = -RLlp6_24*ROlp6_92+RLlp6_34*ROlp6_82
    h_7 = -s.dpt[1,1]+s.dpt[1,2]
    h_8 = POlp5_22-POlp6_24
    h_9 = POlp5_32-POlp6_34
    ROlp7_82 = -C13*S14-S13*C14
    ROlp7_92 = C13*C14-S13*S14
    RLlp7_22 = s.dpt[2,23]*C13
    RLlp7_32 = s.dpt[2,23]*S13
    POlp7_22 = RLlp7_22+s.dpt[2,4]
    RLlp7_24 = ROlp7_82*s.dpt[3,25]
    RLlp7_34 = ROlp7_92*s.dpt[3,25]
    POlp7_24 = POlp7_22+RLlp7_24
    POlp7_34 = RLlp7_32+RLlp7_34
    JTlp7_24_1 = -RLlp7_32-RLlp7_34
    JTlp7_34_1 = RLlp7_22+RLlp7_24
    JTlp7_14_3 = -RLlp7_24*ROlp7_92+RLlp7_34*ROlp7_82
    RLlp8_22 = s.dpt[2,29]*C18
    RLlp8_32 = s.dpt[2,29]*S18
    POlp8_22 = RLlp8_22+s.dpt[2,5]
    POlp8_32 = RLlp8_32+s.dpt[3,5]
    h_10 = s.dpt[1,4]-s.dpt[1,5]
    h_11 = POlp7_24-POlp8_22
    h_12 = POlp7_34-POlp8_32
    ROlp9_52 = C7*C8-S7*S8
    ROlp9_62 = C7*S8+S7*C8
    ROlp9_82 = -C7*S8-S7*C8
    ROlp9_92 = C7*C8-S7*S8
    ROlp9_23 = ROlp9_52*S9
    ROlp9_33 = ROlp9_62*S9
    RLlp9_22 = s.dpt[2,16]*C7
    RLlp9_32 = s.dpt[2,16]*S7
    POlp9_22 = RLlp9_22+s.dpt[2,1]
    RLlp9_14 = s.dpt[1,20]*C9
    RLlp9_24 = ROlp9_23*s.dpt[1,20]
    RLlp9_34 = ROlp9_33*s.dpt[1,20]
    POlp9_14 = RLlp9_14+s.dpt[1,1]
    POlp9_24 = POlp9_22+RLlp9_24
    POlp9_34 = RLlp9_32+RLlp9_34
    JTlp9_24_1 = -RLlp9_32-RLlp9_34
    JTlp9_34_1 = RLlp9_22+RLlp9_24
    JTlp9_14_3 = -RLlp9_24*ROlp9_92+RLlp9_34*ROlp9_82
    JTlp9_24_3 = RLlp9_14*ROlp9_92
    JTlp9_34_3 = -RLlp9_14*ROlp9_82
    ROlp10_52 = C13*C14-S13*S14
    ROlp10_62 = C13*S14+S13*C14
    ROlp10_82 = -C13*S14-S13*C14
    ROlp10_92 = C13*C14-S13*S14
    ROlp10_23 = ROlp10_52*S15
    ROlp10_33 = ROlp10_62*S15
    RLlp10_22 = s.dpt[2,23]*C13
    RLlp10_32 = s.dpt[2,23]*S13
    POlp10_22 = RLlp10_22+s.dpt[2,4]
    RLlp10_14 = s.dpt[1,27]*C15
    RLlp10_24 = ROlp10_23*s.dpt[1,27]
    RLlp10_34 = ROlp10_33*s.dpt[1,27]
    POlp10_14 = RLlp10_14+s.dpt[1,4]
    POlp10_24 = POlp10_22+RLlp10_24
    POlp10_34 = RLlp10_32+RLlp10_34
    JTlp10_24_1 = -RLlp10_32-RLlp10_34
    JTlp10_34_1 = RLlp10_22+RLlp10_24
    JTlp10_14_3 = -RLlp10_24*ROlp10_92+RLlp10_34*ROlp10_82
    JTlp10_24_3 = RLlp10_14*ROlp10_92
    JTlp10_34_3 = -RLlp10_14*ROlp10_82
    Plp11 = -POlp10_14+POlp9_14
    Plp21 = -POlp10_24+POlp9_24
    Plp31 = -POlp10_34+POlp9_34
    P2lp1 = Plp11*Plp11+Plp21*Plp21+Plp31*Plp31
    l2rod1 = s.lrod[1]*s.lrod[1]
    h_13 = (0.50)*(P2lp1-l2rod1)
    Jac_13_7 = JTlp9_24_1*Plp21+JTlp9_34_1*Plp31
    Jac_13_8 = -Plp21*RLlp9_34+Plp31*RLlp9_24
    Jac_13_9 = JTlp9_14_3*Plp11+JTlp9_24_3*Plp21+JTlp9_34_3*Plp31
    Jac_13_13 = -JTlp10_24_1*Plp21-JTlp10_34_1*Plp31
    Jac_13_14 = Plp21*RLlp10_34-Plp31*RLlp10_24
    Jac_13_15 = -JTlp10_14_3*Plp11-JTlp10_24_3*Plp21-JTlp10_34_3*Plp31
    h[1] = h_1
    h[2] = h_2
    h[3] = h_3
    h[4] = h_4
    h[5] = h_5
    h[6] = h_6
    h[7] = h_7
    h[8] = h_8
    h[9] = h_9
    h[10] = h_10
    h[11] = h_11
    h[12] = h_12
    h[13] = h_13
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = 0
    Jac[1,8] = 0
    Jac[1,9] = 0
    Jac[1,10] = 0
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = 0
    Jac[1,17] = 0
    Jac[1,18] = 0
    Jac[1,19] = 0
    Jac[1,20] = -(1.0)
    Jac[1,21] = 0
    Jac[1,22] = 0
    Jac[1,23] = -JTlp2_18_5
    Jac[1,24] = -JTlp2_18_6
    Jac[1,25] = -JTlp2_18_7
    Jac[1,26] = 0
    Jac[1,27] = 0
    Jac[1,28] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = 0
    Jac[2,8] = 0
    Jac[2,9] = 0
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = 0
    Jac[2,18] = 0
    Jac[2,19] = 0
    Jac[2,20] = 0
    Jac[2,21] = -(1.0)
    Jac[2,22] = -JTlp2_28_4
    Jac[2,23] = -JTlp2_28_5
    Jac[2,24] = -JTlp2_28_6
    Jac[2,25] = -JTlp2_28_7
    Jac[2,26] = 0
    Jac[2,27] = 0
    Jac[2,28] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = 0
    Jac[3,8] = 0
    Jac[3,9] = 0
    Jac[3,10] = 0
    Jac[3,11] = 0
    Jac[3,12] = 0
    Jac[3,13] = 0
    Jac[3,14] = 0
    Jac[3,15] = 0
    Jac[3,16] = 0
    Jac[3,17] = 0
    Jac[3,18] = 0
    Jac[3,19] = -(1.0)
    Jac[3,20] = 0
    Jac[3,21] = 0
    Jac[3,22] = -JTlp2_38_4
    Jac[3,23] = -JTlp2_38_5
    Jac[3,24] = -JTlp2_38_6
    Jac[3,25] = -JTlp2_38_7
    Jac[3,26] = 0
    Jac[3,27] = 0
    Jac[3,28] = 0
    Jac[4,1] = 0
    Jac[4,2] = 0
    Jac[4,3] = 0
    Jac[4,4] = 0
    Jac[4,5] = 0
    Jac[4,6] = 0
    Jac[4,7] = 0
    Jac[4,8] = 0
    Jac[4,9] = 0
    Jac[4,10] = 0
    Jac[4,11] = 0
    Jac[4,12] = 0
    Jac[4,13] = 0
    Jac[4,14] = 0
    Jac[4,15] = 0
    Jac[4,16] = 0
    Jac[4,17] = 0
    Jac[4,18] = 0
    Jac[4,19] = 0
    Jac[4,20] = (1.0)
    Jac[4,21] = 0
    Jac[4,22] = 0
    Jac[4,23] = JTlp3_18_5
    Jac[4,24] = JTlp3_18_6
    Jac[4,25] = 0
    Jac[4,26] = JTlp3_18_7
    Jac[4,27] = 0
    Jac[4,28] = 0
    Jac[5,1] = 0
    Jac[5,2] = 0
    Jac[5,3] = 0
    Jac[5,4] = 0
    Jac[5,5] = 0
    Jac[5,6] = 0
    Jac[5,7] = 0
    Jac[5,8] = 0
    Jac[5,9] = 0
    Jac[5,10] = 0
    Jac[5,11] = 0
    Jac[5,12] = 0
    Jac[5,13] = 0
    Jac[5,14] = 0
    Jac[5,15] = 0
    Jac[5,16] = 0
    Jac[5,17] = 0
    Jac[5,18] = 0
    Jac[5,19] = 0
    Jac[5,20] = 0
    Jac[5,21] = (1.0)
    Jac[5,22] = JTlp3_28_4
    Jac[5,23] = JTlp3_28_5
    Jac[5,24] = JTlp3_28_6
    Jac[5,25] = 0
    Jac[5,26] = JTlp3_28_7
    Jac[5,27] = 0
    Jac[5,28] = 0
    Jac[6,1] = 0
    Jac[6,2] = 0
    Jac[6,3] = 0
    Jac[6,4] = 0
    Jac[6,5] = 0
    Jac[6,6] = 0
    Jac[6,7] = 0
    Jac[6,8] = 0
    Jac[6,9] = 0
    Jac[6,10] = 0
    Jac[6,11] = 0
    Jac[6,12] = 0
    Jac[6,13] = 0
    Jac[6,14] = 0
    Jac[6,15] = 0
    Jac[6,16] = 0
    Jac[6,17] = 0
    Jac[6,18] = 0
    Jac[6,19] = (1.0)
    Jac[6,20] = 0
    Jac[6,21] = 0
    Jac[6,22] = JTlp3_38_4
    Jac[6,23] = JTlp3_38_5
    Jac[6,24] = JTlp3_38_6
    Jac[6,25] = 0
    Jac[6,26] = JTlp3_38_7
    Jac[6,27] = 0
    Jac[6,28] = 0
    Jac[7,1] = 0
    Jac[7,2] = 0
    Jac[7,3] = 0
    Jac[7,4] = 0
    Jac[7,5] = 0
    Jac[7,6] = 0
    Jac[7,7] = 0
    Jac[7,8] = 0
    Jac[7,9] = -JTlp6_14_3
    Jac[7,10] = 0
    Jac[7,11] = 0
    Jac[7,12] = 0
    Jac[7,13] = 0
    Jac[7,14] = 0
    Jac[7,15] = 0
    Jac[7,16] = 0
    Jac[7,17] = 0
    Jac[7,18] = 0
    Jac[7,19] = 0
    Jac[7,20] = 0
    Jac[7,21] = 0
    Jac[7,22] = 0
    Jac[7,23] = 0
    Jac[7,24] = 0
    Jac[7,25] = 0
    Jac[7,26] = 0
    Jac[7,27] = 0
    Jac[7,28] = 0
    Jac[8,1] = 0
    Jac[8,2] = 0
    Jac[8,3] = 0
    Jac[8,4] = 0
    Jac[8,5] = 0
    Jac[8,6] = 0
    Jac[8,7] = -JTlp6_24_1
    Jac[8,8] = RLlp6_34
    Jac[8,9] = 0
    Jac[8,10] = 0
    Jac[8,11] = 0
    Jac[8,12] = -RLlp5_32
    Jac[8,13] = 0
    Jac[8,14] = 0
    Jac[8,15] = 0
    Jac[8,16] = 0
    Jac[8,17] = 0
    Jac[8,18] = 0
    Jac[8,19] = 0
    Jac[8,20] = 0
    Jac[8,21] = 0
    Jac[8,22] = 0
    Jac[8,23] = 0
    Jac[8,24] = 0
    Jac[8,25] = 0
    Jac[8,26] = 0
    Jac[8,27] = 0
    Jac[8,28] = 0
    Jac[9,1] = 0
    Jac[9,2] = 0
    Jac[9,3] = 0
    Jac[9,4] = 0
    Jac[9,5] = 0
    Jac[9,6] = 0
    Jac[9,7] = -JTlp6_34_1
    Jac[9,8] = -RLlp6_24
    Jac[9,9] = 0
    Jac[9,10] = 0
    Jac[9,11] = 0
    Jac[9,12] = RLlp5_22
    Jac[9,13] = 0
    Jac[9,14] = 0
    Jac[9,15] = 0
    Jac[9,16] = 0
    Jac[9,17] = 0
    Jac[9,18] = 0
    Jac[9,19] = 0
    Jac[9,20] = 0
    Jac[9,21] = 0
    Jac[9,22] = 0
    Jac[9,23] = 0
    Jac[9,24] = 0
    Jac[9,25] = 0
    Jac[9,26] = 0
    Jac[9,27] = 0
    Jac[9,28] = 0
    Jac[10,1] = 0
    Jac[10,2] = 0
    Jac[10,3] = 0
    Jac[10,4] = 0
    Jac[10,5] = 0
    Jac[10,6] = 0
    Jac[10,7] = 0
    Jac[10,8] = 0
    Jac[10,9] = 0
    Jac[10,10] = 0
    Jac[10,11] = 0
    Jac[10,12] = 0
    Jac[10,13] = 0
    Jac[10,14] = 0
    Jac[10,15] = JTlp7_14_3
    Jac[10,16] = 0
    Jac[10,17] = 0
    Jac[10,18] = 0
    Jac[10,19] = 0
    Jac[10,20] = 0
    Jac[10,21] = 0
    Jac[10,22] = 0
    Jac[10,23] = 0
    Jac[10,24] = 0
    Jac[10,25] = 0
    Jac[10,26] = 0
    Jac[10,27] = 0
    Jac[10,28] = 0
    Jac[11,1] = 0
    Jac[11,2] = 0
    Jac[11,3] = 0
    Jac[11,4] = 0
    Jac[11,5] = 0
    Jac[11,6] = 0
    Jac[11,7] = 0
    Jac[11,8] = 0
    Jac[11,9] = 0
    Jac[11,10] = 0
    Jac[11,11] = 0
    Jac[11,12] = 0
    Jac[11,13] = JTlp7_24_1
    Jac[11,14] = -RLlp7_34
    Jac[11,15] = 0
    Jac[11,16] = 0
    Jac[11,17] = 0
    Jac[11,18] = RLlp8_32
    Jac[11,19] = 0
    Jac[11,20] = 0
    Jac[11,21] = 0
    Jac[11,22] = 0
    Jac[11,23] = 0
    Jac[11,24] = 0
    Jac[11,25] = 0
    Jac[11,26] = 0
    Jac[11,27] = 0
    Jac[11,28] = 0
    Jac[12,1] = 0
    Jac[12,2] = 0
    Jac[12,3] = 0
    Jac[12,4] = 0
    Jac[12,5] = 0
    Jac[12,6] = 0
    Jac[12,7] = 0
    Jac[12,8] = 0
    Jac[12,9] = 0
    Jac[12,10] = 0
    Jac[12,11] = 0
    Jac[12,12] = 0
    Jac[12,13] = JTlp7_34_1
    Jac[12,14] = RLlp7_24
    Jac[12,15] = 0
    Jac[12,16] = 0
    Jac[12,17] = 0
    Jac[12,18] = -RLlp8_22
    Jac[12,19] = 0
    Jac[12,20] = 0
    Jac[12,21] = 0
    Jac[12,22] = 0
    Jac[12,23] = 0
    Jac[12,24] = 0
    Jac[12,25] = 0
    Jac[12,26] = 0
    Jac[12,27] = 0
    Jac[12,28] = 0
    Jac[13,1] = 0
    Jac[13,2] = 0
    Jac[13,3] = 0
    Jac[13,4] = 0
    Jac[13,5] = 0
    Jac[13,6] = 0
    Jac[13,7] = Jac_13_7
    Jac[13,8] = Jac_13_8
    Jac[13,9] = Jac_13_9
    Jac[13,10] = 0
    Jac[13,11] = 0
    Jac[13,12] = 0
    Jac[13,13] = Jac_13_13
    Jac[13,14] = Jac_13_14
    Jac[13,15] = Jac_13_15
    Jac[13,16] = 0
    Jac[13,17] = 0
    Jac[13,18] = 0
    Jac[13,19] = 0
    Jac[13,20] = 0
    Jac[13,21] = 0
    Jac[13,22] = 0
    Jac[13,23] = 0
    Jac[13,24] = 0
    Jac[13,25] = 0
    Jac[13,26] = 0
    Jac[13,27] = 0
    Jac[13,28] = 0

# Number of continuation lines = 0


