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
#	==> Generation Date: Mon Mar 24 16:10:02 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 26
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd1_82 = -C7*S8-S7*C8
    ROjdqd1_92 = C7*C8-S7*S8
    RLjdqd1_22 = s.dpt[2,16]*C7
    RLjdqd1_32 = s.dpt[2,16]*S7
    OMjdqd1_12 = qd[7]+qd[8]
    ORjdqd1_22 = -RLjdqd1_32*qd[7]
    ORjdqd1_32 = RLjdqd1_22*qd[7]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = ORjdqd1_22*qd[7]
    RLjdqd1_23 = ROjdqd1_82*s.dpt[3,18]
    RLjdqd1_33 = ROjdqd1_92*s.dpt[3,18]
    ORjdqd1_23 = -OMjdqd1_12*RLjdqd1_33
    ORjdqd1_33 = OMjdqd1_12*RLjdqd1_23
    Apqpjdqd1_23 = Apqpjdqd1_22-OMjdqd1_12*ORjdqd1_33
    Apqpjdqd1_33 = Apqpjdqd1_32+OMjdqd1_12*ORjdqd1_23
    RLjdqd2_22 = s.dpt[2,21]*C11
    RLjdqd2_32 = s.dpt[2,21]*S11
    ORjdqd2_22 = -RLjdqd2_32*qd[11]
    ORjdqd2_32 = RLjdqd2_22*qd[11]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[11]
    Apqpjdqd2_32 = ORjdqd2_22*qd[11]
    jdqd2 = Apqpjdqd1_23-Apqpjdqd2_22
    jdqd3 = Apqpjdqd1_33-Apqpjdqd2_32
    RLjdqd3_22 = s.dpt[2,27]*C16
    RLjdqd3_32 = s.dpt[2,27]*S16
    ORjdqd3_22 = -RLjdqd3_32*qd[16]
    ORjdqd3_32 = RLjdqd3_22*qd[16]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[16]
    Apqpjdqd3_32 = ORjdqd3_22*qd[16]
    ROjdqd4_82 = -C12*S13-S12*C13
    ROjdqd4_92 = C12*C13-S12*S13
    RLjdqd4_22 = s.dpt[2,22]*C12
    RLjdqd4_32 = s.dpt[2,22]*S12
    OMjdqd4_12 = qd[12]+qd[13]
    ORjdqd4_22 = -RLjdqd4_32*qd[12]
    ORjdqd4_32 = RLjdqd4_22*qd[12]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[12]
    Apqpjdqd4_32 = ORjdqd4_22*qd[12]
    RLjdqd4_23 = ROjdqd4_82*s.dpt[3,24]
    RLjdqd4_33 = ROjdqd4_92*s.dpt[3,24]
    ORjdqd4_23 = -OMjdqd4_12*RLjdqd4_33
    ORjdqd4_33 = OMjdqd4_12*RLjdqd4_23
    Apqpjdqd4_23 = Apqpjdqd4_22-OMjdqd4_12*ORjdqd4_33
    Apqpjdqd4_33 = Apqpjdqd4_32+OMjdqd4_12*ORjdqd4_23
    jdqd5 = Apqpjdqd3_22-Apqpjdqd4_23
    jdqd6 = Apqpjdqd3_32-Apqpjdqd4_33
    ROjdqd6_25 = S20*S21
    ROjdqd6_35 = -C20*S21
    ROjdqd6_85 = -S20*C21
    ROjdqd6_95 = C20*C21
    ROjdqd6_16 = C21*C22
    ROjdqd6_26 = ROjdqd6_25*C22+C20*S22
    ROjdqd6_36 = ROjdqd6_35*C22+S20*S22
    ROjdqd6_46 = -C21*S22
    ROjdqd6_56 = -ROjdqd6_25*S22+C20*C22
    ROjdqd6_66 = -ROjdqd6_35*S22+S20*C22
    ROjdqd6_17 = ROjdqd6_16*C23-S21*S23
    ROjdqd6_27 = ROjdqd6_26*C23-ROjdqd6_85*S23
    ROjdqd6_37 = ROjdqd6_36*C23-ROjdqd6_95*S23
    OMjdqd6_25 = qd[21]*C20
    OMjdqd6_35 = qd[21]*S20
    Ompqpjdqd6_25 = -qd[20]*qd[21]*S20
    Ompqpjdqd6_35 = qd[20]*qd[21]*C20
    OMjdqd6_16 = qd[20]+qd[22]*S21
    OMjdqd6_26 = OMjdqd6_25+ROjdqd6_85*qd[22]
    OMjdqd6_36 = OMjdqd6_35+ROjdqd6_95*qd[22]
    Ompqpjdqd6_16 = qd[22]*(OMjdqd6_25*ROjdqd6_95-OMjdqd6_35*ROjdqd6_85)
    Ompqpjdqd6_26 = Ompqpjdqd6_25+qd[22]*(OMjdqd6_35*S21-ROjdqd6_95*qd[20])
    Ompqpjdqd6_36 = Ompqpjdqd6_35+qd[22]*(-OMjdqd6_25*S21+ROjdqd6_85*qd[20])
    RLjdqd6_17 = ROjdqd6_46*s.dpt[2,30]+s.dpt[3,30]*S21
    RLjdqd6_27 = ROjdqd6_56*s.dpt[2,30]+ROjdqd6_85*s.dpt[3,30]
    RLjdqd6_37 = ROjdqd6_66*s.dpt[2,30]+ROjdqd6_95*s.dpt[3,30]
    OMjdqd6_17 = OMjdqd6_16+ROjdqd6_46*qd[23]
    OMjdqd6_27 = OMjdqd6_26+ROjdqd6_56*qd[23]
    OMjdqd6_37 = OMjdqd6_36+ROjdqd6_66*qd[23]
    ORjdqd6_17 = OMjdqd6_26*RLjdqd6_37-OMjdqd6_36*RLjdqd6_27
    ORjdqd6_27 = -OMjdqd6_16*RLjdqd6_37+OMjdqd6_36*RLjdqd6_17
    ORjdqd6_37 = OMjdqd6_16*RLjdqd6_27-OMjdqd6_26*RLjdqd6_17
    Ompqpjdqd6_17 = Ompqpjdqd6_16+qd[23]*(OMjdqd6_26*ROjdqd6_66-OMjdqd6_36*ROjdqd6_56)
    Ompqpjdqd6_27 = Ompqpjdqd6_26+qd[23]*(-OMjdqd6_16*ROjdqd6_66+OMjdqd6_36*ROjdqd6_46)
    Ompqpjdqd6_37 = Ompqpjdqd6_36+qd[23]*(OMjdqd6_16*ROjdqd6_56-OMjdqd6_26*ROjdqd6_46)
    Apqpjdqd6_17 = OMjdqd6_26*ORjdqd6_37-OMjdqd6_36*ORjdqd6_27+Ompqpjdqd6_26*RLjdqd6_37-Ompqpjdqd6_36*RLjdqd6_27
    Apqpjdqd6_27 = -OMjdqd6_16*ORjdqd6_37+OMjdqd6_36*ORjdqd6_17-Ompqpjdqd6_16*RLjdqd6_37+Ompqpjdqd6_36*RLjdqd6_17
    Apqpjdqd6_37 = OMjdqd6_16*ORjdqd6_27-OMjdqd6_26*ORjdqd6_17+Ompqpjdqd6_16*RLjdqd6_27-Ompqpjdqd6_26*RLjdqd6_17
    RLjdqd6_18 = ROjdqd6_17*s.dpt[1,38]
    RLjdqd6_28 = ROjdqd6_27*s.dpt[1,38]
    RLjdqd6_38 = ROjdqd6_37*s.dpt[1,38]
    ORjdqd6_18 = OMjdqd6_27*RLjdqd6_38-OMjdqd6_37*RLjdqd6_28
    ORjdqd6_28 = -OMjdqd6_17*RLjdqd6_38+OMjdqd6_37*RLjdqd6_18
    ORjdqd6_38 = OMjdqd6_17*RLjdqd6_28-OMjdqd6_27*RLjdqd6_18
    Apqpjdqd6_18 = Apqpjdqd6_17+OMjdqd6_27*ORjdqd6_38-OMjdqd6_37*ORjdqd6_28+Ompqpjdqd6_27*RLjdqd6_38-Ompqpjdqd6_37*RLjdqd6_28
    Apqpjdqd6_28 = Apqpjdqd6_27-OMjdqd6_17*ORjdqd6_38+OMjdqd6_37*ORjdqd6_18-Ompqpjdqd6_17*RLjdqd6_38+Ompqpjdqd6_37*RLjdqd6_18
    Apqpjdqd6_38 = Apqpjdqd6_37+OMjdqd6_17*ORjdqd6_28-OMjdqd6_27*ORjdqd6_18+Ompqpjdqd6_17*RLjdqd6_28-Ompqpjdqd6_27*RLjdqd6_18
    ROjdqd7_25 = S20*S21
    ROjdqd7_35 = -C20*S21
    ROjdqd7_85 = -S20*C21
    ROjdqd7_95 = C20*C21
    ROjdqd7_16 = C21*C22
    ROjdqd7_26 = ROjdqd7_25*C22+C20*S22
    ROjdqd7_36 = ROjdqd7_35*C22+S20*S22
    ROjdqd7_46 = -C21*S22
    ROjdqd7_56 = -ROjdqd7_25*S22+C20*C22
    ROjdqd7_66 = -ROjdqd7_35*S22+S20*C22
    ROjdqd7_17 = ROjdqd7_16*C24-S21*S24
    ROjdqd7_27 = ROjdqd7_26*C24-ROjdqd7_85*S24
    ROjdqd7_37 = ROjdqd7_36*C24-ROjdqd7_95*S24
    OMjdqd7_25 = qd[21]*C20
    OMjdqd7_35 = qd[21]*S20
    Ompqpjdqd7_25 = -qd[20]*qd[21]*S20
    Ompqpjdqd7_35 = qd[20]*qd[21]*C20
    OMjdqd7_16 = qd[20]+qd[22]*S21
    OMjdqd7_26 = OMjdqd7_25+ROjdqd7_85*qd[22]
    OMjdqd7_36 = OMjdqd7_35+ROjdqd7_95*qd[22]
    Ompqpjdqd7_16 = qd[22]*(OMjdqd7_25*ROjdqd7_95-OMjdqd7_35*ROjdqd7_85)
    Ompqpjdqd7_26 = Ompqpjdqd7_25+qd[22]*(OMjdqd7_35*S21-ROjdqd7_95*qd[20])
    Ompqpjdqd7_36 = Ompqpjdqd7_35+qd[22]*(-OMjdqd7_25*S21+ROjdqd7_85*qd[20])
    RLjdqd7_17 = ROjdqd7_46*s.dpt[2,33]+s.dpt[3,33]*S21
    RLjdqd7_27 = ROjdqd7_56*s.dpt[2,33]+ROjdqd7_85*s.dpt[3,33]
    RLjdqd7_37 = ROjdqd7_66*s.dpt[2,33]+ROjdqd7_95*s.dpt[3,33]
    OMjdqd7_17 = OMjdqd7_16+ROjdqd7_46*qd[24]
    OMjdqd7_27 = OMjdqd7_26+ROjdqd7_56*qd[24]
    OMjdqd7_37 = OMjdqd7_36+ROjdqd7_66*qd[24]
    ORjdqd7_17 = OMjdqd7_26*RLjdqd7_37-OMjdqd7_36*RLjdqd7_27
    ORjdqd7_27 = -OMjdqd7_16*RLjdqd7_37+OMjdqd7_36*RLjdqd7_17
    ORjdqd7_37 = OMjdqd7_16*RLjdqd7_27-OMjdqd7_26*RLjdqd7_17
    Ompqpjdqd7_17 = Ompqpjdqd7_16+qd[24]*(OMjdqd7_26*ROjdqd7_66-OMjdqd7_36*ROjdqd7_56)
    Ompqpjdqd7_27 = Ompqpjdqd7_26+qd[24]*(-OMjdqd7_16*ROjdqd7_66+OMjdqd7_36*ROjdqd7_46)
    Ompqpjdqd7_37 = Ompqpjdqd7_36+qd[24]*(OMjdqd7_16*ROjdqd7_56-OMjdqd7_26*ROjdqd7_46)
    Apqpjdqd7_17 = OMjdqd7_26*ORjdqd7_37-OMjdqd7_36*ORjdqd7_27+Ompqpjdqd7_26*RLjdqd7_37-Ompqpjdqd7_36*RLjdqd7_27
    Apqpjdqd7_27 = -OMjdqd7_16*ORjdqd7_37+OMjdqd7_36*ORjdqd7_17-Ompqpjdqd7_16*RLjdqd7_37+Ompqpjdqd7_36*RLjdqd7_17
    Apqpjdqd7_37 = OMjdqd7_16*ORjdqd7_27-OMjdqd7_26*ORjdqd7_17+Ompqpjdqd7_16*RLjdqd7_27-Ompqpjdqd7_26*RLjdqd7_17
    RLjdqd7_18 = ROjdqd7_17*s.dpt[1,39]
    RLjdqd7_28 = ROjdqd7_27*s.dpt[1,39]
    RLjdqd7_38 = ROjdqd7_37*s.dpt[1,39]
    ORjdqd7_18 = OMjdqd7_27*RLjdqd7_38-OMjdqd7_37*RLjdqd7_28
    ORjdqd7_28 = -OMjdqd7_17*RLjdqd7_38+OMjdqd7_37*RLjdqd7_18
    ORjdqd7_38 = OMjdqd7_17*RLjdqd7_28-OMjdqd7_27*RLjdqd7_18
    Apqpjdqd7_18 = Apqpjdqd7_17+OMjdqd7_27*ORjdqd7_38-OMjdqd7_37*ORjdqd7_28+Ompqpjdqd7_27*RLjdqd7_38-Ompqpjdqd7_37*RLjdqd7_28
    Apqpjdqd7_28 = Apqpjdqd7_27-OMjdqd7_17*ORjdqd7_38+OMjdqd7_37*ORjdqd7_18-Ompqpjdqd7_17*RLjdqd7_38+Ompqpjdqd7_37*RLjdqd7_18
    Apqpjdqd7_38 = Apqpjdqd7_37+OMjdqd7_17*ORjdqd7_28-OMjdqd7_27*ORjdqd7_18+Ompqpjdqd7_17*RLjdqd7_28-Ompqpjdqd7_27*RLjdqd7_18
    Jdqd[1] = jdqd2
    Jdqd[2] = jdqd3
    Jdqd[3] = jdqd5
    Jdqd[4] = jdqd6
    Jdqd[5] = -Apqpjdqd6_18
    Jdqd[6] = -Apqpjdqd6_28
    Jdqd[7] = -Apqpjdqd6_38
    Jdqd[8] = Apqpjdqd7_18
    Jdqd[9] = Apqpjdqd7_28
    Jdqd[10] = Apqpjdqd7_38

# Number of continuation lines = 0


