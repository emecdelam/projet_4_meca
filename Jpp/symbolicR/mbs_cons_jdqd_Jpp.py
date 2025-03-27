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
#	==> Generation Date: Thu Mar 27 21:49:43 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 30
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
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd2_25 = S22*S23
    ROjdqd2_35 = -C22*S23
    ROjdqd2_85 = -S22*C23
    ROjdqd2_95 = C22*C23
    ROjdqd2_16 = C23*C24
    ROjdqd2_26 = ROjdqd2_25*C24+C22*S24
    ROjdqd2_36 = ROjdqd2_35*C24+S22*S24
    ROjdqd2_46 = -C23*S24
    ROjdqd2_56 = -ROjdqd2_25*S24+C22*C24
    ROjdqd2_66 = -ROjdqd2_35*S24+S22*C24
    ROjdqd2_17 = ROjdqd2_16*C25-S23*S25
    ROjdqd2_27 = ROjdqd2_26*C25-ROjdqd2_85*S25
    ROjdqd2_37 = ROjdqd2_36*C25-ROjdqd2_95*S25
    OMjdqd2_25 = qd[23]*C22
    OMjdqd2_35 = qd[23]*S22
    Ompqpjdqd2_25 = -qd[22]*qd[23]*S22
    Ompqpjdqd2_35 = qd[22]*qd[23]*C22
    OMjdqd2_16 = qd[22]+qd[24]*S23
    OMjdqd2_26 = OMjdqd2_25+ROjdqd2_85*qd[24]
    OMjdqd2_36 = OMjdqd2_35+ROjdqd2_95*qd[24]
    Ompqpjdqd2_16 = qd[24]*(OMjdqd2_25*ROjdqd2_95-OMjdqd2_35*ROjdqd2_85)
    Ompqpjdqd2_26 = Ompqpjdqd2_25+qd[24]*(OMjdqd2_35*S23-ROjdqd2_95*qd[22])
    Ompqpjdqd2_36 = Ompqpjdqd2_35+qd[24]*(-OMjdqd2_25*S23+ROjdqd2_85*qd[22])
    RLjdqd2_17 = ROjdqd2_46*s.dpt[2,39]+s.dpt[3,39]*S23
    RLjdqd2_27 = ROjdqd2_56*s.dpt[2,39]+ROjdqd2_85*s.dpt[3,39]
    RLjdqd2_37 = ROjdqd2_66*s.dpt[2,39]+ROjdqd2_95*s.dpt[3,39]
    OMjdqd2_17 = OMjdqd2_16+ROjdqd2_46*qd[25]
    OMjdqd2_27 = OMjdqd2_26+ROjdqd2_56*qd[25]
    OMjdqd2_37 = OMjdqd2_36+ROjdqd2_66*qd[25]
    ORjdqd2_17 = OMjdqd2_26*RLjdqd2_37-OMjdqd2_36*RLjdqd2_27
    ORjdqd2_27 = -OMjdqd2_16*RLjdqd2_37+OMjdqd2_36*RLjdqd2_17
    ORjdqd2_37 = OMjdqd2_16*RLjdqd2_27-OMjdqd2_26*RLjdqd2_17
    Ompqpjdqd2_17 = Ompqpjdqd2_16+qd[25]*(OMjdqd2_26*ROjdqd2_66-OMjdqd2_36*ROjdqd2_56)
    Ompqpjdqd2_27 = Ompqpjdqd2_26+qd[25]*(-OMjdqd2_16*ROjdqd2_66+OMjdqd2_36*ROjdqd2_46)
    Ompqpjdqd2_37 = Ompqpjdqd2_36+qd[25]*(OMjdqd2_16*ROjdqd2_56-OMjdqd2_26*ROjdqd2_46)
    Apqpjdqd2_17 = OMjdqd2_26*ORjdqd2_37-OMjdqd2_36*ORjdqd2_27+Ompqpjdqd2_26*RLjdqd2_37-Ompqpjdqd2_36*RLjdqd2_27
    Apqpjdqd2_27 = -OMjdqd2_16*ORjdqd2_37+OMjdqd2_36*ORjdqd2_17-Ompqpjdqd2_16*RLjdqd2_37+Ompqpjdqd2_36*RLjdqd2_17
    Apqpjdqd2_37 = OMjdqd2_16*ORjdqd2_27-OMjdqd2_26*ORjdqd2_17+Ompqpjdqd2_16*RLjdqd2_27-Ompqpjdqd2_26*RLjdqd2_17
    RLjdqd2_18 = ROjdqd2_17*s.dpt[1,51]
    RLjdqd2_28 = ROjdqd2_27*s.dpt[1,51]
    RLjdqd2_38 = ROjdqd2_37*s.dpt[1,51]
    ORjdqd2_18 = OMjdqd2_27*RLjdqd2_38-OMjdqd2_37*RLjdqd2_28
    ORjdqd2_28 = -OMjdqd2_17*RLjdqd2_38+OMjdqd2_37*RLjdqd2_18
    ORjdqd2_38 = OMjdqd2_17*RLjdqd2_28-OMjdqd2_27*RLjdqd2_18
    Apqpjdqd2_18 = Apqpjdqd2_17+OMjdqd2_27*ORjdqd2_38-OMjdqd2_37*ORjdqd2_28+Ompqpjdqd2_27*RLjdqd2_38-Ompqpjdqd2_37*RLjdqd2_28
    Apqpjdqd2_28 = Apqpjdqd2_27-OMjdqd2_17*ORjdqd2_38+OMjdqd2_37*ORjdqd2_18-Ompqpjdqd2_17*RLjdqd2_38+Ompqpjdqd2_37*RLjdqd2_18
    Apqpjdqd2_38 = Apqpjdqd2_37+OMjdqd2_17*ORjdqd2_28-OMjdqd2_27*ORjdqd2_18+Ompqpjdqd2_17*RLjdqd2_28-Ompqpjdqd2_27*RLjdqd2_18
    ROjdqd3_25 = S22*S23
    ROjdqd3_35 = -C22*S23
    ROjdqd3_85 = -S22*C23
    ROjdqd3_95 = C22*C23
    ROjdqd3_16 = C23*C24
    ROjdqd3_26 = ROjdqd3_25*C24+C22*S24
    ROjdqd3_36 = ROjdqd3_35*C24+S22*S24
    ROjdqd3_46 = -C23*S24
    ROjdqd3_56 = -ROjdqd3_25*S24+C22*C24
    ROjdqd3_66 = -ROjdqd3_35*S24+S22*C24
    ROjdqd3_17 = ROjdqd3_16*C26-S23*S26
    ROjdqd3_27 = ROjdqd3_26*C26-ROjdqd3_85*S26
    ROjdqd3_37 = ROjdqd3_36*C26-ROjdqd3_95*S26
    OMjdqd3_25 = qd[23]*C22
    OMjdqd3_35 = qd[23]*S22
    Ompqpjdqd3_25 = -qd[22]*qd[23]*S22
    Ompqpjdqd3_35 = qd[22]*qd[23]*C22
    OMjdqd3_16 = qd[22]+qd[24]*S23
    OMjdqd3_26 = OMjdqd3_25+ROjdqd3_85*qd[24]
    OMjdqd3_36 = OMjdqd3_35+ROjdqd3_95*qd[24]
    Ompqpjdqd3_16 = qd[24]*(OMjdqd3_25*ROjdqd3_95-OMjdqd3_35*ROjdqd3_85)
    Ompqpjdqd3_26 = Ompqpjdqd3_25+qd[24]*(OMjdqd3_35*S23-ROjdqd3_95*qd[22])
    Ompqpjdqd3_36 = Ompqpjdqd3_35+qd[24]*(-OMjdqd3_25*S23+ROjdqd3_85*qd[22])
    RLjdqd3_17 = ROjdqd3_46*s.dpt[2,42]+s.dpt[3,42]*S23
    RLjdqd3_27 = ROjdqd3_56*s.dpt[2,42]+ROjdqd3_85*s.dpt[3,42]
    RLjdqd3_37 = ROjdqd3_66*s.dpt[2,42]+ROjdqd3_95*s.dpt[3,42]
    OMjdqd3_17 = OMjdqd3_16+ROjdqd3_46*qd[26]
    OMjdqd3_27 = OMjdqd3_26+ROjdqd3_56*qd[26]
    OMjdqd3_37 = OMjdqd3_36+ROjdqd3_66*qd[26]
    ORjdqd3_17 = OMjdqd3_26*RLjdqd3_37-OMjdqd3_36*RLjdqd3_27
    ORjdqd3_27 = -OMjdqd3_16*RLjdqd3_37+OMjdqd3_36*RLjdqd3_17
    ORjdqd3_37 = OMjdqd3_16*RLjdqd3_27-OMjdqd3_26*RLjdqd3_17
    Ompqpjdqd3_17 = Ompqpjdqd3_16+qd[26]*(OMjdqd3_26*ROjdqd3_66-OMjdqd3_36*ROjdqd3_56)
    Ompqpjdqd3_27 = Ompqpjdqd3_26+qd[26]*(-OMjdqd3_16*ROjdqd3_66+OMjdqd3_36*ROjdqd3_46)
    Ompqpjdqd3_37 = Ompqpjdqd3_36+qd[26]*(OMjdqd3_16*ROjdqd3_56-OMjdqd3_26*ROjdqd3_46)
    Apqpjdqd3_17 = OMjdqd3_26*ORjdqd3_37-OMjdqd3_36*ORjdqd3_27+Ompqpjdqd3_26*RLjdqd3_37-Ompqpjdqd3_36*RLjdqd3_27
    Apqpjdqd3_27 = -OMjdqd3_16*ORjdqd3_37+OMjdqd3_36*ORjdqd3_17-Ompqpjdqd3_16*RLjdqd3_37+Ompqpjdqd3_36*RLjdqd3_17
    Apqpjdqd3_37 = OMjdqd3_16*ORjdqd3_27-OMjdqd3_26*ORjdqd3_17+Ompqpjdqd3_16*RLjdqd3_27-Ompqpjdqd3_26*RLjdqd3_17
    RLjdqd3_18 = ROjdqd3_17*s.dpt[1,52]
    RLjdqd3_28 = ROjdqd3_27*s.dpt[1,52]
    RLjdqd3_38 = ROjdqd3_37*s.dpt[1,52]
    ORjdqd3_18 = OMjdqd3_27*RLjdqd3_38-OMjdqd3_37*RLjdqd3_28
    ORjdqd3_28 = -OMjdqd3_17*RLjdqd3_38+OMjdqd3_37*RLjdqd3_18
    ORjdqd3_38 = OMjdqd3_17*RLjdqd3_28-OMjdqd3_27*RLjdqd3_18
    Apqpjdqd3_18 = Apqpjdqd3_17+OMjdqd3_27*ORjdqd3_38-OMjdqd3_37*ORjdqd3_28+Ompqpjdqd3_27*RLjdqd3_38-Ompqpjdqd3_37*RLjdqd3_28
    Apqpjdqd3_28 = Apqpjdqd3_27-OMjdqd3_17*ORjdqd3_38+OMjdqd3_37*ORjdqd3_18-Ompqpjdqd3_17*RLjdqd3_38+Ompqpjdqd3_37*RLjdqd3_18
    Apqpjdqd3_38 = Apqpjdqd3_37+OMjdqd3_17*ORjdqd3_28-OMjdqd3_27*ORjdqd3_18+Ompqpjdqd3_17*RLjdqd3_28-Ompqpjdqd3_27*RLjdqd3_18
    RLjdqd5_22 = s.dpt[2,29]*C12
    RLjdqd5_32 = s.dpt[2,29]*S12
    ORjdqd5_22 = -RLjdqd5_32*qd[12]
    ORjdqd5_32 = RLjdqd5_22*qd[12]
    Apqpjdqd5_22 = -ORjdqd5_32*qd[12]
    Apqpjdqd5_32 = ORjdqd5_22*qd[12]
    ROjdqd6_82 = -C7*S8-S7*C8
    ROjdqd6_92 = C7*C8-S7*S8
    RLjdqd6_22 = s.dpt[2,23]*C7
    RLjdqd6_32 = s.dpt[2,23]*S7
    OMjdqd6_12 = qd[7]+qd[8]
    ORjdqd6_22 = -RLjdqd6_32*qd[7]
    ORjdqd6_32 = RLjdqd6_22*qd[7]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[7]
    Apqpjdqd6_32 = ORjdqd6_22*qd[7]
    OMjdqd6_23 = ROjdqd6_82*qd[9]
    OMjdqd6_33 = ROjdqd6_92*qd[9]
    Ompqpjdqd6_23 = -OMjdqd6_12*ROjdqd6_92*qd[9]
    Ompqpjdqd6_33 = OMjdqd6_12*ROjdqd6_82*qd[9]
    RLjdqd6_24 = ROjdqd6_82*s.dpt[3,25]
    RLjdqd6_34 = ROjdqd6_92*s.dpt[3,25]
    ORjdqd6_14 = OMjdqd6_23*RLjdqd6_34-OMjdqd6_33*RLjdqd6_24
    ORjdqd6_24 = -OMjdqd6_12*RLjdqd6_34
    ORjdqd6_34 = OMjdqd6_12*RLjdqd6_24
    Apqpjdqd6_14 = OMjdqd6_23*ORjdqd6_34-OMjdqd6_33*ORjdqd6_24+Ompqpjdqd6_23*RLjdqd6_34-Ompqpjdqd6_33*RLjdqd6_24
    Apqpjdqd6_24 = Apqpjdqd6_22-OMjdqd6_12*ORjdqd6_34+OMjdqd6_33*ORjdqd6_14
    Apqpjdqd6_34 = Apqpjdqd6_32+OMjdqd6_12*ORjdqd6_24-OMjdqd6_23*ORjdqd6_14
    jdqd8 = Apqpjdqd5_22-Apqpjdqd6_24
    jdqd9 = Apqpjdqd5_32-Apqpjdqd6_34
    ROjdqd7_82 = -C13*S14-S13*C14
    ROjdqd7_92 = C13*C14-S13*S14
    RLjdqd7_22 = s.dpt[2,30]*C13
    RLjdqd7_32 = s.dpt[2,30]*S13
    OMjdqd7_12 = qd[13]+qd[14]
    ORjdqd7_22 = -RLjdqd7_32*qd[13]
    ORjdqd7_32 = RLjdqd7_22*qd[13]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[13]
    Apqpjdqd7_32 = ORjdqd7_22*qd[13]
    OMjdqd7_23 = ROjdqd7_82*qd[15]
    OMjdqd7_33 = ROjdqd7_92*qd[15]
    Ompqpjdqd7_23 = -OMjdqd7_12*ROjdqd7_92*qd[15]
    Ompqpjdqd7_33 = OMjdqd7_12*ROjdqd7_82*qd[15]
    RLjdqd7_24 = ROjdqd7_82*s.dpt[3,32]
    RLjdqd7_34 = ROjdqd7_92*s.dpt[3,32]
    ORjdqd7_14 = OMjdqd7_23*RLjdqd7_34-OMjdqd7_33*RLjdqd7_24
    ORjdqd7_24 = -OMjdqd7_12*RLjdqd7_34
    ORjdqd7_34 = OMjdqd7_12*RLjdqd7_24
    Apqpjdqd7_14 = OMjdqd7_23*ORjdqd7_34-OMjdqd7_33*ORjdqd7_24+Ompqpjdqd7_23*RLjdqd7_34-Ompqpjdqd7_33*RLjdqd7_24
    Apqpjdqd7_24 = Apqpjdqd7_22-OMjdqd7_12*ORjdqd7_34+OMjdqd7_33*ORjdqd7_14
    Apqpjdqd7_34 = Apqpjdqd7_32+OMjdqd7_12*ORjdqd7_24-OMjdqd7_23*ORjdqd7_14
    RLjdqd8_22 = s.dpt[2,36]*C18
    RLjdqd8_32 = s.dpt[2,36]*S18
    ORjdqd8_22 = -RLjdqd8_32*qd[18]
    ORjdqd8_32 = RLjdqd8_22*qd[18]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[18]
    Apqpjdqd8_32 = ORjdqd8_22*qd[18]
    jdqd11 = Apqpjdqd7_24-Apqpjdqd8_22
    jdqd12 = Apqpjdqd7_34-Apqpjdqd8_32
    RLjdqd9_12 = s.dpt[1,56]*C29-s.dpt[2,56]*S29
    RLjdqd9_22 = s.dpt[1,56]*S29+s.dpt[2,56]*C29
    POjdqd9_12 = RLjdqd9_12+s.dpt[1,17]
    POjdqd9_22 = RLjdqd9_22+s.dpt[2,17]
    ORjdqd9_12 = -RLjdqd9_22*qd[29]
    ORjdqd9_22 = RLjdqd9_12*qd[29]
    Apqpjdqd9_12 = -ORjdqd9_22*qd[29]
    Apqpjdqd9_22 = ORjdqd9_12*qd[29]
    RLjdqd10_12 = s.dpt[1,58]*C30-s.dpt[2,58]*S30
    RLjdqd10_22 = s.dpt[1,58]*S30+s.dpt[2,58]*C30
    POjdqd10_12 = RLjdqd10_12+s.dpt[1,18]
    POjdqd10_22 = RLjdqd10_22+s.dpt[2,18]
    ORjdqd10_12 = -RLjdqd10_22*qd[30]
    ORjdqd10_22 = RLjdqd10_12*qd[30]
    Apqpjdqd10_12 = -ORjdqd10_22*qd[30]
    Apqpjdqd10_22 = ORjdqd10_12*qd[30]
    jdqd13 = (-Apqpjdqd10_12+Apqpjdqd9_12)*(-POjdqd10_12+POjdqd9_12)+(-Apqpjdqd10_22+Apqpjdqd9_22)*(-POjdqd10_22+POjdqd9_22)+(-ORjdqd10_12+ORjdqd9_12)*(-ORjdqd10_12+ORjdqd9_12)+(-ORjdqd10_22+ORjdqd9_22)*(-ORjdqd10_22+ORjdqd9_22)
    ROjdqd11_52 = C7*C8-S7*S8
    ROjdqd11_62 = C7*S8+S7*C8
    ROjdqd11_82 = -C7*S8-S7*C8
    ROjdqd11_92 = C7*C8-S7*S8
    ROjdqd11_23 = ROjdqd11_52*S9
    ROjdqd11_33 = ROjdqd11_62*S9
    RLjdqd11_22 = s.dpt[2,23]*C7
    RLjdqd11_32 = s.dpt[2,23]*S7
    POjdqd11_22 = RLjdqd11_22+s.dpt[2,2]
    OMjdqd11_12 = qd[7]+qd[8]
    ORjdqd11_22 = -RLjdqd11_32*qd[7]
    ORjdqd11_32 = RLjdqd11_22*qd[7]
    Apqpjdqd11_22 = -ORjdqd11_32*qd[7]
    Apqpjdqd11_32 = ORjdqd11_22*qd[7]
    OMjdqd11_23 = ROjdqd11_82*qd[9]
    OMjdqd11_33 = ROjdqd11_92*qd[9]
    Ompqpjdqd11_23 = -OMjdqd11_12*ROjdqd11_92*qd[9]
    Ompqpjdqd11_33 = OMjdqd11_12*ROjdqd11_82*qd[9]
    RLjdqd11_14 = s.dpt[1,27]*C9
    RLjdqd11_24 = ROjdqd11_23*s.dpt[1,27]
    RLjdqd11_34 = ROjdqd11_33*s.dpt[1,27]
    POjdqd11_14 = RLjdqd11_14+s.dpt[1,2]
    POjdqd11_24 = POjdqd11_22+RLjdqd11_24
    POjdqd11_34 = RLjdqd11_32+RLjdqd11_34
    ORjdqd11_14 = OMjdqd11_23*RLjdqd11_34-OMjdqd11_33*RLjdqd11_24
    ORjdqd11_24 = -OMjdqd11_12*RLjdqd11_34+OMjdqd11_33*RLjdqd11_14
    ORjdqd11_34 = OMjdqd11_12*RLjdqd11_24-OMjdqd11_23*RLjdqd11_14
    VIjdqd11_24 = ORjdqd11_22+ORjdqd11_24
    VIjdqd11_34 = ORjdqd11_32+ORjdqd11_34
    Apqpjdqd11_14 = OMjdqd11_23*ORjdqd11_34-OMjdqd11_33*ORjdqd11_24+Ompqpjdqd11_23*RLjdqd11_34-Ompqpjdqd11_33*RLjdqd11_24
    Apqpjdqd11_24 = Apqpjdqd11_22-OMjdqd11_12*ORjdqd11_34+OMjdqd11_33*ORjdqd11_14+Ompqpjdqd11_33*RLjdqd11_14
    Apqpjdqd11_34 = Apqpjdqd11_32+OMjdqd11_12*ORjdqd11_24-OMjdqd11_23*ORjdqd11_14-Ompqpjdqd11_23*RLjdqd11_14
    RLjdqd12_12 = s.dpt[1,55]*C29-s.dpt[2,55]*S29
    RLjdqd12_22 = s.dpt[1,55]*S29+s.dpt[2,55]*C29
    POjdqd12_12 = RLjdqd12_12+s.dpt[1,17]
    POjdqd12_22 = RLjdqd12_22+s.dpt[2,17]
    ORjdqd12_12 = -RLjdqd12_22*qd[29]
    ORjdqd12_22 = RLjdqd12_12*qd[29]
    Apqpjdqd12_12 = -ORjdqd12_22*qd[29]
    Apqpjdqd12_22 = ORjdqd12_12*qd[29]
    jdqd14 = Apqpjdqd11_34*POjdqd11_34+VIjdqd11_34*VIjdqd11_34+(-ORjdqd12_22+VIjdqd11_24)*(-ORjdqd12_22+VIjdqd11_24)+(Apqpjdqd11_14-Apqpjdqd12_12)*(POjdqd11_14-POjdqd12_12)+(Apqpjdqd11_24-Apqpjdqd12_22)*(POjdqd11_24-POjdqd12_22)+(ORjdqd11_14-ORjdqd12_12)*(ORjdqd11_14-ORjdqd12_12)
    RLjdqd13_12 = s.dpt[1,57]*C30-s.dpt[2,57]*S30
    RLjdqd13_22 = s.dpt[1,57]*S30+s.dpt[2,57]*C30
    POjdqd13_12 = RLjdqd13_12+s.dpt[1,18]
    POjdqd13_22 = RLjdqd13_22+s.dpt[2,18]
    ORjdqd13_12 = -RLjdqd13_22*qd[30]
    ORjdqd13_22 = RLjdqd13_12*qd[30]
    Apqpjdqd13_12 = -ORjdqd13_22*qd[30]
    Apqpjdqd13_22 = ORjdqd13_12*qd[30]
    ROjdqd14_52 = C13*C14-S13*S14
    ROjdqd14_62 = C13*S14+S13*C14
    ROjdqd14_82 = -C13*S14-S13*C14
    ROjdqd14_92 = C13*C14-S13*S14
    ROjdqd14_23 = ROjdqd14_52*S15
    ROjdqd14_33 = ROjdqd14_62*S15
    RLjdqd14_22 = s.dpt[2,30]*C13
    RLjdqd14_32 = s.dpt[2,30]*S13
    POjdqd14_22 = RLjdqd14_22+s.dpt[2,5]
    OMjdqd14_12 = qd[13]+qd[14]
    ORjdqd14_22 = -RLjdqd14_32*qd[13]
    ORjdqd14_32 = RLjdqd14_22*qd[13]
    Apqpjdqd14_22 = -ORjdqd14_32*qd[13]
    Apqpjdqd14_32 = ORjdqd14_22*qd[13]
    OMjdqd14_23 = ROjdqd14_82*qd[15]
    OMjdqd14_33 = ROjdqd14_92*qd[15]
    Ompqpjdqd14_23 = -OMjdqd14_12*ROjdqd14_92*qd[15]
    Ompqpjdqd14_33 = OMjdqd14_12*ROjdqd14_82*qd[15]
    RLjdqd14_14 = s.dpt[1,34]*C15
    RLjdqd14_24 = ROjdqd14_23*s.dpt[1,34]
    RLjdqd14_34 = ROjdqd14_33*s.dpt[1,34]
    POjdqd14_14 = RLjdqd14_14+s.dpt[1,5]
    POjdqd14_24 = POjdqd14_22+RLjdqd14_24
    POjdqd14_34 = RLjdqd14_32+RLjdqd14_34
    ORjdqd14_14 = OMjdqd14_23*RLjdqd14_34-OMjdqd14_33*RLjdqd14_24
    ORjdqd14_24 = -OMjdqd14_12*RLjdqd14_34+OMjdqd14_33*RLjdqd14_14
    ORjdqd14_34 = OMjdqd14_12*RLjdqd14_24-OMjdqd14_23*RLjdqd14_14
    VIjdqd14_24 = ORjdqd14_22+ORjdqd14_24
    VIjdqd14_34 = ORjdqd14_32+ORjdqd14_34
    Apqpjdqd14_14 = OMjdqd14_23*ORjdqd14_34-OMjdqd14_33*ORjdqd14_24+Ompqpjdqd14_23*RLjdqd14_34-Ompqpjdqd14_33*RLjdqd14_24
    Apqpjdqd14_24 = Apqpjdqd14_22-OMjdqd14_12*ORjdqd14_34+OMjdqd14_33*ORjdqd14_14+Ompqpjdqd14_33*RLjdqd14_14
    Apqpjdqd14_34 = Apqpjdqd14_32+OMjdqd14_12*ORjdqd14_24-OMjdqd14_23*ORjdqd14_14-Ompqpjdqd14_23*RLjdqd14_14
    jdqd15 = Apqpjdqd14_34*POjdqd14_34+VIjdqd14_34*VIjdqd14_34+(Apqpjdqd13_12-Apqpjdqd14_14)*(POjdqd13_12-POjdqd14_14)+(Apqpjdqd13_22-Apqpjdqd14_24)*(POjdqd13_22-POjdqd14_24)+(ORjdqd13_12-ORjdqd14_14)*(ORjdqd13_12-ORjdqd14_14)+(ORjdqd13_22-VIjdqd14_24)*(ORjdqd13_22-VIjdqd14_24)
    Jdqd[1] = -Apqpjdqd2_18
    Jdqd[2] = -Apqpjdqd2_28
    Jdqd[3] = -Apqpjdqd2_38
    Jdqd[4] = Apqpjdqd3_18
    Jdqd[5] = Apqpjdqd3_28
    Jdqd[6] = Apqpjdqd3_38
    Jdqd[7] = -Apqpjdqd6_14
    Jdqd[8] = jdqd8
    Jdqd[9] = jdqd9
    Jdqd[10] = Apqpjdqd7_14
    Jdqd[11] = jdqd11
    Jdqd[12] = jdqd12
    Jdqd[13] = jdqd13
    Jdqd[14] = jdqd14
    Jdqd[15] = jdqd15

# Number of continuation lines = 0


