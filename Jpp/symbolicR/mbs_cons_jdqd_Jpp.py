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
#	==> Generation Date: Mon Mar 24 20:40:33 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 28
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
    RLjdqd2_17 = ROjdqd2_46*s.dpt[2,32]+s.dpt[3,32]*S23
    RLjdqd2_27 = ROjdqd2_56*s.dpt[2,32]+ROjdqd2_85*s.dpt[3,32]
    RLjdqd2_37 = ROjdqd2_66*s.dpt[2,32]+ROjdqd2_95*s.dpt[3,32]
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
    RLjdqd2_18 = ROjdqd2_17*s.dpt[1,40]
    RLjdqd2_28 = ROjdqd2_27*s.dpt[1,40]
    RLjdqd2_38 = ROjdqd2_37*s.dpt[1,40]
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
    RLjdqd3_17 = ROjdqd3_46*s.dpt[2,35]+s.dpt[3,35]*S23
    RLjdqd3_27 = ROjdqd3_56*s.dpt[2,35]+ROjdqd3_85*s.dpt[3,35]
    RLjdqd3_37 = ROjdqd3_66*s.dpt[2,35]+ROjdqd3_95*s.dpt[3,35]
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
    RLjdqd3_18 = ROjdqd3_17*s.dpt[1,41]
    RLjdqd3_28 = ROjdqd3_27*s.dpt[1,41]
    RLjdqd3_38 = ROjdqd3_37*s.dpt[1,41]
    ORjdqd3_18 = OMjdqd3_27*RLjdqd3_38-OMjdqd3_37*RLjdqd3_28
    ORjdqd3_28 = -OMjdqd3_17*RLjdqd3_38+OMjdqd3_37*RLjdqd3_18
    ORjdqd3_38 = OMjdqd3_17*RLjdqd3_28-OMjdqd3_27*RLjdqd3_18
    Apqpjdqd3_18 = Apqpjdqd3_17+OMjdqd3_27*ORjdqd3_38-OMjdqd3_37*ORjdqd3_28+Ompqpjdqd3_27*RLjdqd3_38-Ompqpjdqd3_37*RLjdqd3_28
    Apqpjdqd3_28 = Apqpjdqd3_27-OMjdqd3_17*ORjdqd3_38+OMjdqd3_37*ORjdqd3_18-Ompqpjdqd3_17*RLjdqd3_38+Ompqpjdqd3_37*RLjdqd3_18
    Apqpjdqd3_38 = Apqpjdqd3_37+OMjdqd3_17*ORjdqd3_28-OMjdqd3_27*ORjdqd3_18+Ompqpjdqd3_17*RLjdqd3_28-Ompqpjdqd3_27*RLjdqd3_18
    RLjdqd5_22 = s.dpt[2,22]*C12
    RLjdqd5_32 = s.dpt[2,22]*S12
    ORjdqd5_22 = -RLjdqd5_32*qd[12]
    ORjdqd5_32 = RLjdqd5_22*qd[12]
    Apqpjdqd5_22 = -ORjdqd5_32*qd[12]
    Apqpjdqd5_32 = ORjdqd5_22*qd[12]
    ROjdqd6_82 = -C7*S8-S7*C8
    ROjdqd6_92 = C7*C8-S7*S8
    RLjdqd6_22 = s.dpt[2,16]*C7
    RLjdqd6_32 = s.dpt[2,16]*S7
    OMjdqd6_12 = qd[7]+qd[8]
    ORjdqd6_22 = -RLjdqd6_32*qd[7]
    ORjdqd6_32 = RLjdqd6_22*qd[7]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[7]
    Apqpjdqd6_32 = ORjdqd6_22*qd[7]
    OMjdqd6_23 = ROjdqd6_82*qd[9]
    OMjdqd6_33 = ROjdqd6_92*qd[9]
    Ompqpjdqd6_23 = -OMjdqd6_12*ROjdqd6_92*qd[9]
    Ompqpjdqd6_33 = OMjdqd6_12*ROjdqd6_82*qd[9]
    RLjdqd6_24 = ROjdqd6_82*s.dpt[3,18]
    RLjdqd6_34 = ROjdqd6_92*s.dpt[3,18]
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
    RLjdqd7_22 = s.dpt[2,23]*C13
    RLjdqd7_32 = s.dpt[2,23]*S13
    OMjdqd7_12 = qd[13]+qd[14]
    ORjdqd7_22 = -RLjdqd7_32*qd[13]
    ORjdqd7_32 = RLjdqd7_22*qd[13]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[13]
    Apqpjdqd7_32 = ORjdqd7_22*qd[13]
    OMjdqd7_23 = ROjdqd7_82*qd[15]
    OMjdqd7_33 = ROjdqd7_92*qd[15]
    Ompqpjdqd7_23 = -OMjdqd7_12*ROjdqd7_92*qd[15]
    Ompqpjdqd7_33 = OMjdqd7_12*ROjdqd7_82*qd[15]
    RLjdqd7_24 = ROjdqd7_82*s.dpt[3,25]
    RLjdqd7_34 = ROjdqd7_92*s.dpt[3,25]
    ORjdqd7_14 = OMjdqd7_23*RLjdqd7_34-OMjdqd7_33*RLjdqd7_24
    ORjdqd7_24 = -OMjdqd7_12*RLjdqd7_34
    ORjdqd7_34 = OMjdqd7_12*RLjdqd7_24
    Apqpjdqd7_14 = OMjdqd7_23*ORjdqd7_34-OMjdqd7_33*ORjdqd7_24+Ompqpjdqd7_23*RLjdqd7_34-Ompqpjdqd7_33*RLjdqd7_24
    Apqpjdqd7_24 = Apqpjdqd7_22-OMjdqd7_12*ORjdqd7_34+OMjdqd7_33*ORjdqd7_14
    Apqpjdqd7_34 = Apqpjdqd7_32+OMjdqd7_12*ORjdqd7_24-OMjdqd7_23*ORjdqd7_14
    RLjdqd8_22 = s.dpt[2,29]*C18
    RLjdqd8_32 = s.dpt[2,29]*S18
    ORjdqd8_22 = -RLjdqd8_32*qd[18]
    ORjdqd8_32 = RLjdqd8_22*qd[18]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[18]
    Apqpjdqd8_32 = ORjdqd8_22*qd[18]
    jdqd11 = Apqpjdqd7_24-Apqpjdqd8_22
    jdqd12 = Apqpjdqd7_34-Apqpjdqd8_32
    ROjdqd9_52 = C7*C8-S7*S8
    ROjdqd9_62 = C7*S8+S7*C8
    ROjdqd9_82 = -C7*S8-S7*C8
    ROjdqd9_92 = C7*C8-S7*S8
    ROjdqd9_23 = ROjdqd9_52*S9
    ROjdqd9_33 = ROjdqd9_62*S9
    RLjdqd9_22 = s.dpt[2,16]*C7
    RLjdqd9_32 = s.dpt[2,16]*S7
    POjdqd9_22 = RLjdqd9_22+s.dpt[2,1]
    OMjdqd9_12 = qd[7]+qd[8]
    ORjdqd9_22 = -RLjdqd9_32*qd[7]
    ORjdqd9_32 = RLjdqd9_22*qd[7]
    Apqpjdqd9_22 = -ORjdqd9_32*qd[7]
    Apqpjdqd9_32 = ORjdqd9_22*qd[7]
    OMjdqd9_23 = ROjdqd9_82*qd[9]
    OMjdqd9_33 = ROjdqd9_92*qd[9]
    Ompqpjdqd9_23 = -OMjdqd9_12*ROjdqd9_92*qd[9]
    Ompqpjdqd9_33 = OMjdqd9_12*ROjdqd9_82*qd[9]
    RLjdqd9_14 = s.dpt[1,20]*C9
    RLjdqd9_24 = ROjdqd9_23*s.dpt[1,20]
    RLjdqd9_34 = ROjdqd9_33*s.dpt[1,20]
    POjdqd9_14 = RLjdqd9_14+s.dpt[1,1]
    POjdqd9_24 = POjdqd9_22+RLjdqd9_24
    POjdqd9_34 = RLjdqd9_32+RLjdqd9_34
    ORjdqd9_14 = OMjdqd9_23*RLjdqd9_34-OMjdqd9_33*RLjdqd9_24
    ORjdqd9_24 = -OMjdqd9_12*RLjdqd9_34+OMjdqd9_33*RLjdqd9_14
    ORjdqd9_34 = OMjdqd9_12*RLjdqd9_24-OMjdqd9_23*RLjdqd9_14
    VIjdqd9_24 = ORjdqd9_22+ORjdqd9_24
    VIjdqd9_34 = ORjdqd9_32+ORjdqd9_34
    Apqpjdqd9_14 = OMjdqd9_23*ORjdqd9_34-OMjdqd9_33*ORjdqd9_24+Ompqpjdqd9_23*RLjdqd9_34-Ompqpjdqd9_33*RLjdqd9_24
    Apqpjdqd9_24 = Apqpjdqd9_22-OMjdqd9_12*ORjdqd9_34+OMjdqd9_33*ORjdqd9_14+Ompqpjdqd9_33*RLjdqd9_14
    Apqpjdqd9_34 = Apqpjdqd9_32+OMjdqd9_12*ORjdqd9_24-OMjdqd9_23*ORjdqd9_14-Ompqpjdqd9_23*RLjdqd9_14
    ROjdqd10_52 = C13*C14-S13*S14
    ROjdqd10_62 = C13*S14+S13*C14
    ROjdqd10_82 = -C13*S14-S13*C14
    ROjdqd10_92 = C13*C14-S13*S14
    ROjdqd10_23 = ROjdqd10_52*S15
    ROjdqd10_33 = ROjdqd10_62*S15
    RLjdqd10_22 = s.dpt[2,23]*C13
    RLjdqd10_32 = s.dpt[2,23]*S13
    POjdqd10_22 = RLjdqd10_22+s.dpt[2,4]
    OMjdqd10_12 = qd[13]+qd[14]
    ORjdqd10_22 = -RLjdqd10_32*qd[13]
    ORjdqd10_32 = RLjdqd10_22*qd[13]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[13]
    Apqpjdqd10_32 = ORjdqd10_22*qd[13]
    OMjdqd10_23 = ROjdqd10_82*qd[15]
    OMjdqd10_33 = ROjdqd10_92*qd[15]
    Ompqpjdqd10_23 = -OMjdqd10_12*ROjdqd10_92*qd[15]
    Ompqpjdqd10_33 = OMjdqd10_12*ROjdqd10_82*qd[15]
    RLjdqd10_14 = s.dpt[1,27]*C15
    RLjdqd10_24 = ROjdqd10_23*s.dpt[1,27]
    RLjdqd10_34 = ROjdqd10_33*s.dpt[1,27]
    POjdqd10_14 = RLjdqd10_14+s.dpt[1,4]
    POjdqd10_24 = POjdqd10_22+RLjdqd10_24
    POjdqd10_34 = RLjdqd10_32+RLjdqd10_34
    ORjdqd10_14 = OMjdqd10_23*RLjdqd10_34-OMjdqd10_33*RLjdqd10_24
    ORjdqd10_24 = -OMjdqd10_12*RLjdqd10_34+OMjdqd10_33*RLjdqd10_14
    ORjdqd10_34 = OMjdqd10_12*RLjdqd10_24-OMjdqd10_23*RLjdqd10_14
    VIjdqd10_24 = ORjdqd10_22+ORjdqd10_24
    VIjdqd10_34 = ORjdqd10_32+ORjdqd10_34
    Apqpjdqd10_14 = OMjdqd10_23*ORjdqd10_34-OMjdqd10_33*ORjdqd10_24+Ompqpjdqd10_23*RLjdqd10_34-Ompqpjdqd10_33*RLjdqd10_24
    Apqpjdqd10_24 = Apqpjdqd10_22-OMjdqd10_12*ORjdqd10_34+OMjdqd10_33*ORjdqd10_14+Ompqpjdqd10_33*RLjdqd10_14
    Apqpjdqd10_34 = Apqpjdqd10_32+OMjdqd10_12*ORjdqd10_24-OMjdqd10_23*ORjdqd10_14-Ompqpjdqd10_23*RLjdqd10_14
    jdqd13 = (-Apqpjdqd10_14+Apqpjdqd9_14)*(-POjdqd10_14+POjdqd9_14)+(-Apqpjdqd10_24+Apqpjdqd9_24)*(-POjdqd10_24+POjdqd9_24)+(-Apqpjdqd10_34+Apqpjdqd9_34)*(-POjdqd10_34+POjdqd9_34)+(-ORjdqd10_14+ORjdqd9_14)*(-ORjdqd10_14+ORjdqd9_14)+(-VIjdqd10_24+VIjdqd9_24)*(-VIjdqd10_24+VIjdqd9_24)+(-VIjdqd10_34+VIjdqd9_34)*(-VIjdqd10_34+VIjdqd9_34)
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

# Number of continuation lines = 0


