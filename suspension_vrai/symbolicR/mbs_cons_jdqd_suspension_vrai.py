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
#	==> Generation Date: Mon Mar 24 14:14:28 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 32
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
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S32 = sin(q[32])
    C32 = cos(q[32])
 
# Augmented Joint Position Vectors

    Dz191 = q[19]+s.dpt[1,7]
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,16]*C7
    RLjdqd1_32 = s.dpt[2,16]*S7
    ORjdqd1_22 = -RLjdqd1_32*qd[7]
    ORjdqd1_32 = RLjdqd1_22*qd[7]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = ORjdqd1_22*qd[7]
    ROjdqd2_82 = -C8*S9-S8*C9
    ROjdqd2_92 = C8*C9-S8*S9
    RLjdqd2_22 = s.dpt[2,17]*C8
    RLjdqd2_32 = s.dpt[2,17]*S8
    OMjdqd2_12 = qd[8]+qd[9]
    ORjdqd2_22 = -RLjdqd2_32*qd[8]
    ORjdqd2_32 = RLjdqd2_22*qd[8]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[8]
    Apqpjdqd2_32 = ORjdqd2_22*qd[8]
    OMjdqd2_23 = ROjdqd2_82*qd[10]
    OMjdqd2_33 = ROjdqd2_92*qd[10]
    Ompqpjdqd2_23 = -OMjdqd2_12*ROjdqd2_92*qd[10]
    Ompqpjdqd2_33 = OMjdqd2_12*ROjdqd2_82*qd[10]
    RLjdqd2_24 = ROjdqd2_82*s.dpt[3,19]
    RLjdqd2_34 = ROjdqd2_92*s.dpt[3,19]
    ORjdqd2_14 = OMjdqd2_23*RLjdqd2_34-OMjdqd2_33*RLjdqd2_24
    ORjdqd2_24 = -OMjdqd2_12*RLjdqd2_34
    ORjdqd2_34 = OMjdqd2_12*RLjdqd2_24
    Apqpjdqd2_14 = OMjdqd2_23*ORjdqd2_34-OMjdqd2_33*ORjdqd2_24+Ompqpjdqd2_23*RLjdqd2_34-Ompqpjdqd2_33*RLjdqd2_24
    Apqpjdqd2_24 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_34+OMjdqd2_33*ORjdqd2_14
    Apqpjdqd2_34 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_24-OMjdqd2_23*ORjdqd2_14
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_24
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_34
    RLjdqd3_22 = s.dpt[2,22]*C13
    RLjdqd3_32 = s.dpt[2,22]*S13
    ORjdqd3_22 = -RLjdqd3_32*qd[13]
    ORjdqd3_32 = RLjdqd3_22*qd[13]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[13]
    Apqpjdqd3_32 = ORjdqd3_22*qd[13]
    ROjdqd4_52 = C14*C15-S14*S15
    ROjdqd4_62 = C14*S15+S14*C15
    ROjdqd4_82 = -C14*S15-S14*C15
    ROjdqd4_92 = C14*C15-S14*S15
    ROjdqd4_53 = ROjdqd4_52*C16
    ROjdqd4_63 = ROjdqd4_62*C16
    RLjdqd4_22 = s.dpt[2,23]*C14
    RLjdqd4_32 = s.dpt[2,23]*S14
    OMjdqd4_12 = qd[14]+qd[15]
    ORjdqd4_22 = -RLjdqd4_32*qd[14]
    ORjdqd4_32 = RLjdqd4_22*qd[14]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[14]
    Apqpjdqd4_32 = ORjdqd4_22*qd[14]
    OMjdqd4_23 = ROjdqd4_82*qd[16]
    OMjdqd4_33 = ROjdqd4_92*qd[16]
    Ompqpjdqd4_23 = -OMjdqd4_12*ROjdqd4_92*qd[16]
    Ompqpjdqd4_33 = OMjdqd4_12*ROjdqd4_82*qd[16]
    RLjdqd4_14 = -s.dpt[2,25]*S16
    RLjdqd4_24 = ROjdqd4_53*s.dpt[2,25]
    RLjdqd4_34 = ROjdqd4_63*s.dpt[2,25]
    ORjdqd4_14 = OMjdqd4_23*RLjdqd4_34-OMjdqd4_33*RLjdqd4_24
    ORjdqd4_24 = -OMjdqd4_12*RLjdqd4_34+OMjdqd4_33*RLjdqd4_14
    ORjdqd4_34 = OMjdqd4_12*RLjdqd4_24-OMjdqd4_23*RLjdqd4_14
    Apqpjdqd4_14 = OMjdqd4_23*ORjdqd4_34-OMjdqd4_33*ORjdqd4_24+Ompqpjdqd4_23*RLjdqd4_34-Ompqpjdqd4_33*RLjdqd4_24
    Apqpjdqd4_24 = Apqpjdqd4_22-OMjdqd4_12*ORjdqd4_34+OMjdqd4_33*ORjdqd4_14+Ompqpjdqd4_33*RLjdqd4_14
    Apqpjdqd4_34 = Apqpjdqd4_32+OMjdqd4_12*ORjdqd4_24-OMjdqd4_23*ORjdqd4_14-Ompqpjdqd4_23*RLjdqd4_14
    jdqd5 = Apqpjdqd3_22-Apqpjdqd4_24
    jdqd6 = Apqpjdqd3_32-Apqpjdqd4_34
    ROjdqd6_15 = C22*C23
    ROjdqd6_25 = S22*C23
    ROjdqd6_75 = C22*S23
    ROjdqd6_85 = S22*S23
    ROjdqd6_46 = ROjdqd6_75*S24-S22*C24
    ROjdqd6_56 = ROjdqd6_85*S24+C22*C24
    ROjdqd6_66 = C23*S24
    ROjdqd6_76 = ROjdqd6_75*C24+S22*S24
    ROjdqd6_86 = ROjdqd6_85*C24-C22*S24
    ROjdqd6_96 = C23*C24
    ROjdqd6_17 = ROjdqd6_15*C27-ROjdqd6_76*S27
    ROjdqd6_27 = ROjdqd6_25*C27-ROjdqd6_86*S27
    ROjdqd6_37 = -ROjdqd6_96*S27-S23*C27
    OMjdqd6_15 = -qd[23]*S22
    OMjdqd6_25 = qd[23]*C22
    Ompqpjdqd6_15 = -qd[22]*qd[23]*C22
    Ompqpjdqd6_25 = -qd[22]*qd[23]*S22
    OMjdqd6_16 = OMjdqd6_15+ROjdqd6_15*qd[24]
    OMjdqd6_26 = OMjdqd6_25+ROjdqd6_25*qd[24]
    OMjdqd6_36 = qd[22]-qd[24]*S23
    Ompqpjdqd6_16 = Ompqpjdqd6_15+qd[24]*(-OMjdqd6_25*S23-ROjdqd6_25*qd[22])
    Ompqpjdqd6_26 = Ompqpjdqd6_25+qd[24]*(OMjdqd6_15*S23+ROjdqd6_15*qd[22])
    Ompqpjdqd6_36 = qd[24]*(OMjdqd6_15*ROjdqd6_25-OMjdqd6_25*ROjdqd6_15)
    RLjdqd6_17 = ROjdqd6_76*s.dpt[3,34]
    RLjdqd6_27 = ROjdqd6_86*s.dpt[3,34]
    RLjdqd6_37 = ROjdqd6_96*s.dpt[3,34]
    OMjdqd6_17 = OMjdqd6_16+ROjdqd6_46*qd[27]
    OMjdqd6_27 = OMjdqd6_26+ROjdqd6_56*qd[27]
    OMjdqd6_37 = OMjdqd6_36+ROjdqd6_66*qd[27]
    ORjdqd6_17 = OMjdqd6_26*RLjdqd6_37-OMjdqd6_36*RLjdqd6_27
    ORjdqd6_27 = -OMjdqd6_16*RLjdqd6_37+OMjdqd6_36*RLjdqd6_17
    ORjdqd6_37 = OMjdqd6_16*RLjdqd6_27-OMjdqd6_26*RLjdqd6_17
    Ompqpjdqd6_17 = Ompqpjdqd6_16+qd[27]*(OMjdqd6_26*ROjdqd6_66-OMjdqd6_36*ROjdqd6_56)
    Ompqpjdqd6_27 = Ompqpjdqd6_26+qd[27]*(-OMjdqd6_16*ROjdqd6_66+OMjdqd6_36*ROjdqd6_46)
    Ompqpjdqd6_37 = Ompqpjdqd6_36+qd[27]*(OMjdqd6_16*ROjdqd6_56-OMjdqd6_26*ROjdqd6_46)
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
    ROjdqd8_15 = C22*C23
    ROjdqd8_25 = S22*C23
    ROjdqd8_75 = C22*S23
    ROjdqd8_85 = S22*S23
    ROjdqd8_46 = ROjdqd8_75*S24-S22*C24
    ROjdqd8_56 = ROjdqd8_85*S24+C22*C24
    ROjdqd8_66 = C23*S24
    ROjdqd8_76 = ROjdqd8_75*C24+S22*S24
    ROjdqd8_86 = ROjdqd8_85*C24-C22*S24
    ROjdqd8_96 = C23*C24
    ROjdqd8_17 = ROjdqd8_15*C30+ROjdqd8_46*S30
    ROjdqd8_27 = ROjdqd8_25*C30+ROjdqd8_56*S30
    ROjdqd8_37 = ROjdqd8_66*S30-S23*C30
    ROjdqd8_47 = -ROjdqd8_15*S30+ROjdqd8_46*C30
    ROjdqd8_57 = -ROjdqd8_25*S30+ROjdqd8_56*C30
    ROjdqd8_67 = ROjdqd8_66*C30+S23*S30
    ROjdqd8_18 = ROjdqd8_17*C31-ROjdqd8_76*S31
    ROjdqd8_28 = ROjdqd8_27*C31-ROjdqd8_86*S31
    ROjdqd8_38 = ROjdqd8_37*C31-ROjdqd8_96*S31
    OMjdqd8_15 = -qd[23]*S22
    OMjdqd8_25 = qd[23]*C22
    Ompqpjdqd8_15 = -qd[22]*qd[23]*C22
    Ompqpjdqd8_25 = -qd[22]*qd[23]*S22
    OMjdqd8_16 = OMjdqd8_15+ROjdqd8_15*qd[24]
    OMjdqd8_26 = OMjdqd8_25+ROjdqd8_25*qd[24]
    OMjdqd8_36 = qd[22]-qd[24]*S23
    Ompqpjdqd8_16 = Ompqpjdqd8_15+qd[24]*(-OMjdqd8_25*S23-ROjdqd8_25*qd[22])
    Ompqpjdqd8_26 = Ompqpjdqd8_25+qd[24]*(OMjdqd8_15*S23+ROjdqd8_15*qd[22])
    Ompqpjdqd8_36 = qd[24]*(OMjdqd8_15*ROjdqd8_25-OMjdqd8_25*ROjdqd8_15)
    RLjdqd8_17 = ROjdqd8_76*s.dpt[3,36]
    RLjdqd8_27 = ROjdqd8_86*s.dpt[3,36]
    RLjdqd8_37 = ROjdqd8_96*s.dpt[3,36]
    OMjdqd8_17 = OMjdqd8_16+ROjdqd8_76*qd[30]
    OMjdqd8_27 = OMjdqd8_26+ROjdqd8_86*qd[30]
    OMjdqd8_37 = OMjdqd8_36+ROjdqd8_96*qd[30]
    ORjdqd8_17 = OMjdqd8_26*RLjdqd8_37-OMjdqd8_36*RLjdqd8_27
    ORjdqd8_27 = -OMjdqd8_16*RLjdqd8_37+OMjdqd8_36*RLjdqd8_17
    ORjdqd8_37 = OMjdqd8_16*RLjdqd8_27-OMjdqd8_26*RLjdqd8_17
    Ompqpjdqd8_17 = Ompqpjdqd8_16+qd[30]*(OMjdqd8_26*ROjdqd8_96-OMjdqd8_36*ROjdqd8_86)
    Ompqpjdqd8_27 = Ompqpjdqd8_26+qd[30]*(-OMjdqd8_16*ROjdqd8_96+OMjdqd8_36*ROjdqd8_76)
    Ompqpjdqd8_37 = Ompqpjdqd8_36+qd[30]*(OMjdqd8_16*ROjdqd8_86-OMjdqd8_26*ROjdqd8_76)
    Apqpjdqd8_17 = OMjdqd8_26*ORjdqd8_37-OMjdqd8_36*ORjdqd8_27+Ompqpjdqd8_26*RLjdqd8_37-Ompqpjdqd8_36*RLjdqd8_27
    Apqpjdqd8_27 = -OMjdqd8_16*ORjdqd8_37+OMjdqd8_36*ORjdqd8_17-Ompqpjdqd8_16*RLjdqd8_37+Ompqpjdqd8_36*RLjdqd8_17
    Apqpjdqd8_37 = OMjdqd8_16*ORjdqd8_27-OMjdqd8_26*ORjdqd8_17+Ompqpjdqd8_16*RLjdqd8_27-Ompqpjdqd8_26*RLjdqd8_17
    OMjdqd8_18 = OMjdqd8_17+ROjdqd8_47*qd[31]
    OMjdqd8_28 = OMjdqd8_27+ROjdqd8_57*qd[31]
    OMjdqd8_38 = OMjdqd8_37+ROjdqd8_67*qd[31]
    Ompqpjdqd8_18 = Ompqpjdqd8_17+qd[31]*(OMjdqd8_27*ROjdqd8_67-OMjdqd8_37*ROjdqd8_57)
    Ompqpjdqd8_28 = Ompqpjdqd8_27+qd[31]*(-OMjdqd8_17*ROjdqd8_67+OMjdqd8_37*ROjdqd8_47)
    Ompqpjdqd8_38 = Ompqpjdqd8_37+qd[31]*(OMjdqd8_17*ROjdqd8_57-OMjdqd8_27*ROjdqd8_47)
    RLjdqd8_19 = ROjdqd8_18*s.dpt[1,40]
    RLjdqd8_29 = ROjdqd8_28*s.dpt[1,40]
    RLjdqd8_39 = ROjdqd8_38*s.dpt[1,40]
    ORjdqd8_19 = OMjdqd8_28*RLjdqd8_39-OMjdqd8_38*RLjdqd8_29
    ORjdqd8_29 = -OMjdqd8_18*RLjdqd8_39+OMjdqd8_38*RLjdqd8_19
    ORjdqd8_39 = OMjdqd8_18*RLjdqd8_29-OMjdqd8_28*RLjdqd8_19
    Apqpjdqd8_19 = Apqpjdqd8_17+OMjdqd8_28*ORjdqd8_39-OMjdqd8_38*ORjdqd8_29+Ompqpjdqd8_28*RLjdqd8_39-Ompqpjdqd8_38*RLjdqd8_29
    Apqpjdqd8_29 = Apqpjdqd8_27-OMjdqd8_18*ORjdqd8_39+OMjdqd8_38*ORjdqd8_19-Ompqpjdqd8_18*RLjdqd8_39+Ompqpjdqd8_38*RLjdqd8_19
    Apqpjdqd8_39 = Apqpjdqd8_37+OMjdqd8_18*ORjdqd8_29-OMjdqd8_28*ORjdqd8_19+Ompqpjdqd8_18*RLjdqd8_29-Ompqpjdqd8_28*RLjdqd8_19
    ROjdqd10_15 = C22*C23
    ROjdqd10_25 = S22*C23
    ROjdqd10_75 = C22*S23
    ROjdqd10_85 = S22*S23
    ROjdqd10_46 = ROjdqd10_75*S24-S22*C24
    ROjdqd10_56 = ROjdqd10_85*S24+C22*C24
    ROjdqd10_66 = C23*S24
    ROjdqd10_76 = ROjdqd10_75*C24+S22*S24
    ROjdqd10_86 = ROjdqd10_85*C24-C22*S24
    ROjdqd10_96 = C23*C24
    ROjdqd10_17 = ROjdqd10_15*C28+ROjdqd10_46*S28
    ROjdqd10_27 = ROjdqd10_25*C28+ROjdqd10_56*S28
    ROjdqd10_37 = ROjdqd10_66*S28-S23*C28
    ROjdqd10_47 = -ROjdqd10_15*S28+ROjdqd10_46*C28
    ROjdqd10_57 = -ROjdqd10_25*S28+ROjdqd10_56*C28
    ROjdqd10_67 = ROjdqd10_66*C28+S23*S28
    ROjdqd10_18 = ROjdqd10_17*C29-ROjdqd10_76*S29
    ROjdqd10_28 = ROjdqd10_27*C29-ROjdqd10_86*S29
    ROjdqd10_38 = ROjdqd10_37*C29-ROjdqd10_96*S29
    OMjdqd10_15 = -qd[23]*S22
    OMjdqd10_25 = qd[23]*C22
    Ompqpjdqd10_15 = -qd[22]*qd[23]*C22
    Ompqpjdqd10_25 = -qd[22]*qd[23]*S22
    OMjdqd10_16 = OMjdqd10_15+ROjdqd10_15*qd[24]
    OMjdqd10_26 = OMjdqd10_25+ROjdqd10_25*qd[24]
    OMjdqd10_36 = qd[22]-qd[24]*S23
    Ompqpjdqd10_16 = Ompqpjdqd10_15+qd[24]*(-OMjdqd10_25*S23-ROjdqd10_25*qd[22])
    Ompqpjdqd10_26 = Ompqpjdqd10_25+qd[24]*(OMjdqd10_15*S23+ROjdqd10_15*qd[22])
    Ompqpjdqd10_36 = qd[24]*(OMjdqd10_15*ROjdqd10_25-OMjdqd10_25*ROjdqd10_15)
    RLjdqd10_17 = ROjdqd10_76*s.dpt[3,35]
    RLjdqd10_27 = ROjdqd10_86*s.dpt[3,35]
    RLjdqd10_37 = ROjdqd10_96*s.dpt[3,35]
    OMjdqd10_17 = OMjdqd10_16+ROjdqd10_76*qd[28]
    OMjdqd10_27 = OMjdqd10_26+ROjdqd10_86*qd[28]
    OMjdqd10_37 = OMjdqd10_36+ROjdqd10_96*qd[28]
    ORjdqd10_17 = OMjdqd10_26*RLjdqd10_37-OMjdqd10_36*RLjdqd10_27
    ORjdqd10_27 = -OMjdqd10_16*RLjdqd10_37+OMjdqd10_36*RLjdqd10_17
    ORjdqd10_37 = OMjdqd10_16*RLjdqd10_27-OMjdqd10_26*RLjdqd10_17
    Ompqpjdqd10_17 = Ompqpjdqd10_16+qd[28]*(OMjdqd10_26*ROjdqd10_96-OMjdqd10_36*ROjdqd10_86)
    Ompqpjdqd10_27 = Ompqpjdqd10_26+qd[28]*(-OMjdqd10_16*ROjdqd10_96+OMjdqd10_36*ROjdqd10_76)
    Ompqpjdqd10_37 = Ompqpjdqd10_36+qd[28]*(OMjdqd10_16*ROjdqd10_86-OMjdqd10_26*ROjdqd10_76)
    Apqpjdqd10_17 = OMjdqd10_26*ORjdqd10_37-OMjdqd10_36*ORjdqd10_27+Ompqpjdqd10_26*RLjdqd10_37-Ompqpjdqd10_36*RLjdqd10_27
    Apqpjdqd10_27 = -OMjdqd10_16*ORjdqd10_37+OMjdqd10_36*ORjdqd10_17-Ompqpjdqd10_16*RLjdqd10_37+Ompqpjdqd10_36*RLjdqd10_17
    Apqpjdqd10_37 = OMjdqd10_16*ORjdqd10_27-OMjdqd10_26*ORjdqd10_17+Ompqpjdqd10_16*RLjdqd10_27-Ompqpjdqd10_26*RLjdqd10_17
    OMjdqd10_18 = OMjdqd10_17+ROjdqd10_47*qd[29]
    OMjdqd10_28 = OMjdqd10_27+ROjdqd10_57*qd[29]
    OMjdqd10_38 = OMjdqd10_37+ROjdqd10_67*qd[29]
    Ompqpjdqd10_18 = Ompqpjdqd10_17+qd[29]*(OMjdqd10_27*ROjdqd10_67-OMjdqd10_37*ROjdqd10_57)
    Ompqpjdqd10_28 = Ompqpjdqd10_27+qd[29]*(-OMjdqd10_17*ROjdqd10_67+OMjdqd10_37*ROjdqd10_47)
    Ompqpjdqd10_38 = Ompqpjdqd10_37+qd[29]*(OMjdqd10_17*ROjdqd10_57-OMjdqd10_27*ROjdqd10_47)
    RLjdqd10_19 = ROjdqd10_18*s.dpt[1,39]
    RLjdqd10_29 = ROjdqd10_28*s.dpt[1,39]
    RLjdqd10_39 = ROjdqd10_38*s.dpt[1,39]
    ORjdqd10_19 = OMjdqd10_28*RLjdqd10_39-OMjdqd10_38*RLjdqd10_29
    ORjdqd10_29 = -OMjdqd10_18*RLjdqd10_39+OMjdqd10_38*RLjdqd10_19
    ORjdqd10_39 = OMjdqd10_18*RLjdqd10_29-OMjdqd10_28*RLjdqd10_19
    Apqpjdqd10_19 = Apqpjdqd10_17+OMjdqd10_28*ORjdqd10_39-OMjdqd10_38*ORjdqd10_29+Ompqpjdqd10_28*RLjdqd10_39-Ompqpjdqd10_38*RLjdqd10_29
    Apqpjdqd10_29 = Apqpjdqd10_27-OMjdqd10_18*ORjdqd10_39+OMjdqd10_38*ORjdqd10_19-Ompqpjdqd10_18*RLjdqd10_39+Ompqpjdqd10_38*RLjdqd10_19
    Apqpjdqd10_39 = Apqpjdqd10_37+OMjdqd10_18*ORjdqd10_29-OMjdqd10_28*ORjdqd10_19+Ompqpjdqd10_18*RLjdqd10_29-Ompqpjdqd10_28*RLjdqd10_19
    ROjdqd12_15 = C22*C23
    ROjdqd12_25 = S22*C23
    ROjdqd12_75 = C22*S23
    ROjdqd12_85 = S22*S23
    ROjdqd12_46 = ROjdqd12_75*S24-S22*C24
    ROjdqd12_56 = ROjdqd12_85*S24+C22*C24
    ROjdqd12_66 = C23*S24
    ROjdqd12_76 = ROjdqd12_75*C24+S22*S24
    ROjdqd12_86 = ROjdqd12_85*C24-C22*S24
    ROjdqd12_96 = C23*C24
    ROjdqd12_17 = ROjdqd12_15*C32-ROjdqd12_76*S32
    ROjdqd12_27 = ROjdqd12_25*C32-ROjdqd12_86*S32
    ROjdqd12_37 = -ROjdqd12_96*S32-S23*C32
    OMjdqd12_15 = -qd[23]*S22
    OMjdqd12_25 = qd[23]*C22
    Ompqpjdqd12_15 = -qd[22]*qd[23]*C22
    Ompqpjdqd12_25 = -qd[22]*qd[23]*S22
    OMjdqd12_16 = OMjdqd12_15+ROjdqd12_15*qd[24]
    OMjdqd12_26 = OMjdqd12_25+ROjdqd12_25*qd[24]
    OMjdqd12_36 = qd[22]-qd[24]*S23
    Ompqpjdqd12_16 = Ompqpjdqd12_15+qd[24]*(-OMjdqd12_25*S23-ROjdqd12_25*qd[22])
    Ompqpjdqd12_26 = Ompqpjdqd12_25+qd[24]*(OMjdqd12_15*S23+ROjdqd12_15*qd[22])
    Ompqpjdqd12_36 = qd[24]*(OMjdqd12_15*ROjdqd12_25-OMjdqd12_25*ROjdqd12_15)
    RLjdqd12_17 = ROjdqd12_76*s.dpt[3,37]
    RLjdqd12_27 = ROjdqd12_86*s.dpt[3,37]
    RLjdqd12_37 = ROjdqd12_96*s.dpt[3,37]
    OMjdqd12_17 = OMjdqd12_16+ROjdqd12_46*qd[32]
    OMjdqd12_27 = OMjdqd12_26+ROjdqd12_56*qd[32]
    OMjdqd12_37 = OMjdqd12_36+ROjdqd12_66*qd[32]
    ORjdqd12_17 = OMjdqd12_26*RLjdqd12_37-OMjdqd12_36*RLjdqd12_27
    ORjdqd12_27 = -OMjdqd12_16*RLjdqd12_37+OMjdqd12_36*RLjdqd12_17
    ORjdqd12_37 = OMjdqd12_16*RLjdqd12_27-OMjdqd12_26*RLjdqd12_17
    Ompqpjdqd12_17 = Ompqpjdqd12_16+qd[32]*(OMjdqd12_26*ROjdqd12_66-OMjdqd12_36*ROjdqd12_56)
    Ompqpjdqd12_27 = Ompqpjdqd12_26+qd[32]*(-OMjdqd12_16*ROjdqd12_66+OMjdqd12_36*ROjdqd12_46)
    Ompqpjdqd12_37 = Ompqpjdqd12_36+qd[32]*(OMjdqd12_16*ROjdqd12_56-OMjdqd12_26*ROjdqd12_46)
    Apqpjdqd12_17 = OMjdqd12_26*ORjdqd12_37-OMjdqd12_36*ORjdqd12_27+Ompqpjdqd12_26*RLjdqd12_37-Ompqpjdqd12_36*RLjdqd12_27
    Apqpjdqd12_27 = -OMjdqd12_16*ORjdqd12_37+OMjdqd12_36*ORjdqd12_17-Ompqpjdqd12_16*RLjdqd12_37+Ompqpjdqd12_36*RLjdqd12_17
    Apqpjdqd12_37 = OMjdqd12_16*ORjdqd12_27-OMjdqd12_26*ORjdqd12_17+Ompqpjdqd12_16*RLjdqd12_27-Ompqpjdqd12_26*RLjdqd12_17
    RLjdqd12_18 = ROjdqd12_17*s.dpt[1,41]
    RLjdqd12_28 = ROjdqd12_27*s.dpt[1,41]
    RLjdqd12_38 = ROjdqd12_37*s.dpt[1,41]
    ORjdqd12_18 = OMjdqd12_27*RLjdqd12_38-OMjdqd12_37*RLjdqd12_28
    ORjdqd12_28 = -OMjdqd12_17*RLjdqd12_38+OMjdqd12_37*RLjdqd12_18
    ORjdqd12_38 = OMjdqd12_17*RLjdqd12_28-OMjdqd12_27*RLjdqd12_18
    Apqpjdqd12_18 = Apqpjdqd12_17+OMjdqd12_27*ORjdqd12_38-OMjdqd12_37*ORjdqd12_28+Ompqpjdqd12_27*RLjdqd12_38-Ompqpjdqd12_37*RLjdqd12_28
    Apqpjdqd12_28 = Apqpjdqd12_27-OMjdqd12_17*ORjdqd12_38+OMjdqd12_37*ORjdqd12_18-Ompqpjdqd12_17*RLjdqd12_38+Ompqpjdqd12_37*RLjdqd12_18
    Apqpjdqd12_38 = Apqpjdqd12_37+OMjdqd12_17*ORjdqd12_28-OMjdqd12_27*ORjdqd12_18+Ompqpjdqd12_17*RLjdqd12_28-Ompqpjdqd12_27*RLjdqd12_18
    ROjdqd13_52 = C14*C15-S14*S15
    ROjdqd13_62 = C14*S15+S14*C15
    ROjdqd13_82 = -C14*S15-S14*C15
    ROjdqd13_92 = C14*C15-S14*S15
    ROjdqd13_23 = ROjdqd13_52*S16
    ROjdqd13_33 = ROjdqd13_62*S16
    RLjdqd13_22 = s.dpt[2,23]*C14
    RLjdqd13_32 = s.dpt[2,23]*S14
    POjdqd13_22 = RLjdqd13_22+s.dpt[2,6]
    OMjdqd13_12 = qd[14]+qd[15]
    ORjdqd13_22 = -RLjdqd13_32*qd[14]
    ORjdqd13_32 = RLjdqd13_22*qd[14]
    Apqpjdqd13_22 = -ORjdqd13_32*qd[14]
    Apqpjdqd13_32 = ORjdqd13_22*qd[14]
    OMjdqd13_23 = ROjdqd13_82*qd[16]
    OMjdqd13_33 = ROjdqd13_92*qd[16]
    Ompqpjdqd13_23 = -OMjdqd13_12*ROjdqd13_92*qd[16]
    Ompqpjdqd13_33 = OMjdqd13_12*ROjdqd13_82*qd[16]
    RLjdqd13_14 = s.dpt[1,27]*C16
    RLjdqd13_24 = ROjdqd13_23*s.dpt[1,27]
    RLjdqd13_34 = ROjdqd13_33*s.dpt[1,27]
    POjdqd13_14 = RLjdqd13_14+s.dpt[1,6]
    POjdqd13_24 = POjdqd13_22+RLjdqd13_24
    POjdqd13_34 = RLjdqd13_32+RLjdqd13_34
    ORjdqd13_14 = OMjdqd13_23*RLjdqd13_34-OMjdqd13_33*RLjdqd13_24
    ORjdqd13_24 = -OMjdqd13_12*RLjdqd13_34+OMjdqd13_33*RLjdqd13_14
    ORjdqd13_34 = OMjdqd13_12*RLjdqd13_24-OMjdqd13_23*RLjdqd13_14
    VIjdqd13_24 = ORjdqd13_22+ORjdqd13_24
    VIjdqd13_34 = ORjdqd13_32+ORjdqd13_34
    Apqpjdqd13_14 = OMjdqd13_23*ORjdqd13_34-OMjdqd13_33*ORjdqd13_24+Ompqpjdqd13_23*RLjdqd13_34-Ompqpjdqd13_33*RLjdqd13_24
    Apqpjdqd13_24 = Apqpjdqd13_22-OMjdqd13_12*ORjdqd13_34+OMjdqd13_33*ORjdqd13_14+Ompqpjdqd13_33*RLjdqd13_14
    Apqpjdqd13_34 = Apqpjdqd13_32+OMjdqd13_12*ORjdqd13_24-OMjdqd13_23*ORjdqd13_14-Ompqpjdqd13_23*RLjdqd13_14
    ROjdqd14_52 = C8*C9-S8*S9
    ROjdqd14_62 = C8*S9+S8*C9
    ROjdqd14_82 = -C8*S9-S8*C9
    ROjdqd14_92 = C8*C9-S8*S9
    ROjdqd14_23 = ROjdqd14_52*S10
    ROjdqd14_33 = ROjdqd14_62*S10
    RLjdqd14_22 = s.dpt[2,17]*C8
    RLjdqd14_32 = s.dpt[2,17]*S8
    POjdqd14_22 = RLjdqd14_22+s.dpt[2,3]
    OMjdqd14_12 = qd[8]+qd[9]
    ORjdqd14_22 = -RLjdqd14_32*qd[8]
    ORjdqd14_32 = RLjdqd14_22*qd[8]
    Apqpjdqd14_22 = -ORjdqd14_32*qd[8]
    Apqpjdqd14_32 = ORjdqd14_22*qd[8]
    OMjdqd14_23 = ROjdqd14_82*qd[10]
    OMjdqd14_33 = ROjdqd14_92*qd[10]
    Ompqpjdqd14_23 = -OMjdqd14_12*ROjdqd14_92*qd[10]
    Ompqpjdqd14_33 = OMjdqd14_12*ROjdqd14_82*qd[10]
    RLjdqd14_14 = s.dpt[1,21]*C10
    RLjdqd14_24 = ROjdqd14_23*s.dpt[1,21]
    RLjdqd14_34 = ROjdqd14_33*s.dpt[1,21]
    POjdqd14_14 = RLjdqd14_14+s.dpt[1,3]
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
    jdqd19 = (Apqpjdqd13_14-Apqpjdqd14_14)*(POjdqd13_14-POjdqd14_14)+(Apqpjdqd13_24-Apqpjdqd14_24)*(POjdqd13_24-POjdqd14_24)+(Apqpjdqd13_34-Apqpjdqd14_34)*(POjdqd13_34-POjdqd14_34)+(ORjdqd13_14-ORjdqd14_14)*(ORjdqd13_14-ORjdqd14_14)+(VIjdqd13_24-VIjdqd14_24)*(VIjdqd13_24-VIjdqd14_24)+(VIjdqd13_34-VIjdqd14_34)*(VIjdqd13_34-VIjdqd14_34)
    Jdqd[1] = -Apqpjdqd2_14
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = -Apqpjdqd4_14
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6
    Jdqd[7] = -Apqpjdqd6_18
    Jdqd[8] = -Apqpjdqd6_28
    Jdqd[9] = -Apqpjdqd6_38
    Jdqd[10] = -Apqpjdqd8_19
    Jdqd[11] = -Apqpjdqd8_29
    Jdqd[12] = -Apqpjdqd8_39
    Jdqd[13] = -Apqpjdqd10_19
    Jdqd[14] = -Apqpjdqd10_29
    Jdqd[15] = -Apqpjdqd10_39
    Jdqd[16] = -Apqpjdqd12_18
    Jdqd[17] = -Apqpjdqd12_28
    Jdqd[18] = -Apqpjdqd12_38
    Jdqd[19] = jdqd19

# Number of continuation lines = 0


