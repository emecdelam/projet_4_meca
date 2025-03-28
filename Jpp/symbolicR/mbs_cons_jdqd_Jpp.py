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
#	==> Generation Date: Fri Mar 28 02:55:51 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 24
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
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,29]*C12
    RLjdqd1_32 = s.dpt[2,29]*S12
    ORjdqd1_22 = -RLjdqd1_32*qd[12]
    ORjdqd1_32 = RLjdqd1_22*qd[12]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[12]
    Apqpjdqd1_32 = ORjdqd1_22*qd[12]
    ROjdqd2_82 = -C7*S8-S7*C8
    ROjdqd2_92 = C7*C8-S7*S8
    RLjdqd2_22 = s.dpt[2,23]*C7
    RLjdqd2_32 = s.dpt[2,23]*S7
    OMjdqd2_12 = qd[7]+qd[8]
    ORjdqd2_22 = -RLjdqd2_32*qd[7]
    ORjdqd2_32 = RLjdqd2_22*qd[7]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[7]
    Apqpjdqd2_32 = ORjdqd2_22*qd[7]
    OMjdqd2_23 = ROjdqd2_82*qd[9]
    OMjdqd2_33 = ROjdqd2_92*qd[9]
    Ompqpjdqd2_23 = -OMjdqd2_12*ROjdqd2_92*qd[9]
    Ompqpjdqd2_33 = OMjdqd2_12*ROjdqd2_82*qd[9]
    RLjdqd2_24 = ROjdqd2_82*s.dpt[3,25]
    RLjdqd2_34 = ROjdqd2_92*s.dpt[3,25]
    ORjdqd2_14 = OMjdqd2_23*RLjdqd2_34-OMjdqd2_33*RLjdqd2_24
    ORjdqd2_24 = -OMjdqd2_12*RLjdqd2_34
    ORjdqd2_34 = OMjdqd2_12*RLjdqd2_24
    Apqpjdqd2_14 = OMjdqd2_23*ORjdqd2_34-OMjdqd2_33*ORjdqd2_24+Ompqpjdqd2_23*RLjdqd2_34-Ompqpjdqd2_33*RLjdqd2_24
    Apqpjdqd2_24 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_34+OMjdqd2_33*ORjdqd2_14
    Apqpjdqd2_34 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_24-OMjdqd2_23*ORjdqd2_14
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_24
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_34
    ROjdqd3_82 = -C13*S14-S13*C14
    ROjdqd3_92 = C13*C14-S13*S14
    RLjdqd3_22 = s.dpt[2,30]*C13
    RLjdqd3_32 = s.dpt[2,30]*S13
    OMjdqd3_12 = qd[13]+qd[14]
    ORjdqd3_22 = -RLjdqd3_32*qd[13]
    ORjdqd3_32 = RLjdqd3_22*qd[13]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[13]
    Apqpjdqd3_32 = ORjdqd3_22*qd[13]
    OMjdqd3_23 = ROjdqd3_82*qd[15]
    OMjdqd3_33 = ROjdqd3_92*qd[15]
    Ompqpjdqd3_23 = -OMjdqd3_12*ROjdqd3_92*qd[15]
    Ompqpjdqd3_33 = OMjdqd3_12*ROjdqd3_82*qd[15]
    RLjdqd3_24 = ROjdqd3_82*s.dpt[3,32]
    RLjdqd3_34 = ROjdqd3_92*s.dpt[3,32]
    ORjdqd3_14 = OMjdqd3_23*RLjdqd3_34-OMjdqd3_33*RLjdqd3_24
    ORjdqd3_24 = -OMjdqd3_12*RLjdqd3_34
    ORjdqd3_34 = OMjdqd3_12*RLjdqd3_24
    Apqpjdqd3_14 = OMjdqd3_23*ORjdqd3_34-OMjdqd3_33*ORjdqd3_24+Ompqpjdqd3_23*RLjdqd3_34-Ompqpjdqd3_33*RLjdqd3_24
    Apqpjdqd3_24 = Apqpjdqd3_22-OMjdqd3_12*ORjdqd3_34+OMjdqd3_33*ORjdqd3_14
    Apqpjdqd3_34 = Apqpjdqd3_32+OMjdqd3_12*ORjdqd3_24-OMjdqd3_23*ORjdqd3_14
    RLjdqd4_22 = s.dpt[2,36]*C18
    RLjdqd4_32 = s.dpt[2,36]*S18
    ORjdqd4_22 = -RLjdqd4_32*qd[18]
    ORjdqd4_32 = RLjdqd4_22*qd[18]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[18]
    Apqpjdqd4_32 = ORjdqd4_22*qd[18]
    jdqd5 = Apqpjdqd3_24-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_34-Apqpjdqd4_32
    RLjdqd5_12 = s.dpt[1,54]*C23-s.dpt[2,54]*S23
    RLjdqd5_22 = s.dpt[1,54]*S23+s.dpt[2,54]*C23
    POjdqd5_12 = RLjdqd5_12+s.dpt[1,17]
    POjdqd5_22 = RLjdqd5_22+s.dpt[2,17]
    ORjdqd5_12 = -RLjdqd5_22*qd[23]
    ORjdqd5_22 = RLjdqd5_12*qd[23]
    Apqpjdqd5_12 = -ORjdqd5_22*qd[23]
    Apqpjdqd5_22 = ORjdqd5_12*qd[23]
    RLjdqd6_12 = s.dpt[1,56]*C24-s.dpt[2,56]*S24
    RLjdqd6_22 = s.dpt[1,56]*S24+s.dpt[2,56]*C24
    POjdqd6_12 = RLjdqd6_12+s.dpt[1,18]
    POjdqd6_22 = RLjdqd6_22+s.dpt[2,18]
    ORjdqd6_12 = -RLjdqd6_22*qd[24]
    ORjdqd6_22 = RLjdqd6_12*qd[24]
    Apqpjdqd6_12 = -ORjdqd6_22*qd[24]
    Apqpjdqd6_22 = ORjdqd6_12*qd[24]
    jdqd7 = (Apqpjdqd5_12-Apqpjdqd6_12)*(POjdqd5_12-POjdqd6_12)+(Apqpjdqd5_22-Apqpjdqd6_22)*(POjdqd5_22-POjdqd6_22)+(ORjdqd5_12-ORjdqd6_12)*(ORjdqd5_12-ORjdqd6_12)+(ORjdqd5_22-ORjdqd6_22)*(ORjdqd5_22-ORjdqd6_22)
    ROjdqd7_52 = C7*C8-S7*S8
    ROjdqd7_62 = C7*S8+S7*C8
    ROjdqd7_82 = -C7*S8-S7*C8
    ROjdqd7_92 = C7*C8-S7*S8
    ROjdqd7_23 = ROjdqd7_52*S9
    ROjdqd7_33 = ROjdqd7_62*S9
    RLjdqd7_22 = s.dpt[2,23]*C7
    RLjdqd7_32 = s.dpt[2,23]*S7
    POjdqd7_22 = RLjdqd7_22+s.dpt[2,2]
    OMjdqd7_12 = qd[7]+qd[8]
    ORjdqd7_22 = -RLjdqd7_32*qd[7]
    ORjdqd7_32 = RLjdqd7_22*qd[7]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[7]
    Apqpjdqd7_32 = ORjdqd7_22*qd[7]
    OMjdqd7_23 = ROjdqd7_82*qd[9]
    OMjdqd7_33 = ROjdqd7_92*qd[9]
    Ompqpjdqd7_23 = -OMjdqd7_12*ROjdqd7_92*qd[9]
    Ompqpjdqd7_33 = OMjdqd7_12*ROjdqd7_82*qd[9]
    RLjdqd7_14 = s.dpt[1,27]*C9
    RLjdqd7_24 = ROjdqd7_23*s.dpt[1,27]
    RLjdqd7_34 = ROjdqd7_33*s.dpt[1,27]
    POjdqd7_14 = RLjdqd7_14+s.dpt[1,2]
    POjdqd7_24 = POjdqd7_22+RLjdqd7_24
    POjdqd7_34 = RLjdqd7_32+RLjdqd7_34
    ORjdqd7_14 = OMjdqd7_23*RLjdqd7_34-OMjdqd7_33*RLjdqd7_24
    ORjdqd7_24 = -OMjdqd7_12*RLjdqd7_34+OMjdqd7_33*RLjdqd7_14
    ORjdqd7_34 = OMjdqd7_12*RLjdqd7_24-OMjdqd7_23*RLjdqd7_14
    VIjdqd7_24 = ORjdqd7_22+ORjdqd7_24
    VIjdqd7_34 = ORjdqd7_32+ORjdqd7_34
    Apqpjdqd7_14 = OMjdqd7_23*ORjdqd7_34-OMjdqd7_33*ORjdqd7_24+Ompqpjdqd7_23*RLjdqd7_34-Ompqpjdqd7_33*RLjdqd7_24
    Apqpjdqd7_24 = Apqpjdqd7_22-OMjdqd7_12*ORjdqd7_34+OMjdqd7_33*ORjdqd7_14+Ompqpjdqd7_33*RLjdqd7_14
    Apqpjdqd7_34 = Apqpjdqd7_32+OMjdqd7_12*ORjdqd7_24-OMjdqd7_23*ORjdqd7_14-Ompqpjdqd7_23*RLjdqd7_14
    RLjdqd8_12 = s.dpt[1,53]*C23-s.dpt[2,53]*S23
    RLjdqd8_22 = s.dpt[1,53]*S23+s.dpt[2,53]*C23
    POjdqd8_12 = RLjdqd8_12+s.dpt[1,17]
    POjdqd8_22 = RLjdqd8_22+s.dpt[2,17]
    ORjdqd8_12 = -RLjdqd8_22*qd[23]
    ORjdqd8_22 = RLjdqd8_12*qd[23]
    Apqpjdqd8_12 = -ORjdqd8_22*qd[23]
    Apqpjdqd8_22 = ORjdqd8_12*qd[23]
    jdqd8 = Apqpjdqd7_34*POjdqd7_34+VIjdqd7_34*VIjdqd7_34+(-ORjdqd8_22+VIjdqd7_24)*(-ORjdqd8_22+VIjdqd7_24)+(Apqpjdqd7_14-Apqpjdqd8_12)*(POjdqd7_14-POjdqd8_12)+(Apqpjdqd7_24-Apqpjdqd8_22)*(POjdqd7_24-POjdqd8_22)+(ORjdqd7_14-ORjdqd8_12)*(ORjdqd7_14-ORjdqd8_12)
    RLjdqd9_12 = s.dpt[1,55]*C24-s.dpt[2,55]*S24
    RLjdqd9_22 = s.dpt[1,55]*S24+s.dpt[2,55]*C24
    POjdqd9_12 = RLjdqd9_12+s.dpt[1,18]
    POjdqd9_22 = RLjdqd9_22+s.dpt[2,18]
    ORjdqd9_12 = -RLjdqd9_22*qd[24]
    ORjdqd9_22 = RLjdqd9_12*qd[24]
    Apqpjdqd9_12 = -ORjdqd9_22*qd[24]
    Apqpjdqd9_22 = ORjdqd9_12*qd[24]
    ROjdqd10_52 = C13*C14-S13*S14
    ROjdqd10_62 = C13*S14+S13*C14
    ROjdqd10_82 = -C13*S14-S13*C14
    ROjdqd10_92 = C13*C14-S13*S14
    ROjdqd10_23 = ROjdqd10_52*S15
    ROjdqd10_33 = ROjdqd10_62*S15
    RLjdqd10_22 = s.dpt[2,30]*C13
    RLjdqd10_32 = s.dpt[2,30]*S13
    POjdqd10_22 = RLjdqd10_22+s.dpt[2,5]
    OMjdqd10_12 = qd[13]+qd[14]
    ORjdqd10_22 = -RLjdqd10_32*qd[13]
    ORjdqd10_32 = RLjdqd10_22*qd[13]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[13]
    Apqpjdqd10_32 = ORjdqd10_22*qd[13]
    OMjdqd10_23 = ROjdqd10_82*qd[15]
    OMjdqd10_33 = ROjdqd10_92*qd[15]
    Ompqpjdqd10_23 = -OMjdqd10_12*ROjdqd10_92*qd[15]
    Ompqpjdqd10_33 = OMjdqd10_12*ROjdqd10_82*qd[15]
    RLjdqd10_14 = s.dpt[1,34]*C15
    RLjdqd10_24 = ROjdqd10_23*s.dpt[1,34]
    RLjdqd10_34 = ROjdqd10_33*s.dpt[1,34]
    POjdqd10_14 = RLjdqd10_14+s.dpt[1,5]
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
    jdqd9 = Apqpjdqd10_34*POjdqd10_34+VIjdqd10_34*VIjdqd10_34+(-Apqpjdqd10_14+Apqpjdqd9_12)*(-POjdqd10_14+POjdqd9_12)+(-Apqpjdqd10_24+Apqpjdqd9_22)*(-POjdqd10_24+POjdqd9_22)+(-ORjdqd10_14+ORjdqd9_12)*(-ORjdqd10_14+ORjdqd9_12)+(ORjdqd9_22-VIjdqd10_24)*(ORjdqd9_22-VIjdqd10_24)
    ROjdqd12_82 = -S19*C20
    ROjdqd12_92 = C19*C20
    OMjdqd12_22 = qd[20]*C19
    OMjdqd12_32 = qd[20]*S19
    Ompqpjdqd12_22 = -qd[19]*qd[20]*S19
    Ompqpjdqd12_32 = qd[19]*qd[20]*C19
    RLjdqd12_13 = s.dpt[3,39]*S20
    RLjdqd12_23 = ROjdqd12_82*s.dpt[3,39]+s.dpt[2,39]*C19
    RLjdqd12_33 = ROjdqd12_92*s.dpt[3,39]+s.dpt[2,39]*S19
    POjdqd12_13 = RLjdqd12_13+s.dpt[1,14]
    ORjdqd12_13 = OMjdqd12_22*RLjdqd12_33-OMjdqd12_32*RLjdqd12_23
    ORjdqd12_23 = OMjdqd12_32*RLjdqd12_13-RLjdqd12_33*qd[19]
    ORjdqd12_33 = -OMjdqd12_22*RLjdqd12_13+RLjdqd12_23*qd[19]
    Apqpjdqd12_13 = OMjdqd12_22*ORjdqd12_33-OMjdqd12_32*ORjdqd12_23+Ompqpjdqd12_22*RLjdqd12_33-Ompqpjdqd12_32*RLjdqd12_23
    Apqpjdqd12_23 = OMjdqd12_32*ORjdqd12_13-ORjdqd12_33*qd[19]+Ompqpjdqd12_32*RLjdqd12_13
    Apqpjdqd12_33 = -OMjdqd12_22*ORjdqd12_13+ORjdqd12_23*qd[19]-Ompqpjdqd12_22*RLjdqd12_13
    jdqd10 = -Apqpjdqd12_13*(-POjdqd12_13+s.dpt[1,10])-Apqpjdqd12_23*(-RLjdqd12_23+s.dpt[2,10])-Apqpjdqd12_33*(-RLjdqd12_33+s.dpt[3,10])+ORjdqd12_13*ORjdqd12_13+ORjdqd12_23*ORjdqd12_23+ORjdqd12_33*ORjdqd12_33
    OMjdqd14_22 = qd[20]*C19
    OMjdqd14_32 = qd[20]*S19
    Ompqpjdqd14_22 = -qd[19]*qd[20]*S19
    Ompqpjdqd14_32 = qd[19]*qd[20]*C19
    RLjdqd14_23 = s.dpt[2,46]*C19
    RLjdqd14_33 = s.dpt[2,46]*S19
    ORjdqd14_13 = OMjdqd14_22*RLjdqd14_33-OMjdqd14_32*RLjdqd14_23
    ORjdqd14_23 = -RLjdqd14_33*qd[19]
    ORjdqd14_33 = RLjdqd14_23*qd[19]
    Apqpjdqd14_13 = OMjdqd14_22*ORjdqd14_33-OMjdqd14_32*ORjdqd14_23+Ompqpjdqd14_22*RLjdqd14_33-Ompqpjdqd14_32*RLjdqd14_23
    Apqpjdqd14_23 = OMjdqd14_32*ORjdqd14_13-ORjdqd14_33*qd[19]
    Apqpjdqd14_33 = -OMjdqd14_22*ORjdqd14_13+ORjdqd14_23*qd[19]
    jdqd11 = -Apqpjdqd14_13*(-s.dpt[1,14]+s.dpt[1,15])-Apqpjdqd14_23*(-RLjdqd14_23+s.dpt[2,15])+Apqpjdqd14_33*RLjdqd14_33+ORjdqd14_13*ORjdqd14_13+ORjdqd14_23*ORjdqd14_23+ORjdqd14_33*ORjdqd14_33
    OMjdqd16_22 = qd[20]*C19
    OMjdqd16_32 = qd[20]*S19
    Ompqpjdqd16_22 = -qd[19]*qd[20]*S19
    Ompqpjdqd16_32 = qd[19]*qd[20]*C19
    RLjdqd16_23 = s.dpt[2,45]*C19
    RLjdqd16_33 = s.dpt[2,45]*S19
    ORjdqd16_13 = OMjdqd16_22*RLjdqd16_33-OMjdqd16_32*RLjdqd16_23
    ORjdqd16_23 = -RLjdqd16_33*qd[19]
    ORjdqd16_33 = RLjdqd16_23*qd[19]
    Apqpjdqd16_13 = OMjdqd16_22*ORjdqd16_33-OMjdqd16_32*ORjdqd16_23+Ompqpjdqd16_22*RLjdqd16_33-Ompqpjdqd16_32*RLjdqd16_23
    Apqpjdqd16_23 = OMjdqd16_32*ORjdqd16_13-ORjdqd16_33*qd[19]
    Apqpjdqd16_33 = -OMjdqd16_22*ORjdqd16_13+ORjdqd16_23*qd[19]
    jdqd12 = -Apqpjdqd16_13*(-s.dpt[1,14]+s.dpt[1,16])-Apqpjdqd16_23*(-RLjdqd16_23+s.dpt[2,16])+Apqpjdqd16_33*RLjdqd16_33+ORjdqd16_13*ORjdqd16_13+ORjdqd16_23*ORjdqd16_23+ORjdqd16_33*ORjdqd16_33
    ROjdqd18_82 = -S19*C20
    ROjdqd18_92 = C19*C20
    OMjdqd18_22 = qd[20]*C19
    OMjdqd18_32 = qd[20]*S19
    Ompqpjdqd18_22 = -qd[19]*qd[20]*S19
    Ompqpjdqd18_32 = qd[19]*qd[20]*C19
    RLjdqd18_13 = s.dpt[3,42]*S20
    RLjdqd18_23 = ROjdqd18_82*s.dpt[3,42]+s.dpt[2,42]*C19
    RLjdqd18_33 = ROjdqd18_92*s.dpt[3,42]+s.dpt[2,42]*S19
    POjdqd18_13 = RLjdqd18_13+s.dpt[1,14]
    ORjdqd18_13 = OMjdqd18_22*RLjdqd18_33-OMjdqd18_32*RLjdqd18_23
    ORjdqd18_23 = OMjdqd18_32*RLjdqd18_13-RLjdqd18_33*qd[19]
    ORjdqd18_33 = -OMjdqd18_22*RLjdqd18_13+RLjdqd18_23*qd[19]
    Apqpjdqd18_13 = OMjdqd18_22*ORjdqd18_33-OMjdqd18_32*ORjdqd18_23+Ompqpjdqd18_22*RLjdqd18_33-Ompqpjdqd18_32*RLjdqd18_23
    Apqpjdqd18_23 = OMjdqd18_32*ORjdqd18_13-ORjdqd18_33*qd[19]+Ompqpjdqd18_32*RLjdqd18_13
    Apqpjdqd18_33 = -OMjdqd18_22*ORjdqd18_13+ORjdqd18_23*qd[19]-Ompqpjdqd18_22*RLjdqd18_13
    jdqd13 = -Apqpjdqd18_13*(-POjdqd18_13+s.dpt[1,11])-Apqpjdqd18_23*(-RLjdqd18_23+s.dpt[2,11])-Apqpjdqd18_33*(-RLjdqd18_33+s.dpt[3,11])+ORjdqd18_13*ORjdqd18_13+ORjdqd18_23*ORjdqd18_23+ORjdqd18_33*ORjdqd18_33
    Jdqd[1] = -Apqpjdqd2_14
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = Apqpjdqd3_14
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6
    Jdqd[7] = jdqd7
    Jdqd[8] = jdqd8
    Jdqd[9] = jdqd9
    Jdqd[10] = jdqd10
    Jdqd[11] = jdqd11
    Jdqd[12] = jdqd12
    Jdqd[13] = jdqd13

# Number of continuation lines = 0


