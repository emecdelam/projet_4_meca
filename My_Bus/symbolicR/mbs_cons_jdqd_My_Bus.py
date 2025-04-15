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
#	==> Generation Date: Tue Apr 15 11:13:20 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: My_Bus
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

    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
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

    ROjdqd1_82 = -C8*S9-S8*C9
    ROjdqd1_92 = C8*C9-S8*S9
    RLjdqd1_22 = s.dpt[2,24]*C8
    RLjdqd1_32 = s.dpt[2,24]*S8
    OMjdqd1_12 = qd[8]+qd[9]
    ORjdqd1_22 = -RLjdqd1_32*qd[8]
    ORjdqd1_32 = RLjdqd1_22*qd[8]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[8]
    Apqpjdqd1_32 = ORjdqd1_22*qd[8]
    OMjdqd1_23 = ROjdqd1_82*qd[10]
    OMjdqd1_33 = ROjdqd1_92*qd[10]
    Ompqpjdqd1_23 = -OMjdqd1_12*ROjdqd1_92*qd[10]
    Ompqpjdqd1_33 = OMjdqd1_12*ROjdqd1_82*qd[10]
    RLjdqd1_24 = ROjdqd1_82*s.dpt[3,26]
    RLjdqd1_34 = ROjdqd1_92*s.dpt[3,26]
    ORjdqd1_14 = OMjdqd1_23*RLjdqd1_34-OMjdqd1_33*RLjdqd1_24
    ORjdqd1_24 = -OMjdqd1_12*RLjdqd1_34
    ORjdqd1_34 = OMjdqd1_12*RLjdqd1_24
    Apqpjdqd1_14 = OMjdqd1_23*ORjdqd1_34-OMjdqd1_33*ORjdqd1_24+Ompqpjdqd1_23*RLjdqd1_34-Ompqpjdqd1_33*RLjdqd1_24
    Apqpjdqd1_24 = Apqpjdqd1_22-OMjdqd1_12*ORjdqd1_34+OMjdqd1_33*ORjdqd1_14
    Apqpjdqd1_34 = Apqpjdqd1_32+OMjdqd1_12*ORjdqd1_24-OMjdqd1_23*ORjdqd1_14
    RLjdqd2_22 = s.dpt[2,23]*C7
    RLjdqd2_32 = s.dpt[2,23]*S7
    ORjdqd2_22 = -RLjdqd2_32*qd[7]
    ORjdqd2_32 = RLjdqd2_22*qd[7]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[7]
    Apqpjdqd2_32 = ORjdqd2_22*qd[7]
    jdqd2 = Apqpjdqd1_24-Apqpjdqd2_22
    jdqd3 = Apqpjdqd1_34-Apqpjdqd2_32
    RLjdqd3_22 = s.dpt[2,29]*C13
    RLjdqd3_32 = s.dpt[2,29]*S13
    ORjdqd3_22 = -RLjdqd3_32*qd[13]
    ORjdqd3_32 = RLjdqd3_22*qd[13]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[13]
    Apqpjdqd3_32 = ORjdqd3_22*qd[13]
    ROjdqd4_82 = -C14*S15-S14*C15
    ROjdqd4_92 = C14*C15-S14*S15
    RLjdqd4_22 = s.dpt[2,30]*C14
    RLjdqd4_32 = s.dpt[2,30]*S14
    OMjdqd4_12 = qd[14]+qd[15]
    ORjdqd4_22 = -RLjdqd4_32*qd[14]
    ORjdqd4_32 = RLjdqd4_22*qd[14]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[14]
    Apqpjdqd4_32 = ORjdqd4_22*qd[14]
    OMjdqd4_23 = ROjdqd4_82*qd[16]
    OMjdqd4_33 = ROjdqd4_92*qd[16]
    Ompqpjdqd4_23 = -OMjdqd4_12*ROjdqd4_92*qd[16]
    Ompqpjdqd4_33 = OMjdqd4_12*ROjdqd4_82*qd[16]
    RLjdqd4_24 = ROjdqd4_82*s.dpt[3,32]
    RLjdqd4_34 = ROjdqd4_92*s.dpt[3,32]
    ORjdqd4_14 = OMjdqd4_23*RLjdqd4_34-OMjdqd4_33*RLjdqd4_24
    ORjdqd4_24 = -OMjdqd4_12*RLjdqd4_34
    ORjdqd4_34 = OMjdqd4_12*RLjdqd4_24
    Apqpjdqd4_14 = OMjdqd4_23*ORjdqd4_34-OMjdqd4_33*ORjdqd4_24+Ompqpjdqd4_23*RLjdqd4_34-Ompqpjdqd4_33*RLjdqd4_24
    Apqpjdqd4_24 = Apqpjdqd4_22-OMjdqd4_12*ORjdqd4_34+OMjdqd4_33*ORjdqd4_14
    Apqpjdqd4_34 = Apqpjdqd4_32+OMjdqd4_12*ORjdqd4_24-OMjdqd4_23*ORjdqd4_14
    jdqd5 = Apqpjdqd3_22-Apqpjdqd4_24
    jdqd6 = Apqpjdqd3_32-Apqpjdqd4_34
    RLjdqd5_12 = s.dpt[1,49]*C23-s.dpt[2,49]*S23
    RLjdqd5_22 = s.dpt[1,49]*S23+s.dpt[2,49]*C23
    POjdqd5_12 = RLjdqd5_12+s.dpt[1,20]
    POjdqd5_22 = RLjdqd5_22+s.dpt[2,20]
    ORjdqd5_12 = -RLjdqd5_22*qd[23]
    ORjdqd5_22 = RLjdqd5_12*qd[23]
    Apqpjdqd5_12 = -ORjdqd5_22*qd[23]
    Apqpjdqd5_22 = ORjdqd5_12*qd[23]
    RLjdqd6_12 = s.dpt[1,51]*C24-s.dpt[2,51]*S24
    RLjdqd6_22 = s.dpt[1,51]*S24+s.dpt[2,51]*C24
    POjdqd6_12 = RLjdqd6_12+s.dpt[1,21]
    POjdqd6_22 = RLjdqd6_22+s.dpt[2,21]
    ORjdqd6_12 = -RLjdqd6_22*qd[24]
    ORjdqd6_22 = RLjdqd6_12*qd[24]
    Apqpjdqd6_12 = -ORjdqd6_22*qd[24]
    Apqpjdqd6_22 = ORjdqd6_12*qd[24]
    jdqd7 = (Apqpjdqd5_12-Apqpjdqd6_12)*(POjdqd5_12-POjdqd6_12)+(Apqpjdqd5_22-Apqpjdqd6_22)*(POjdqd5_22-POjdqd6_22)+(ORjdqd5_12-ORjdqd6_12)*(ORjdqd5_12-ORjdqd6_12)+(ORjdqd5_22-ORjdqd6_22)*(ORjdqd5_22-ORjdqd6_22)
    ROjdqd7_52 = C8*C9-S8*S9
    ROjdqd7_62 = C8*S9+S8*C9
    ROjdqd7_82 = -C8*S9-S8*C9
    ROjdqd7_92 = C8*C9-S8*S9
    ROjdqd7_23 = ROjdqd7_52*S10
    ROjdqd7_33 = ROjdqd7_62*S10
    RLjdqd7_22 = s.dpt[2,24]*C8
    RLjdqd7_32 = s.dpt[2,24]*S8
    POjdqd7_22 = RLjdqd7_22+s.dpt[2,4]
    OMjdqd7_12 = qd[8]+qd[9]
    ORjdqd7_22 = -RLjdqd7_32*qd[8]
    ORjdqd7_32 = RLjdqd7_22*qd[8]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[8]
    Apqpjdqd7_32 = ORjdqd7_22*qd[8]
    OMjdqd7_23 = ROjdqd7_82*qd[10]
    OMjdqd7_33 = ROjdqd7_92*qd[10]
    Ompqpjdqd7_23 = -OMjdqd7_12*ROjdqd7_92*qd[10]
    Ompqpjdqd7_33 = OMjdqd7_12*ROjdqd7_82*qd[10]
    RLjdqd7_14 = s.dpt[1,28]*C10
    RLjdqd7_24 = ROjdqd7_23*s.dpt[1,28]
    RLjdqd7_34 = ROjdqd7_33*s.dpt[1,28]
    POjdqd7_14 = RLjdqd7_14+s.dpt[1,4]
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
    RLjdqd8_12 = s.dpt[1,50]*C23-s.dpt[2,50]*S23
    RLjdqd8_22 = s.dpt[1,50]*S23+s.dpt[2,50]*C23
    POjdqd8_12 = RLjdqd8_12+s.dpt[1,20]
    POjdqd8_22 = RLjdqd8_22+s.dpt[2,20]
    ORjdqd8_12 = -RLjdqd8_22*qd[23]
    ORjdqd8_22 = RLjdqd8_12*qd[23]
    Apqpjdqd8_12 = -ORjdqd8_22*qd[23]
    Apqpjdqd8_22 = ORjdqd8_12*qd[23]
    jdqd8 = Apqpjdqd7_34*POjdqd7_34+VIjdqd7_34*VIjdqd7_34+(-ORjdqd8_22+VIjdqd7_24)*(-ORjdqd8_22+VIjdqd7_24)+(Apqpjdqd7_14-Apqpjdqd8_12)*(POjdqd7_14-POjdqd8_12)+(Apqpjdqd7_24-Apqpjdqd8_22)*(POjdqd7_24-POjdqd8_22)+(ORjdqd7_14-ORjdqd8_12)*(ORjdqd7_14-ORjdqd8_12)
    RLjdqd9_12 = s.dpt[1,52]*C24-s.dpt[2,52]*S24
    RLjdqd9_22 = s.dpt[1,52]*S24+s.dpt[2,52]*C24
    POjdqd9_12 = RLjdqd9_12+s.dpt[1,21]
    POjdqd9_22 = RLjdqd9_22+s.dpt[2,21]
    ORjdqd9_12 = -RLjdqd9_22*qd[24]
    ORjdqd9_22 = RLjdqd9_12*qd[24]
    Apqpjdqd9_12 = -ORjdqd9_22*qd[24]
    Apqpjdqd9_22 = ORjdqd9_12*qd[24]
    ROjdqd10_52 = C14*C15-S14*S15
    ROjdqd10_62 = C14*S15+S14*C15
    ROjdqd10_82 = -C14*S15-S14*C15
    ROjdqd10_92 = C14*C15-S14*S15
    ROjdqd10_23 = ROjdqd10_52*S16
    ROjdqd10_33 = ROjdqd10_62*S16
    RLjdqd10_22 = s.dpt[2,30]*C14
    RLjdqd10_32 = s.dpt[2,30]*S14
    POjdqd10_22 = RLjdqd10_22+s.dpt[2,7]
    OMjdqd10_12 = qd[14]+qd[15]
    ORjdqd10_22 = -RLjdqd10_32*qd[14]
    ORjdqd10_32 = RLjdqd10_22*qd[14]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[14]
    Apqpjdqd10_32 = ORjdqd10_22*qd[14]
    OMjdqd10_23 = ROjdqd10_82*qd[16]
    OMjdqd10_33 = ROjdqd10_92*qd[16]
    Ompqpjdqd10_23 = -OMjdqd10_12*ROjdqd10_92*qd[16]
    Ompqpjdqd10_33 = OMjdqd10_12*ROjdqd10_82*qd[16]
    RLjdqd10_14 = s.dpt[1,34]*C16
    RLjdqd10_24 = ROjdqd10_23*s.dpt[1,34]
    RLjdqd10_34 = ROjdqd10_33*s.dpt[1,34]
    POjdqd10_14 = RLjdqd10_14+s.dpt[1,7]
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
    ROjdqd11_42 = S19*S20
    ROjdqd11_62 = C19*S20
    OMjdqd11_12 = qd[20]*C19
    OMjdqd11_32 = -qd[20]*S19
    Ompqpjdqd11_12 = -qd[19]*qd[20]*S19
    Ompqpjdqd11_32 = -qd[19]*qd[20]*C19
    RLjdqd11_13 = ROjdqd11_42*s.dpt[2,35]
    RLjdqd11_23 = s.dpt[2,35]*C20
    RLjdqd11_33 = ROjdqd11_62*s.dpt[2,35]
    POjdqd11_13 = RLjdqd11_13+s.dpt[1,15]
    ORjdqd11_13 = -OMjdqd11_32*RLjdqd11_23+RLjdqd11_33*qd[19]
    ORjdqd11_23 = -OMjdqd11_12*RLjdqd11_33+OMjdqd11_32*RLjdqd11_13
    ORjdqd11_33 = OMjdqd11_12*RLjdqd11_23-RLjdqd11_13*qd[19]
    Apqpjdqd11_13 = -OMjdqd11_32*ORjdqd11_23+ORjdqd11_33*qd[19]-Ompqpjdqd11_32*RLjdqd11_23
    Apqpjdqd11_23 = -OMjdqd11_12*ORjdqd11_33+OMjdqd11_32*ORjdqd11_13-Ompqpjdqd11_12*RLjdqd11_33+Ompqpjdqd11_32*RLjdqd11_13
    Apqpjdqd11_33 = OMjdqd11_12*ORjdqd11_23-ORjdqd11_13*qd[19]+Ompqpjdqd11_12*RLjdqd11_23
    jdqd10 = Apqpjdqd11_13*(POjdqd11_13-s.dpt[1,12])+Apqpjdqd11_23*(RLjdqd11_23-s.dpt[2,12])+Apqpjdqd11_33*RLjdqd11_33+ORjdqd11_13*ORjdqd11_13+ORjdqd11_23*ORjdqd11_23+ORjdqd11_33*ORjdqd11_33
    ROjdqd13_42 = S19*S20
    ROjdqd13_62 = C19*S20
    OMjdqd13_12 = qd[20]*C19
    OMjdqd13_32 = -qd[20]*S19
    Ompqpjdqd13_12 = -qd[19]*qd[20]*S19
    Ompqpjdqd13_32 = -qd[19]*qd[20]*C19
    RLjdqd13_13 = ROjdqd13_42*s.dpt[2,37]
    RLjdqd13_23 = s.dpt[2,37]*C20
    RLjdqd13_33 = ROjdqd13_62*s.dpt[2,37]
    POjdqd13_13 = RLjdqd13_13+s.dpt[1,15]
    ORjdqd13_13 = -OMjdqd13_32*RLjdqd13_23+RLjdqd13_33*qd[19]
    ORjdqd13_23 = -OMjdqd13_12*RLjdqd13_33+OMjdqd13_32*RLjdqd13_13
    ORjdqd13_33 = OMjdqd13_12*RLjdqd13_23-RLjdqd13_13*qd[19]
    Apqpjdqd13_13 = -OMjdqd13_32*ORjdqd13_23+ORjdqd13_33*qd[19]-Ompqpjdqd13_32*RLjdqd13_23
    Apqpjdqd13_23 = -OMjdqd13_12*ORjdqd13_33+OMjdqd13_32*ORjdqd13_13-Ompqpjdqd13_12*RLjdqd13_33+Ompqpjdqd13_32*RLjdqd13_13
    Apqpjdqd13_33 = OMjdqd13_12*ORjdqd13_23-ORjdqd13_13*qd[19]+Ompqpjdqd13_12*RLjdqd13_23
    jdqd11 = Apqpjdqd13_13*(POjdqd13_13-s.dpt[1,14])+Apqpjdqd13_23*(RLjdqd13_23-s.dpt[2,14])+Apqpjdqd13_33*RLjdqd13_33+ORjdqd13_13*ORjdqd13_13+ORjdqd13_23*ORjdqd13_23+ORjdqd13_33*ORjdqd13_33
    ROjdqd16_42 = S19*S20
    ROjdqd16_62 = C19*S20
    ROjdqd16_72 = S19*C20
    ROjdqd16_92 = C19*C20
    OMjdqd16_12 = qd[20]*C19
    OMjdqd16_32 = -qd[20]*S19
    Ompqpjdqd16_12 = -qd[19]*qd[20]*S19
    Ompqpjdqd16_32 = -qd[19]*qd[20]*C19
    RLjdqd16_13 = ROjdqd16_42*s.dpt[2,38]+ROjdqd16_72*s.dpt[3,38]
    RLjdqd16_23 = s.dpt[2,38]*C20-s.dpt[3,38]*S20
    RLjdqd16_33 = ROjdqd16_62*s.dpt[2,38]+ROjdqd16_92*s.dpt[3,38]
    POjdqd16_13 = RLjdqd16_13+s.dpt[1,15]
    ORjdqd16_13 = -OMjdqd16_32*RLjdqd16_23+RLjdqd16_33*qd[19]
    ORjdqd16_23 = -OMjdqd16_12*RLjdqd16_33+OMjdqd16_32*RLjdqd16_13
    ORjdqd16_33 = OMjdqd16_12*RLjdqd16_23-RLjdqd16_13*qd[19]
    Apqpjdqd16_13 = -OMjdqd16_32*ORjdqd16_23+ORjdqd16_33*qd[19]-Ompqpjdqd16_32*RLjdqd16_23
    Apqpjdqd16_23 = -OMjdqd16_12*ORjdqd16_33+OMjdqd16_32*ORjdqd16_13-Ompqpjdqd16_12*RLjdqd16_33+Ompqpjdqd16_32*RLjdqd16_13
    Apqpjdqd16_33 = OMjdqd16_12*ORjdqd16_23-ORjdqd16_13*qd[19]+Ompqpjdqd16_12*RLjdqd16_23
    jdqd12 = -Apqpjdqd16_13*(-POjdqd16_13+s.dpt[1,13])-Apqpjdqd16_23*(-RLjdqd16_23+s.dpt[2,13])-Apqpjdqd16_33*(-RLjdqd16_33+s.dpt[3,13])+ORjdqd16_13*ORjdqd16_13+ORjdqd16_23*ORjdqd16_23+ORjdqd16_33*ORjdqd16_33
    ROjdqd18_42 = S19*S20
    ROjdqd18_62 = C19*S20
    ROjdqd18_72 = S19*C20
    ROjdqd18_92 = C19*C20
    OMjdqd18_12 = qd[20]*C19
    OMjdqd18_32 = -qd[20]*S19
    Ompqpjdqd18_12 = -qd[19]*qd[20]*S19
    Ompqpjdqd18_32 = -qd[19]*qd[20]*C19
    RLjdqd18_13 = ROjdqd18_42*s.dpt[2,36]+ROjdqd18_72*s.dpt[3,36]
    RLjdqd18_23 = s.dpt[2,36]*C20-s.dpt[3,36]*S20
    RLjdqd18_33 = ROjdqd18_62*s.dpt[2,36]+ROjdqd18_92*s.dpt[3,36]
    POjdqd18_13 = RLjdqd18_13+s.dpt[1,15]
    ORjdqd18_13 = -OMjdqd18_32*RLjdqd18_23+RLjdqd18_33*qd[19]
    ORjdqd18_23 = -OMjdqd18_12*RLjdqd18_33+OMjdqd18_32*RLjdqd18_13
    ORjdqd18_33 = OMjdqd18_12*RLjdqd18_23-RLjdqd18_13*qd[19]
    Apqpjdqd18_13 = -OMjdqd18_32*ORjdqd18_23+ORjdqd18_33*qd[19]-Ompqpjdqd18_32*RLjdqd18_23
    Apqpjdqd18_23 = -OMjdqd18_12*ORjdqd18_33+OMjdqd18_32*ORjdqd18_13-Ompqpjdqd18_12*RLjdqd18_33+Ompqpjdqd18_32*RLjdqd18_13
    Apqpjdqd18_33 = OMjdqd18_12*ORjdqd18_23-ORjdqd18_13*qd[19]+Ompqpjdqd18_12*RLjdqd18_23
    jdqd13 = -Apqpjdqd18_13*(-POjdqd18_13+s.dpt[1,11])-Apqpjdqd18_23*(-RLjdqd18_23+s.dpt[2,11])-Apqpjdqd18_33*(-RLjdqd18_33+s.dpt[3,11])+ORjdqd18_13*ORjdqd18_13+ORjdqd18_23*ORjdqd18_23+ORjdqd18_33*ORjdqd18_33
    Jdqd[1] = Apqpjdqd1_14
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = -Apqpjdqd4_14
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


