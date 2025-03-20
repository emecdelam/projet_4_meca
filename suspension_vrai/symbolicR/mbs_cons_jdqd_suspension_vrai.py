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
#	==> Generation Date: Thu Mar 20 15:02:22 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 13
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
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S11 = sin(q[11])
    C11 = cos(q[11])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,7]*C7
    RLjdqd1_32 = s.dpt[2,7]*S7
    ORjdqd1_22 = -RLjdqd1_32*qd[7]
    ORjdqd1_32 = RLjdqd1_22*qd[7]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = ORjdqd1_22*qd[7]
    ROjdqd2_82 = -S8*C9
    ROjdqd2_92 = C8*C9
    RLjdqd2_22 = s.dpt[2,8]*C8
    RLjdqd2_32 = s.dpt[2,8]*S8
    OMjdqd2_22 = qd[9]*C8
    OMjdqd2_32 = qd[9]*S8
    ORjdqd2_22 = -RLjdqd2_32*qd[8]
    ORjdqd2_32 = RLjdqd2_22*qd[8]
    Ompqpjdqd2_22 = -qd[8]*qd[9]*S8
    Ompqpjdqd2_32 = qd[8]*qd[9]*C8
    Apqpjdqd2_22 = -ORjdqd2_32*qd[8]
    Apqpjdqd2_32 = ORjdqd2_22*qd[8]
    RLjdqd2_13 = s.dpt[3,10]*S9
    RLjdqd2_23 = ROjdqd2_82*s.dpt[3,10]
    RLjdqd2_33 = ROjdqd2_92*s.dpt[3,10]
    ORjdqd2_13 = OMjdqd2_22*RLjdqd2_33-OMjdqd2_32*RLjdqd2_23
    ORjdqd2_23 = OMjdqd2_32*RLjdqd2_13-RLjdqd2_33*qd[8]
    ORjdqd2_33 = -OMjdqd2_22*RLjdqd2_13+RLjdqd2_23*qd[8]
    Apqpjdqd2_13 = OMjdqd2_22*ORjdqd2_33-OMjdqd2_32*ORjdqd2_23+Ompqpjdqd2_22*RLjdqd2_33-Ompqpjdqd2_32*RLjdqd2_23
    Apqpjdqd2_23 = Apqpjdqd2_22+OMjdqd2_32*ORjdqd2_13-ORjdqd2_33*qd[8]+Ompqpjdqd2_32*RLjdqd2_13
    Apqpjdqd2_33 = Apqpjdqd2_32-OMjdqd2_22*ORjdqd2_13+ORjdqd2_23*qd[8]-Ompqpjdqd2_22*RLjdqd2_13
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_23
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    ROjdqd3_82 = -S12*C13
    ROjdqd3_92 = C12*C13
    RLjdqd3_22 = s.dpt[2,13]*C12
    RLjdqd3_32 = s.dpt[2,13]*S12
    OMjdqd3_22 = qd[13]*C12
    OMjdqd3_32 = qd[13]*S12
    ORjdqd3_22 = -RLjdqd3_32*qd[12]
    ORjdqd3_32 = RLjdqd3_22*qd[12]
    Ompqpjdqd3_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd3_32 = qd[12]*qd[13]*C12
    Apqpjdqd3_22 = -ORjdqd3_32*qd[12]
    Apqpjdqd3_32 = ORjdqd3_22*qd[12]
    RLjdqd3_13 = s.dpt[3,15]*S13
    RLjdqd3_23 = ROjdqd3_82*s.dpt[3,15]
    RLjdqd3_33 = ROjdqd3_92*s.dpt[3,15]
    ORjdqd3_13 = OMjdqd3_22*RLjdqd3_33-OMjdqd3_32*RLjdqd3_23
    ORjdqd3_23 = OMjdqd3_32*RLjdqd3_13-RLjdqd3_33*qd[12]
    ORjdqd3_33 = -OMjdqd3_22*RLjdqd3_13+RLjdqd3_23*qd[12]
    Apqpjdqd3_13 = OMjdqd3_22*ORjdqd3_33-OMjdqd3_32*ORjdqd3_23+Ompqpjdqd3_22*RLjdqd3_33-Ompqpjdqd3_32*RLjdqd3_23
    Apqpjdqd3_23 = Apqpjdqd3_22+OMjdqd3_32*ORjdqd3_13-ORjdqd3_33*qd[12]+Ompqpjdqd3_32*RLjdqd3_13
    Apqpjdqd3_33 = Apqpjdqd3_32-OMjdqd3_22*ORjdqd3_13+ORjdqd3_23*qd[12]-Ompqpjdqd3_22*RLjdqd3_13
    RLjdqd4_22 = s.dpt[2,12]*C11
    RLjdqd4_32 = s.dpt[2,12]*S11
    ORjdqd4_22 = -RLjdqd4_32*qd[11]
    ORjdqd4_32 = RLjdqd4_22*qd[11]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[11]
    Apqpjdqd4_32 = ORjdqd4_22*qd[11]
    jdqd5 = Apqpjdqd3_23-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_33-Apqpjdqd4_32
    Jdqd[1] = -Apqpjdqd2_13
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = Apqpjdqd3_13
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6

# Number of continuation lines = 0


