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
#	==> Generation Date: Sun Mar 16 15:08:23 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 10
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
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_12 = s.dpt[1,4]*C7
    RLjdqd1_32 = -s.dpt[1,4]*S7
    ORjdqd1_12 = RLjdqd1_32*qd[7]
    ORjdqd1_32 = -RLjdqd1_12*qd[7]
    Apqpjdqd1_12 = ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = -ORjdqd1_12*qd[7]
    ROjdqd2_72 = C8*S9+S8*C9
    ROjdqd2_92 = C8*C9-S8*S9
    RLjdqd2_12 = s.dpt[1,5]*C8
    RLjdqd2_32 = -s.dpt[1,5]*S8
    OMjdqd2_22 = qd[8]+qd[9]
    ORjdqd2_12 = RLjdqd2_32*qd[8]
    ORjdqd2_32 = -RLjdqd2_12*qd[8]
    Apqpjdqd2_12 = ORjdqd2_32*qd[8]
    Apqpjdqd2_32 = -ORjdqd2_12*qd[8]
    RLjdqd2_13 = ROjdqd2_72*s.dpt[3,7]
    RLjdqd2_33 = ROjdqd2_92*s.dpt[3,7]
    ORjdqd2_13 = OMjdqd2_22*RLjdqd2_33
    ORjdqd2_33 = -OMjdqd2_22*RLjdqd2_13
    Apqpjdqd2_13 = Apqpjdqd2_12+OMjdqd2_22*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32-OMjdqd2_22*ORjdqd2_13
    jdqd1 = Apqpjdqd1_12-Apqpjdqd2_13
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd3

# Number of continuation lines = 0


