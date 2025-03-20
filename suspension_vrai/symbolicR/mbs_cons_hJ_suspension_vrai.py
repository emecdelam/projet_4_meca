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

    RLlp1_22 = s.dpt[2,7]*C7
    RLlp1_32 = s.dpt[2,7]*S7
    POlp1_22 = RLlp1_22+s.dpt[2,2]
    POlp1_32 = RLlp1_32+s.dpt[3,2]
    ROlp2_82 = -S8*C9
    ROlp2_92 = C8*C9
    RLlp2_22 = s.dpt[2,8]*C8
    RLlp2_32 = s.dpt[2,8]*S8
    POlp2_22 = RLlp2_22+s.dpt[2,3]
    RLlp2_13 = s.dpt[3,10]*S9
    RLlp2_23 = ROlp2_82*s.dpt[3,10]
    RLlp2_33 = ROlp2_92*s.dpt[3,10]
    POlp2_13 = RLlp2_13+s.dpt[1,3]
    POlp2_23 = POlp2_22+RLlp2_23
    POlp2_33 = RLlp2_32+RLlp2_33
    JTlp2_23_1 = -RLlp2_32-RLlp2_33
    JTlp2_33_1 = RLlp2_22+RLlp2_23
    JTlp2_13_2 = -RLlp2_23*S8+RLlp2_33*C8
    JTlp2_23_2 = RLlp2_13*S8
    JTlp2_33_2 = -RLlp2_13*C8
    h_1 = -POlp2_13+s.dpt[1,2]
    h_2 = POlp1_22-POlp2_23
    h_3 = POlp1_32-POlp2_33
    ROlp3_82 = -S12*C13
    ROlp3_92 = C12*C13
    RLlp3_22 = s.dpt[2,13]*C12
    RLlp3_32 = s.dpt[2,13]*S12
    POlp3_22 = RLlp3_22+s.dpt[2,6]
    RLlp3_13 = s.dpt[3,15]*S13
    RLlp3_23 = ROlp3_82*s.dpt[3,15]
    RLlp3_33 = ROlp3_92*s.dpt[3,15]
    POlp3_13 = RLlp3_13+s.dpt[1,6]
    POlp3_23 = POlp3_22+RLlp3_23
    POlp3_33 = RLlp3_32+RLlp3_33
    JTlp3_23_1 = -RLlp3_32-RLlp3_33
    JTlp3_33_1 = RLlp3_22+RLlp3_23
    JTlp3_13_2 = -RLlp3_23*S12+RLlp3_33*C12
    JTlp3_23_2 = RLlp3_13*S12
    JTlp3_33_2 = -RLlp3_13*C12
    RLlp4_22 = s.dpt[2,12]*C11
    RLlp4_32 = s.dpt[2,12]*S11
    POlp4_22 = RLlp4_22+s.dpt[2,5]
    POlp4_32 = RLlp4_32+s.dpt[3,5]
    h_4 = POlp3_13-s.dpt[1,5]
    h_5 = POlp3_23-POlp4_22
    h_6 = POlp3_33-POlp4_32
    h[1] = h_1
    h[2] = h_2
    h[3] = h_3
    h[4] = h_4
    h[5] = h_5
    h[6] = h_6
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = 0
    Jac[1,8] = 0
    Jac[1,9] = -JTlp2_13_2
    Jac[1,10] = 0
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = -RLlp1_32
    Jac[2,8] = -JTlp2_23_1
    Jac[2,9] = -JTlp2_23_2
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = RLlp1_22
    Jac[3,8] = -JTlp2_33_1
    Jac[3,9] = -JTlp2_33_2
    Jac[3,10] = 0
    Jac[3,11] = 0
    Jac[3,12] = 0
    Jac[3,13] = 0
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
    Jac[4,13] = JTlp3_13_2
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
    Jac[5,11] = RLlp4_32
    Jac[5,12] = JTlp3_23_1
    Jac[5,13] = JTlp3_23_2
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
    Jac[6,11] = -RLlp4_22
    Jac[6,12] = JTlp3_33_1
    Jac[6,13] = JTlp3_33_2

# Number of continuation lines = 0


