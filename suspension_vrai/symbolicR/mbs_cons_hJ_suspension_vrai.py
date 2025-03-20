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
#	==> Generation Date: Thu Mar 20 14:58:16 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 12
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
    h[1] = h_1
    h[2] = h_2
    h[3] = h_3
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

# Number of continuation lines = 0


