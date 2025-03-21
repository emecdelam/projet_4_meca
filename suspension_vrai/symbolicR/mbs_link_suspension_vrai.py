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
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S8 = sin(q[8])
    C8 = cos(q[8])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,9]*C8-s.dpt[3,9]*S8
    RLlnk2_32 = s.dpt[2,9]*S8+s.dpt[3,9]*C8
    POlnk2_22 = RLlnk2_22+s.dpt[2,3]
    ORlnk2_22 = -qd[8]*RLlnk2_32
    ORlnk2_32 = qd[8]*RLlnk2_22
    Plnk11 = -s.dpt[1,1]+s.dpt[1,3]
    Plnk21 = POlnk2_22-s.dpt[2,1]
    Plnk31 = RLlnk2_32-s.dpt[3,1]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*s.dpt[3,1]+fPlnk31*s.dpt[2,1]
    trqlnk6_1_2 = fPlnk11*s.dpt[3,1]-fPlnk31*s.dpt[1,1]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,1]+fPlnk21*s.dpt[1,1]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C8+e31*S8)
    fSlnk31 = Flink1*(-e21*S8+e31*C8)
    trqlnk8_1_1 = fSlnk21*s.dpt[3,9]-fSlnk31*s.dpt[2,9]
    trqlnk8_1_2 = -fSlnk11*s.dpt[3,9]
    trqlnk8_1_3 = fSlnk11*s.dpt[2,9]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+fPlnk11
    frc[2,6] = s.frc[2,6]+fPlnk21
    frc[3,6] = s.frc[3,6]+fPlnk31
    trq[1,6] = s.trq[1,6]+trqlnk6_1_1
    trq[2,6] = s.trq[2,6]+trqlnk6_1_2
    trq[3,6] = s.trq[3,6]+trqlnk6_1_3
    frc[1,8] = s.frc[1,8]-fSlnk11
    frc[2,8] = s.frc[2,8]-fSlnk21
    frc[3,8] = s.frc[3,8]-fSlnk31
    trq[1,8] = s.trq[1,8]+trqlnk8_1_1
    trq[2,8] = s.trq[2,8]+trqlnk8_1_2
    trq[3,8] = s.trq[3,8]+trqlnk8_1_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


