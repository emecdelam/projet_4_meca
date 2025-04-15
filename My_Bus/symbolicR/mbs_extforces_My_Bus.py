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
#	==> Function: F19 - External Forces
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos
from numpy import zeros

def extforces(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    qdd = s.qdd
    frc = s.frc
    trq = s.trq

    PxF1 = zeros(4)
    RxF1 = zeros((4, 4))
    VxF1 = zeros(4)
    OMxF1 = zeros(4)
    AxF1 = zeros(4)
    OMPxF1 = zeros(4)

 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

    ROcp1_45 = -S4*C5
    ROcp1_55 = C4*C5
    ROcp1_75 = S4*S5
    ROcp1_85 = -C4*S5
    ROcp1_16 = -ROcp1_75*S6+C4*C6
    ROcp1_26 = -ROcp1_85*S6+S4*C6
    ROcp1_36 = -C5*S6
    ROcp1_76 = ROcp1_75*C6+C4*S6
    ROcp1_86 = ROcp1_85*C6+S4*S6
    ROcp1_96 = C5*C6
    OMcp1_15 = qd[5]*C4
    OMcp1_25 = qd[5]*S4
    OPcp1_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp1_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp1_16 = OMcp1_15+qd[6]*ROcp1_45
    OMcp1_26 = OMcp1_25+qd[6]*ROcp1_55
    OMcp1_36 = qd[4]+qd[6]*S5
    OPcp1_16 = OPcp1_15+qd[6]*(-qd[4]*ROcp1_55+OMcp1_25*S5)+qdd[6]*ROcp1_45
    OPcp1_26 = OPcp1_25+qd[6]*(qd[4]*ROcp1_45-OMcp1_15*S5)+qdd[6]*ROcp1_55
    OPcp1_36 = qdd[4]+qd[6]*(OMcp1_15*ROcp1_55-OMcp1_25*ROcp1_45)+qdd[6]*S5
    PxF1[1] = q[1]
    PxF1[2] = q[2]
    PxF1[3] = q[3]
    RxF1[1,1] = ROcp1_16
    RxF1[1,2] = ROcp1_26
    RxF1[1,3] = ROcp1_36
    RxF1[2,1] = ROcp1_45
    RxF1[2,2] = ROcp1_55
    RxF1[2,3] = S5
    RxF1[3,1] = ROcp1_76
    RxF1[3,2] = ROcp1_86
    RxF1[3,3] = ROcp1_96
    VxF1[1] = qd[1]
    VxF1[2] = qd[2]
    VxF1[3] = qd[3]
    OMxF1[1] = OMcp1_16
    OMxF1[2] = OMcp1_26
    OMxF1[3] = OMcp1_36
    AxF1[1] = qdd[1]
    AxF1[2] = qdd[2]
    AxF1[3] = qdd[3]
    OMPxF1[1] = OPcp1_16
    OMPxF1[2] = OPcp1_26
    OMPxF1[3] = OPcp1_36
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_6_0 = xtrq11-xfrc21*(SWr1[9]-s.l[3,6])+xfrc31*SWr1[8]
    trqext_2_6_0 = xtrq21+xfrc11*(SWr1[9]-s.l[3,6])-xfrc31*SWr1[7]
    trqext_3_6_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+xfrc11
    frc[2,6] = s.frc[2,6]+xfrc21
    frc[3,6] = s.frc[3,6]+xfrc31
    trq[1,6] = s.trq[1,6]+trqext_1_6_0
    trq[2,6] = s.trq[2,6]+trqext_2_6_0
    trq[3,6] = s.trq[3,6]+trqext_3_6_0

# Number of continuation lines = 0


