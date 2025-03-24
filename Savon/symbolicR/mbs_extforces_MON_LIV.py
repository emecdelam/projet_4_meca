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
#	==> Generation Date: Mon Mar 24 13:26:35 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: MON_LIV
#
#	==> Number of joints: 12
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

    PxF2 = zeros(4)
    RxF2 = zeros((4, 4))
    VxF2 = zeros(4)
    OMxF2 = zeros(4)
    AxF2 = zeros(4)
    OMPxF2 = zeros(4)

    PxF3 = zeros(4)
    RxF3 = zeros((4, 4))
    VxF3 = zeros(4)
    OMxF3 = zeros(4)
    AxF3 = zeros(4)
    OMPxF3 = zeros(4)

    PxF4 = zeros(4)
    RxF4 = zeros((4, 4))
    VxF4 = zeros(4)
    OMxF4 = zeros(4)
    AxF4 = zeros(4)
    OMPxF4 = zeros(4)

 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

    ROcp1_25 = S4*S5
    ROcp1_35 = -C4*S5
    ROcp1_85 = -S4*C5
    ROcp1_95 = C4*C5
    ROcp1_16 = C5*C6
    ROcp1_26 = ROcp1_25*C6+C4*S6
    ROcp1_36 = ROcp1_35*C6+S4*S6
    ROcp1_46 = -C5*S6
    ROcp1_56 = -ROcp1_25*S6+C4*C6
    ROcp1_66 = -ROcp1_35*S6+S4*C6
    ROcp1_17 = ROcp1_16*C7+ROcp1_46*S7
    ROcp1_27 = ROcp1_26*C7+ROcp1_56*S7
    ROcp1_37 = ROcp1_36*C7+ROcp1_66*S7
    ROcp1_47 = -ROcp1_16*S7+ROcp1_46*C7
    ROcp1_57 = -ROcp1_26*S7+ROcp1_56*C7
    ROcp1_67 = -ROcp1_36*S7+ROcp1_66*C7
    ROcp1_18 = ROcp1_17*C8-S5*S8
    ROcp1_28 = ROcp1_27*C8-ROcp1_85*S8
    ROcp1_38 = ROcp1_37*C8-ROcp1_95*S8
    ROcp1_78 = ROcp1_17*S8+S5*C8
    ROcp1_88 = ROcp1_27*S8+ROcp1_85*C8
    ROcp1_98 = ROcp1_37*S8+ROcp1_95*C8
    OMcp1_25 = qd[5]*C4
    OMcp1_35 = qd[5]*S4
    OPcp1_25 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp1_35 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp1_16 = qd[4]+qd[6]*S5
    OMcp1_26 = OMcp1_25+qd[6]*ROcp1_85
    OMcp1_36 = OMcp1_35+qd[6]*ROcp1_95
    OPcp1_16 = qdd[4]+qd[6]*(OMcp1_25*ROcp1_95-OMcp1_35*ROcp1_85)+qdd[6]*S5
    OPcp1_26 = OPcp1_25+qd[6]*(-qd[4]*ROcp1_95+OMcp1_35*S5)+qdd[6]*ROcp1_85
    OPcp1_36 = OPcp1_35+qd[6]*(qd[4]*ROcp1_85-OMcp1_25*S5)+qdd[6]*ROcp1_95
    RLcp1_17 = ROcp1_16*s.dpt[1,1]+ROcp1_46*s.dpt[2,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]+ROcp1_56*s.dpt[2,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]+ROcp1_66*s.dpt[2,1]
    POcp1_17 = q[1]+RLcp1_17
    POcp1_27 = q[2]+RLcp1_27
    POcp1_37 = q[3]+RLcp1_37
    OMcp1_17 = OMcp1_16+qd[7]*S5
    OMcp1_27 = OMcp1_26+qd[7]*ROcp1_85
    OMcp1_37 = OMcp1_36+qd[7]*ROcp1_95
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = qd[1]+ORcp1_17
    VIcp1_27 = qd[2]+ORcp1_27
    VIcp1_37 = qd[3]+ORcp1_37
    OPcp1_17 = OPcp1_16+qd[7]*(OMcp1_26*ROcp1_95-OMcp1_36*ROcp1_85)+qdd[7]*S5
    OPcp1_27 = OPcp1_26+qd[7]*(-OMcp1_16*ROcp1_95+OMcp1_36*S5)+qdd[7]*ROcp1_85
    OPcp1_37 = OPcp1_36+qd[7]*(OMcp1_16*ROcp1_85-OMcp1_26*S5)+qdd[7]*ROcp1_95
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    OMcp1_18 = OMcp1_17+qd[8]*ROcp1_47
    OMcp1_28 = OMcp1_27+qd[8]*ROcp1_57
    OMcp1_38 = OMcp1_37+qd[8]*ROcp1_67
    OPcp1_18 = OPcp1_17+qd[8]*(OMcp1_27*ROcp1_67-OMcp1_37*ROcp1_57)+qdd[8]*ROcp1_47
    OPcp1_28 = OPcp1_27+qd[8]*(-OMcp1_17*ROcp1_67+OMcp1_37*ROcp1_47)+qdd[8]*ROcp1_57
    OPcp1_38 = OPcp1_37+qd[8]*(OMcp1_17*ROcp1_57-OMcp1_27*ROcp1_47)+qdd[8]*ROcp1_67
    PxF1[1] = POcp1_17
    PxF1[2] = POcp1_27
    PxF1[3] = POcp1_37
    RxF1[1,1] = ROcp1_18
    RxF1[1,2] = ROcp1_28
    RxF1[1,3] = ROcp1_38
    RxF1[2,1] = ROcp1_47
    RxF1[2,2] = ROcp1_57
    RxF1[2,3] = ROcp1_67
    RxF1[3,1] = ROcp1_78
    RxF1[3,2] = ROcp1_88
    RxF1[3,3] = ROcp1_98
    VxF1[1] = VIcp1_17
    VxF1[2] = VIcp1_27
    VxF1[3] = VIcp1_37
    OMxF1[1] = OMcp1_18
    OMxF1[2] = OMcp1_28
    OMxF1[3] = OMcp1_38
    AxF1[1] = ACcp1_17
    AxF1[2] = ACcp1_27
    AxF1[3] = ACcp1_37
    OMPxF1[1] = OPcp1_18
    OMPxF1[2] = OPcp1_28
    OMPxF1[3] = OPcp1_38
    ROcp2_25 = S4*S5
    ROcp2_35 = -C4*S5
    ROcp2_85 = -S4*C5
    ROcp2_95 = C4*C5
    ROcp2_16 = C5*C6
    ROcp2_26 = ROcp2_25*C6+C4*S6
    ROcp2_36 = ROcp2_35*C6+S4*S6
    ROcp2_46 = -C5*S6
    ROcp2_56 = -ROcp2_25*S6+C4*C6
    ROcp2_66 = -ROcp2_35*S6+S4*C6
    ROcp2_19 = ROcp2_16*C9-S5*S9
    ROcp2_29 = ROcp2_26*C9-ROcp2_85*S9
    ROcp2_39 = ROcp2_36*C9-ROcp2_95*S9
    ROcp2_79 = ROcp2_16*S9+S5*C9
    ROcp2_89 = ROcp2_26*S9+ROcp2_85*C9
    ROcp2_99 = ROcp2_36*S9+ROcp2_95*C9
    OMcp2_25 = qd[5]*C4
    OMcp2_35 = qd[5]*S4
    OPcp2_25 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp2_35 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp2_16 = qd[4]+qd[6]*S5
    OMcp2_26 = OMcp2_25+qd[6]*ROcp2_85
    OMcp2_36 = OMcp2_35+qd[6]*ROcp2_95
    OPcp2_16 = qdd[4]+qd[6]*(OMcp2_25*ROcp2_95-OMcp2_35*ROcp2_85)+qdd[6]*S5
    OPcp2_26 = OPcp2_25+qd[6]*(-qd[4]*ROcp2_95+OMcp2_35*S5)+qdd[6]*ROcp2_85
    OPcp2_36 = OPcp2_35+qd[6]*(qd[4]*ROcp2_85-OMcp2_25*S5)+qdd[6]*ROcp2_95
    RLcp2_17 = ROcp2_16*s.dpt[1,2]+ROcp2_46*s.dpt[2,2]
    RLcp2_27 = ROcp2_26*s.dpt[1,2]+ROcp2_56*s.dpt[2,2]
    RLcp2_37 = ROcp2_36*s.dpt[1,2]+ROcp2_66*s.dpt[2,2]
    POcp2_17 = q[1]+RLcp2_17
    POcp2_27 = q[2]+RLcp2_27
    POcp2_37 = q[3]+RLcp2_37
    OMcp2_17 = OMcp2_16+qd[9]*ROcp2_46
    OMcp2_27 = OMcp2_26+qd[9]*ROcp2_56
    OMcp2_37 = OMcp2_36+qd[9]*ROcp2_66
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = qd[1]+ORcp2_17
    VIcp2_27 = qd[2]+ORcp2_27
    VIcp2_37 = qd[3]+ORcp2_37
    OPcp2_17 = OPcp2_16+qd[9]*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qdd[9]*ROcp2_46
    OPcp2_27 = OPcp2_26+qd[9]*(-OMcp2_16*ROcp2_66+OMcp2_36*ROcp2_46)+qdd[9]*ROcp2_56
    OPcp2_37 = OPcp2_36+qd[9]*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qdd[9]*ROcp2_66
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    PxF2[1] = POcp2_17
    PxF2[2] = POcp2_27
    PxF2[3] = POcp2_37
    RxF2[1,1] = ROcp2_19
    RxF2[1,2] = ROcp2_29
    RxF2[1,3] = ROcp2_39
    RxF2[2,1] = ROcp2_46
    RxF2[2,2] = ROcp2_56
    RxF2[2,3] = ROcp2_66
    RxF2[3,1] = ROcp2_79
    RxF2[3,2] = ROcp2_89
    RxF2[3,3] = ROcp2_99
    VxF2[1] = VIcp2_17
    VxF2[2] = VIcp2_27
    VxF2[3] = VIcp2_37
    OMxF2[1] = OMcp2_17
    OMxF2[2] = OMcp2_27
    OMxF2[3] = OMcp2_37
    AxF2[1] = ACcp2_17
    AxF2[2] = ACcp2_27
    AxF2[3] = ACcp2_37
    OMPxF2[1] = OPcp2_17
    OMPxF2[2] = OPcp2_27
    OMPxF2[3] = OPcp2_37
    ROcp3_25 = S4*S5
    ROcp3_35 = -C4*S5
    ROcp3_85 = -S4*C5
    ROcp3_95 = C4*C5
    ROcp3_16 = C5*C6
    ROcp3_26 = ROcp3_25*C6+C4*S6
    ROcp3_36 = ROcp3_35*C6+S4*S6
    ROcp3_46 = -C5*S6
    ROcp3_56 = -ROcp3_25*S6+C4*C6
    ROcp3_66 = -ROcp3_35*S6+S4*C6
    ROcp3_110 = ROcp3_16*C10-S10*S5
    ROcp3_210 = ROcp3_26*C10-ROcp3_85*S10
    ROcp3_310 = ROcp3_36*C10-ROcp3_95*S10
    ROcp3_710 = ROcp3_16*S10+C10*S5
    ROcp3_810 = ROcp3_26*S10+ROcp3_85*C10
    ROcp3_910 = ROcp3_36*S10+ROcp3_95*C10
    OMcp3_25 = qd[5]*C4
    OMcp3_35 = qd[5]*S4
    OPcp3_25 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp3_35 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp3_16 = qd[4]+qd[6]*S5
    OMcp3_26 = OMcp3_25+qd[6]*ROcp3_85
    OMcp3_36 = OMcp3_35+qd[6]*ROcp3_95
    OPcp3_16 = qdd[4]+qd[6]*(OMcp3_25*ROcp3_95-OMcp3_35*ROcp3_85)+qdd[6]*S5
    OPcp3_26 = OPcp3_25+qd[6]*(-qd[4]*ROcp3_95+OMcp3_35*S5)+qdd[6]*ROcp3_85
    OPcp3_36 = OPcp3_35+qd[6]*(qd[4]*ROcp3_85-OMcp3_25*S5)+qdd[6]*ROcp3_95
    RLcp3_17 = ROcp3_16*s.dpt[1,3]+ROcp3_46*s.dpt[2,3]
    RLcp3_27 = ROcp3_26*s.dpt[1,3]+ROcp3_56*s.dpt[2,3]
    RLcp3_37 = ROcp3_36*s.dpt[1,3]+ROcp3_66*s.dpt[2,3]
    POcp3_17 = q[1]+RLcp3_17
    POcp3_27 = q[2]+RLcp3_27
    POcp3_37 = q[3]+RLcp3_37
    OMcp3_17 = OMcp3_16+qd[10]*ROcp3_46
    OMcp3_27 = OMcp3_26+qd[10]*ROcp3_56
    OMcp3_37 = OMcp3_36+qd[10]*ROcp3_66
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = qd[1]+ORcp3_17
    VIcp3_27 = qd[2]+ORcp3_27
    VIcp3_37 = qd[3]+ORcp3_37
    OPcp3_17 = OPcp3_16+qd[10]*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qdd[10]*ROcp3_46
    OPcp3_27 = OPcp3_26+qd[10]*(-OMcp3_16*ROcp3_66+OMcp3_36*ROcp3_46)+qdd[10]*ROcp3_56
    OPcp3_37 = OPcp3_36+qd[10]*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qdd[10]*ROcp3_66
    ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    PxF3[1] = POcp3_17
    PxF3[2] = POcp3_27
    PxF3[3] = POcp3_37
    RxF3[1,1] = ROcp3_110
    RxF3[1,2] = ROcp3_210
    RxF3[1,3] = ROcp3_310
    RxF3[2,1] = ROcp3_46
    RxF3[2,2] = ROcp3_56
    RxF3[2,3] = ROcp3_66
    RxF3[3,1] = ROcp3_710
    RxF3[3,2] = ROcp3_810
    RxF3[3,3] = ROcp3_910
    VxF3[1] = VIcp3_17
    VxF3[2] = VIcp3_27
    VxF3[3] = VIcp3_37
    OMxF3[1] = OMcp3_17
    OMxF3[2] = OMcp3_27
    OMxF3[3] = OMcp3_37
    AxF3[1] = ACcp3_17
    AxF3[2] = ACcp3_27
    AxF3[3] = ACcp3_37
    OMPxF3[1] = OPcp3_17
    OMPxF3[2] = OPcp3_27
    OMPxF3[3] = OPcp3_37
    ROcp4_25 = S4*S5
    ROcp4_35 = -C4*S5
    ROcp4_85 = -S4*C5
    ROcp4_95 = C4*C5
    ROcp4_16 = C5*C6
    ROcp4_26 = ROcp4_25*C6+C4*S6
    ROcp4_36 = ROcp4_35*C6+S4*S6
    ROcp4_46 = -C5*S6
    ROcp4_56 = -ROcp4_25*S6+C4*C6
    ROcp4_66 = -ROcp4_35*S6+S4*C6
    ROcp4_111 = ROcp4_16*C11+ROcp4_46*S11
    ROcp4_211 = ROcp4_26*C11+ROcp4_56*S11
    ROcp4_311 = ROcp4_36*C11+ROcp4_66*S11
    ROcp4_411 = -ROcp4_16*S11+ROcp4_46*C11
    ROcp4_511 = -ROcp4_26*S11+ROcp4_56*C11
    ROcp4_611 = -ROcp4_36*S11+ROcp4_66*C11
    ROcp4_112 = ROcp4_111*C12-S12*S5
    ROcp4_212 = ROcp4_211*C12-ROcp4_85*S12
    ROcp4_312 = ROcp4_311*C12-ROcp4_95*S12
    ROcp4_712 = ROcp4_111*S12+C12*S5
    ROcp4_812 = ROcp4_211*S12+ROcp4_85*C12
    ROcp4_912 = ROcp4_311*S12+ROcp4_95*C12
    OMcp4_25 = qd[5]*C4
    OMcp4_35 = qd[5]*S4
    OPcp4_25 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp4_35 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp4_16 = qd[4]+qd[6]*S5
    OMcp4_26 = OMcp4_25+qd[6]*ROcp4_85
    OMcp4_36 = OMcp4_35+qd[6]*ROcp4_95
    OPcp4_16 = qdd[4]+qd[6]*(OMcp4_25*ROcp4_95-OMcp4_35*ROcp4_85)+qdd[6]*S5
    OPcp4_26 = OPcp4_25+qd[6]*(-qd[4]*ROcp4_95+OMcp4_35*S5)+qdd[6]*ROcp4_85
    OPcp4_36 = OPcp4_35+qd[6]*(qd[4]*ROcp4_85-OMcp4_25*S5)+qdd[6]*ROcp4_95
    RLcp4_17 = ROcp4_16*s.dpt[1,4]+ROcp4_46*s.dpt[2,4]
    RLcp4_27 = ROcp4_26*s.dpt[1,4]+ROcp4_56*s.dpt[2,4]
    RLcp4_37 = ROcp4_36*s.dpt[1,4]+ROcp4_66*s.dpt[2,4]
    POcp4_17 = q[1]+RLcp4_17
    POcp4_27 = q[2]+RLcp4_27
    POcp4_37 = q[3]+RLcp4_37
    OMcp4_17 = OMcp4_16+qd[11]*S5
    OMcp4_27 = OMcp4_26+qd[11]*ROcp4_85
    OMcp4_37 = OMcp4_36+qd[11]*ROcp4_95
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = qd[1]+ORcp4_17
    VIcp4_27 = qd[2]+ORcp4_27
    VIcp4_37 = qd[3]+ORcp4_37
    OPcp4_17 = OPcp4_16+qd[11]*(OMcp4_26*ROcp4_95-OMcp4_36*ROcp4_85)+qdd[11]*S5
    OPcp4_27 = OPcp4_26+qd[11]*(-OMcp4_16*ROcp4_95+OMcp4_36*S5)+qdd[11]*ROcp4_85
    OPcp4_37 = OPcp4_36+qd[11]*(OMcp4_16*ROcp4_85-OMcp4_26*S5)+qdd[11]*ROcp4_95
    ACcp4_17 = qdd[1]+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27
    ACcp4_27 = qdd[2]-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17
    ACcp4_37 = qdd[3]+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17
    OMcp4_18 = OMcp4_17+qd[12]*ROcp4_411
    OMcp4_28 = OMcp4_27+qd[12]*ROcp4_511
    OMcp4_38 = OMcp4_37+qd[12]*ROcp4_611
    OPcp4_18 = OPcp4_17+qd[12]*(OMcp4_27*ROcp4_611-OMcp4_37*ROcp4_511)+qdd[12]*ROcp4_411
    OPcp4_28 = OPcp4_27+qd[12]*(-OMcp4_17*ROcp4_611+OMcp4_37*ROcp4_411)+qdd[12]*ROcp4_511
    OPcp4_38 = OPcp4_37+qd[12]*(OMcp4_17*ROcp4_511-OMcp4_27*ROcp4_411)+qdd[12]*ROcp4_611
    PxF4[1] = POcp4_17
    PxF4[2] = POcp4_27
    PxF4[3] = POcp4_37
    RxF4[1,1] = ROcp4_112
    RxF4[1,2] = ROcp4_212
    RxF4[1,3] = ROcp4_312
    RxF4[2,1] = ROcp4_411
    RxF4[2,2] = ROcp4_511
    RxF4[2,3] = ROcp4_611
    RxF4[3,1] = ROcp4_712
    RxF4[3,2] = ROcp4_812
    RxF4[3,3] = ROcp4_912
    VxF4[1] = VIcp4_17
    VxF4[2] = VIcp4_27
    VxF4[3] = VIcp4_37
    OMxF4[1] = OMcp4_18
    OMxF4[2] = OMcp4_28
    OMxF4[3] = OMcp4_38
    AxF4[1] = ACcp4_17
    AxF4[2] = ACcp4_27
    AxF4[3] = ACcp4_37
    OMPxF4[1] = OPcp4_18
    OMPxF4[2] = OPcp4_28
    OMPxF4[3] = OPcp4_38
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    SWr2 = s.user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2)
    SWr3 = s.user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3)
    SWr4 = s.user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_8_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*SWr1[8]
    trqext_2_8_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_8_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
    xfrc12 = RxF2[1,1]*SWr2[1]+RxF2[1,2]*SWr2[2]+RxF2[1,3]*SWr2[3]
    xfrc22 = RxF2[2,1]*SWr2[1]+RxF2[2,2]*SWr2[2]+RxF2[2,3]*SWr2[3]
    xfrc32 = RxF2[3,1]*SWr2[1]+RxF2[3,2]*SWr2[2]+RxF2[3,3]*SWr2[3]
    xtrq12 = RxF2[1,1]*SWr2[4]+RxF2[1,2]*SWr2[5]+RxF2[1,3]*SWr2[6]
    xtrq22 = RxF2[2,1]*SWr2[4]+RxF2[2,2]*SWr2[5]+RxF2[2,3]*SWr2[6]
    xtrq32 = RxF2[3,1]*SWr2[4]+RxF2[3,2]*SWr2[5]+RxF2[3,3]*SWr2[6]
    trqext_1_9_1 = xtrq12-xfrc22*SWr2[9]+xfrc32*SWr2[8]
    trqext_2_9_1 = xtrq22+xfrc12*SWr2[9]-xfrc32*SWr2[7]
    trqext_3_9_1 = xtrq32-xfrc12*SWr2[8]+xfrc22*SWr2[7]
    xfrc13 = RxF3[1,1]*SWr3[1]+RxF3[1,2]*SWr3[2]+RxF3[1,3]*SWr3[3]
    xfrc23 = RxF3[2,1]*SWr3[1]+RxF3[2,2]*SWr3[2]+RxF3[2,3]*SWr3[3]
    xfrc33 = RxF3[3,1]*SWr3[1]+RxF3[3,2]*SWr3[2]+RxF3[3,3]*SWr3[3]
    xtrq13 = RxF3[1,1]*SWr3[4]+RxF3[1,2]*SWr3[5]+RxF3[1,3]*SWr3[6]
    xtrq23 = RxF3[2,1]*SWr3[4]+RxF3[2,2]*SWr3[5]+RxF3[2,3]*SWr3[6]
    xtrq33 = RxF3[3,1]*SWr3[4]+RxF3[3,2]*SWr3[5]+RxF3[3,3]*SWr3[6]
    trqext_1_10_2 = xtrq13-xfrc23*SWr3[9]+xfrc33*SWr3[8]
    trqext_2_10_2 = xtrq23+xfrc13*SWr3[9]-xfrc33*SWr3[7]
    trqext_3_10_2 = xtrq33-xfrc13*SWr3[8]+xfrc23*SWr3[7]
    xfrc14 = RxF4[1,1]*SWr4[1]+RxF4[1,2]*SWr4[2]+RxF4[1,3]*SWr4[3]
    xfrc24 = RxF4[2,1]*SWr4[1]+RxF4[2,2]*SWr4[2]+RxF4[2,3]*SWr4[3]
    xfrc34 = RxF4[3,1]*SWr4[1]+RxF4[3,2]*SWr4[2]+RxF4[3,3]*SWr4[3]
    xtrq14 = RxF4[1,1]*SWr4[4]+RxF4[1,2]*SWr4[5]+RxF4[1,3]*SWr4[6]
    xtrq24 = RxF4[2,1]*SWr4[4]+RxF4[2,2]*SWr4[5]+RxF4[2,3]*SWr4[6]
    xtrq34 = RxF4[3,1]*SWr4[4]+RxF4[3,2]*SWr4[5]+RxF4[3,3]*SWr4[6]
    trqext_1_12_3 = xtrq14-xfrc24*SWr4[9]+xfrc34*SWr4[8]
    trqext_2_12_3 = xtrq24+xfrc14*SWr4[9]-xfrc34*SWr4[7]
    trqext_3_12_3 = xtrq34-xfrc14*SWr4[8]+xfrc24*SWr4[7]
 
# Symbolic model output

    frc[1,8] = s.frc[1,8]+xfrc11
    frc[2,8] = s.frc[2,8]+xfrc21
    frc[3,8] = s.frc[3,8]+xfrc31
    trq[1,8] = s.trq[1,8]+trqext_1_8_0
    trq[2,8] = s.trq[2,8]+trqext_2_8_0
    trq[3,8] = s.trq[3,8]+trqext_3_8_0
    frc[1,9] = s.frc[1,9]+xfrc12
    frc[2,9] = s.frc[2,9]+xfrc22
    frc[3,9] = s.frc[3,9]+xfrc32
    trq[1,9] = s.trq[1,9]+trqext_1_9_1
    trq[2,9] = s.trq[2,9]+trqext_2_9_1
    trq[3,9] = s.trq[3,9]+trqext_3_9_1
    frc[1,10] = s.frc[1,10]+xfrc13
    frc[2,10] = s.frc[2,10]+xfrc23
    frc[3,10] = s.frc[3,10]+xfrc33
    trq[1,10] = s.trq[1,10]+trqext_1_10_2
    trq[2,10] = s.trq[2,10]+trqext_2_10_2
    trq[3,10] = s.trq[3,10]+trqext_3_10_2
    frc[1,12] = s.frc[1,12]+xfrc14
    frc[2,12] = s.frc[2,12]+xfrc24
    frc[3,12] = s.frc[3,12]+xfrc34
    trq[1,12] = s.trq[1,12]+trqext_1_12_3
    trq[2,12] = s.trq[2,12]+trqext_2_12_3
    trq[3,12] = s.trq[3,12]+trqext_3_12_3

# Number of continuation lines = 0


