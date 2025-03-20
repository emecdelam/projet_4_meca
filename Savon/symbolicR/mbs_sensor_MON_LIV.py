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
#	==> Generation Date: Tue Mar 18 12:43:11 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: MON_LIV
#
#	==> Number of joints: 12
#
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
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


  if (isens == 1): 

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
    OPcp1_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp1_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp1_16 = qd[4]+qd[6]*S5
    OMcp1_26 = OMcp1_25+ROcp1_85*qd[6]
    OMcp1_36 = OMcp1_35+ROcp1_95*qd[6]
    OPcp1_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp1_25*ROcp1_95-OMcp1_35*ROcp1_85)
    OPcp1_26 = OPcp1_25+ROcp1_85*qdd[6]+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4])
    OPcp1_36 = OPcp1_35+ROcp1_95*qdd[6]+qd[6]*(-OMcp1_25*S5+ROcp1_85*qd[4])
    RLcp1_17 = ROcp1_16*s.dpt[1,1]+ROcp1_46*s.dpt[2,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]+ROcp1_56*s.dpt[2,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]+ROcp1_66*s.dpt[2,1]
    POcp1_17 = RLcp1_17+q[1]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[3]
    OMcp1_17 = OMcp1_16+qd[7]*S5
    OMcp1_27 = OMcp1_26+ROcp1_85*qd[7]
    OMcp1_37 = OMcp1_36+ROcp1_95*qd[7]
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[1]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[3]
    OPcp1_17 = OPcp1_16+qdd[7]*S5+qd[7]*(OMcp1_26*ROcp1_95-OMcp1_36*ROcp1_85)
    OPcp1_27 = OPcp1_26+ROcp1_85*qdd[7]+qd[7]*(-OMcp1_16*ROcp1_95+OMcp1_36*S5)
    OPcp1_37 = OPcp1_36+ROcp1_95*qdd[7]+qd[7]*(OMcp1_16*ROcp1_85-OMcp1_26*S5)
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    OMcp1_18 = OMcp1_17+ROcp1_47*qd[8]
    OMcp1_28 = OMcp1_27+ROcp1_57*qd[8]
    OMcp1_38 = OMcp1_37+ROcp1_67*qd[8]
    OPcp1_18 = OPcp1_17+ROcp1_47*qdd[8]+qd[8]*(OMcp1_27*ROcp1_67-OMcp1_37*ROcp1_57)
    OPcp1_28 = OPcp1_27+ROcp1_57*qdd[8]+qd[8]*(-OMcp1_17*ROcp1_67+OMcp1_37*ROcp1_47)
    OPcp1_38 = OPcp1_37+ROcp1_67*qdd[8]+qd[8]*(OMcp1_17*ROcp1_57-OMcp1_27*ROcp1_47)
    sens.P[1] = POcp1_17
    sens.P[2] = POcp1_27
    sens.P[3] = POcp1_37
    sens.R[1,1] = ROcp1_18
    sens.R[1,2] = ROcp1_28
    sens.R[1,3] = ROcp1_38
    sens.R[2,1] = ROcp1_47
    sens.R[2,2] = ROcp1_57
    sens.R[2,3] = ROcp1_67
    sens.R[3,1] = ROcp1_78
    sens.R[3,2] = ROcp1_88
    sens.R[3,3] = ROcp1_98
    sens.V[1] = VIcp1_17
    sens.V[2] = VIcp1_27
    sens.V[3] = VIcp1_37
    sens.OM[1] = OMcp1_18
    sens.OM[2] = OMcp1_28
    sens.OM[3] = OMcp1_38
    sens.A[1] = ACcp1_17
    sens.A[2] = ACcp1_27
    sens.A[3] = ACcp1_37
    sens.OMP[1] = OPcp1_18
    sens.OMP[2] = OPcp1_28
    sens.OMP[3] = OPcp1_38

  if (isens == 2): 

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
    OPcp2_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp2_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp2_16 = qd[4]+qd[6]*S5
    OMcp2_26 = OMcp2_25+ROcp2_85*qd[6]
    OMcp2_36 = OMcp2_35+ROcp2_95*qd[6]
    OPcp2_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp2_25*ROcp2_95-OMcp2_35*ROcp2_85)
    OPcp2_26 = OPcp2_25+ROcp2_85*qdd[6]+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4])
    OPcp2_36 = OPcp2_35+ROcp2_95*qdd[6]+qd[6]*(-OMcp2_25*S5+ROcp2_85*qd[4])
    RLcp2_17 = ROcp2_16*s.dpt[1,2]+ROcp2_46*s.dpt[2,2]
    RLcp2_27 = ROcp2_26*s.dpt[1,2]+ROcp2_56*s.dpt[2,2]
    RLcp2_37 = ROcp2_36*s.dpt[1,2]+ROcp2_66*s.dpt[2,2]
    POcp2_17 = RLcp2_17+q[1]
    POcp2_27 = RLcp2_27+q[2]
    POcp2_37 = RLcp2_37+q[3]
    OMcp2_17 = OMcp2_16+ROcp2_46*qd[9]
    OMcp2_27 = OMcp2_26+ROcp2_56*qd[9]
    OMcp2_37 = OMcp2_36+ROcp2_66*qd[9]
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+qd[1]
    VIcp2_27 = ORcp2_27+qd[2]
    VIcp2_37 = ORcp2_37+qd[3]
    OPcp2_17 = OPcp2_16+ROcp2_46*qdd[9]+qd[9]*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)
    OPcp2_27 = OPcp2_26+ROcp2_56*qdd[9]+qd[9]*(-OMcp2_16*ROcp2_66+OMcp2_36*ROcp2_46)
    OPcp2_37 = OPcp2_36+ROcp2_66*qdd[9]+qd[9]*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    sens.P[1] = POcp2_17
    sens.P[2] = POcp2_27
    sens.P[3] = POcp2_37
    sens.R[1,1] = ROcp2_19
    sens.R[1,2] = ROcp2_29
    sens.R[1,3] = ROcp2_39
    sens.R[2,1] = ROcp2_46
    sens.R[2,2] = ROcp2_56
    sens.R[2,3] = ROcp2_66
    sens.R[3,1] = ROcp2_79
    sens.R[3,2] = ROcp2_89
    sens.R[3,3] = ROcp2_99
    sens.V[1] = VIcp2_17
    sens.V[2] = VIcp2_27
    sens.V[3] = VIcp2_37
    sens.OM[1] = OMcp2_17
    sens.OM[2] = OMcp2_27
    sens.OM[3] = OMcp2_37
    sens.A[1] = ACcp2_17
    sens.A[2] = ACcp2_27
    sens.A[3] = ACcp2_37
    sens.OMP[1] = OPcp2_17
    sens.OMP[2] = OPcp2_27
    sens.OMP[3] = OPcp2_37

  if (isens == 3): 

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
    OPcp3_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp3_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp3_16 = qd[4]+qd[6]*S5
    OMcp3_26 = OMcp3_25+ROcp3_85*qd[6]
    OMcp3_36 = OMcp3_35+ROcp3_95*qd[6]
    OPcp3_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp3_25*ROcp3_95-OMcp3_35*ROcp3_85)
    OPcp3_26 = OPcp3_25+ROcp3_85*qdd[6]+qd[6]*(OMcp3_35*S5-ROcp3_95*qd[4])
    OPcp3_36 = OPcp3_35+ROcp3_95*qdd[6]+qd[6]*(-OMcp3_25*S5+ROcp3_85*qd[4])
    RLcp3_17 = ROcp3_16*s.dpt[1,3]+ROcp3_46*s.dpt[2,3]
    RLcp3_27 = ROcp3_26*s.dpt[1,3]+ROcp3_56*s.dpt[2,3]
    RLcp3_37 = ROcp3_36*s.dpt[1,3]+ROcp3_66*s.dpt[2,3]
    POcp3_17 = RLcp3_17+q[1]
    POcp3_27 = RLcp3_27+q[2]
    POcp3_37 = RLcp3_37+q[3]
    OMcp3_17 = OMcp3_16+ROcp3_46*qd[10]
    OMcp3_27 = OMcp3_26+ROcp3_56*qd[10]
    OMcp3_37 = OMcp3_36+ROcp3_66*qd[10]
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+qd[1]
    VIcp3_27 = ORcp3_27+qd[2]
    VIcp3_37 = ORcp3_37+qd[3]
    OPcp3_17 = OPcp3_16+ROcp3_46*qdd[10]+qd[10]*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)
    OPcp3_27 = OPcp3_26+ROcp3_56*qdd[10]+qd[10]*(-OMcp3_16*ROcp3_66+OMcp3_36*ROcp3_46)
    OPcp3_37 = OPcp3_36+ROcp3_66*qdd[10]+qd[10]*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)
    ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    sens.P[1] = POcp3_17
    sens.P[2] = POcp3_27
    sens.P[3] = POcp3_37
    sens.R[1,1] = ROcp3_110
    sens.R[1,2] = ROcp3_210
    sens.R[1,3] = ROcp3_310
    sens.R[2,1] = ROcp3_46
    sens.R[2,2] = ROcp3_56
    sens.R[2,3] = ROcp3_66
    sens.R[3,1] = ROcp3_710
    sens.R[3,2] = ROcp3_810
    sens.R[3,3] = ROcp3_910
    sens.V[1] = VIcp3_17
    sens.V[2] = VIcp3_27
    sens.V[3] = VIcp3_37
    sens.OM[1] = OMcp3_17
    sens.OM[2] = OMcp3_27
    sens.OM[3] = OMcp3_37
    sens.A[1] = ACcp3_17
    sens.A[2] = ACcp3_27
    sens.A[3] = ACcp3_37
    sens.OMP[1] = OPcp3_17
    sens.OMP[2] = OPcp3_27
    sens.OMP[3] = OPcp3_37

  if (isens == 4): 

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
    OPcp4_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp4_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp4_16 = qd[4]+qd[6]*S5
    OMcp4_26 = OMcp4_25+ROcp4_85*qd[6]
    OMcp4_36 = OMcp4_35+ROcp4_95*qd[6]
    OPcp4_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp4_25*ROcp4_95-OMcp4_35*ROcp4_85)
    OPcp4_26 = OPcp4_25+ROcp4_85*qdd[6]+qd[6]*(OMcp4_35*S5-ROcp4_95*qd[4])
    OPcp4_36 = OPcp4_35+ROcp4_95*qdd[6]+qd[6]*(-OMcp4_25*S5+ROcp4_85*qd[4])
    RLcp4_17 = ROcp4_16*s.dpt[1,4]+ROcp4_46*s.dpt[2,4]
    RLcp4_27 = ROcp4_26*s.dpt[1,4]+ROcp4_56*s.dpt[2,4]
    RLcp4_37 = ROcp4_36*s.dpt[1,4]+ROcp4_66*s.dpt[2,4]
    POcp4_17 = RLcp4_17+q[1]
    POcp4_27 = RLcp4_27+q[2]
    POcp4_37 = RLcp4_37+q[3]
    OMcp4_17 = OMcp4_16+qd[11]*S5
    OMcp4_27 = OMcp4_26+ROcp4_85*qd[11]
    OMcp4_37 = OMcp4_36+ROcp4_95*qd[11]
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = ORcp4_17+qd[1]
    VIcp4_27 = ORcp4_27+qd[2]
    VIcp4_37 = ORcp4_37+qd[3]
    OPcp4_17 = OPcp4_16+qdd[11]*S5+qd[11]*(OMcp4_26*ROcp4_95-OMcp4_36*ROcp4_85)
    OPcp4_27 = OPcp4_26+ROcp4_85*qdd[11]+qd[11]*(-OMcp4_16*ROcp4_95+OMcp4_36*S5)
    OPcp4_37 = OPcp4_36+ROcp4_95*qdd[11]+qd[11]*(OMcp4_16*ROcp4_85-OMcp4_26*S5)
    ACcp4_17 = qdd[1]+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27
    ACcp4_27 = qdd[2]-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17
    ACcp4_37 = qdd[3]+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17
    OMcp4_18 = OMcp4_17+ROcp4_411*qd[12]
    OMcp4_28 = OMcp4_27+ROcp4_511*qd[12]
    OMcp4_38 = OMcp4_37+ROcp4_611*qd[12]
    OPcp4_18 = OPcp4_17+ROcp4_411*qdd[12]+qd[12]*(OMcp4_27*ROcp4_611-OMcp4_37*ROcp4_511)
    OPcp4_28 = OPcp4_27+ROcp4_511*qdd[12]+qd[12]*(-OMcp4_17*ROcp4_611+OMcp4_37*ROcp4_411)
    OPcp4_38 = OPcp4_37+ROcp4_611*qdd[12]+qd[12]*(OMcp4_17*ROcp4_511-OMcp4_27*ROcp4_411)
    sens.P[1] = POcp4_17
    sens.P[2] = POcp4_27
    sens.P[3] = POcp4_37
    sens.R[1,1] = ROcp4_112
    sens.R[1,2] = ROcp4_212
    sens.R[1,3] = ROcp4_312
    sens.R[2,1] = ROcp4_411
    sens.R[2,2] = ROcp4_511
    sens.R[2,3] = ROcp4_611
    sens.R[3,1] = ROcp4_712
    sens.R[3,2] = ROcp4_812
    sens.R[3,3] = ROcp4_912
    sens.V[1] = VIcp4_17
    sens.V[2] = VIcp4_27
    sens.V[3] = VIcp4_37
    sens.OM[1] = OMcp4_18
    sens.OM[2] = OMcp4_28
    sens.OM[3] = OMcp4_38
    sens.A[1] = ACcp4_17
    sens.A[2] = ACcp4_27
    sens.A[3] = ACcp4_37
    sens.OMP[1] = OPcp4_18
    sens.OMP[2] = OPcp4_28
    sens.OMP[3] = OPcp4_38

 


# Number of continuation lines = 0


