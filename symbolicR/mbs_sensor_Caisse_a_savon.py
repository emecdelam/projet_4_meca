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
#	==> Generation Date: Tue Mar 11 09:40:44 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Caisse_a_savon
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
    ROcp1_17 = ROcp1_16*C7-ROcp1_76*S7
    ROcp1_27 = ROcp1_26*C7-ROcp1_86*S7
    ROcp1_37 = ROcp1_36*C7-ROcp1_96*S7
    ROcp1_77 = ROcp1_16*S7+ROcp1_76*C7
    ROcp1_87 = ROcp1_26*S7+ROcp1_86*C7
    ROcp1_97 = ROcp1_36*S7+ROcp1_96*C7
    OMcp1_15 = qd[5]*C4
    OMcp1_25 = qd[5]*S4
    OPcp1_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp1_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp1_16 = OMcp1_15+ROcp1_45*qd[6]
    OMcp1_26 = OMcp1_25+ROcp1_55*qd[6]
    OMcp1_36 = qd[4]+qd[6]*S5
    OPcp1_16 = OPcp1_15+ROcp1_45*qdd[6]+qd[6]*(OMcp1_25*S5-ROcp1_55*qd[4])
    OPcp1_26 = OPcp1_25+ROcp1_55*qdd[6]+qd[6]*(-OMcp1_15*S5+ROcp1_45*qd[4])
    OPcp1_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp1_15*ROcp1_55-OMcp1_25*ROcp1_45)
    RLcp1_17 = ROcp1_16*s.dpt[1,1]+ROcp1_45*s.dpt[2,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]+ROcp1_55*s.dpt[2,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]+s.dpt[2,1]*S5
    POcp1_17 = RLcp1_17+q[1]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[3]
    OMcp1_17 = OMcp1_16+ROcp1_45*qd[7]
    OMcp1_27 = OMcp1_26+ROcp1_55*qd[7]
    OMcp1_37 = OMcp1_36+qd[7]*S5
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[1]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[3]
    OPcp1_17 = OPcp1_16+ROcp1_45*qdd[7]+qd[7]*(OMcp1_26*S5-OMcp1_36*ROcp1_55)
    OPcp1_27 = OPcp1_26+ROcp1_55*qdd[7]+qd[7]*(-OMcp1_16*S5+OMcp1_36*ROcp1_45)
    OPcp1_37 = OPcp1_36+qdd[7]*S5+qd[7]*(OMcp1_16*ROcp1_55-OMcp1_26*ROcp1_45)
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    sens.P[1] = POcp1_17
    sens.P[2] = POcp1_27
    sens.P[3] = POcp1_37
    sens.R[1,1] = ROcp1_17
    sens.R[1,2] = ROcp1_27
    sens.R[1,3] = ROcp1_37
    sens.R[2,1] = ROcp1_45
    sens.R[2,2] = ROcp1_55
    sens.R[2,3] = S5
    sens.R[3,1] = ROcp1_77
    sens.R[3,2] = ROcp1_87
    sens.R[3,3] = ROcp1_97
    sens.V[1] = VIcp1_17
    sens.V[2] = VIcp1_27
    sens.V[3] = VIcp1_37
    sens.OM[1] = OMcp1_17
    sens.OM[2] = OMcp1_27
    sens.OM[3] = OMcp1_37
    sens.A[1] = ACcp1_17
    sens.A[2] = ACcp1_27
    sens.A[3] = ACcp1_37
    sens.OMP[1] = OPcp1_17
    sens.OMP[2] = OPcp1_27
    sens.OMP[3] = OPcp1_37

  if (isens == 2): 

    ROcp2_45 = -S4*C5
    ROcp2_55 = C4*C5
    ROcp2_75 = S4*S5
    ROcp2_85 = -C4*S5
    ROcp2_16 = -ROcp2_75*S6+C4*C6
    ROcp2_26 = -ROcp2_85*S6+S4*C6
    ROcp2_36 = -C5*S6
    ROcp2_76 = ROcp2_75*C6+C4*S6
    ROcp2_86 = ROcp2_85*C6+S4*S6
    ROcp2_96 = C5*C6
    ROcp2_18 = ROcp2_16*C8-ROcp2_76*S8
    ROcp2_28 = ROcp2_26*C8-ROcp2_86*S8
    ROcp2_38 = ROcp2_36*C8-ROcp2_96*S8
    ROcp2_78 = ROcp2_16*S8+ROcp2_76*C8
    ROcp2_88 = ROcp2_26*S8+ROcp2_86*C8
    ROcp2_98 = ROcp2_36*S8+ROcp2_96*C8
    OMcp2_15 = qd[5]*C4
    OMcp2_25 = qd[5]*S4
    OPcp2_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp2_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp2_16 = OMcp2_15+ROcp2_45*qd[6]
    OMcp2_26 = OMcp2_25+ROcp2_55*qd[6]
    OMcp2_36 = qd[4]+qd[6]*S5
    OPcp2_16 = OPcp2_15+ROcp2_45*qdd[6]+qd[6]*(OMcp2_25*S5-ROcp2_55*qd[4])
    OPcp2_26 = OPcp2_25+ROcp2_55*qdd[6]+qd[6]*(-OMcp2_15*S5+ROcp2_45*qd[4])
    OPcp2_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp2_15*ROcp2_55-OMcp2_25*ROcp2_45)
    RLcp2_17 = ROcp2_16*s.dpt[1,2]+ROcp2_45*s.dpt[2,2]
    RLcp2_27 = ROcp2_26*s.dpt[1,2]+ROcp2_55*s.dpt[2,2]
    RLcp2_37 = ROcp2_36*s.dpt[1,2]+s.dpt[2,2]*S5
    POcp2_17 = RLcp2_17+q[1]
    POcp2_27 = RLcp2_27+q[2]
    POcp2_37 = RLcp2_37+q[3]
    OMcp2_17 = OMcp2_16+ROcp2_45*qd[8]
    OMcp2_27 = OMcp2_26+ROcp2_55*qd[8]
    OMcp2_37 = OMcp2_36+qd[8]*S5
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+qd[1]
    VIcp2_27 = ORcp2_27+qd[2]
    VIcp2_37 = ORcp2_37+qd[3]
    OPcp2_17 = OPcp2_16+ROcp2_45*qdd[8]+qd[8]*(OMcp2_26*S5-OMcp2_36*ROcp2_55)
    OPcp2_27 = OPcp2_26+ROcp2_55*qdd[8]+qd[8]*(-OMcp2_16*S5+OMcp2_36*ROcp2_45)
    OPcp2_37 = OPcp2_36+qdd[8]*S5+qd[8]*(OMcp2_16*ROcp2_55-OMcp2_26*ROcp2_45)
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    sens.P[1] = POcp2_17
    sens.P[2] = POcp2_27
    sens.P[3] = POcp2_37
    sens.R[1,1] = ROcp2_18
    sens.R[1,2] = ROcp2_28
    sens.R[1,3] = ROcp2_38
    sens.R[2,1] = ROcp2_45
    sens.R[2,2] = ROcp2_55
    sens.R[2,3] = S5
    sens.R[3,1] = ROcp2_78
    sens.R[3,2] = ROcp2_88
    sens.R[3,3] = ROcp2_98
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

    ROcp3_45 = -S4*C5
    ROcp3_55 = C4*C5
    ROcp3_75 = S4*S5
    ROcp3_85 = -C4*S5
    ROcp3_16 = -ROcp3_75*S6+C4*C6
    ROcp3_26 = -ROcp3_85*S6+S4*C6
    ROcp3_36 = -C5*S6
    ROcp3_76 = ROcp3_75*C6+C4*S6
    ROcp3_86 = ROcp3_85*C6+S4*S6
    ROcp3_96 = C5*C6
    ROcp3_19 = ROcp3_16*C9+ROcp3_45*S9
    ROcp3_29 = ROcp3_26*C9+ROcp3_55*S9
    ROcp3_39 = ROcp3_36*C9+S5*S9
    ROcp3_49 = -ROcp3_16*S9+ROcp3_45*C9
    ROcp3_59 = -ROcp3_26*S9+ROcp3_55*C9
    ROcp3_69 = -ROcp3_36*S9+S5*C9
    ROcp3_110 = ROcp3_19*C10-ROcp3_76*S10
    ROcp3_210 = ROcp3_29*C10-ROcp3_86*S10
    ROcp3_310 = ROcp3_39*C10-ROcp3_96*S10
    ROcp3_710 = ROcp3_19*S10+ROcp3_76*C10
    ROcp3_810 = ROcp3_29*S10+ROcp3_86*C10
    ROcp3_910 = ROcp3_39*S10+ROcp3_96*C10
    OMcp3_15 = qd[5]*C4
    OMcp3_25 = qd[5]*S4
    OPcp3_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp3_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp3_16 = OMcp3_15+ROcp3_45*qd[6]
    OMcp3_26 = OMcp3_25+ROcp3_55*qd[6]
    OMcp3_36 = qd[4]+qd[6]*S5
    OPcp3_16 = OPcp3_15+ROcp3_45*qdd[6]+qd[6]*(OMcp3_25*S5-ROcp3_55*qd[4])
    OPcp3_26 = OPcp3_25+ROcp3_55*qdd[6]+qd[6]*(-OMcp3_15*S5+ROcp3_45*qd[4])
    OPcp3_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp3_15*ROcp3_55-OMcp3_25*ROcp3_45)
    RLcp3_17 = ROcp3_16*s.dpt[1,3]+ROcp3_45*s.dpt[2,3]
    RLcp3_27 = ROcp3_26*s.dpt[1,3]+ROcp3_55*s.dpt[2,3]
    RLcp3_37 = ROcp3_36*s.dpt[1,3]+s.dpt[2,3]*S5
    POcp3_17 = RLcp3_17+q[1]
    POcp3_27 = RLcp3_27+q[2]
    POcp3_37 = RLcp3_37+q[3]
    OMcp3_17 = OMcp3_16+ROcp3_76*qd[9]
    OMcp3_27 = OMcp3_26+ROcp3_86*qd[9]
    OMcp3_37 = OMcp3_36+ROcp3_96*qd[9]
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+qd[1]
    VIcp3_27 = ORcp3_27+qd[2]
    VIcp3_37 = ORcp3_37+qd[3]
    OPcp3_17 = OPcp3_16+ROcp3_76*qdd[9]+qd[9]*(OMcp3_26*ROcp3_96-OMcp3_36*ROcp3_86)
    OPcp3_27 = OPcp3_26+ROcp3_86*qdd[9]+qd[9]*(-OMcp3_16*ROcp3_96+OMcp3_36*ROcp3_76)
    OPcp3_37 = OPcp3_36+ROcp3_96*qdd[9]+qd[9]*(OMcp3_16*ROcp3_86-OMcp3_26*ROcp3_76)
    ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    OMcp3_18 = OMcp3_17+ROcp3_49*qd[10]
    OMcp3_28 = OMcp3_27+ROcp3_59*qd[10]
    OMcp3_38 = OMcp3_37+ROcp3_69*qd[10]
    OPcp3_18 = OPcp3_17+ROcp3_49*qdd[10]+qd[10]*(OMcp3_27*ROcp3_69-OMcp3_37*ROcp3_59)
    OPcp3_28 = OPcp3_27+ROcp3_59*qdd[10]+qd[10]*(-OMcp3_17*ROcp3_69+OMcp3_37*ROcp3_49)
    OPcp3_38 = OPcp3_37+ROcp3_69*qdd[10]+qd[10]*(OMcp3_17*ROcp3_59-OMcp3_27*ROcp3_49)
    sens.P[1] = POcp3_17
    sens.P[2] = POcp3_27
    sens.P[3] = POcp3_37
    sens.R[1,1] = ROcp3_110
    sens.R[1,2] = ROcp3_210
    sens.R[1,3] = ROcp3_310
    sens.R[2,1] = ROcp3_49
    sens.R[2,2] = ROcp3_59
    sens.R[2,3] = ROcp3_69
    sens.R[3,1] = ROcp3_710
    sens.R[3,2] = ROcp3_810
    sens.R[3,3] = ROcp3_910
    sens.V[1] = VIcp3_17
    sens.V[2] = VIcp3_27
    sens.V[3] = VIcp3_37
    sens.OM[1] = OMcp3_18
    sens.OM[2] = OMcp3_28
    sens.OM[3] = OMcp3_38
    sens.A[1] = ACcp3_17
    sens.A[2] = ACcp3_27
    sens.A[3] = ACcp3_37
    sens.OMP[1] = OPcp3_18
    sens.OMP[2] = OPcp3_28
    sens.OMP[3] = OPcp3_38

  if (isens == 4): 

    ROcp4_45 = -S4*C5
    ROcp4_55 = C4*C5
    ROcp4_75 = S4*S5
    ROcp4_85 = -C4*S5
    ROcp4_16 = -ROcp4_75*S6+C4*C6
    ROcp4_26 = -ROcp4_85*S6+S4*C6
    ROcp4_36 = -C5*S6
    ROcp4_76 = ROcp4_75*C6+C4*S6
    ROcp4_86 = ROcp4_85*C6+S4*S6
    ROcp4_96 = C5*C6
    ROcp4_111 = ROcp4_16*C11+ROcp4_45*S11
    ROcp4_211 = ROcp4_26*C11+ROcp4_55*S11
    ROcp4_311 = ROcp4_36*C11+S11*S5
    ROcp4_411 = -ROcp4_16*S11+ROcp4_45*C11
    ROcp4_511 = -ROcp4_26*S11+ROcp4_55*C11
    ROcp4_611 = -ROcp4_36*S11+C11*S5
    ROcp4_112 = ROcp4_111*C12-ROcp4_76*S12
    ROcp4_212 = ROcp4_211*C12-ROcp4_86*S12
    ROcp4_312 = ROcp4_311*C12-ROcp4_96*S12
    ROcp4_712 = ROcp4_111*S12+ROcp4_76*C12
    ROcp4_812 = ROcp4_211*S12+ROcp4_86*C12
    ROcp4_912 = ROcp4_311*S12+ROcp4_96*C12
    OMcp4_15 = qd[5]*C4
    OMcp4_25 = qd[5]*S4
    OPcp4_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp4_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp4_16 = OMcp4_15+ROcp4_45*qd[6]
    OMcp4_26 = OMcp4_25+ROcp4_55*qd[6]
    OMcp4_36 = qd[4]+qd[6]*S5
    OPcp4_16 = OPcp4_15+ROcp4_45*qdd[6]+qd[6]*(OMcp4_25*S5-ROcp4_55*qd[4])
    OPcp4_26 = OPcp4_25+ROcp4_55*qdd[6]+qd[6]*(-OMcp4_15*S5+ROcp4_45*qd[4])
    OPcp4_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp4_15*ROcp4_55-OMcp4_25*ROcp4_45)
    RLcp4_17 = ROcp4_16*s.dpt[1,4]+ROcp4_45*s.dpt[2,4]
    RLcp4_27 = ROcp4_26*s.dpt[1,4]+ROcp4_55*s.dpt[2,4]
    RLcp4_37 = ROcp4_36*s.dpt[1,4]+s.dpt[2,4]*S5
    POcp4_17 = RLcp4_17+q[1]
    POcp4_27 = RLcp4_27+q[2]
    POcp4_37 = RLcp4_37+q[3]
    OMcp4_17 = OMcp4_16+ROcp4_76*qd[11]
    OMcp4_27 = OMcp4_26+ROcp4_86*qd[11]
    OMcp4_37 = OMcp4_36+ROcp4_96*qd[11]
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = ORcp4_17+qd[1]
    VIcp4_27 = ORcp4_27+qd[2]
    VIcp4_37 = ORcp4_37+qd[3]
    OPcp4_17 = OPcp4_16+ROcp4_76*qdd[11]+qd[11]*(OMcp4_26*ROcp4_96-OMcp4_36*ROcp4_86)
    OPcp4_27 = OPcp4_26+ROcp4_86*qdd[11]+qd[11]*(-OMcp4_16*ROcp4_96+OMcp4_36*ROcp4_76)
    OPcp4_37 = OPcp4_36+ROcp4_96*qdd[11]+qd[11]*(OMcp4_16*ROcp4_86-OMcp4_26*ROcp4_76)
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


