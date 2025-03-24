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
#	==> Generation Date: Mon Mar 24 19:02:42 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 26
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
  S12 = sin(q[12])
  C12 = cos(q[12])
  S13 = sin(q[13])
  C13 = cos(q[13])
  S14 = sin(q[14])
  C14 = cos(q[14])
  S15 = sin(q[15])
  C15 = cos(q[15])
  S20 = sin(q[20])
  C20 = cos(q[20])
  S21 = sin(q[21])
  C21 = cos(q[21])
  S22 = sin(q[22])
  C22 = cos(q[22])
  S25 = sin(q[25])
  C25 = cos(q[25])
  S26 = sin(q[26])
  C26 = cos(q[26])
 
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
    ROcp1_47 = ROcp1_45*C7+ROcp1_76*S7
    ROcp1_57 = ROcp1_55*C7+ROcp1_86*S7
    ROcp1_67 = ROcp1_96*S7+S5*C7
    ROcp1_77 = -ROcp1_45*S7+ROcp1_76*C7
    ROcp1_87 = -ROcp1_55*S7+ROcp1_86*C7
    ROcp1_97 = ROcp1_96*C7-S5*S7
    ROcp1_48 = ROcp1_47*C8+ROcp1_77*S8
    ROcp1_58 = ROcp1_57*C8+ROcp1_87*S8
    ROcp1_68 = ROcp1_67*C8+ROcp1_97*S8
    ROcp1_78 = -ROcp1_47*S8+ROcp1_77*C8
    ROcp1_88 = -ROcp1_57*S8+ROcp1_87*C8
    ROcp1_98 = -ROcp1_67*S8+ROcp1_97*C8
    ROcp1_49 = ROcp1_48*C9+ROcp1_78*S9
    ROcp1_59 = ROcp1_58*C9+ROcp1_88*S9
    ROcp1_69 = ROcp1_68*C9+ROcp1_98*S9
    ROcp1_79 = -ROcp1_48*S9+ROcp1_78*C9
    ROcp1_89 = -ROcp1_58*S9+ROcp1_88*C9
    ROcp1_99 = -ROcp1_68*S9+ROcp1_98*C9
    ROcp1_110 = ROcp1_16*C10-ROcp1_79*S10
    ROcp1_210 = ROcp1_26*C10-ROcp1_89*S10
    ROcp1_310 = ROcp1_36*C10-ROcp1_99*S10
    ROcp1_710 = ROcp1_16*S10+ROcp1_79*C10
    ROcp1_810 = ROcp1_26*S10+ROcp1_89*C10
    ROcp1_910 = ROcp1_36*S10+ROcp1_99*C10
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
    POcp1_17 = RLcp1_17+q[3]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[1]
    OMcp1_17 = OMcp1_16+ROcp1_16*qd[7]
    OMcp1_27 = OMcp1_26+ROcp1_26*qd[7]
    OMcp1_37 = OMcp1_36+ROcp1_36*qd[7]
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[3]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[1]
    OPcp1_17 = OPcp1_16+ROcp1_16*qdd[7]+qd[7]*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26)
    OPcp1_27 = OPcp1_26+ROcp1_26*qdd[7]+qd[7]*(-OMcp1_16*ROcp1_36+OMcp1_36*ROcp1_16)
    OPcp1_37 = OPcp1_36+ROcp1_36*qdd[7]+qd[7]*(OMcp1_16*ROcp1_26-OMcp1_26*ROcp1_16)
    ACcp1_17 = qdd[3]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[1]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    RLcp1_18 = ROcp1_47*s.dpt[2,16]
    RLcp1_28 = ROcp1_57*s.dpt[2,16]
    RLcp1_38 = ROcp1_67*s.dpt[2,16]
    POcp1_18 = POcp1_17+RLcp1_18
    POcp1_28 = POcp1_27+RLcp1_28
    POcp1_38 = POcp1_37+RLcp1_38
    OMcp1_18 = OMcp1_17+ROcp1_16*qd[8]
    OMcp1_28 = OMcp1_27+ROcp1_26*qd[8]
    OMcp1_38 = OMcp1_37+ROcp1_36*qd[8]
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28
    ORcp1_28 = -OMcp1_17*RLcp1_38+OMcp1_37*RLcp1_18
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18
    VIcp1_18 = ORcp1_18+VIcp1_17
    VIcp1_28 = ORcp1_28+VIcp1_27
    VIcp1_38 = ORcp1_38+VIcp1_37
    OPcp1_18 = OPcp1_17+ROcp1_16*qdd[8]+qd[8]*(OMcp1_27*ROcp1_36-OMcp1_37*ROcp1_26)
    OPcp1_28 = OPcp1_27+ROcp1_26*qdd[8]+qd[8]*(-OMcp1_17*ROcp1_36+OMcp1_37*ROcp1_16)
    OPcp1_38 = OPcp1_37+ROcp1_36*qdd[8]+qd[8]*(OMcp1_17*ROcp1_26-OMcp1_27*ROcp1_16)
    ACcp1_18 = ACcp1_17+OMcp1_27*ORcp1_38-OMcp1_37*ORcp1_28+OPcp1_27*RLcp1_38-OPcp1_37*RLcp1_28
    ACcp1_28 = ACcp1_27-OMcp1_17*ORcp1_38+OMcp1_37*ORcp1_18-OPcp1_17*RLcp1_38+OPcp1_37*RLcp1_18
    ACcp1_38 = ACcp1_37+OMcp1_17*ORcp1_28-OMcp1_27*ORcp1_18+OPcp1_17*RLcp1_28-OPcp1_27*RLcp1_18
    RLcp1_19 = ROcp1_48*s.dpt[2,19]+ROcp1_78*s.dpt[3,19]
    RLcp1_29 = ROcp1_58*s.dpt[2,19]+ROcp1_88*s.dpt[3,19]
    RLcp1_39 = ROcp1_68*s.dpt[2,19]+ROcp1_98*s.dpt[3,19]
    POcp1_19 = POcp1_18+RLcp1_19
    POcp1_29 = POcp1_28+RLcp1_29
    POcp1_39 = POcp1_38+RLcp1_39
    OMcp1_19 = OMcp1_18+ROcp1_16*qd[9]
    OMcp1_29 = OMcp1_28+ROcp1_26*qd[9]
    OMcp1_39 = OMcp1_38+ROcp1_36*qd[9]
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29
    ORcp1_29 = -OMcp1_18*RLcp1_39+OMcp1_38*RLcp1_19
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_18
    VIcp1_29 = ORcp1_29+VIcp1_28
    VIcp1_39 = ORcp1_39+VIcp1_38
    OPcp1_19 = OPcp1_18+ROcp1_16*qdd[9]+qd[9]*(OMcp1_28*ROcp1_36-OMcp1_38*ROcp1_26)
    OPcp1_29 = OPcp1_28+ROcp1_26*qdd[9]+qd[9]*(-OMcp1_18*ROcp1_36+OMcp1_38*ROcp1_16)
    OPcp1_39 = OPcp1_38+ROcp1_36*qdd[9]+qd[9]*(OMcp1_18*ROcp1_26-OMcp1_28*ROcp1_16)
    ACcp1_19 = ACcp1_18+OMcp1_28*ORcp1_39-OMcp1_38*ORcp1_29+OPcp1_28*RLcp1_39-OPcp1_38*RLcp1_29
    ACcp1_29 = ACcp1_28-OMcp1_18*ORcp1_39+OMcp1_38*ORcp1_19-OPcp1_18*RLcp1_39+OPcp1_38*RLcp1_19
    ACcp1_39 = ACcp1_38+OMcp1_18*ORcp1_29-OMcp1_28*ORcp1_19+OPcp1_18*RLcp1_29-OPcp1_28*RLcp1_19
    OMcp1_110 = OMcp1_19+ROcp1_49*qd[10]
    OMcp1_210 = OMcp1_29+ROcp1_59*qd[10]
    OMcp1_310 = OMcp1_39+ROcp1_69*qd[10]
    OPcp1_110 = OPcp1_19+ROcp1_49*qdd[10]+qd[10]*(OMcp1_29*ROcp1_69-OMcp1_39*ROcp1_59)
    OPcp1_210 = OPcp1_29+ROcp1_59*qdd[10]+qd[10]*(-OMcp1_19*ROcp1_69+OMcp1_39*ROcp1_49)
    OPcp1_310 = OPcp1_39+ROcp1_69*qdd[10]+qd[10]*(OMcp1_19*ROcp1_59-OMcp1_29*ROcp1_49)
    sens.P[1] = POcp1_19
    sens.P[2] = POcp1_29
    sens.P[3] = POcp1_39
    sens.R[1,1] = ROcp1_110
    sens.R[1,2] = ROcp1_210
    sens.R[1,3] = ROcp1_310
    sens.R[2,1] = ROcp1_49
    sens.R[2,2] = ROcp1_59
    sens.R[2,3] = ROcp1_69
    sens.R[3,1] = ROcp1_710
    sens.R[3,2] = ROcp1_810
    sens.R[3,3] = ROcp1_910
    sens.V[1] = VIcp1_19
    sens.V[2] = VIcp1_29
    sens.V[3] = VIcp1_39
    sens.OM[1] = OMcp1_110
    sens.OM[2] = OMcp1_210
    sens.OM[3] = OMcp1_310
    sens.A[1] = ACcp1_19
    sens.A[2] = ACcp1_29
    sens.A[3] = ACcp1_39
    sens.OMP[1] = OPcp1_110
    sens.OMP[2] = OPcp1_210
    sens.OMP[3] = OPcp1_310

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
    ROcp2_412 = ROcp2_45*C12+ROcp2_76*S12
    ROcp2_512 = ROcp2_55*C12+ROcp2_86*S12
    ROcp2_612 = ROcp2_96*S12+C12*S5
    ROcp2_712 = -ROcp2_45*S12+ROcp2_76*C12
    ROcp2_812 = -ROcp2_55*S12+ROcp2_86*C12
    ROcp2_912 = ROcp2_96*C12-S12*S5
    ROcp2_413 = ROcp2_412*C13+ROcp2_712*S13
    ROcp2_513 = ROcp2_512*C13+ROcp2_812*S13
    ROcp2_613 = ROcp2_612*C13+ROcp2_912*S13
    ROcp2_713 = -ROcp2_412*S13+ROcp2_712*C13
    ROcp2_813 = -ROcp2_512*S13+ROcp2_812*C13
    ROcp2_913 = -ROcp2_612*S13+ROcp2_912*C13
    ROcp2_414 = ROcp2_413*C14+ROcp2_713*S14
    ROcp2_514 = ROcp2_513*C14+ROcp2_813*S14
    ROcp2_614 = ROcp2_613*C14+ROcp2_913*S14
    ROcp2_714 = -ROcp2_413*S14+ROcp2_713*C14
    ROcp2_814 = -ROcp2_513*S14+ROcp2_813*C14
    ROcp2_914 = -ROcp2_613*S14+ROcp2_913*C14
    ROcp2_115 = ROcp2_16*C15-ROcp2_714*S15
    ROcp2_215 = ROcp2_26*C15-ROcp2_814*S15
    ROcp2_315 = ROcp2_36*C15-ROcp2_914*S15
    ROcp2_715 = ROcp2_16*S15+ROcp2_714*C15
    ROcp2_815 = ROcp2_26*S15+ROcp2_814*C15
    ROcp2_915 = ROcp2_36*S15+ROcp2_914*C15
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
    RLcp2_17 = ROcp2_16*s.dpt[1,4]+ROcp2_45*s.dpt[2,4]
    RLcp2_27 = ROcp2_26*s.dpt[1,4]+ROcp2_55*s.dpt[2,4]
    RLcp2_37 = ROcp2_36*s.dpt[1,4]+s.dpt[2,4]*S5
    POcp2_17 = RLcp2_17+q[3]
    POcp2_27 = RLcp2_27+q[2]
    POcp2_37 = RLcp2_37+q[1]
    OMcp2_17 = OMcp2_16+ROcp2_16*qd[12]
    OMcp2_27 = OMcp2_26+ROcp2_26*qd[12]
    OMcp2_37 = OMcp2_36+ROcp2_36*qd[12]
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+qd[3]
    VIcp2_27 = ORcp2_27+qd[2]
    VIcp2_37 = ORcp2_37+qd[1]
    OPcp2_17 = OPcp2_16+ROcp2_16*qdd[12]+qd[12]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26)
    OPcp2_27 = OPcp2_26+ROcp2_26*qdd[12]+qd[12]*(-OMcp2_16*ROcp2_36+OMcp2_36*ROcp2_16)
    OPcp2_37 = OPcp2_36+ROcp2_36*qdd[12]+qd[12]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16)
    ACcp2_17 = qdd[3]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[1]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    RLcp2_18 = ROcp2_412*s.dpt[2,22]
    RLcp2_28 = ROcp2_512*s.dpt[2,22]
    RLcp2_38 = ROcp2_612*s.dpt[2,22]
    POcp2_18 = POcp2_17+RLcp2_18
    POcp2_28 = POcp2_27+RLcp2_28
    POcp2_38 = POcp2_37+RLcp2_38
    OMcp2_18 = OMcp2_17+ROcp2_16*qd[13]
    OMcp2_28 = OMcp2_27+ROcp2_26*qd[13]
    OMcp2_38 = OMcp2_37+ROcp2_36*qd[13]
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28
    ORcp2_28 = -OMcp2_17*RLcp2_38+OMcp2_37*RLcp2_18
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18
    VIcp2_18 = ORcp2_18+VIcp2_17
    VIcp2_28 = ORcp2_28+VIcp2_27
    VIcp2_38 = ORcp2_38+VIcp2_37
    OPcp2_18 = OPcp2_17+ROcp2_16*qdd[13]+qd[13]*(OMcp2_27*ROcp2_36-OMcp2_37*ROcp2_26)
    OPcp2_28 = OPcp2_27+ROcp2_26*qdd[13]+qd[13]*(-OMcp2_17*ROcp2_36+OMcp2_37*ROcp2_16)
    OPcp2_38 = OPcp2_37+ROcp2_36*qdd[13]+qd[13]*(OMcp2_17*ROcp2_26-OMcp2_27*ROcp2_16)
    ACcp2_18 = ACcp2_17+OMcp2_27*ORcp2_38-OMcp2_37*ORcp2_28+OPcp2_27*RLcp2_38-OPcp2_37*RLcp2_28
    ACcp2_28 = ACcp2_27-OMcp2_17*ORcp2_38+OMcp2_37*ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18
    ACcp2_38 = ACcp2_37+OMcp2_17*ORcp2_28-OMcp2_27*ORcp2_18+OPcp2_17*RLcp2_28-OPcp2_27*RLcp2_18
    RLcp2_19 = ROcp2_413*s.dpt[2,25]+ROcp2_713*s.dpt[3,25]
    RLcp2_29 = ROcp2_513*s.dpt[2,25]+ROcp2_813*s.dpt[3,25]
    RLcp2_39 = ROcp2_613*s.dpt[2,25]+ROcp2_913*s.dpt[3,25]
    POcp2_19 = POcp2_18+RLcp2_19
    POcp2_29 = POcp2_28+RLcp2_29
    POcp2_39 = POcp2_38+RLcp2_39
    OMcp2_19 = OMcp2_18+ROcp2_16*qd[14]
    OMcp2_29 = OMcp2_28+ROcp2_26*qd[14]
    OMcp2_39 = OMcp2_38+ROcp2_36*qd[14]
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_18
    VIcp2_29 = ORcp2_29+VIcp2_28
    VIcp2_39 = ORcp2_39+VIcp2_38
    OPcp2_19 = OPcp2_18+ROcp2_16*qdd[14]+qd[14]*(OMcp2_28*ROcp2_36-OMcp2_38*ROcp2_26)
    OPcp2_29 = OPcp2_28+ROcp2_26*qdd[14]+qd[14]*(-OMcp2_18*ROcp2_36+OMcp2_38*ROcp2_16)
    OPcp2_39 = OPcp2_38+ROcp2_36*qdd[14]+qd[14]*(OMcp2_18*ROcp2_26-OMcp2_28*ROcp2_16)
    ACcp2_19 = ACcp2_18+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_28-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_38+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    OMcp2_110 = OMcp2_19+ROcp2_414*qd[15]
    OMcp2_210 = OMcp2_29+ROcp2_514*qd[15]
    OMcp2_310 = OMcp2_39+ROcp2_614*qd[15]
    OPcp2_110 = OPcp2_19+ROcp2_414*qdd[15]+qd[15]*(OMcp2_29*ROcp2_614-OMcp2_39*ROcp2_514)
    OPcp2_210 = OPcp2_29+ROcp2_514*qdd[15]+qd[15]*(-OMcp2_19*ROcp2_614+OMcp2_39*ROcp2_414)
    OPcp2_310 = OPcp2_39+ROcp2_614*qdd[15]+qd[15]*(OMcp2_19*ROcp2_514-OMcp2_29*ROcp2_414)
    sens.P[1] = POcp2_19
    sens.P[2] = POcp2_29
    sens.P[3] = POcp2_39
    sens.R[1,1] = ROcp2_115
    sens.R[1,2] = ROcp2_215
    sens.R[1,3] = ROcp2_315
    sens.R[2,1] = ROcp2_414
    sens.R[2,2] = ROcp2_514
    sens.R[2,3] = ROcp2_614
    sens.R[3,1] = ROcp2_715
    sens.R[3,2] = ROcp2_815
    sens.R[3,3] = ROcp2_915
    sens.V[1] = VIcp2_19
    sens.V[2] = VIcp2_29
    sens.V[3] = VIcp2_39
    sens.OM[1] = OMcp2_110
    sens.OM[2] = OMcp2_210
    sens.OM[3] = OMcp2_310
    sens.A[1] = ACcp2_19
    sens.A[2] = ACcp2_29
    sens.A[3] = ACcp2_39
    sens.OMP[1] = OPcp2_110
    sens.OMP[2] = OPcp2_210
    sens.OMP[3] = OPcp2_310

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
    ROcp3_420 = ROcp3_45*C20+ROcp3_76*S20
    ROcp3_520 = ROcp3_55*C20+ROcp3_86*S20
    ROcp3_620 = ROcp3_96*S20+C20*S5
    ROcp3_720 = -ROcp3_45*S20+ROcp3_76*C20
    ROcp3_820 = -ROcp3_55*S20+ROcp3_86*C20
    ROcp3_920 = ROcp3_96*C20-S20*S5
    ROcp3_121 = ROcp3_16*C21-ROcp3_720*S21
    ROcp3_221 = ROcp3_26*C21-ROcp3_820*S21
    ROcp3_321 = ROcp3_36*C21-ROcp3_920*S21
    ROcp3_721 = ROcp3_16*S21+ROcp3_720*C21
    ROcp3_821 = ROcp3_26*S21+ROcp3_820*C21
    ROcp3_921 = ROcp3_36*S21+ROcp3_920*C21
    ROcp3_122 = ROcp3_121*C22+ROcp3_420*S22
    ROcp3_222 = ROcp3_221*C22+ROcp3_520*S22
    ROcp3_322 = ROcp3_321*C22+ROcp3_620*S22
    ROcp3_422 = -ROcp3_121*S22+ROcp3_420*C22
    ROcp3_522 = -ROcp3_221*S22+ROcp3_520*C22
    ROcp3_622 = -ROcp3_321*S22+ROcp3_620*C22
    ROcp3_125 = ROcp3_122*C25-ROcp3_721*S25
    ROcp3_225 = ROcp3_222*C25-ROcp3_821*S25
    ROcp3_325 = ROcp3_322*C25-ROcp3_921*S25
    ROcp3_725 = ROcp3_122*S25+ROcp3_721*C25
    ROcp3_825 = ROcp3_222*S25+ROcp3_821*C25
    ROcp3_925 = ROcp3_322*S25+ROcp3_921*C25
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
    RLcp3_17 = ROcp3_16*s.dpt[1,13]+ROcp3_76*q[17]
    RLcp3_27 = ROcp3_26*s.dpt[1,13]+ROcp3_86*q[17]
    RLcp3_37 = ROcp3_36*s.dpt[1,13]+ROcp3_96*q[17]
    POcp3_17 = RLcp3_17+q[3]
    POcp3_27 = RLcp3_27+q[2]
    POcp3_37 = RLcp3_37+q[1]
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+qd[3]+ROcp3_76*qd[17]
    VIcp3_27 = ORcp3_27+qd[2]+ROcp3_86*qd[17]
    VIcp3_37 = ORcp3_37+qd[1]+ROcp3_96*qd[17]
    ACcp3_17 = qdd[3]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27+ROcp3_76*qdd[17]+(2.0)*qd[17]*(OMcp3_26*ROcp3_96-OMcp3_36*ROcp3_86)
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17+ROcp3_86*qdd[17]+(2.0)*qd[17]*(-OMcp3_16*ROcp3_96+OMcp3_36*ROcp3_76)
    ACcp3_37 = qdd[1]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17+ROcp3_96*qdd[17]+(2.0)*qd[17]*(OMcp3_16*ROcp3_86-OMcp3_26*ROcp3_76)
    RLcp3_18 = ROcp3_16*q[18]
    RLcp3_28 = ROcp3_26*q[18]
    RLcp3_38 = ROcp3_36*q[18]
    POcp3_18 = POcp3_17+RLcp3_18
    POcp3_28 = POcp3_27+RLcp3_28
    POcp3_38 = POcp3_37+RLcp3_38
    ORcp3_18 = OMcp3_26*RLcp3_38-OMcp3_36*RLcp3_28
    ORcp3_28 = -OMcp3_16*RLcp3_38+OMcp3_36*RLcp3_18
    ORcp3_38 = OMcp3_16*RLcp3_28-OMcp3_26*RLcp3_18
    VIcp3_18 = ORcp3_18+VIcp3_17+ROcp3_16*qd[18]
    VIcp3_28 = ORcp3_28+VIcp3_27+ROcp3_26*qd[18]
    VIcp3_38 = ORcp3_38+VIcp3_37+ROcp3_36*qd[18]
    ACcp3_18 = ACcp3_17+OMcp3_26*ORcp3_38-OMcp3_36*ORcp3_28+OPcp3_26*RLcp3_38-OPcp3_36*RLcp3_28+ROcp3_16*qdd[18]+(2.0)*qd[18]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26)
    ACcp3_28 = ACcp3_27-OMcp3_16*ORcp3_38+OMcp3_36*ORcp3_18-OPcp3_16*RLcp3_38+OPcp3_36*RLcp3_18+ROcp3_26*qdd[18]+(2.0)*qd[18]*(-OMcp3_16*ROcp3_36+OMcp3_36*ROcp3_16)
    ACcp3_38 = ACcp3_37+OMcp3_16*ORcp3_28-OMcp3_26*ORcp3_18+OPcp3_16*RLcp3_28-OPcp3_26*RLcp3_18+ROcp3_36*qdd[18]+(2.0)*qd[18]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16)
    RLcp3_19 = ROcp3_45*q[19]
    RLcp3_29 = ROcp3_55*q[19]
    RLcp3_39 = q[19]*S5
    POcp3_19 = POcp3_18+RLcp3_19
    POcp3_29 = POcp3_28+RLcp3_29
    POcp3_39 = POcp3_38+RLcp3_39
    ORcp3_19 = OMcp3_26*RLcp3_39-OMcp3_36*RLcp3_29
    ORcp3_29 = -OMcp3_16*RLcp3_39+OMcp3_36*RLcp3_19
    ORcp3_39 = OMcp3_16*RLcp3_29-OMcp3_26*RLcp3_19
    VIcp3_19 = ORcp3_19+VIcp3_18+ROcp3_45*qd[19]
    VIcp3_29 = ORcp3_29+VIcp3_28+ROcp3_55*qd[19]
    VIcp3_39 = ORcp3_39+VIcp3_38+qd[19]*S5
    ACcp3_19 = ACcp3_18+OMcp3_26*ORcp3_39-OMcp3_36*ORcp3_29+OPcp3_26*RLcp3_39-OPcp3_36*RLcp3_29+ROcp3_45*qdd[19]+(2.0)*qd[19]*(OMcp3_26*S5-OMcp3_36*ROcp3_55)
    ACcp3_29 = ACcp3_28-OMcp3_16*ORcp3_39+OMcp3_36*ORcp3_19-OPcp3_16*RLcp3_39+OPcp3_36*RLcp3_19+ROcp3_55*qdd[19]+(2.0)*qd[19]*(-OMcp3_16*S5+OMcp3_36*ROcp3_45)
    ACcp3_39 = ACcp3_38+OMcp3_16*ORcp3_29-OMcp3_26*ORcp3_19+OPcp3_16*RLcp3_29-OPcp3_26*RLcp3_19+qdd[19]*S5+(2.0)*qd[19]*(OMcp3_16*ROcp3_55-OMcp3_26*ROcp3_45)
    OMcp3_110 = OMcp3_16+ROcp3_16*qd[20]
    OMcp3_210 = OMcp3_26+ROcp3_26*qd[20]
    OMcp3_310 = OMcp3_36+ROcp3_36*qd[20]
    OPcp3_110 = OPcp3_16+ROcp3_16*qdd[20]+qd[20]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26)
    OPcp3_210 = OPcp3_26+ROcp3_26*qdd[20]+qd[20]*(-OMcp3_16*ROcp3_36+OMcp3_36*ROcp3_16)
    OPcp3_310 = OPcp3_36+ROcp3_36*qdd[20]+qd[20]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16)
    OMcp3_111 = OMcp3_110+ROcp3_420*qd[21]
    OMcp3_211 = OMcp3_210+ROcp3_520*qd[21]
    OMcp3_311 = OMcp3_310+ROcp3_620*qd[21]
    OPcp3_111 = OPcp3_110+ROcp3_420*qdd[21]+qd[21]*(OMcp3_210*ROcp3_620-OMcp3_310*ROcp3_520)
    OPcp3_211 = OPcp3_210+ROcp3_520*qdd[21]+qd[21]*(-OMcp3_110*ROcp3_620+OMcp3_310*ROcp3_420)
    OPcp3_311 = OPcp3_310+ROcp3_620*qdd[21]+qd[21]*(OMcp3_110*ROcp3_520-OMcp3_210*ROcp3_420)
    OMcp3_112 = OMcp3_111+ROcp3_721*qd[22]
    OMcp3_212 = OMcp3_211+ROcp3_821*qd[22]
    OMcp3_312 = OMcp3_311+ROcp3_921*qd[22]
    OPcp3_112 = OPcp3_111+ROcp3_721*qdd[22]+qd[22]*(OMcp3_211*ROcp3_921-OMcp3_311*ROcp3_821)
    OPcp3_212 = OPcp3_211+ROcp3_821*qdd[22]+qd[22]*(-OMcp3_111*ROcp3_921+OMcp3_311*ROcp3_721)
    OPcp3_312 = OPcp3_311+ROcp3_921*qdd[22]+qd[22]*(OMcp3_111*ROcp3_821-OMcp3_211*ROcp3_721)
    RLcp3_113 = ROcp3_422*s.dpt[2,34]
    RLcp3_213 = ROcp3_522*s.dpt[2,34]
    RLcp3_313 = ROcp3_622*s.dpt[2,34]
    POcp3_113 = POcp3_19+RLcp3_113
    POcp3_213 = POcp3_29+RLcp3_213
    POcp3_313 = POcp3_39+RLcp3_313
    OMcp3_113 = OMcp3_112+ROcp3_422*qd[25]
    OMcp3_213 = OMcp3_212+ROcp3_522*qd[25]
    OMcp3_313 = OMcp3_312+ROcp3_622*qd[25]
    ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213
    ORcp3_213 = -OMcp3_112*RLcp3_313+OMcp3_312*RLcp3_113
    ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113
    VIcp3_113 = ORcp3_113+VIcp3_19
    VIcp3_213 = ORcp3_213+VIcp3_29
    VIcp3_313 = ORcp3_313+VIcp3_39
    OPcp3_113 = OPcp3_112+ROcp3_422*qdd[25]+qd[25]*(OMcp3_212*ROcp3_622-OMcp3_312*ROcp3_522)
    OPcp3_213 = OPcp3_212+ROcp3_522*qdd[25]+qd[25]*(-OMcp3_112*ROcp3_622+OMcp3_312*ROcp3_422)
    OPcp3_313 = OPcp3_312+ROcp3_622*qdd[25]+qd[25]*(OMcp3_112*ROcp3_522-OMcp3_212*ROcp3_422)
    ACcp3_113 = ACcp3_19+OMcp3_212*ORcp3_313-OMcp3_312*ORcp3_213+OPcp3_212*RLcp3_313-OPcp3_312*RLcp3_213
    ACcp3_213 = ACcp3_29-OMcp3_112*ORcp3_313+OMcp3_312*ORcp3_113-OPcp3_112*RLcp3_313+OPcp3_312*RLcp3_113
    ACcp3_313 = ACcp3_39+OMcp3_112*ORcp3_213-OMcp3_212*ORcp3_113+OPcp3_112*RLcp3_213-OPcp3_212*RLcp3_113
    sens.P[1] = POcp3_113
    sens.P[2] = POcp3_213
    sens.P[3] = POcp3_313
    sens.R[1,1] = ROcp3_125
    sens.R[1,2] = ROcp3_225
    sens.R[1,3] = ROcp3_325
    sens.R[2,1] = ROcp3_422
    sens.R[2,2] = ROcp3_522
    sens.R[2,3] = ROcp3_622
    sens.R[3,1] = ROcp3_725
    sens.R[3,2] = ROcp3_825
    sens.R[3,3] = ROcp3_925
    sens.V[1] = VIcp3_113
    sens.V[2] = VIcp3_213
    sens.V[3] = VIcp3_313
    sens.OM[1] = OMcp3_113
    sens.OM[2] = OMcp3_213
    sens.OM[3] = OMcp3_313
    sens.A[1] = ACcp3_113
    sens.A[2] = ACcp3_213
    sens.A[3] = ACcp3_313
    sens.OMP[1] = OPcp3_113
    sens.OMP[2] = OPcp3_213
    sens.OMP[3] = OPcp3_313

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
    ROcp4_420 = ROcp4_45*C20+ROcp4_76*S20
    ROcp4_520 = ROcp4_55*C20+ROcp4_86*S20
    ROcp4_620 = ROcp4_96*S20+C20*S5
    ROcp4_720 = -ROcp4_45*S20+ROcp4_76*C20
    ROcp4_820 = -ROcp4_55*S20+ROcp4_86*C20
    ROcp4_920 = ROcp4_96*C20-S20*S5
    ROcp4_121 = ROcp4_16*C21-ROcp4_720*S21
    ROcp4_221 = ROcp4_26*C21-ROcp4_820*S21
    ROcp4_321 = ROcp4_36*C21-ROcp4_920*S21
    ROcp4_721 = ROcp4_16*S21+ROcp4_720*C21
    ROcp4_821 = ROcp4_26*S21+ROcp4_820*C21
    ROcp4_921 = ROcp4_36*S21+ROcp4_920*C21
    ROcp4_122 = ROcp4_121*C22+ROcp4_420*S22
    ROcp4_222 = ROcp4_221*C22+ROcp4_520*S22
    ROcp4_322 = ROcp4_321*C22+ROcp4_620*S22
    ROcp4_422 = -ROcp4_121*S22+ROcp4_420*C22
    ROcp4_522 = -ROcp4_221*S22+ROcp4_520*C22
    ROcp4_622 = -ROcp4_321*S22+ROcp4_620*C22
    ROcp4_126 = ROcp4_122*C26-ROcp4_721*S26
    ROcp4_226 = ROcp4_222*C26-ROcp4_821*S26
    ROcp4_326 = ROcp4_322*C26-ROcp4_921*S26
    ROcp4_726 = ROcp4_122*S26+ROcp4_721*C26
    ROcp4_826 = ROcp4_222*S26+ROcp4_821*C26
    ROcp4_926 = ROcp4_322*S26+ROcp4_921*C26
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
    RLcp4_17 = ROcp4_16*s.dpt[1,13]+ROcp4_76*q[17]
    RLcp4_27 = ROcp4_26*s.dpt[1,13]+ROcp4_86*q[17]
    RLcp4_37 = ROcp4_36*s.dpt[1,13]+ROcp4_96*q[17]
    POcp4_17 = RLcp4_17+q[3]
    POcp4_27 = RLcp4_27+q[2]
    POcp4_37 = RLcp4_37+q[1]
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = ORcp4_17+qd[3]+ROcp4_76*qd[17]
    VIcp4_27 = ORcp4_27+qd[2]+ROcp4_86*qd[17]
    VIcp4_37 = ORcp4_37+qd[1]+ROcp4_96*qd[17]
    ACcp4_17 = qdd[3]+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27+ROcp4_76*qdd[17]+(2.0)*qd[17]*(OMcp4_26*ROcp4_96-OMcp4_36*ROcp4_86)
    ACcp4_27 = qdd[2]-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17+ROcp4_86*qdd[17]+(2.0)*qd[17]*(-OMcp4_16*ROcp4_96+OMcp4_36*ROcp4_76)
    ACcp4_37 = qdd[1]+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17+ROcp4_96*qdd[17]+(2.0)*qd[17]*(OMcp4_16*ROcp4_86-OMcp4_26*ROcp4_76)
    RLcp4_18 = ROcp4_16*q[18]
    RLcp4_28 = ROcp4_26*q[18]
    RLcp4_38 = ROcp4_36*q[18]
    POcp4_18 = POcp4_17+RLcp4_18
    POcp4_28 = POcp4_27+RLcp4_28
    POcp4_38 = POcp4_37+RLcp4_38
    ORcp4_18 = OMcp4_26*RLcp4_38-OMcp4_36*RLcp4_28
    ORcp4_28 = -OMcp4_16*RLcp4_38+OMcp4_36*RLcp4_18
    ORcp4_38 = OMcp4_16*RLcp4_28-OMcp4_26*RLcp4_18
    VIcp4_18 = ORcp4_18+VIcp4_17+ROcp4_16*qd[18]
    VIcp4_28 = ORcp4_28+VIcp4_27+ROcp4_26*qd[18]
    VIcp4_38 = ORcp4_38+VIcp4_37+ROcp4_36*qd[18]
    ACcp4_18 = ACcp4_17+OMcp4_26*ORcp4_38-OMcp4_36*ORcp4_28+OPcp4_26*RLcp4_38-OPcp4_36*RLcp4_28+ROcp4_16*qdd[18]+(2.0)*qd[18]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)
    ACcp4_28 = ACcp4_27-OMcp4_16*ORcp4_38+OMcp4_36*ORcp4_18-OPcp4_16*RLcp4_38+OPcp4_36*RLcp4_18+ROcp4_26*qdd[18]+(2.0)*qd[18]*(-OMcp4_16*ROcp4_36+OMcp4_36*ROcp4_16)
    ACcp4_38 = ACcp4_37+OMcp4_16*ORcp4_28-OMcp4_26*ORcp4_18+OPcp4_16*RLcp4_28-OPcp4_26*RLcp4_18+ROcp4_36*qdd[18]+(2.0)*qd[18]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)
    RLcp4_19 = ROcp4_45*q[19]
    RLcp4_29 = ROcp4_55*q[19]
    RLcp4_39 = q[19]*S5
    POcp4_19 = POcp4_18+RLcp4_19
    POcp4_29 = POcp4_28+RLcp4_29
    POcp4_39 = POcp4_38+RLcp4_39
    ORcp4_19 = OMcp4_26*RLcp4_39-OMcp4_36*RLcp4_29
    ORcp4_29 = -OMcp4_16*RLcp4_39+OMcp4_36*RLcp4_19
    ORcp4_39 = OMcp4_16*RLcp4_29-OMcp4_26*RLcp4_19
    VIcp4_19 = ORcp4_19+VIcp4_18+ROcp4_45*qd[19]
    VIcp4_29 = ORcp4_29+VIcp4_28+ROcp4_55*qd[19]
    VIcp4_39 = ORcp4_39+VIcp4_38+qd[19]*S5
    ACcp4_19 = ACcp4_18+OMcp4_26*ORcp4_39-OMcp4_36*ORcp4_29+OPcp4_26*RLcp4_39-OPcp4_36*RLcp4_29+ROcp4_45*qdd[19]+(2.0)*qd[19]*(OMcp4_26*S5-OMcp4_36*ROcp4_55)
    ACcp4_29 = ACcp4_28-OMcp4_16*ORcp4_39+OMcp4_36*ORcp4_19-OPcp4_16*RLcp4_39+OPcp4_36*RLcp4_19+ROcp4_55*qdd[19]+(2.0)*qd[19]*(-OMcp4_16*S5+OMcp4_36*ROcp4_45)
    ACcp4_39 = ACcp4_38+OMcp4_16*ORcp4_29-OMcp4_26*ORcp4_19+OPcp4_16*RLcp4_29-OPcp4_26*RLcp4_19+qdd[19]*S5+(2.0)*qd[19]*(OMcp4_16*ROcp4_55-OMcp4_26*ROcp4_45)
    OMcp4_110 = OMcp4_16+ROcp4_16*qd[20]
    OMcp4_210 = OMcp4_26+ROcp4_26*qd[20]
    OMcp4_310 = OMcp4_36+ROcp4_36*qd[20]
    OPcp4_110 = OPcp4_16+ROcp4_16*qdd[20]+qd[20]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)
    OPcp4_210 = OPcp4_26+ROcp4_26*qdd[20]+qd[20]*(-OMcp4_16*ROcp4_36+OMcp4_36*ROcp4_16)
    OPcp4_310 = OPcp4_36+ROcp4_36*qdd[20]+qd[20]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)
    OMcp4_111 = OMcp4_110+ROcp4_420*qd[21]
    OMcp4_211 = OMcp4_210+ROcp4_520*qd[21]
    OMcp4_311 = OMcp4_310+ROcp4_620*qd[21]
    OPcp4_111 = OPcp4_110+ROcp4_420*qdd[21]+qd[21]*(OMcp4_210*ROcp4_620-OMcp4_310*ROcp4_520)
    OPcp4_211 = OPcp4_210+ROcp4_520*qdd[21]+qd[21]*(-OMcp4_110*ROcp4_620+OMcp4_310*ROcp4_420)
    OPcp4_311 = OPcp4_310+ROcp4_620*qdd[21]+qd[21]*(OMcp4_110*ROcp4_520-OMcp4_210*ROcp4_420)
    OMcp4_112 = OMcp4_111+ROcp4_721*qd[22]
    OMcp4_212 = OMcp4_211+ROcp4_821*qd[22]
    OMcp4_312 = OMcp4_311+ROcp4_921*qd[22]
    OPcp4_112 = OPcp4_111+ROcp4_721*qdd[22]+qd[22]*(OMcp4_211*ROcp4_921-OMcp4_311*ROcp4_821)
    OPcp4_212 = OPcp4_211+ROcp4_821*qdd[22]+qd[22]*(-OMcp4_111*ROcp4_921+OMcp4_311*ROcp4_721)
    OPcp4_312 = OPcp4_311+ROcp4_921*qdd[22]+qd[22]*(OMcp4_111*ROcp4_821-OMcp4_211*ROcp4_721)
    RLcp4_113 = ROcp4_422*s.dpt[2,35]
    RLcp4_213 = ROcp4_522*s.dpt[2,35]
    RLcp4_313 = ROcp4_622*s.dpt[2,35]
    POcp4_113 = POcp4_19+RLcp4_113
    POcp4_213 = POcp4_29+RLcp4_213
    POcp4_313 = POcp4_39+RLcp4_313
    OMcp4_113 = OMcp4_112+ROcp4_422*qd[26]
    OMcp4_213 = OMcp4_212+ROcp4_522*qd[26]
    OMcp4_313 = OMcp4_312+ROcp4_622*qd[26]
    ORcp4_113 = OMcp4_212*RLcp4_313-OMcp4_312*RLcp4_213
    ORcp4_213 = -OMcp4_112*RLcp4_313+OMcp4_312*RLcp4_113
    ORcp4_313 = OMcp4_112*RLcp4_213-OMcp4_212*RLcp4_113
    VIcp4_113 = ORcp4_113+VIcp4_19
    VIcp4_213 = ORcp4_213+VIcp4_29
    VIcp4_313 = ORcp4_313+VIcp4_39
    OPcp4_113 = OPcp4_112+ROcp4_422*qdd[26]+qd[26]*(OMcp4_212*ROcp4_622-OMcp4_312*ROcp4_522)
    OPcp4_213 = OPcp4_212+ROcp4_522*qdd[26]+qd[26]*(-OMcp4_112*ROcp4_622+OMcp4_312*ROcp4_422)
    OPcp4_313 = OPcp4_312+ROcp4_622*qdd[26]+qd[26]*(OMcp4_112*ROcp4_522-OMcp4_212*ROcp4_422)
    ACcp4_113 = ACcp4_19+OMcp4_212*ORcp4_313-OMcp4_312*ORcp4_213+OPcp4_212*RLcp4_313-OPcp4_312*RLcp4_213
    ACcp4_213 = ACcp4_29-OMcp4_112*ORcp4_313+OMcp4_312*ORcp4_113-OPcp4_112*RLcp4_313+OPcp4_312*RLcp4_113
    ACcp4_313 = ACcp4_39+OMcp4_112*ORcp4_213-OMcp4_212*ORcp4_113+OPcp4_112*RLcp4_213-OPcp4_212*RLcp4_113
    sens.P[1] = POcp4_113
    sens.P[2] = POcp4_213
    sens.P[3] = POcp4_313
    sens.R[1,1] = ROcp4_126
    sens.R[1,2] = ROcp4_226
    sens.R[1,3] = ROcp4_326
    sens.R[2,1] = ROcp4_422
    sens.R[2,2] = ROcp4_522
    sens.R[2,3] = ROcp4_622
    sens.R[3,1] = ROcp4_726
    sens.R[3,2] = ROcp4_826
    sens.R[3,3] = ROcp4_926
    sens.V[1] = VIcp4_113
    sens.V[2] = VIcp4_213
    sens.V[3] = VIcp4_313
    sens.OM[1] = OMcp4_113
    sens.OM[2] = OMcp4_213
    sens.OM[3] = OMcp4_313
    sens.A[1] = ACcp4_113
    sens.A[2] = ACcp4_213
    sens.A[3] = ACcp4_313
    sens.OMP[1] = OPcp4_113
    sens.OMP[2] = OPcp4_213
    sens.OMP[3] = OPcp4_313

 


# Number of continuation lines = 0


