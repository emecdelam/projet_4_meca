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
  S8 = sin(q[8])
  C8 = cos(q[8])
  S9 = sin(q[9])
  C9 = cos(q[9])
  S10 = sin(q[10])
  C10 = cos(q[10])
 
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
    ROcp1_18 = ROcp1_16*C8-S5*S8
    ROcp1_28 = ROcp1_26*C8-ROcp1_85*S8
    ROcp1_38 = ROcp1_36*C8-ROcp1_95*S8
    ROcp1_78 = ROcp1_16*S8+S5*C8
    ROcp1_88 = ROcp1_26*S8+ROcp1_85*C8
    ROcp1_98 = ROcp1_36*S8+ROcp1_95*C8
    ROcp1_19 = ROcp1_18*C9-ROcp1_78*S9
    ROcp1_29 = ROcp1_28*C9-ROcp1_88*S9
    ROcp1_39 = ROcp1_38*C9-ROcp1_98*S9
    ROcp1_79 = ROcp1_18*S9+ROcp1_78*C9
    ROcp1_89 = ROcp1_28*S9+ROcp1_88*C9
    ROcp1_99 = ROcp1_38*S9+ROcp1_98*C9
    ROcp1_110 = ROcp1_19*C10-ROcp1_79*S10
    ROcp1_210 = ROcp1_29*C10-ROcp1_89*S10
    ROcp1_310 = ROcp1_39*C10-ROcp1_99*S10
    ROcp1_710 = ROcp1_19*S10+ROcp1_79*C10
    ROcp1_810 = ROcp1_29*S10+ROcp1_89*C10
    ROcp1_910 = ROcp1_39*S10+ROcp1_99*C10
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
    RLcp1_17 = ROcp1_16*s.dpt[1,3]
    RLcp1_27 = ROcp1_26*s.dpt[1,3]
    RLcp1_37 = ROcp1_36*s.dpt[1,3]
    POcp1_17 = RLcp1_17+q[1]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[3]
    OMcp1_17 = OMcp1_16+ROcp1_46*qd[8]
    OMcp1_27 = OMcp1_26+ROcp1_56*qd[8]
    OMcp1_37 = OMcp1_36+ROcp1_66*qd[8]
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[1]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[3]
    OPcp1_17 = OPcp1_16+ROcp1_46*qdd[8]+qd[8]*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56)
    OPcp1_27 = OPcp1_26+ROcp1_56*qdd[8]+qd[8]*(-OMcp1_16*ROcp1_66+OMcp1_36*ROcp1_46)
    OPcp1_37 = OPcp1_36+ROcp1_66*qdd[8]+qd[8]*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46)
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    RLcp1_18 = ROcp1_18*s.dpt[1,5]
    RLcp1_28 = ROcp1_28*s.dpt[1,5]
    RLcp1_38 = ROcp1_38*s.dpt[1,5]
    POcp1_18 = POcp1_17+RLcp1_18
    POcp1_28 = POcp1_27+RLcp1_28
    POcp1_38 = POcp1_37+RLcp1_38
    OMcp1_18 = OMcp1_17+ROcp1_46*qd[9]
    OMcp1_28 = OMcp1_27+ROcp1_56*qd[9]
    OMcp1_38 = OMcp1_37+ROcp1_66*qd[9]
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28
    ORcp1_28 = -OMcp1_17*RLcp1_38+OMcp1_37*RLcp1_18
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18
    VIcp1_18 = ORcp1_18+VIcp1_17
    VIcp1_28 = ORcp1_28+VIcp1_27
    VIcp1_38 = ORcp1_38+VIcp1_37
    OPcp1_18 = OPcp1_17+ROcp1_46*qdd[9]+qd[9]*(OMcp1_27*ROcp1_66-OMcp1_37*ROcp1_56)
    OPcp1_28 = OPcp1_27+ROcp1_56*qdd[9]+qd[9]*(-OMcp1_17*ROcp1_66+OMcp1_37*ROcp1_46)
    OPcp1_38 = OPcp1_37+ROcp1_66*qdd[9]+qd[9]*(OMcp1_17*ROcp1_56-OMcp1_27*ROcp1_46)
    ACcp1_18 = ACcp1_17+OMcp1_27*ORcp1_38-OMcp1_37*ORcp1_28+OPcp1_27*RLcp1_38-OPcp1_37*RLcp1_28
    ACcp1_28 = ACcp1_27-OMcp1_17*ORcp1_38+OMcp1_37*ORcp1_18-OPcp1_17*RLcp1_38+OPcp1_37*RLcp1_18
    ACcp1_38 = ACcp1_37+OMcp1_17*ORcp1_28-OMcp1_27*ORcp1_18+OPcp1_17*RLcp1_28-OPcp1_27*RLcp1_18
    RLcp1_19 = ROcp1_19*s.dpt[1,8]+ROcp1_79*s.dpt[3,8]
    RLcp1_29 = ROcp1_29*s.dpt[1,8]+ROcp1_89*s.dpt[3,8]
    RLcp1_39 = ROcp1_39*s.dpt[1,8]+ROcp1_99*s.dpt[3,8]
    POcp1_19 = POcp1_18+RLcp1_19
    POcp1_29 = POcp1_28+RLcp1_29
    POcp1_39 = POcp1_38+RLcp1_39
    OMcp1_19 = OMcp1_18+ROcp1_46*qd[10]
    OMcp1_29 = OMcp1_28+ROcp1_56*qd[10]
    OMcp1_39 = OMcp1_38+ROcp1_66*qd[10]
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29
    ORcp1_29 = -OMcp1_18*RLcp1_39+OMcp1_38*RLcp1_19
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_18
    VIcp1_29 = ORcp1_29+VIcp1_28
    VIcp1_39 = ORcp1_39+VIcp1_38
    OPcp1_19 = OPcp1_18+ROcp1_46*qdd[10]+qd[10]*(OMcp1_28*ROcp1_66-OMcp1_38*ROcp1_56)
    OPcp1_29 = OPcp1_28+ROcp1_56*qdd[10]+qd[10]*(-OMcp1_18*ROcp1_66+OMcp1_38*ROcp1_46)
    OPcp1_39 = OPcp1_38+ROcp1_66*qdd[10]+qd[10]*(OMcp1_18*ROcp1_56-OMcp1_28*ROcp1_46)
    ACcp1_19 = ACcp1_18+OMcp1_28*ORcp1_39-OMcp1_38*ORcp1_29+OPcp1_28*RLcp1_39-OPcp1_38*RLcp1_29
    ACcp1_29 = ACcp1_28-OMcp1_18*ORcp1_39+OMcp1_38*ORcp1_19-OPcp1_18*RLcp1_39+OPcp1_38*RLcp1_19
    ACcp1_39 = ACcp1_38+OMcp1_18*ORcp1_29-OMcp1_28*ORcp1_19+OPcp1_18*RLcp1_29-OPcp1_28*RLcp1_19
    sens.P[1] = POcp1_19
    sens.P[2] = POcp1_29
    sens.P[3] = POcp1_39
    sens.R[1,1] = ROcp1_110
    sens.R[1,2] = ROcp1_210
    sens.R[1,3] = ROcp1_310
    sens.R[2,1] = ROcp1_46
    sens.R[2,2] = ROcp1_56
    sens.R[2,3] = ROcp1_66
    sens.R[3,1] = ROcp1_710
    sens.R[3,2] = ROcp1_810
    sens.R[3,3] = ROcp1_910
    sens.V[1] = VIcp1_19
    sens.V[2] = VIcp1_29
    sens.V[3] = VIcp1_39
    sens.OM[1] = OMcp1_19
    sens.OM[2] = OMcp1_29
    sens.OM[3] = OMcp1_39
    sens.A[1] = ACcp1_19
    sens.A[2] = ACcp1_29
    sens.A[3] = ACcp1_39
    sens.OMP[1] = OPcp1_19
    sens.OMP[2] = OPcp1_29
    sens.OMP[3] = OPcp1_39

 


# Number of continuation lines = 0


