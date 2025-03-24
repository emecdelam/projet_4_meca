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
#	==> Generation Date: Mon Mar 24 20:37:20 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 28
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

    S7 = sin(q[7])
    C7 = cos(q[7])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,17]*C7-s.dpt[3,17]*S7
    RLlnk2_32 = s.dpt[2,17]*S7+s.dpt[3,17]*C7
    POlnk2_22 = RLlnk2_22+s.dpt[2,1]
    ORlnk2_22 = -qd[7]*RLlnk2_32
    ORlnk2_32 = qd[7]*RLlnk2_22
    Plnk11 = s.dpt[1,1]-s.dpt[1,3]
    Plnk21 = POlnk2_22-s.dpt[2,3]
    Plnk31 = RLlnk2_32-s.dpt[3,3]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk4_22 = s.dpt[2,24]*C13-s.dpt[3,24]*S13
    RLlnk4_32 = s.dpt[2,24]*S13+s.dpt[3,24]*C13
    POlnk4_22 = RLlnk4_22+s.dpt[2,4]
    ORlnk4_22 = -qd[13]*RLlnk4_32
    ORlnk4_32 = qd[13]*RLlnk4_22
    Plnk12 = s.dpt[1,4]-s.dpt[1,6]
    Plnk22 = POlnk4_22-s.dpt[2,6]
    Plnk32 = RLlnk4_32-s.dpt[3,6]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    ROlnk6_223 = S22*S23
    ROlnk6_323 = -C22*S23
    ROlnk6_823 = -S22*C23
    ROlnk6_923 = C22*C23
    ROlnk6_124 = C23*C24
    ROlnk6_224 = ROlnk6_223*C24+C22*S24
    ROlnk6_324 = ROlnk6_323*C24+S22*S24
    ROlnk6_424 = -C23*S24
    ROlnk6_524 = -ROlnk6_223*S24+C22*C24
    ROlnk6_624 = -ROlnk6_323*S24+S22*C24
    POlnk6_12 = q[20]+s.dpt[1,13]
    OMlnk6_25 = qd[23]*C22
    OMlnk6_35 = qd[23]*S22
    OMlnk6_16 = qd[22]+qd[24]*S23
    OMlnk6_26 = OMlnk6_25+qd[24]*ROlnk6_823
    OMlnk6_36 = OMlnk6_35+qd[24]*ROlnk6_923
    RLlnk6_17 = ROlnk6_124*s.dpt[1,30]+ROlnk6_424*s.dpt[2,30]
    RLlnk6_27 = ROlnk6_224*s.dpt[1,30]+ROlnk6_524*s.dpt[2,30]
    RLlnk6_37 = ROlnk6_324*s.dpt[1,30]+ROlnk6_624*s.dpt[2,30]
    POlnk6_17 = POlnk6_12+RLlnk6_17
    POlnk6_27 = q[21]+RLlnk6_27
    POlnk6_37 = q[19]+RLlnk6_37
    ORlnk6_17 = OMlnk6_26*RLlnk6_37-OMlnk6_36*RLlnk6_27
    ORlnk6_27 = -OMlnk6_16*RLlnk6_37+OMlnk6_36*RLlnk6_17
    ORlnk6_37 = OMlnk6_16*RLlnk6_27-OMlnk6_26*RLlnk6_17
    VIlnk6_17 = qd[20]+ORlnk6_17
    VIlnk6_27 = qd[21]+ORlnk6_27
    VIlnk6_37 = qd[19]+ORlnk6_37
    Plnk13 = POlnk6_17-s.dpt[1,7]
    Plnk23 = POlnk6_27-s.dpt[2,7]
    Plnk33 = POlnk6_37-s.dpt[3,7]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = VIlnk6_17*e13+VIlnk6_27*e23+VIlnk6_37*e33
    ROlnk8_223 = S22*S23
    ROlnk8_323 = -C22*S23
    ROlnk8_823 = -S22*C23
    ROlnk8_923 = C22*C23
    ROlnk8_124 = C23*C24
    ROlnk8_224 = ROlnk8_223*C24+C22*S24
    ROlnk8_324 = ROlnk8_323*C24+S22*S24
    ROlnk8_424 = -C23*S24
    ROlnk8_524 = -ROlnk8_223*S24+C22*C24
    ROlnk8_624 = -ROlnk8_323*S24+S22*C24
    POlnk8_12 = q[20]+s.dpt[1,13]
    OMlnk8_25 = qd[23]*C22
    OMlnk8_35 = qd[23]*S22
    OMlnk8_16 = qd[22]+qd[24]*S23
    OMlnk8_26 = OMlnk8_25+qd[24]*ROlnk8_823
    OMlnk8_36 = OMlnk8_35+qd[24]*ROlnk8_923
    RLlnk8_17 = ROlnk8_124*s.dpt[1,31]+ROlnk8_424*s.dpt[2,31]
    RLlnk8_27 = ROlnk8_224*s.dpt[1,31]+ROlnk8_524*s.dpt[2,31]
    RLlnk8_37 = ROlnk8_324*s.dpt[1,31]+ROlnk8_624*s.dpt[2,31]
    POlnk8_17 = POlnk8_12+RLlnk8_17
    POlnk8_27 = q[21]+RLlnk8_27
    POlnk8_37 = q[19]+RLlnk8_37
    ORlnk8_17 = OMlnk8_26*RLlnk8_37-OMlnk8_36*RLlnk8_27
    ORlnk8_27 = -OMlnk8_16*RLlnk8_37+OMlnk8_36*RLlnk8_17
    ORlnk8_37 = OMlnk8_16*RLlnk8_27-OMlnk8_26*RLlnk8_17
    VIlnk8_17 = qd[20]+ORlnk8_17
    VIlnk8_27 = qd[21]+ORlnk8_27
    VIlnk8_37 = qd[19]+ORlnk8_37
    Plnk14 = POlnk8_17-s.dpt[1,8]
    Plnk24 = POlnk8_27-s.dpt[2,8]
    Plnk34 = POlnk8_37-s.dpt[3,8]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = VIlnk8_17*e14+VIlnk8_27*e24+VIlnk8_37*e34
    ROlnk10_223 = S22*S23
    ROlnk10_323 = -C22*S23
    ROlnk10_823 = -S22*C23
    ROlnk10_923 = C22*C23
    ROlnk10_124 = C23*C24
    ROlnk10_224 = ROlnk10_223*C24+C22*S24
    ROlnk10_324 = ROlnk10_323*C24+S22*S24
    ROlnk10_424 = -C23*S24
    ROlnk10_524 = -ROlnk10_223*S24+C22*C24
    ROlnk10_624 = -ROlnk10_323*S24+S22*C24
    POlnk10_12 = q[20]+s.dpt[1,13]
    OMlnk10_25 = qd[23]*C22
    OMlnk10_35 = qd[23]*S22
    OMlnk10_16 = qd[22]+qd[24]*S23
    OMlnk10_26 = OMlnk10_25+qd[24]*ROlnk10_823
    OMlnk10_36 = OMlnk10_35+qd[24]*ROlnk10_923
    RLlnk10_17 = ROlnk10_124*s.dpt[1,33]+ROlnk10_424*s.dpt[2,33]
    RLlnk10_27 = ROlnk10_224*s.dpt[1,33]+ROlnk10_524*s.dpt[2,33]
    RLlnk10_37 = ROlnk10_324*s.dpt[1,33]+ROlnk10_624*s.dpt[2,33]
    POlnk10_17 = POlnk10_12+RLlnk10_17
    POlnk10_27 = q[21]+RLlnk10_27
    POlnk10_37 = q[19]+RLlnk10_37
    ORlnk10_17 = OMlnk10_26*RLlnk10_37-OMlnk10_36*RLlnk10_27
    ORlnk10_27 = -OMlnk10_16*RLlnk10_37+OMlnk10_36*RLlnk10_17
    ORlnk10_37 = OMlnk10_16*RLlnk10_27-OMlnk10_26*RLlnk10_17
    VIlnk10_17 = qd[20]+ORlnk10_17
    VIlnk10_27 = qd[21]+ORlnk10_27
    VIlnk10_37 = qd[19]+ORlnk10_37
    Plnk15 = POlnk10_17-s.dpt[1,11]
    Plnk25 = POlnk10_27-s.dpt[2,11]
    Plnk35 = POlnk10_37-s.dpt[3,11]
    PPlnk5 = Plnk15*Plnk15+Plnk25*Plnk25+Plnk35*Plnk35
    Z5 = sqrt(PPlnk5)
    e15 = Plnk15/Z5
    e25 = Plnk25/Z5
    e35 = Plnk35/Z5
    Zd5 = VIlnk10_17*e15+VIlnk10_27*e25+VIlnk10_37*e35
    ROlnk12_223 = S22*S23
    ROlnk12_323 = -C22*S23
    ROlnk12_823 = -S22*C23
    ROlnk12_923 = C22*C23
    ROlnk12_124 = C23*C24
    ROlnk12_224 = ROlnk12_223*C24+C22*S24
    ROlnk12_324 = ROlnk12_323*C24+S22*S24
    ROlnk12_424 = -C23*S24
    ROlnk12_524 = -ROlnk12_223*S24+C22*C24
    ROlnk12_624 = -ROlnk12_323*S24+S22*C24
    POlnk12_12 = q[20]+s.dpt[1,13]
    OMlnk12_25 = qd[23]*C22
    OMlnk12_35 = qd[23]*S22
    OMlnk12_16 = qd[22]+qd[24]*S23
    OMlnk12_26 = OMlnk12_25+qd[24]*ROlnk12_823
    OMlnk12_36 = OMlnk12_35+qd[24]*ROlnk12_923
    RLlnk12_17 = ROlnk12_124*s.dpt[1,34]+ROlnk12_424*s.dpt[2,34]
    RLlnk12_27 = ROlnk12_224*s.dpt[1,34]+ROlnk12_524*s.dpt[2,34]
    RLlnk12_37 = ROlnk12_324*s.dpt[1,34]+ROlnk12_624*s.dpt[2,34]
    POlnk12_17 = POlnk12_12+RLlnk12_17
    POlnk12_27 = q[21]+RLlnk12_27
    POlnk12_37 = q[19]+RLlnk12_37
    ORlnk12_17 = OMlnk12_26*RLlnk12_37-OMlnk12_36*RLlnk12_27
    ORlnk12_27 = -OMlnk12_16*RLlnk12_37+OMlnk12_36*RLlnk12_17
    ORlnk12_37 = OMlnk12_16*RLlnk12_27-OMlnk12_26*RLlnk12_17
    VIlnk12_17 = qd[20]+ORlnk12_17
    VIlnk12_27 = qd[21]+ORlnk12_27
    VIlnk12_37 = qd[19]+ORlnk12_37
    Plnk16 = POlnk12_17-s.dpt[1,12]
    Plnk26 = POlnk12_27-s.dpt[2,12]
    Plnk36 = POlnk12_37-s.dpt[3,12]
    PPlnk6 = Plnk16*Plnk16+Plnk26*Plnk26+Plnk36*Plnk36
    Z6 = sqrt(PPlnk6)
    e16 = Plnk16/Z6
    e26 = Plnk26/Z6
    e36 = Plnk36/Z6
    Zd6 = VIlnk12_17*e16+VIlnk12_27*e26+VIlnk12_37*e36
    ROlnk14_223 = S22*S23
    ROlnk14_323 = -C22*S23
    ROlnk14_823 = -S22*C23
    ROlnk14_923 = C22*C23
    ROlnk14_124 = C23*C24
    ROlnk14_224 = ROlnk14_223*C24+C22*S24
    ROlnk14_324 = ROlnk14_323*C24+S22*S24
    ROlnk14_424 = -C23*S24
    ROlnk14_524 = -ROlnk14_223*S24+C22*C24
    ROlnk14_624 = -ROlnk14_323*S24+S22*C24
    POlnk14_12 = q[20]+s.dpt[1,13]
    OMlnk14_25 = qd[23]*C22
    OMlnk14_35 = qd[23]*S22
    OMlnk14_16 = qd[22]+qd[24]*S23
    OMlnk14_26 = OMlnk14_25+qd[24]*ROlnk14_823
    OMlnk14_36 = OMlnk14_35+qd[24]*ROlnk14_923
    RLlnk14_17 = ROlnk14_424*s.dpt[2,39]
    RLlnk14_27 = ROlnk14_524*s.dpt[2,39]
    RLlnk14_37 = ROlnk14_624*s.dpt[2,39]
    POlnk14_17 = POlnk14_12+RLlnk14_17
    POlnk14_27 = q[21]+RLlnk14_27
    POlnk14_37 = q[19]+RLlnk14_37
    ORlnk14_17 = OMlnk14_26*RLlnk14_37-OMlnk14_36*RLlnk14_27
    ORlnk14_27 = -OMlnk14_16*RLlnk14_37+OMlnk14_36*RLlnk14_17
    ORlnk14_37 = OMlnk14_16*RLlnk14_27-OMlnk14_26*RLlnk14_17
    VIlnk14_17 = qd[20]+ORlnk14_17
    VIlnk14_27 = qd[21]+ORlnk14_27
    VIlnk14_37 = qd[19]+ORlnk14_37
    Plnk17 = POlnk14_17-s.dpt[1,14]
    Plnk27 = POlnk14_27-s.dpt[2,14]
    PPlnk7 = POlnk14_37*POlnk14_37+Plnk17*Plnk17+Plnk27*Plnk27
    Z7 = sqrt(PPlnk7)
    e17 = Plnk17/Z7
    e27 = Plnk27/Z7
    e37 = POlnk14_37/Z7
    Zd7 = VIlnk14_17*e17+VIlnk14_27*e27+VIlnk14_37*e37
    ROlnk16_223 = S22*S23
    ROlnk16_323 = -C22*S23
    ROlnk16_823 = -S22*C23
    ROlnk16_923 = C22*C23
    ROlnk16_124 = C23*C24
    ROlnk16_224 = ROlnk16_223*C24+C22*S24
    ROlnk16_324 = ROlnk16_323*C24+S22*S24
    ROlnk16_424 = -C23*S24
    ROlnk16_524 = -ROlnk16_223*S24+C22*C24
    ROlnk16_624 = -ROlnk16_323*S24+S22*C24
    POlnk16_12 = q[20]+s.dpt[1,13]
    OMlnk16_25 = qd[23]*C22
    OMlnk16_35 = qd[23]*S22
    OMlnk16_16 = qd[22]+qd[24]*S23
    OMlnk16_26 = OMlnk16_25+qd[24]*ROlnk16_823
    OMlnk16_36 = OMlnk16_35+qd[24]*ROlnk16_923
    RLlnk16_17 = ROlnk16_424*s.dpt[2,38]
    RLlnk16_27 = ROlnk16_524*s.dpt[2,38]
    RLlnk16_37 = ROlnk16_624*s.dpt[2,38]
    POlnk16_17 = POlnk16_12+RLlnk16_17
    POlnk16_27 = q[21]+RLlnk16_27
    POlnk16_37 = q[19]+RLlnk16_37
    ORlnk16_17 = OMlnk16_26*RLlnk16_37-OMlnk16_36*RLlnk16_27
    ORlnk16_27 = -OMlnk16_16*RLlnk16_37+OMlnk16_36*RLlnk16_17
    ORlnk16_37 = OMlnk16_16*RLlnk16_27-OMlnk16_26*RLlnk16_17
    VIlnk16_17 = qd[20]+ORlnk16_17
    VIlnk16_27 = qd[21]+ORlnk16_27
    VIlnk16_37 = qd[19]+ORlnk16_37
    Plnk18 = POlnk16_17-s.dpt[1,15]
    Plnk28 = POlnk16_27-s.dpt[2,15]
    PPlnk8 = POlnk16_37*POlnk16_37+Plnk18*Plnk18+Plnk28*Plnk28
    Z8 = sqrt(PPlnk8)
    e18 = Plnk18/Z8
    e28 = Plnk28/Z8
    e38 = POlnk16_37/Z8
    Zd8 = VIlnk16_17*e18+VIlnk16_27*e28+VIlnk16_37*e38

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
    Flink5 = s.user_LinkForces(Z5,Zd5,s,tsim,5)
    Flink6 = s.user_LinkForces(Z6,Zd6,s,tsim,6)
    Flink7 = s.user_LinkForces(Z7,Zd7,s,tsim,7)
    Flink8 = s.user_LinkForces(Z8,Zd8,s,tsim,8)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*s.dpt[3,3]+fPlnk31*s.dpt[2,3]
    trqlnk6_1_2 = fPlnk11*s.dpt[3,3]-fPlnk31*s.dpt[1,3]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,3]+fPlnk21*s.dpt[1,3]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C7+e31*S7)
    fSlnk31 = Flink1*(-e21*S7+e31*C7)
    trqlnk7_1_1 = fSlnk21*s.dpt[3,17]-fSlnk31*s.dpt[2,17]
    trqlnk7_1_2 = -fSlnk11*s.dpt[3,17]
    trqlnk7_1_3 = fSlnk11*s.dpt[2,17]
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11+fPlnk12
    frclnk6_2_2 = fPlnk21+fPlnk22
    frclnk6_2_3 = fPlnk31+fPlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*s.dpt[3,6]+fPlnk32*s.dpt[2,6]
    trqlnk6_2_2 = trqlnk6_1_2+fPlnk12*s.dpt[3,6]-fPlnk32*s.dpt[1,6]
    trqlnk6_2_3 = trqlnk6_1_3-fPlnk12*s.dpt[2,6]+fPlnk22*s.dpt[1,6]
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*(e22*C13+e32*S13)
    fSlnk32 = Flink2*(-e22*S13+e32*C13)
    trqlnk13_2_1 = fSlnk22*s.dpt[3,24]-fSlnk32*s.dpt[2,24]
    trqlnk13_2_2 = -fSlnk12*s.dpt[3,24]
    trqlnk13_2_3 = fSlnk12*s.dpt[2,24]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_1 = fPlnk13+frclnk6_2_1
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*s.dpt[3,7]+fPlnk33*s.dpt[2,7]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*s.dpt[3,7]-fPlnk33*s.dpt[1,7]
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,7]+fPlnk23*s.dpt[1,7]
    fSlnk13 = Flink3*(ROlnk6_124*e13+ROlnk6_224*e23+ROlnk6_324*e33)
    fSlnk23 = Flink3*(ROlnk6_424*e13+ROlnk6_524*e23+ROlnk6_624*e33)
    fSlnk33 = Flink3*(ROlnk6_823*e23+ROlnk6_923*e33+e13*S23)
    trqlnk24_3_1 = -fSlnk33*s.dpt[2,30]
    trqlnk24_3_2 = fSlnk33*s.dpt[1,30]
    trqlnk24_3_3 = fSlnk13*s.dpt[2,30]-fSlnk23*s.dpt[1,30]
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk14+frclnk6_3_1
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*s.dpt[3,8]+fPlnk34*s.dpt[2,8]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*s.dpt[3,8]-fPlnk34*s.dpt[1,8]
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,8]+fPlnk24*s.dpt[1,8]
    fSlnk14 = Flink4*(ROlnk8_124*e14+ROlnk8_224*e24+ROlnk8_324*e34)
    fSlnk24 = Flink4*(ROlnk8_424*e14+ROlnk8_524*e24+ROlnk8_624*e34)
    fSlnk34 = Flink4*(ROlnk8_823*e24+ROlnk8_923*e34+e14*S23)
    frclnk24_4_1 = -fSlnk13-fSlnk14
    frclnk24_4_2 = -fSlnk23-fSlnk24
    frclnk24_4_3 = -fSlnk33-fSlnk34
    trqlnk24_4_1 = trqlnk24_3_1-fSlnk34*s.dpt[2,31]
    trqlnk24_4_2 = trqlnk24_3_2+fSlnk34*s.dpt[1,31]
    trqlnk24_4_3 = trqlnk24_3_3+fSlnk14*s.dpt[2,31]-fSlnk24*s.dpt[1,31]
    fPlnk15 = Flink5*e15
    fPlnk25 = Flink5*e25
    fPlnk35 = Flink5*e35
    frclnk6_5_1 = fPlnk15+frclnk6_4_1
    frclnk6_5_2 = fPlnk25+frclnk6_4_2
    frclnk6_5_3 = fPlnk35+frclnk6_4_3
    trqlnk6_5_1 = trqlnk6_4_1-fPlnk25*s.dpt[3,11]+fPlnk35*s.dpt[2,11]
    trqlnk6_5_2 = trqlnk6_4_2+fPlnk15*s.dpt[3,11]-fPlnk35*s.dpt[1,11]
    trqlnk6_5_3 = trqlnk6_4_3-fPlnk15*s.dpt[2,11]+fPlnk25*s.dpt[1,11]
    fSlnk15 = Flink5*(ROlnk10_124*e15+ROlnk10_224*e25+ROlnk10_324*e35)
    fSlnk25 = Flink5*(ROlnk10_424*e15+ROlnk10_524*e25+ROlnk10_624*e35)
    fSlnk35 = Flink5*(ROlnk10_823*e25+ROlnk10_923*e35+e15*S23)
    frclnk24_5_1 = -fSlnk15+frclnk24_4_1
    frclnk24_5_2 = -fSlnk25+frclnk24_4_2
    frclnk24_5_3 = -fSlnk35+frclnk24_4_3
    trqlnk24_5_1 = trqlnk24_4_1-fSlnk35*s.dpt[2,33]
    trqlnk24_5_2 = trqlnk24_4_2+fSlnk35*s.dpt[1,33]
    trqlnk24_5_3 = trqlnk24_4_3+fSlnk15*s.dpt[2,33]-fSlnk25*s.dpt[1,33]
    fPlnk16 = Flink6*e16
    fPlnk26 = Flink6*e26
    fPlnk36 = Flink6*e36
    frclnk6_6_1 = fPlnk16+frclnk6_5_1
    frclnk6_6_2 = fPlnk26+frclnk6_5_2
    frclnk6_6_3 = fPlnk36+frclnk6_5_3
    trqlnk6_6_1 = trqlnk6_5_1-fPlnk26*s.dpt[3,12]+fPlnk36*s.dpt[2,12]
    trqlnk6_6_2 = trqlnk6_5_2+fPlnk16*s.dpt[3,12]-fPlnk36*s.dpt[1,12]
    trqlnk6_6_3 = trqlnk6_5_3-fPlnk16*s.dpt[2,12]+fPlnk26*s.dpt[1,12]
    fSlnk16 = Flink6*(ROlnk12_124*e16+ROlnk12_224*e26+ROlnk12_324*e36)
    fSlnk26 = Flink6*(ROlnk12_424*e16+ROlnk12_524*e26+ROlnk12_624*e36)
    fSlnk36 = Flink6*(ROlnk12_823*e26+ROlnk12_923*e36+e16*S23)
    frclnk24_6_1 = -fSlnk16+frclnk24_5_1
    frclnk24_6_2 = -fSlnk26+frclnk24_5_2
    frclnk24_6_3 = -fSlnk36+frclnk24_5_3
    trqlnk24_6_1 = trqlnk24_5_1-fSlnk36*s.dpt[2,34]
    trqlnk24_6_2 = trqlnk24_5_2+fSlnk36*s.dpt[1,34]
    trqlnk24_6_3 = trqlnk24_5_3+fSlnk16*s.dpt[2,34]-fSlnk26*s.dpt[1,34]
    fPlnk17 = Flink7*e17
    fPlnk27 = Flink7*e27
    fPlnk37 = Flink7*e37
    frclnk6_7_1 = fPlnk17+frclnk6_6_1
    frclnk6_7_2 = fPlnk27+frclnk6_6_2
    frclnk6_7_3 = fPlnk37+frclnk6_6_3
    trqlnk6_7_1 = trqlnk6_6_1+fPlnk37*s.dpt[2,14]
    trqlnk6_7_2 = trqlnk6_6_2-fPlnk37*s.dpt[1,14]
    trqlnk6_7_3 = trqlnk6_6_3-fPlnk17*s.dpt[2,14]+fPlnk27*s.dpt[1,14]
    fSlnk17 = Flink7*(ROlnk14_124*e17+ROlnk14_224*e27+ROlnk14_324*e37)
    fSlnk27 = Flink7*(ROlnk14_424*e17+ROlnk14_524*e27+ROlnk14_624*e37)
    fSlnk37 = Flink7*(ROlnk14_823*e27+ROlnk14_923*e37+e17*S23)
    frclnk24_7_1 = -fSlnk17+frclnk24_6_1
    frclnk24_7_2 = -fSlnk27+frclnk24_6_2
    frclnk24_7_3 = -fSlnk37+frclnk24_6_3
    trqlnk24_7_1 = trqlnk24_6_1-fSlnk37*s.dpt[2,39]
    trqlnk24_7_3 = trqlnk24_6_3+fSlnk17*s.dpt[2,39]
    fPlnk18 = Flink8*e18
    fPlnk28 = Flink8*e28
    fPlnk38 = Flink8*e38
    frclnk6_8_1 = fPlnk18+frclnk6_7_1
    frclnk6_8_2 = fPlnk28+frclnk6_7_2
    frclnk6_8_3 = fPlnk38+frclnk6_7_3
    trqlnk6_8_1 = trqlnk6_7_1+fPlnk38*s.dpt[2,15]
    trqlnk6_8_2 = trqlnk6_7_2-fPlnk38*s.dpt[1,15]
    trqlnk6_8_3 = trqlnk6_7_3-fPlnk18*s.dpt[2,15]+fPlnk28*s.dpt[1,15]
    fSlnk18 = Flink8*(ROlnk16_124*e18+ROlnk16_224*e28+ROlnk16_324*e38)
    fSlnk28 = Flink8*(ROlnk16_424*e18+ROlnk16_524*e28+ROlnk16_624*e38)
    fSlnk38 = Flink8*(ROlnk16_823*e28+ROlnk16_923*e38+e18*S23)
    frclnk24_8_1 = -fSlnk18+frclnk24_7_1
    frclnk24_8_2 = -fSlnk28+frclnk24_7_2
    frclnk24_8_3 = -fSlnk38+frclnk24_7_3
    trqlnk24_8_1 = trqlnk24_7_1-fSlnk38*s.dpt[2,38]
    trqlnk24_8_3 = trqlnk24_7_3+fSlnk18*s.dpt[2,38]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_8_1
    frc[2,6] = s.frc[2,6]+frclnk6_8_2
    frc[3,6] = s.frc[3,6]+frclnk6_8_3
    trq[1,6] = s.trq[1,6]+trqlnk6_8_1
    trq[2,6] = s.trq[2,6]+trqlnk6_8_2
    trq[3,6] = s.trq[3,6]+trqlnk6_8_3
    frc[1,7] = s.frc[1,7]-fSlnk11
    frc[2,7] = s.frc[2,7]-fSlnk21
    frc[3,7] = s.frc[3,7]-fSlnk31
    trq[1,7] = s.trq[1,7]+trqlnk7_1_1
    trq[2,7] = s.trq[2,7]+trqlnk7_1_2
    trq[3,7] = s.trq[3,7]+trqlnk7_1_3
    frc[1,13] = s.frc[1,13]-fSlnk12
    frc[2,13] = s.frc[2,13]-fSlnk22
    frc[3,13] = s.frc[3,13]-fSlnk32
    trq[1,13] = s.trq[1,13]+trqlnk13_2_1
    trq[2,13] = s.trq[2,13]+trqlnk13_2_2
    trq[3,13] = s.trq[3,13]+trqlnk13_2_3
    frc[1,24] = s.frc[1,24]+frclnk24_8_1
    frc[2,24] = s.frc[2,24]+frclnk24_8_2
    frc[3,24] = s.frc[3,24]+frclnk24_8_3
    trq[1,24] = s.trq[1,24]+trqlnk24_8_1
    trq[2,24] = s.trq[2,24]+trqlnk24_6_2
    trq[3,24] = s.trq[3,24]+trqlnk24_8_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3
    Z[4] = Z4
    Zd[4] = Zd4
    Flink[4] = Flink4
    Z[5] = Z5
    Zd[5] = Zd5
    Flink[5] = Flink5
    Z[6] = Z6
    Zd[6] = Zd6
    Flink[6] = Flink6
    Z[7] = Z7
    Zd[7] = Zd7
    Flink[7] = Flink7
    Z[8] = Z8
    Zd[8] = Zd8
    Flink[8] = Flink8

# Number of continuation lines = 0


