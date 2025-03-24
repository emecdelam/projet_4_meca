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
    S12 = sin(q[12])
    C12 = cos(q[12])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
 
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
    RLlnk4_22 = s.dpt[2,23]*C12-s.dpt[3,23]*S12
    RLlnk4_32 = s.dpt[2,23]*S12+s.dpt[3,23]*C12
    POlnk4_22 = RLlnk4_22+s.dpt[2,4]
    ORlnk4_22 = -qd[12]*RLlnk4_32
    ORlnk4_32 = qd[12]*RLlnk4_22
    Plnk12 = s.dpt[1,4]-s.dpt[1,6]
    Plnk22 = POlnk4_22-s.dpt[2,6]
    Plnk32 = RLlnk4_32-s.dpt[3,6]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    ROlnk6_221 = S20*S21
    ROlnk6_321 = -C20*S21
    ROlnk6_821 = -S20*C21
    ROlnk6_921 = C20*C21
    ROlnk6_122 = C21*C22
    ROlnk6_222 = ROlnk6_221*C22+C20*S22
    ROlnk6_322 = ROlnk6_321*C22+S20*S22
    ROlnk6_422 = -C21*S22
    ROlnk6_522 = -ROlnk6_221*S22+C20*C22
    ROlnk6_622 = -ROlnk6_321*S22+S20*C22
    POlnk6_12 = q[18]+s.dpt[1,13]
    OMlnk6_25 = qd[21]*C20
    OMlnk6_35 = qd[21]*S20
    OMlnk6_16 = qd[20]+qd[22]*S21
    OMlnk6_26 = OMlnk6_25+qd[22]*ROlnk6_821
    OMlnk6_36 = OMlnk6_35+qd[22]*ROlnk6_921
    RLlnk6_17 = ROlnk6_122*s.dpt[1,28]+ROlnk6_422*s.dpt[2,28]
    RLlnk6_27 = ROlnk6_222*s.dpt[1,28]+ROlnk6_522*s.dpt[2,28]
    RLlnk6_37 = ROlnk6_322*s.dpt[1,28]+ROlnk6_622*s.dpt[2,28]
    POlnk6_17 = POlnk6_12+RLlnk6_17
    POlnk6_27 = q[19]+RLlnk6_27
    POlnk6_37 = q[17]+RLlnk6_37
    ORlnk6_17 = OMlnk6_26*RLlnk6_37-OMlnk6_36*RLlnk6_27
    ORlnk6_27 = -OMlnk6_16*RLlnk6_37+OMlnk6_36*RLlnk6_17
    ORlnk6_37 = OMlnk6_16*RLlnk6_27-OMlnk6_26*RLlnk6_17
    VIlnk6_17 = qd[18]+ORlnk6_17
    VIlnk6_27 = qd[19]+ORlnk6_27
    VIlnk6_37 = qd[17]+ORlnk6_37
    Plnk13 = POlnk6_17-s.dpt[1,7]
    Plnk23 = POlnk6_27-s.dpt[2,7]
    Plnk33 = POlnk6_37-s.dpt[3,7]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = VIlnk6_17*e13+VIlnk6_27*e23+VIlnk6_37*e33
    ROlnk8_221 = S20*S21
    ROlnk8_321 = -C20*S21
    ROlnk8_821 = -S20*C21
    ROlnk8_921 = C20*C21
    ROlnk8_122 = C21*C22
    ROlnk8_222 = ROlnk8_221*C22+C20*S22
    ROlnk8_322 = ROlnk8_321*C22+S20*S22
    ROlnk8_422 = -C21*S22
    ROlnk8_522 = -ROlnk8_221*S22+C20*C22
    ROlnk8_622 = -ROlnk8_321*S22+S20*C22
    POlnk8_12 = q[18]+s.dpt[1,13]
    OMlnk8_25 = qd[21]*C20
    OMlnk8_35 = qd[21]*S20
    OMlnk8_16 = qd[20]+qd[22]*S21
    OMlnk8_26 = OMlnk8_25+qd[22]*ROlnk8_821
    OMlnk8_36 = OMlnk8_35+qd[22]*ROlnk8_921
    RLlnk8_17 = ROlnk8_122*s.dpt[1,29]+ROlnk8_422*s.dpt[2,29]
    RLlnk8_27 = ROlnk8_222*s.dpt[1,29]+ROlnk8_522*s.dpt[2,29]
    RLlnk8_37 = ROlnk8_322*s.dpt[1,29]+ROlnk8_622*s.dpt[2,29]
    POlnk8_17 = POlnk8_12+RLlnk8_17
    POlnk8_27 = q[19]+RLlnk8_27
    POlnk8_37 = q[17]+RLlnk8_37
    ORlnk8_17 = OMlnk8_26*RLlnk8_37-OMlnk8_36*RLlnk8_27
    ORlnk8_27 = -OMlnk8_16*RLlnk8_37+OMlnk8_36*RLlnk8_17
    ORlnk8_37 = OMlnk8_16*RLlnk8_27-OMlnk8_26*RLlnk8_17
    VIlnk8_17 = qd[18]+ORlnk8_17
    VIlnk8_27 = qd[19]+ORlnk8_27
    VIlnk8_37 = qd[17]+ORlnk8_37
    Plnk14 = POlnk8_17-s.dpt[1,8]
    Plnk24 = POlnk8_27-s.dpt[2,8]
    Plnk34 = POlnk8_37-s.dpt[3,8]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = VIlnk8_17*e14+VIlnk8_27*e24+VIlnk8_37*e34
    ROlnk10_221 = S20*S21
    ROlnk10_321 = -C20*S21
    ROlnk10_821 = -S20*C21
    ROlnk10_921 = C20*C21
    ROlnk10_122 = C21*C22
    ROlnk10_222 = ROlnk10_221*C22+C20*S22
    ROlnk10_322 = ROlnk10_321*C22+S20*S22
    ROlnk10_422 = -C21*S22
    ROlnk10_522 = -ROlnk10_221*S22+C20*C22
    ROlnk10_622 = -ROlnk10_321*S22+S20*C22
    POlnk10_12 = q[18]+s.dpt[1,13]
    OMlnk10_25 = qd[21]*C20
    OMlnk10_35 = qd[21]*S20
    OMlnk10_16 = qd[20]+qd[22]*S21
    OMlnk10_26 = OMlnk10_25+qd[22]*ROlnk10_821
    OMlnk10_36 = OMlnk10_35+qd[22]*ROlnk10_921
    RLlnk10_17 = ROlnk10_122*s.dpt[1,31]+ROlnk10_422*s.dpt[2,31]
    RLlnk10_27 = ROlnk10_222*s.dpt[1,31]+ROlnk10_522*s.dpt[2,31]
    RLlnk10_37 = ROlnk10_322*s.dpt[1,31]+ROlnk10_622*s.dpt[2,31]
    POlnk10_17 = POlnk10_12+RLlnk10_17
    POlnk10_27 = q[19]+RLlnk10_27
    POlnk10_37 = q[17]+RLlnk10_37
    ORlnk10_17 = OMlnk10_26*RLlnk10_37-OMlnk10_36*RLlnk10_27
    ORlnk10_27 = -OMlnk10_16*RLlnk10_37+OMlnk10_36*RLlnk10_17
    ORlnk10_37 = OMlnk10_16*RLlnk10_27-OMlnk10_26*RLlnk10_17
    VIlnk10_17 = qd[18]+ORlnk10_17
    VIlnk10_27 = qd[19]+ORlnk10_27
    VIlnk10_37 = qd[17]+ORlnk10_37
    Plnk15 = POlnk10_17-s.dpt[1,11]
    Plnk25 = POlnk10_27-s.dpt[2,11]
    Plnk35 = POlnk10_37-s.dpt[3,11]
    PPlnk5 = Plnk15*Plnk15+Plnk25*Plnk25+Plnk35*Plnk35
    Z5 = sqrt(PPlnk5)
    e15 = Plnk15/Z5
    e25 = Plnk25/Z5
    e35 = Plnk35/Z5
    Zd5 = VIlnk10_17*e15+VIlnk10_27*e25+VIlnk10_37*e35
    ROlnk12_221 = S20*S21
    ROlnk12_321 = -C20*S21
    ROlnk12_821 = -S20*C21
    ROlnk12_921 = C20*C21
    ROlnk12_122 = C21*C22
    ROlnk12_222 = ROlnk12_221*C22+C20*S22
    ROlnk12_322 = ROlnk12_321*C22+S20*S22
    ROlnk12_422 = -C21*S22
    ROlnk12_522 = -ROlnk12_221*S22+C20*C22
    ROlnk12_622 = -ROlnk12_321*S22+S20*C22
    POlnk12_12 = q[18]+s.dpt[1,13]
    OMlnk12_25 = qd[21]*C20
    OMlnk12_35 = qd[21]*S20
    OMlnk12_16 = qd[20]+qd[22]*S21
    OMlnk12_26 = OMlnk12_25+qd[22]*ROlnk12_821
    OMlnk12_36 = OMlnk12_35+qd[22]*ROlnk12_921
    RLlnk12_17 = ROlnk12_122*s.dpt[1,32]+ROlnk12_422*s.dpt[2,32]
    RLlnk12_27 = ROlnk12_222*s.dpt[1,32]+ROlnk12_522*s.dpt[2,32]
    RLlnk12_37 = ROlnk12_322*s.dpt[1,32]+ROlnk12_622*s.dpt[2,32]
    POlnk12_17 = POlnk12_12+RLlnk12_17
    POlnk12_27 = q[19]+RLlnk12_27
    POlnk12_37 = q[17]+RLlnk12_37
    ORlnk12_17 = OMlnk12_26*RLlnk12_37-OMlnk12_36*RLlnk12_27
    ORlnk12_27 = -OMlnk12_16*RLlnk12_37+OMlnk12_36*RLlnk12_17
    ORlnk12_37 = OMlnk12_16*RLlnk12_27-OMlnk12_26*RLlnk12_17
    VIlnk12_17 = qd[18]+ORlnk12_17
    VIlnk12_27 = qd[19]+ORlnk12_27
    VIlnk12_37 = qd[17]+ORlnk12_37
    Plnk16 = POlnk12_17-s.dpt[1,12]
    Plnk26 = POlnk12_27-s.dpt[2,12]
    Plnk36 = POlnk12_37-s.dpt[3,12]
    PPlnk6 = Plnk16*Plnk16+Plnk26*Plnk26+Plnk36*Plnk36
    Z6 = sqrt(PPlnk6)
    e16 = Plnk16/Z6
    e26 = Plnk26/Z6
    e36 = Plnk36/Z6
    Zd6 = VIlnk12_17*e16+VIlnk12_27*e26+VIlnk12_37*e36
    ROlnk14_221 = S20*S21
    ROlnk14_321 = -C20*S21
    ROlnk14_821 = -S20*C21
    ROlnk14_921 = C20*C21
    ROlnk14_122 = C21*C22
    ROlnk14_222 = ROlnk14_221*C22+C20*S22
    ROlnk14_322 = ROlnk14_321*C22+S20*S22
    ROlnk14_422 = -C21*S22
    ROlnk14_522 = -ROlnk14_221*S22+C20*C22
    ROlnk14_622 = -ROlnk14_321*S22+S20*C22
    POlnk14_12 = q[18]+s.dpt[1,13]
    OMlnk14_25 = qd[21]*C20
    OMlnk14_35 = qd[21]*S20
    OMlnk14_16 = qd[20]+qd[22]*S21
    OMlnk14_26 = OMlnk14_25+qd[22]*ROlnk14_821
    OMlnk14_36 = OMlnk14_35+qd[22]*ROlnk14_921
    RLlnk14_17 = ROlnk14_422*s.dpt[2,37]
    RLlnk14_27 = ROlnk14_522*s.dpt[2,37]
    RLlnk14_37 = ROlnk14_622*s.dpt[2,37]
    POlnk14_17 = POlnk14_12+RLlnk14_17
    POlnk14_27 = q[19]+RLlnk14_27
    POlnk14_37 = q[17]+RLlnk14_37
    ORlnk14_17 = OMlnk14_26*RLlnk14_37-OMlnk14_36*RLlnk14_27
    ORlnk14_27 = -OMlnk14_16*RLlnk14_37+OMlnk14_36*RLlnk14_17
    ORlnk14_37 = OMlnk14_16*RLlnk14_27-OMlnk14_26*RLlnk14_17
    VIlnk14_17 = qd[18]+ORlnk14_17
    VIlnk14_27 = qd[19]+ORlnk14_27
    VIlnk14_37 = qd[17]+ORlnk14_37
    Plnk17 = POlnk14_17-s.dpt[1,14]
    Plnk27 = POlnk14_27-s.dpt[2,14]
    PPlnk7 = POlnk14_37*POlnk14_37+Plnk17*Plnk17+Plnk27*Plnk27
    Z7 = sqrt(PPlnk7)
    e17 = Plnk17/Z7
    e27 = Plnk27/Z7
    e37 = POlnk14_37/Z7
    Zd7 = VIlnk14_17*e17+VIlnk14_27*e27+VIlnk14_37*e37
    ROlnk16_221 = S20*S21
    ROlnk16_321 = -C20*S21
    ROlnk16_821 = -S20*C21
    ROlnk16_921 = C20*C21
    ROlnk16_122 = C21*C22
    ROlnk16_222 = ROlnk16_221*C22+C20*S22
    ROlnk16_322 = ROlnk16_321*C22+S20*S22
    ROlnk16_422 = -C21*S22
    ROlnk16_522 = -ROlnk16_221*S22+C20*C22
    ROlnk16_622 = -ROlnk16_321*S22+S20*C22
    POlnk16_12 = q[18]+s.dpt[1,13]
    OMlnk16_25 = qd[21]*C20
    OMlnk16_35 = qd[21]*S20
    OMlnk16_16 = qd[20]+qd[22]*S21
    OMlnk16_26 = OMlnk16_25+qd[22]*ROlnk16_821
    OMlnk16_36 = OMlnk16_35+qd[22]*ROlnk16_921
    RLlnk16_17 = ROlnk16_422*s.dpt[2,36]
    RLlnk16_27 = ROlnk16_522*s.dpt[2,36]
    RLlnk16_37 = ROlnk16_622*s.dpt[2,36]
    POlnk16_17 = POlnk16_12+RLlnk16_17
    POlnk16_27 = q[19]+RLlnk16_27
    POlnk16_37 = q[17]+RLlnk16_37
    ORlnk16_17 = OMlnk16_26*RLlnk16_37-OMlnk16_36*RLlnk16_27
    ORlnk16_27 = -OMlnk16_16*RLlnk16_37+OMlnk16_36*RLlnk16_17
    ORlnk16_37 = OMlnk16_16*RLlnk16_27-OMlnk16_26*RLlnk16_17
    VIlnk16_17 = qd[18]+ORlnk16_17
    VIlnk16_27 = qd[19]+ORlnk16_27
    VIlnk16_37 = qd[17]+ORlnk16_37
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
    fSlnk22 = Flink2*(e22*C12+e32*S12)
    fSlnk32 = Flink2*(-e22*S12+e32*C12)
    trqlnk12_2_1 = fSlnk22*s.dpt[3,23]-fSlnk32*s.dpt[2,23]
    trqlnk12_2_2 = -fSlnk12*s.dpt[3,23]
    trqlnk12_2_3 = fSlnk12*s.dpt[2,23]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_1 = fPlnk13+frclnk6_2_1
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*s.dpt[3,7]+fPlnk33*s.dpt[2,7]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*s.dpt[3,7]-fPlnk33*s.dpt[1,7]
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,7]+fPlnk23*s.dpt[1,7]
    fSlnk13 = Flink3*(ROlnk6_122*e13+ROlnk6_222*e23+ROlnk6_322*e33)
    fSlnk23 = Flink3*(ROlnk6_422*e13+ROlnk6_522*e23+ROlnk6_622*e33)
    fSlnk33 = Flink3*(ROlnk6_821*e23+ROlnk6_921*e33+e13*S21)
    trqlnk22_3_1 = -fSlnk33*s.dpt[2,28]
    trqlnk22_3_2 = fSlnk33*s.dpt[1,28]
    trqlnk22_3_3 = fSlnk13*s.dpt[2,28]-fSlnk23*s.dpt[1,28]
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk14+frclnk6_3_1
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*s.dpt[3,8]+fPlnk34*s.dpt[2,8]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*s.dpt[3,8]-fPlnk34*s.dpt[1,8]
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,8]+fPlnk24*s.dpt[1,8]
    fSlnk14 = Flink4*(ROlnk8_122*e14+ROlnk8_222*e24+ROlnk8_322*e34)
    fSlnk24 = Flink4*(ROlnk8_422*e14+ROlnk8_522*e24+ROlnk8_622*e34)
    fSlnk34 = Flink4*(ROlnk8_821*e24+ROlnk8_921*e34+e14*S21)
    frclnk22_4_1 = -fSlnk13-fSlnk14
    frclnk22_4_2 = -fSlnk23-fSlnk24
    frclnk22_4_3 = -fSlnk33-fSlnk34
    trqlnk22_4_1 = trqlnk22_3_1-fSlnk34*s.dpt[2,29]
    trqlnk22_4_2 = trqlnk22_3_2+fSlnk34*s.dpt[1,29]
    trqlnk22_4_3 = trqlnk22_3_3+fSlnk14*s.dpt[2,29]-fSlnk24*s.dpt[1,29]
    fPlnk15 = Flink5*e15
    fPlnk25 = Flink5*e25
    fPlnk35 = Flink5*e35
    frclnk6_5_1 = fPlnk15+frclnk6_4_1
    frclnk6_5_2 = fPlnk25+frclnk6_4_2
    frclnk6_5_3 = fPlnk35+frclnk6_4_3
    trqlnk6_5_1 = trqlnk6_4_1-fPlnk25*s.dpt[3,11]+fPlnk35*s.dpt[2,11]
    trqlnk6_5_2 = trqlnk6_4_2+fPlnk15*s.dpt[3,11]-fPlnk35*s.dpt[1,11]
    trqlnk6_5_3 = trqlnk6_4_3-fPlnk15*s.dpt[2,11]+fPlnk25*s.dpt[1,11]
    fSlnk15 = Flink5*(ROlnk10_122*e15+ROlnk10_222*e25+ROlnk10_322*e35)
    fSlnk25 = Flink5*(ROlnk10_422*e15+ROlnk10_522*e25+ROlnk10_622*e35)
    fSlnk35 = Flink5*(ROlnk10_821*e25+ROlnk10_921*e35+e15*S21)
    frclnk22_5_1 = -fSlnk15+frclnk22_4_1
    frclnk22_5_2 = -fSlnk25+frclnk22_4_2
    frclnk22_5_3 = -fSlnk35+frclnk22_4_3
    trqlnk22_5_1 = trqlnk22_4_1-fSlnk35*s.dpt[2,31]
    trqlnk22_5_2 = trqlnk22_4_2+fSlnk35*s.dpt[1,31]
    trqlnk22_5_3 = trqlnk22_4_3+fSlnk15*s.dpt[2,31]-fSlnk25*s.dpt[1,31]
    fPlnk16 = Flink6*e16
    fPlnk26 = Flink6*e26
    fPlnk36 = Flink6*e36
    frclnk6_6_1 = fPlnk16+frclnk6_5_1
    frclnk6_6_2 = fPlnk26+frclnk6_5_2
    frclnk6_6_3 = fPlnk36+frclnk6_5_3
    trqlnk6_6_1 = trqlnk6_5_1-fPlnk26*s.dpt[3,12]+fPlnk36*s.dpt[2,12]
    trqlnk6_6_2 = trqlnk6_5_2+fPlnk16*s.dpt[3,12]-fPlnk36*s.dpt[1,12]
    trqlnk6_6_3 = trqlnk6_5_3-fPlnk16*s.dpt[2,12]+fPlnk26*s.dpt[1,12]
    fSlnk16 = Flink6*(ROlnk12_122*e16+ROlnk12_222*e26+ROlnk12_322*e36)
    fSlnk26 = Flink6*(ROlnk12_422*e16+ROlnk12_522*e26+ROlnk12_622*e36)
    fSlnk36 = Flink6*(ROlnk12_821*e26+ROlnk12_921*e36+e16*S21)
    frclnk22_6_1 = -fSlnk16+frclnk22_5_1
    frclnk22_6_2 = -fSlnk26+frclnk22_5_2
    frclnk22_6_3 = -fSlnk36+frclnk22_5_3
    trqlnk22_6_1 = trqlnk22_5_1-fSlnk36*s.dpt[2,32]
    trqlnk22_6_2 = trqlnk22_5_2+fSlnk36*s.dpt[1,32]
    trqlnk22_6_3 = trqlnk22_5_3+fSlnk16*s.dpt[2,32]-fSlnk26*s.dpt[1,32]
    fPlnk17 = Flink7*e17
    fPlnk27 = Flink7*e27
    fPlnk37 = Flink7*e37
    frclnk6_7_1 = fPlnk17+frclnk6_6_1
    frclnk6_7_2 = fPlnk27+frclnk6_6_2
    frclnk6_7_3 = fPlnk37+frclnk6_6_3
    trqlnk6_7_1 = trqlnk6_6_1+fPlnk37*s.dpt[2,14]
    trqlnk6_7_2 = trqlnk6_6_2-fPlnk37*s.dpt[1,14]
    trqlnk6_7_3 = trqlnk6_6_3-fPlnk17*s.dpt[2,14]+fPlnk27*s.dpt[1,14]
    fSlnk17 = Flink7*(ROlnk14_122*e17+ROlnk14_222*e27+ROlnk14_322*e37)
    fSlnk27 = Flink7*(ROlnk14_422*e17+ROlnk14_522*e27+ROlnk14_622*e37)
    fSlnk37 = Flink7*(ROlnk14_821*e27+ROlnk14_921*e37+e17*S21)
    frclnk22_7_1 = -fSlnk17+frclnk22_6_1
    frclnk22_7_2 = -fSlnk27+frclnk22_6_2
    frclnk22_7_3 = -fSlnk37+frclnk22_6_3
    trqlnk22_7_1 = trqlnk22_6_1-fSlnk37*s.dpt[2,37]
    trqlnk22_7_3 = trqlnk22_6_3+fSlnk17*s.dpt[2,37]
    fPlnk18 = Flink8*e18
    fPlnk28 = Flink8*e28
    fPlnk38 = Flink8*e38
    frclnk6_8_1 = fPlnk18+frclnk6_7_1
    frclnk6_8_2 = fPlnk28+frclnk6_7_2
    frclnk6_8_3 = fPlnk38+frclnk6_7_3
    trqlnk6_8_1 = trqlnk6_7_1+fPlnk38*s.dpt[2,15]
    trqlnk6_8_2 = trqlnk6_7_2-fPlnk38*s.dpt[1,15]
    trqlnk6_8_3 = trqlnk6_7_3-fPlnk18*s.dpt[2,15]+fPlnk28*s.dpt[1,15]
    fSlnk18 = Flink8*(ROlnk16_122*e18+ROlnk16_222*e28+ROlnk16_322*e38)
    fSlnk28 = Flink8*(ROlnk16_422*e18+ROlnk16_522*e28+ROlnk16_622*e38)
    fSlnk38 = Flink8*(ROlnk16_821*e28+ROlnk16_921*e38+e18*S21)
    frclnk22_8_1 = -fSlnk18+frclnk22_7_1
    frclnk22_8_2 = -fSlnk28+frclnk22_7_2
    frclnk22_8_3 = -fSlnk38+frclnk22_7_3
    trqlnk22_8_1 = trqlnk22_7_1-fSlnk38*s.dpt[2,36]
    trqlnk22_8_3 = trqlnk22_7_3+fSlnk18*s.dpt[2,36]
 
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
    frc[1,12] = s.frc[1,12]-fSlnk12
    frc[2,12] = s.frc[2,12]-fSlnk22
    frc[3,12] = s.frc[3,12]-fSlnk32
    trq[1,12] = s.trq[1,12]+trqlnk12_2_1
    trq[2,12] = s.trq[2,12]+trqlnk12_2_2
    trq[3,12] = s.trq[3,12]+trqlnk12_2_3
    frc[1,22] = s.frc[1,22]+frclnk22_8_1
    frc[2,22] = s.frc[2,22]+frclnk22_8_2
    frc[3,22] = s.frc[3,22]+frclnk22_8_3
    trq[1,22] = s.trq[1,22]+trqlnk22_8_1
    trq[2,22] = s.trq[2,22]+trqlnk22_6_2
    trq[3,22] = s.trq[3,22]+trqlnk22_8_3
 
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


