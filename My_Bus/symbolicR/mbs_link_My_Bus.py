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
    S14 = sin(q[14])
    C14 = cos(q[14])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,25]*C8-s.dpt[3,25]*S8
    RLlnk2_32 = s.dpt[2,25]*S8+s.dpt[3,25]*C8
    POlnk2_22 = RLlnk2_22+s.dpt[2,4]
    ORlnk2_22 = -qd[8]*RLlnk2_32
    ORlnk2_32 = qd[8]*RLlnk2_22
    Plnk11 = -s.dpt[1,2]+s.dpt[1,4]
    Plnk21 = POlnk2_22-s.dpt[2,2]
    Plnk31 = RLlnk2_32-s.dpt[3,2]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk4_22 = s.dpt[2,31]*C14-s.dpt[3,31]*S14
    RLlnk4_32 = s.dpt[2,31]*S14+s.dpt[3,31]*C14
    POlnk4_22 = RLlnk4_22+s.dpt[2,7]
    ORlnk4_22 = -qd[14]*RLlnk4_32
    ORlnk4_32 = qd[14]*RLlnk4_22
    Plnk12 = -s.dpt[1,5]+s.dpt[1,7]
    Plnk22 = POlnk4_22-s.dpt[2,5]
    Plnk32 = RLlnk4_32-s.dpt[3,5]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    ROlnk6_420 = S19*S20
    ROlnk6_620 = C19*S20
    ROlnk6_720 = S19*C20
    ROlnk6_920 = C19*C20
    OMlnk6_12 = qd[20]*C19
    OMlnk6_32 = -qd[20]*S19
    RLlnk6_13 = ROlnk6_420*s.dpt[2,41]+s.dpt[1,41]*C19
    RLlnk6_23 = s.dpt[2,41]*C20
    RLlnk6_33 = ROlnk6_620*s.dpt[2,41]-s.dpt[1,41]*S19
    POlnk6_13 = RLlnk6_13+s.dpt[1,15]
    ORlnk6_13 = qd[19]*RLlnk6_33-OMlnk6_32*RLlnk6_23
    ORlnk6_23 = -OMlnk6_12*RLlnk6_33+OMlnk6_32*RLlnk6_13
    ORlnk6_33 = -qd[19]*RLlnk6_13+OMlnk6_12*RLlnk6_23
    Plnk13 = POlnk6_13-s.dpt[1,8]
    Plnk23 = RLlnk6_23-s.dpt[2,8]
    Plnk33 = RLlnk6_33-s.dpt[3,8]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_13*e13+ORlnk6_23*e23+ORlnk6_33*e33
    ROlnk8_420 = S19*S20
    ROlnk8_620 = C19*S20
    ROlnk8_720 = S19*C20
    ROlnk8_920 = C19*C20
    OMlnk8_12 = qd[20]*C19
    OMlnk8_32 = -qd[20]*S19
    RLlnk8_13 = ROlnk8_420*s.dpt[2,42]+s.dpt[1,42]*C19
    RLlnk8_23 = s.dpt[2,42]*C20
    RLlnk8_33 = ROlnk8_620*s.dpt[2,42]-s.dpt[1,42]*S19
    POlnk8_13 = RLlnk8_13+s.dpt[1,15]
    ORlnk8_13 = qd[19]*RLlnk8_33-OMlnk8_32*RLlnk8_23
    ORlnk8_23 = -OMlnk8_12*RLlnk8_33+OMlnk8_32*RLlnk8_13
    ORlnk8_33 = -qd[19]*RLlnk8_13+OMlnk8_12*RLlnk8_23
    Plnk14 = POlnk8_13-s.dpt[1,9]
    Plnk24 = RLlnk8_23-s.dpt[2,9]
    Plnk34 = RLlnk8_33-s.dpt[3,9]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = ORlnk8_13*e14+ORlnk8_23*e24+ORlnk8_33*e34
    ROlnk10_420 = S19*S20
    ROlnk10_620 = C19*S20
    ROlnk10_720 = S19*C20
    ROlnk10_920 = C19*C20
    OMlnk10_12 = qd[20]*C19
    OMlnk10_32 = -qd[20]*S19
    RLlnk10_13 = ROlnk10_420*s.dpt[2,45]+s.dpt[1,45]*C19
    RLlnk10_23 = s.dpt[2,45]*C20
    RLlnk10_33 = ROlnk10_620*s.dpt[2,45]-s.dpt[1,45]*S19
    POlnk10_13 = RLlnk10_13+s.dpt[1,15]
    ORlnk10_13 = qd[19]*RLlnk10_33-OMlnk10_32*RLlnk10_23
    ORlnk10_23 = -OMlnk10_12*RLlnk10_33+OMlnk10_32*RLlnk10_13
    ORlnk10_33 = -qd[19]*RLlnk10_13+OMlnk10_12*RLlnk10_23
    Plnk15 = POlnk10_13-s.dpt[1,22]
    Plnk25 = RLlnk10_23-s.dpt[2,22]
    Plnk35 = RLlnk10_33-s.dpt[3,22]
    PPlnk5 = Plnk15*Plnk15+Plnk25*Plnk25+Plnk35*Plnk35
    Z5 = sqrt(PPlnk5)
    e15 = Plnk15/Z5
    e25 = Plnk25/Z5
    e35 = Plnk35/Z5
    Zd5 = ORlnk10_13*e15+ORlnk10_23*e25+ORlnk10_33*e35
    ROlnk12_420 = S19*S20
    ROlnk12_620 = C19*S20
    ROlnk12_720 = S19*C20
    ROlnk12_920 = C19*C20
    OMlnk12_12 = qd[20]*C19
    OMlnk12_32 = -qd[20]*S19
    RLlnk12_13 = ROlnk12_420*s.dpt[2,46]+s.dpt[1,46]*C19
    RLlnk12_23 = s.dpt[2,46]*C20
    RLlnk12_33 = ROlnk12_620*s.dpt[2,46]-s.dpt[1,46]*S19
    POlnk12_13 = RLlnk12_13+s.dpt[1,15]
    ORlnk12_13 = qd[19]*RLlnk12_33-OMlnk12_32*RLlnk12_23
    ORlnk12_23 = -OMlnk12_12*RLlnk12_33+OMlnk12_32*RLlnk12_13
    ORlnk12_33 = -qd[19]*RLlnk12_13+OMlnk12_12*RLlnk12_23
    Plnk16 = POlnk12_13-s.dpt[1,10]
    Plnk26 = RLlnk12_23-s.dpt[2,10]
    Plnk36 = RLlnk12_33-s.dpt[3,10]
    PPlnk6 = Plnk16*Plnk16+Plnk26*Plnk26+Plnk36*Plnk36
    Z6 = sqrt(PPlnk6)
    e16 = Plnk16/Z6
    e26 = Plnk26/Z6
    e36 = Plnk36/Z6
    Zd6 = ORlnk12_13*e16+ORlnk12_23*e26+ORlnk12_33*e36
    ROlnk14_420 = S19*S20
    ROlnk14_620 = C19*S20
    ROlnk14_720 = S19*C20
    ROlnk14_920 = C19*C20
    OMlnk14_12 = qd[20]*C19
    OMlnk14_32 = -qd[20]*S19
    RLlnk14_13 = ROlnk14_420*s.dpt[2,43]+s.dpt[1,43]*C19
    RLlnk14_23 = s.dpt[2,43]*C20
    RLlnk14_33 = ROlnk14_620*s.dpt[2,43]-s.dpt[1,43]*S19
    POlnk14_13 = RLlnk14_13+s.dpt[1,15]
    ORlnk14_13 = qd[19]*RLlnk14_33-OMlnk14_32*RLlnk14_23
    ORlnk14_23 = -OMlnk14_12*RLlnk14_33+OMlnk14_32*RLlnk14_13
    ORlnk14_33 = -qd[19]*RLlnk14_13+OMlnk14_12*RLlnk14_23
    Plnk17 = POlnk14_13-s.dpt[1,16]
    Plnk27 = RLlnk14_23-s.dpt[2,16]
    Plnk37 = RLlnk14_33-s.dpt[3,16]
    PPlnk7 = Plnk17*Plnk17+Plnk27*Plnk27+Plnk37*Plnk37
    Z7 = sqrt(PPlnk7)
    e17 = Plnk17/Z7
    e27 = Plnk27/Z7
    e37 = Plnk37/Z7
    Zd7 = ORlnk14_13*e17+ORlnk14_23*e27+ORlnk14_33*e37
    ROlnk16_420 = S19*S20
    ROlnk16_620 = C19*S20
    ROlnk16_720 = S19*C20
    ROlnk16_920 = C19*C20
    OMlnk16_12 = qd[20]*C19
    OMlnk16_32 = -qd[20]*S19
    RLlnk16_13 = ROlnk16_420*s.dpt[2,44]+s.dpt[1,44]*C19
    RLlnk16_23 = s.dpt[2,44]*C20
    RLlnk16_33 = ROlnk16_620*s.dpt[2,44]-s.dpt[1,44]*S19
    POlnk16_13 = RLlnk16_13+s.dpt[1,15]
    ORlnk16_13 = qd[19]*RLlnk16_33-OMlnk16_32*RLlnk16_23
    ORlnk16_23 = -OMlnk16_12*RLlnk16_33+OMlnk16_32*RLlnk16_13
    ORlnk16_33 = -qd[19]*RLlnk16_13+OMlnk16_12*RLlnk16_23
    Plnk18 = POlnk16_13-s.dpt[1,17]
    Plnk28 = RLlnk16_23-s.dpt[2,17]
    Plnk38 = RLlnk16_33-s.dpt[3,17]
    PPlnk8 = Plnk18*Plnk18+Plnk28*Plnk28+Plnk38*Plnk38
    Z8 = sqrt(PPlnk8)
    e18 = Plnk18/Z8
    e28 = Plnk28/Z8
    e38 = Plnk38/Z8
    Zd8 = ORlnk16_13*e18+ORlnk16_23*e28+ORlnk16_33*e38
    ROlnk18_420 = S19*S20
    ROlnk18_620 = C19*S20
    ROlnk18_720 = S19*C20
    ROlnk18_920 = C19*C20
    OMlnk18_12 = qd[20]*C19
    OMlnk18_32 = -qd[20]*S19
    RLlnk18_13 = ROlnk18_420*s.dpt[2,47]+s.dpt[1,47]*C19
    RLlnk18_23 = s.dpt[2,47]*C20
    RLlnk18_33 = ROlnk18_620*s.dpt[2,47]-s.dpt[1,47]*S19
    POlnk18_13 = RLlnk18_13+s.dpt[1,15]
    ORlnk18_13 = qd[19]*RLlnk18_33-OMlnk18_32*RLlnk18_23
    ORlnk18_23 = -OMlnk18_12*RLlnk18_33+OMlnk18_32*RLlnk18_13
    ORlnk18_33 = -qd[19]*RLlnk18_13+OMlnk18_12*RLlnk18_23
    Plnk19 = POlnk18_13-s.dpt[1,18]
    Plnk29 = RLlnk18_23-s.dpt[2,18]
    Plnk39 = RLlnk18_33-s.dpt[3,18]
    PPlnk9 = Plnk19*Plnk19+Plnk29*Plnk29+Plnk39*Plnk39
    Z9 = sqrt(PPlnk9)
    e19 = Plnk19/Z9
    e29 = Plnk29/Z9
    e39 = Plnk39/Z9
    Zd9 = ORlnk18_13*e19+ORlnk18_23*e29+ORlnk18_33*e39
    ROlnk20_420 = S19*S20
    ROlnk20_620 = C19*S20
    ROlnk20_720 = S19*C20
    ROlnk20_920 = C19*C20
    OMlnk20_12 = qd[20]*C19
    OMlnk20_32 = -qd[20]*S19
    RLlnk20_13 = ROlnk20_420*s.dpt[2,48]+s.dpt[1,48]*C19
    RLlnk20_23 = s.dpt[2,48]*C20
    RLlnk20_33 = ROlnk20_620*s.dpt[2,48]-s.dpt[1,48]*S19
    POlnk20_13 = RLlnk20_13+s.dpt[1,15]
    ORlnk20_13 = qd[19]*RLlnk20_33-OMlnk20_32*RLlnk20_23
    ORlnk20_23 = -OMlnk20_12*RLlnk20_33+OMlnk20_32*RLlnk20_13
    ORlnk20_33 = -qd[19]*RLlnk20_13+OMlnk20_12*RLlnk20_23
    Plnk110 = POlnk20_13-s.dpt[1,19]
    Plnk210 = RLlnk20_23-s.dpt[2,19]
    Plnk310 = RLlnk20_33-s.dpt[3,19]
    PPlnk10 = Plnk110*Plnk110+Plnk210*Plnk210+Plnk310*Plnk310
    Z10 = sqrt(PPlnk10)
    e110 = Plnk110/Z10
    e210 = Plnk210/Z10
    e310 = Plnk310/Z10
    Zd10 = ORlnk20_13*e110+ORlnk20_23*e210+ORlnk20_33*e310

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
    Flink5 = s.user_LinkForces(Z5,Zd5,s,tsim,5)
    Flink6 = s.user_LinkForces(Z6,Zd6,s,tsim,6)
    Flink7 = s.user_LinkForces(Z7,Zd7,s,tsim,7)
    Flink8 = s.user_LinkForces(Z8,Zd8,s,tsim,8)
    Flink9 = s.user_LinkForces(Z9,Zd9,s,tsim,9)
    Flink10 = s.user_LinkForces(Z10,Zd10,s,tsim,10)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,2]-s.l[3,6])+fPlnk31*s.dpt[2,2]
    trqlnk6_1_2 = fPlnk11*(s.dpt[3,2]-s.l[3,6])-fPlnk31*s.dpt[1,2]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,2]+fPlnk21*s.dpt[1,2]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C8+e31*S8)
    fSlnk31 = Flink1*(-e21*S8+e31*C8)
    trqlnk8_1_1 = fSlnk21*s.dpt[3,25]-fSlnk31*s.dpt[2,25]
    trqlnk8_1_2 = -fSlnk11*s.dpt[3,25]
    trqlnk8_1_3 = fSlnk11*s.dpt[2,25]
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11+fPlnk12
    frclnk6_2_2 = fPlnk21+fPlnk22
    frclnk6_2_3 = fPlnk31+fPlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*(s.dpt[3,5]-s.l[3,6])+fPlnk32*s.dpt[2,5]
    trqlnk6_2_2 = trqlnk6_1_2+fPlnk12*(s.dpt[3,5]-s.l[3,6])-fPlnk32*s.dpt[1,5]
    trqlnk6_2_3 = trqlnk6_1_3-fPlnk12*s.dpt[2,5]+fPlnk22*s.dpt[1,5]
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*(e22*C14+e32*S14)
    fSlnk32 = Flink2*(-e22*S14+e32*C14)
    trqlnk14_2_1 = fSlnk22*s.dpt[3,31]-fSlnk32*s.dpt[2,31]
    trqlnk14_2_2 = -fSlnk12*s.dpt[3,31]
    trqlnk14_2_3 = fSlnk12*s.dpt[2,31]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_1 = fPlnk13+frclnk6_2_1
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*(s.dpt[3,8]-s.l[3,6])+fPlnk33*s.dpt[2,8]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*(s.dpt[3,8]-s.l[3,6])-fPlnk33*s.dpt[1,8]
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,8]+fPlnk23*s.dpt[1,8]
    fSlnk13 = Flink3*(e13*C19-e33*S19)
    fSlnk23 = Flink3*(ROlnk6_420*e13+ROlnk6_620*e33+e23*C20)
    fSlnk33 = Flink3*(ROlnk6_720*e13+ROlnk6_920*e33-e23*S20)
    trqlnk20_3_1 = -fSlnk33*s.dpt[2,41]
    trqlnk20_3_2 = fSlnk33*s.dpt[1,41]
    trqlnk20_3_3 = fSlnk13*s.dpt[2,41]-fSlnk23*s.dpt[1,41]
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk14+frclnk6_3_1
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*(s.dpt[3,9]-s.l[3,6])+fPlnk34*s.dpt[2,9]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*(s.dpt[3,9]-s.l[3,6])-fPlnk34*s.dpt[1,9]
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,9]+fPlnk24*s.dpt[1,9]
    fSlnk14 = Flink4*(e14*C19-e34*S19)
    fSlnk24 = Flink4*(ROlnk8_420*e14+ROlnk8_620*e34+e24*C20)
    fSlnk34 = Flink4*(ROlnk8_720*e14+ROlnk8_920*e34-e24*S20)
    frclnk20_4_1 = -fSlnk13-fSlnk14
    frclnk20_4_2 = -fSlnk23-fSlnk24
    frclnk20_4_3 = -fSlnk33-fSlnk34
    trqlnk20_4_1 = trqlnk20_3_1-fSlnk34*s.dpt[2,42]
    trqlnk20_4_2 = trqlnk20_3_2+fSlnk34*s.dpt[1,42]
    trqlnk20_4_3 = trqlnk20_3_3+fSlnk14*s.dpt[2,42]-fSlnk24*s.dpt[1,42]
    fPlnk15 = Flink5*e15
    fPlnk25 = Flink5*e25
    fPlnk35 = Flink5*e35
    frclnk6_5_1 = fPlnk15+frclnk6_4_1
    frclnk6_5_2 = fPlnk25+frclnk6_4_2
    frclnk6_5_3 = fPlnk35+frclnk6_4_3
    trqlnk6_5_1 = trqlnk6_4_1-fPlnk25*(s.dpt[3,22]-s.l[3,6])+fPlnk35*s.dpt[2,22]
    trqlnk6_5_2 = trqlnk6_4_2+fPlnk15*(s.dpt[3,22]-s.l[3,6])-fPlnk35*s.dpt[1,22]
    trqlnk6_5_3 = trqlnk6_4_3-fPlnk15*s.dpt[2,22]+fPlnk25*s.dpt[1,22]
    fSlnk15 = Flink5*(e15*C19-e35*S19)
    fSlnk25 = Flink5*(ROlnk10_420*e15+ROlnk10_620*e35+e25*C20)
    fSlnk35 = Flink5*(ROlnk10_720*e15+ROlnk10_920*e35-e25*S20)
    frclnk20_5_1 = -fSlnk15+frclnk20_4_1
    frclnk20_5_2 = -fSlnk25+frclnk20_4_2
    frclnk20_5_3 = -fSlnk35+frclnk20_4_3
    trqlnk20_5_1 = trqlnk20_4_1-fSlnk35*s.dpt[2,45]
    trqlnk20_5_2 = trqlnk20_4_2+fSlnk35*s.dpt[1,45]
    trqlnk20_5_3 = trqlnk20_4_3+fSlnk15*s.dpt[2,45]-fSlnk25*s.dpt[1,45]
    fPlnk16 = Flink6*e16
    fPlnk26 = Flink6*e26
    fPlnk36 = Flink6*e36
    frclnk6_6_1 = fPlnk16+frclnk6_5_1
    frclnk6_6_2 = fPlnk26+frclnk6_5_2
    frclnk6_6_3 = fPlnk36+frclnk6_5_3
    trqlnk6_6_1 = trqlnk6_5_1-fPlnk26*(s.dpt[3,10]-s.l[3,6])+fPlnk36*s.dpt[2,10]
    trqlnk6_6_2 = trqlnk6_5_2+fPlnk16*(s.dpt[3,10]-s.l[3,6])-fPlnk36*s.dpt[1,10]
    trqlnk6_6_3 = trqlnk6_5_3-fPlnk16*s.dpt[2,10]+fPlnk26*s.dpt[1,10]
    fSlnk16 = Flink6*(e16*C19-e36*S19)
    fSlnk26 = Flink6*(ROlnk12_420*e16+ROlnk12_620*e36+e26*C20)
    fSlnk36 = Flink6*(ROlnk12_720*e16+ROlnk12_920*e36-e26*S20)
    frclnk20_6_1 = -fSlnk16+frclnk20_5_1
    frclnk20_6_2 = -fSlnk26+frclnk20_5_2
    frclnk20_6_3 = -fSlnk36+frclnk20_5_3
    trqlnk20_6_1 = trqlnk20_5_1-fSlnk36*s.dpt[2,46]
    trqlnk20_6_2 = trqlnk20_5_2+fSlnk36*s.dpt[1,46]
    trqlnk20_6_3 = trqlnk20_5_3+fSlnk16*s.dpt[2,46]-fSlnk26*s.dpt[1,46]
    fPlnk17 = Flink7*e17
    fPlnk27 = Flink7*e27
    fPlnk37 = Flink7*e37
    frclnk6_7_1 = fPlnk17+frclnk6_6_1
    frclnk6_7_2 = fPlnk27+frclnk6_6_2
    frclnk6_7_3 = fPlnk37+frclnk6_6_3
    trqlnk6_7_1 = trqlnk6_6_1-fPlnk27*(s.dpt[3,16]-s.l[3,6])+fPlnk37*s.dpt[2,16]
    trqlnk6_7_2 = trqlnk6_6_2+fPlnk17*(s.dpt[3,16]-s.l[3,6])-fPlnk37*s.dpt[1,16]
    trqlnk6_7_3 = trqlnk6_6_3-fPlnk17*s.dpt[2,16]+fPlnk27*s.dpt[1,16]
    fSlnk17 = Flink7*(e17*C19-e37*S19)
    fSlnk27 = Flink7*(ROlnk14_420*e17+ROlnk14_620*e37+e27*C20)
    fSlnk37 = Flink7*(ROlnk14_720*e17+ROlnk14_920*e37-e27*S20)
    frclnk20_7_1 = -fSlnk17+frclnk20_6_1
    frclnk20_7_2 = -fSlnk27+frclnk20_6_2
    frclnk20_7_3 = -fSlnk37+frclnk20_6_3
    trqlnk20_7_1 = trqlnk20_6_1-fSlnk37*s.dpt[2,43]
    trqlnk20_7_2 = trqlnk20_6_2+fSlnk37*s.dpt[1,43]
    trqlnk20_7_3 = trqlnk20_6_3+fSlnk17*s.dpt[2,43]-fSlnk27*s.dpt[1,43]
    fPlnk18 = Flink8*e18
    fPlnk28 = Flink8*e28
    fPlnk38 = Flink8*e38
    frclnk6_8_1 = fPlnk18+frclnk6_7_1
    frclnk6_8_2 = fPlnk28+frclnk6_7_2
    frclnk6_8_3 = fPlnk38+frclnk6_7_3
    trqlnk6_8_1 = trqlnk6_7_1-fPlnk28*(s.dpt[3,17]-s.l[3,6])+fPlnk38*s.dpt[2,17]
    trqlnk6_8_2 = trqlnk6_7_2+fPlnk18*(s.dpt[3,17]-s.l[3,6])-fPlnk38*s.dpt[1,17]
    trqlnk6_8_3 = trqlnk6_7_3-fPlnk18*s.dpt[2,17]+fPlnk28*s.dpt[1,17]
    fSlnk18 = Flink8*(e18*C19-e38*S19)
    fSlnk28 = Flink8*(ROlnk16_420*e18+ROlnk16_620*e38+e28*C20)
    fSlnk38 = Flink8*(ROlnk16_720*e18+ROlnk16_920*e38-e28*S20)
    frclnk20_8_1 = -fSlnk18+frclnk20_7_1
    frclnk20_8_2 = -fSlnk28+frclnk20_7_2
    frclnk20_8_3 = -fSlnk38+frclnk20_7_3
    trqlnk20_8_1 = trqlnk20_7_1-fSlnk38*s.dpt[2,44]
    trqlnk20_8_2 = trqlnk20_7_2+fSlnk38*s.dpt[1,44]
    trqlnk20_8_3 = trqlnk20_7_3+fSlnk18*s.dpt[2,44]-fSlnk28*s.dpt[1,44]
    fPlnk19 = Flink9*e19
    fPlnk29 = Flink9*e29
    fPlnk39 = Flink9*e39
    frclnk6_9_1 = fPlnk19+frclnk6_8_1
    frclnk6_9_2 = fPlnk29+frclnk6_8_2
    frclnk6_9_3 = fPlnk39+frclnk6_8_3
    trqlnk6_9_1 = trqlnk6_8_1-fPlnk29*(s.dpt[3,18]-s.l[3,6])+fPlnk39*s.dpt[2,18]
    trqlnk6_9_2 = trqlnk6_8_2+fPlnk19*(s.dpt[3,18]-s.l[3,6])-fPlnk39*s.dpt[1,18]
    trqlnk6_9_3 = trqlnk6_8_3-fPlnk19*s.dpt[2,18]+fPlnk29*s.dpt[1,18]
    fSlnk19 = Flink9*(e19*C19-e39*S19)
    fSlnk29 = Flink9*(ROlnk18_420*e19+ROlnk18_620*e39+e29*C20)
    fSlnk39 = Flink9*(ROlnk18_720*e19+ROlnk18_920*e39-e29*S20)
    frclnk20_9_1 = -fSlnk19+frclnk20_8_1
    frclnk20_9_2 = -fSlnk29+frclnk20_8_2
    frclnk20_9_3 = -fSlnk39+frclnk20_8_3
    trqlnk20_9_1 = trqlnk20_8_1-fSlnk39*s.dpt[2,47]
    trqlnk20_9_2 = trqlnk20_8_2+fSlnk39*s.dpt[1,47]
    trqlnk20_9_3 = trqlnk20_8_3+fSlnk19*s.dpt[2,47]-fSlnk29*s.dpt[1,47]
    fPlnk110 = Flink10*e110
    fPlnk210 = Flink10*e210
    fPlnk310 = Flink10*e310
    frclnk6_10_1 = fPlnk110+frclnk6_9_1
    frclnk6_10_2 = fPlnk210+frclnk6_9_2
    frclnk6_10_3 = fPlnk310+frclnk6_9_3
    trqlnk6_10_1 = trqlnk6_9_1-fPlnk210*(s.dpt[3,19]-s.l[3,6])+fPlnk310*s.dpt[2,19]
    trqlnk6_10_2 = trqlnk6_9_2+fPlnk110*(s.dpt[3,19]-s.l[3,6])-fPlnk310*s.dpt[1,19]
    trqlnk6_10_3 = trqlnk6_9_3-fPlnk110*s.dpt[2,19]+fPlnk210*s.dpt[1,19]
    fSlnk110 = Flink10*(e110*C19-e310*S19)
    fSlnk210 = Flink10*(ROlnk20_420*e110+ROlnk20_620*e310+e210*C20)
    fSlnk310 = Flink10*(ROlnk20_720*e110+ROlnk20_920*e310-e210*S20)
    frclnk20_10_1 = -fSlnk110+frclnk20_9_1
    frclnk20_10_2 = -fSlnk210+frclnk20_9_2
    frclnk20_10_3 = -fSlnk310+frclnk20_9_3
    trqlnk20_10_1 = trqlnk20_9_1-fSlnk310*s.dpt[2,48]
    trqlnk20_10_2 = trqlnk20_9_2+fSlnk310*s.dpt[1,48]
    trqlnk20_10_3 = trqlnk20_9_3+fSlnk110*s.dpt[2,48]-fSlnk210*s.dpt[1,48]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_10_1
    frc[2,6] = s.frc[2,6]+frclnk6_10_2
    frc[3,6] = s.frc[3,6]+frclnk6_10_3
    trq[1,6] = s.trq[1,6]+trqlnk6_10_1
    trq[2,6] = s.trq[2,6]+trqlnk6_10_2
    trq[3,6] = s.trq[3,6]+trqlnk6_10_3
    frc[1,8] = s.frc[1,8]-fSlnk11
    frc[2,8] = s.frc[2,8]-fSlnk21
    frc[3,8] = s.frc[3,8]-fSlnk31
    trq[1,8] = s.trq[1,8]+trqlnk8_1_1
    trq[2,8] = s.trq[2,8]+trqlnk8_1_2
    trq[3,8] = s.trq[3,8]+trqlnk8_1_3
    frc[1,14] = s.frc[1,14]-fSlnk12
    frc[2,14] = s.frc[2,14]-fSlnk22
    frc[3,14] = s.frc[3,14]-fSlnk32
    trq[1,14] = s.trq[1,14]+trqlnk14_2_1
    trq[2,14] = s.trq[2,14]+trqlnk14_2_2
    trq[3,14] = s.trq[3,14]+trqlnk14_2_3
    frc[1,20] = s.frc[1,20]+frclnk20_10_1
    frc[2,20] = s.frc[2,20]+frclnk20_10_2
    frc[3,20] = s.frc[3,20]+frclnk20_10_3
    trq[1,20] = s.trq[1,20]+trqlnk20_10_1
    trq[2,20] = s.trq[2,20]+trqlnk20_10_2
    trq[3,20] = s.trq[3,20]+trqlnk20_10_3
 
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
    Z[9] = Z9
    Zd[9] = Zd9
    Flink[9] = Flink9
    Z[10] = Z10
    Zd[10] = Zd10
    Flink[10] = Flink10

# Number of continuation lines = 0


