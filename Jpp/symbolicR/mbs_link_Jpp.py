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
#	==> Generation Date: Fri Mar 28 02:55:51 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
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

    S7 = sin(q[7])
    C7 = cos(q[7])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,24]*C7-s.dpt[3,24]*S7
    RLlnk2_32 = s.dpt[2,24]*S7+s.dpt[3,24]*C7
    POlnk2_22 = RLlnk2_22+s.dpt[2,2]
    ORlnk2_22 = -qd[7]*RLlnk2_32
    ORlnk2_32 = qd[7]*RLlnk2_22
    Plnk11 = s.dpt[1,2]-s.dpt[1,4]
    Plnk21 = POlnk2_22-s.dpt[2,4]
    Plnk31 = RLlnk2_32-s.dpt[3,4]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk4_22 = s.dpt[2,31]*C13-s.dpt[3,31]*S13
    RLlnk4_32 = s.dpt[2,31]*S13+s.dpt[3,31]*C13
    POlnk4_22 = RLlnk4_22+s.dpt[2,5]
    ORlnk4_22 = -qd[13]*RLlnk4_32
    ORlnk4_32 = qd[13]*RLlnk4_22
    Plnk12 = s.dpt[1,5]-s.dpt[1,7]
    Plnk22 = POlnk4_22-s.dpt[2,7]
    Plnk32 = RLlnk4_32-s.dpt[3,7]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    ROlnk6_220 = S19*S20
    ROlnk6_320 = -C19*S20
    ROlnk6_820 = -S19*C20
    ROlnk6_920 = C19*C20
    OMlnk6_22 = qd[20]*C19
    OMlnk6_32 = qd[20]*S19
    RLlnk6_13 = s.dpt[1,37]*C20
    RLlnk6_23 = ROlnk6_220*s.dpt[1,37]+s.dpt[2,37]*C19
    RLlnk6_33 = ROlnk6_320*s.dpt[1,37]+s.dpt[2,37]*S19
    POlnk6_13 = RLlnk6_13+s.dpt[1,14]
    ORlnk6_13 = OMlnk6_22*RLlnk6_33-OMlnk6_32*RLlnk6_23
    ORlnk6_23 = -qd[19]*RLlnk6_33+OMlnk6_32*RLlnk6_13
    ORlnk6_33 = qd[19]*RLlnk6_23-OMlnk6_22*RLlnk6_13
    Plnk13 = POlnk6_13-s.dpt[1,8]
    Plnk23 = RLlnk6_23-s.dpt[2,8]
    Plnk33 = RLlnk6_33-s.dpt[3,8]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_13*e13+ORlnk6_23*e23+ORlnk6_33*e33
    ROlnk8_220 = S19*S20
    ROlnk8_320 = -C19*S20
    ROlnk8_820 = -S19*C20
    ROlnk8_920 = C19*C20
    OMlnk8_22 = qd[20]*C19
    OMlnk8_32 = qd[20]*S19
    RLlnk8_13 = s.dpt[1,38]*C20
    RLlnk8_23 = ROlnk8_220*s.dpt[1,38]+s.dpt[2,38]*C19
    RLlnk8_33 = ROlnk8_320*s.dpt[1,38]+s.dpt[2,38]*S19
    POlnk8_13 = RLlnk8_13+s.dpt[1,14]
    ORlnk8_13 = OMlnk8_22*RLlnk8_33-OMlnk8_32*RLlnk8_23
    ORlnk8_23 = -qd[19]*RLlnk8_33+OMlnk8_32*RLlnk8_13
    ORlnk8_33 = qd[19]*RLlnk8_23-OMlnk8_22*RLlnk8_13
    Plnk14 = POlnk8_13-s.dpt[1,9]
    Plnk24 = RLlnk8_23-s.dpt[2,9]
    Plnk34 = RLlnk8_33-s.dpt[3,9]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = ORlnk8_13*e14+ORlnk8_23*e24+ORlnk8_33*e34
    ROlnk10_220 = S19*S20
    ROlnk10_320 = -C19*S20
    ROlnk10_820 = -S19*C20
    ROlnk10_920 = C19*C20
    OMlnk10_22 = qd[20]*C19
    OMlnk10_32 = qd[20]*S19
    RLlnk10_13 = s.dpt[1,40]*C20
    RLlnk10_23 = ROlnk10_220*s.dpt[1,40]+s.dpt[2,40]*C19
    RLlnk10_33 = ROlnk10_320*s.dpt[1,40]+s.dpt[2,40]*S19
    POlnk10_13 = RLlnk10_13+s.dpt[1,14]
    ORlnk10_13 = OMlnk10_22*RLlnk10_33-OMlnk10_32*RLlnk10_23
    ORlnk10_23 = -qd[19]*RLlnk10_33+OMlnk10_32*RLlnk10_13
    ORlnk10_33 = qd[19]*RLlnk10_23-OMlnk10_22*RLlnk10_13
    Plnk15 = POlnk10_13-s.dpt[1,12]
    Plnk25 = RLlnk10_23-s.dpt[2,12]
    Plnk35 = RLlnk10_33-s.dpt[3,12]
    PPlnk5 = Plnk15*Plnk15+Plnk25*Plnk25+Plnk35*Plnk35
    Z5 = sqrt(PPlnk5)
    e15 = Plnk15/Z5
    e25 = Plnk25/Z5
    e35 = Plnk35/Z5
    Zd5 = ORlnk10_13*e15+ORlnk10_23*e25+ORlnk10_33*e35
    ROlnk12_220 = S19*S20
    ROlnk12_320 = -C19*S20
    ROlnk12_820 = -S19*C20
    ROlnk12_920 = C19*C20
    OMlnk12_22 = qd[20]*C19
    OMlnk12_32 = qd[20]*S19
    RLlnk12_13 = s.dpt[1,41]*C20
    RLlnk12_23 = ROlnk12_220*s.dpt[1,41]+s.dpt[2,41]*C19
    RLlnk12_33 = ROlnk12_320*s.dpt[1,41]+s.dpt[2,41]*S19
    POlnk12_13 = RLlnk12_13+s.dpt[1,14]
    ORlnk12_13 = OMlnk12_22*RLlnk12_33-OMlnk12_32*RLlnk12_23
    ORlnk12_23 = -qd[19]*RLlnk12_33+OMlnk12_32*RLlnk12_13
    ORlnk12_33 = qd[19]*RLlnk12_23-OMlnk12_22*RLlnk12_13
    Plnk16 = POlnk12_13-s.dpt[1,13]
    Plnk26 = RLlnk12_23-s.dpt[2,13]
    Plnk36 = RLlnk12_33-s.dpt[3,13]
    PPlnk6 = Plnk16*Plnk16+Plnk26*Plnk26+Plnk36*Plnk36
    Z6 = sqrt(PPlnk6)
    e16 = Plnk16/Z6
    e26 = Plnk26/Z6
    e36 = Plnk36/Z6
    Zd6 = ORlnk12_13*e16+ORlnk12_23*e26+ORlnk12_33*e36
    ROlnk14_220 = S19*S20
    ROlnk14_320 = -C19*S20
    ROlnk14_820 = -S19*C20
    ROlnk14_920 = C19*C20
    OMlnk14_22 = qd[20]*C19
    OMlnk14_32 = qd[20]*S19
    RLlnk14_13 = s.dpt[1,48]*C20
    RLlnk14_23 = ROlnk14_220*s.dpt[1,48]+s.dpt[2,48]*C19
    RLlnk14_33 = ROlnk14_320*s.dpt[1,48]+s.dpt[2,48]*S19
    POlnk14_13 = RLlnk14_13+s.dpt[1,14]
    ORlnk14_13 = OMlnk14_22*RLlnk14_33-OMlnk14_32*RLlnk14_23
    ORlnk14_23 = -qd[19]*RLlnk14_33+OMlnk14_32*RLlnk14_13
    ORlnk14_33 = qd[19]*RLlnk14_23-OMlnk14_22*RLlnk14_13
    Plnk17 = POlnk14_13-s.dpt[1,19]
    Plnk27 = RLlnk14_23-s.dpt[2,19]
    Plnk37 = RLlnk14_33-s.dpt[3,19]
    PPlnk7 = Plnk17*Plnk17+Plnk27*Plnk27+Plnk37*Plnk37
    Z7 = sqrt(PPlnk7)
    e17 = Plnk17/Z7
    e27 = Plnk27/Z7
    e37 = Plnk37/Z7
    Zd7 = ORlnk14_13*e17+ORlnk14_23*e27+ORlnk14_33*e37
    ROlnk16_220 = S19*S20
    ROlnk16_320 = -C19*S20
    ROlnk16_820 = -S19*C20
    ROlnk16_920 = C19*C20
    OMlnk16_22 = qd[20]*C19
    OMlnk16_32 = qd[20]*S19
    RLlnk16_13 = s.dpt[1,47]*C20
    RLlnk16_23 = ROlnk16_220*s.dpt[1,47]+s.dpt[2,47]*C19
    RLlnk16_33 = ROlnk16_320*s.dpt[1,47]+s.dpt[2,47]*S19
    POlnk16_13 = RLlnk16_13+s.dpt[1,14]
    ORlnk16_13 = OMlnk16_22*RLlnk16_33-OMlnk16_32*RLlnk16_23
    ORlnk16_23 = -qd[19]*RLlnk16_33+OMlnk16_32*RLlnk16_13
    ORlnk16_33 = qd[19]*RLlnk16_23-OMlnk16_22*RLlnk16_13
    Plnk18 = POlnk16_13-s.dpt[1,21]
    Plnk28 = RLlnk16_23-s.dpt[2,21]
    Plnk38 = RLlnk16_33-s.dpt[3,21]
    PPlnk8 = Plnk18*Plnk18+Plnk28*Plnk28+Plnk38*Plnk38
    Z8 = sqrt(PPlnk8)
    e18 = Plnk18/Z8
    e28 = Plnk28/Z8
    e38 = Plnk38/Z8
    Zd8 = ORlnk16_13*e18+ORlnk16_23*e28+ORlnk16_33*e38
    ROlnk18_220 = S19*S20
    ROlnk18_320 = -C19*S20
    ROlnk18_820 = -S19*C20
    ROlnk18_920 = C19*C20
    OMlnk18_22 = qd[20]*C19
    OMlnk18_32 = qd[20]*S19
    RLlnk18_13 = s.dpt[1,49]*C20
    RLlnk18_23 = ROlnk18_220*s.dpt[1,49]+s.dpt[2,49]*C19
    RLlnk18_33 = ROlnk18_320*s.dpt[1,49]+s.dpt[2,49]*S19
    POlnk18_13 = RLlnk18_13+s.dpt[1,14]
    ORlnk18_13 = OMlnk18_22*RLlnk18_33-OMlnk18_32*RLlnk18_23
    ORlnk18_23 = -qd[19]*RLlnk18_33+OMlnk18_32*RLlnk18_13
    ORlnk18_33 = qd[19]*RLlnk18_23-OMlnk18_22*RLlnk18_13
    Plnk19 = POlnk18_13-s.dpt[1,20]
    Plnk29 = RLlnk18_23-s.dpt[2,20]
    Plnk39 = RLlnk18_33-s.dpt[3,20]
    PPlnk9 = Plnk19*Plnk19+Plnk29*Plnk29+Plnk39*Plnk39
    Z9 = sqrt(PPlnk9)
    e19 = Plnk19/Z9
    e29 = Plnk29/Z9
    e39 = Plnk39/Z9
    Zd9 = ORlnk18_13*e19+ORlnk18_23*e29+ORlnk18_33*e39
    ROlnk19_220 = S19*S20
    ROlnk19_320 = -C19*S20
    ROlnk19_820 = -S19*C20
    ROlnk19_920 = C19*C20
    OMlnk19_22 = qd[20]*C19
    OMlnk19_32 = qd[20]*S19
    RLlnk19_13 = s.dpt[1,50]*C20
    RLlnk19_23 = ROlnk19_220*s.dpt[1,50]+s.dpt[2,50]*C19
    RLlnk19_33 = ROlnk19_320*s.dpt[1,50]+s.dpt[2,50]*S19
    POlnk19_13 = RLlnk19_13+s.dpt[1,14]
    ORlnk19_13 = OMlnk19_22*RLlnk19_33-OMlnk19_32*RLlnk19_23
    ORlnk19_23 = -qd[19]*RLlnk19_33+OMlnk19_32*RLlnk19_13
    ORlnk19_33 = qd[19]*RLlnk19_23-OMlnk19_22*RLlnk19_13
    Plnk110 = -POlnk19_13+s.dpt[1,22]
    Plnk210 = -RLlnk19_23+s.dpt[2,22]
    Plnk310 = -RLlnk19_33+s.dpt[3,22]
    PPlnk10 = Plnk110*Plnk110+Plnk210*Plnk210+Plnk310*Plnk310
    Z10 = sqrt(PPlnk10)
    e110 = Plnk110/Z10
    e210 = Plnk210/Z10
    e310 = Plnk310/Z10
    Zd10 = -ORlnk19_13*e110-ORlnk19_23*e210-ORlnk19_33*e310

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
    trqlnk6_1_1 = -fPlnk21*s.dpt[3,4]+fPlnk31*s.dpt[2,4]
    trqlnk6_1_2 = fPlnk11*s.dpt[3,4]-fPlnk31*s.dpt[1,4]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,4]+fPlnk21*s.dpt[1,4]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C7+e31*S7)
    fSlnk31 = Flink1*(-e21*S7+e31*C7)
    trqlnk7_1_1 = fSlnk21*s.dpt[3,24]-fSlnk31*s.dpt[2,24]
    trqlnk7_1_2 = -fSlnk11*s.dpt[3,24]
    trqlnk7_1_3 = fSlnk11*s.dpt[2,24]
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11+fPlnk12
    frclnk6_2_2 = fPlnk21+fPlnk22
    frclnk6_2_3 = fPlnk31+fPlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*s.dpt[3,7]+fPlnk32*s.dpt[2,7]
    trqlnk6_2_2 = trqlnk6_1_2+fPlnk12*s.dpt[3,7]-fPlnk32*s.dpt[1,7]
    trqlnk6_2_3 = trqlnk6_1_3-fPlnk12*s.dpt[2,7]+fPlnk22*s.dpt[1,7]
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*(e22*C13+e32*S13)
    fSlnk32 = Flink2*(-e22*S13+e32*C13)
    trqlnk13_2_1 = fSlnk22*s.dpt[3,31]-fSlnk32*s.dpt[2,31]
    trqlnk13_2_2 = -fSlnk12*s.dpt[3,31]
    trqlnk13_2_3 = fSlnk12*s.dpt[2,31]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_1 = fPlnk13+frclnk6_2_1
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*s.dpt[3,8]+fPlnk33*s.dpt[2,8]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*s.dpt[3,8]-fPlnk33*s.dpt[1,8]
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,8]+fPlnk23*s.dpt[1,8]
    fSlnk13 = Flink3*(ROlnk6_220*e23+ROlnk6_320*e33+e13*C20)
    fSlnk23 = Flink3*(e23*C19+e33*S19)
    fSlnk33 = Flink3*(ROlnk6_820*e23+ROlnk6_920*e33+e13*S20)
    trqlnk20_3_1 = -fSlnk33*s.dpt[2,37]
    trqlnk20_3_2 = fSlnk33*s.dpt[1,37]
    trqlnk20_3_3 = fSlnk13*s.dpt[2,37]-fSlnk23*s.dpt[1,37]
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk14+frclnk6_3_1
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*s.dpt[3,9]+fPlnk34*s.dpt[2,9]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*s.dpt[3,9]-fPlnk34*s.dpt[1,9]
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,9]+fPlnk24*s.dpt[1,9]
    fSlnk14 = Flink4*(ROlnk8_220*e24+ROlnk8_320*e34+e14*C20)
    fSlnk24 = Flink4*(e24*C19+e34*S19)
    fSlnk34 = Flink4*(ROlnk8_820*e24+ROlnk8_920*e34+e14*S20)
    frclnk20_4_1 = -fSlnk13-fSlnk14
    frclnk20_4_2 = -fSlnk23-fSlnk24
    frclnk20_4_3 = -fSlnk33-fSlnk34
    trqlnk20_4_1 = trqlnk20_3_1-fSlnk34*s.dpt[2,38]
    trqlnk20_4_2 = trqlnk20_3_2+fSlnk34*s.dpt[1,38]
    trqlnk20_4_3 = trqlnk20_3_3+fSlnk14*s.dpt[2,38]-fSlnk24*s.dpt[1,38]
    fPlnk15 = Flink5*e15
    fPlnk25 = Flink5*e25
    fPlnk35 = Flink5*e35
    frclnk6_5_1 = fPlnk15+frclnk6_4_1
    frclnk6_5_2 = fPlnk25+frclnk6_4_2
    frclnk6_5_3 = fPlnk35+frclnk6_4_3
    trqlnk6_5_1 = trqlnk6_4_1-fPlnk25*s.dpt[3,12]+fPlnk35*s.dpt[2,12]
    trqlnk6_5_2 = trqlnk6_4_2+fPlnk15*s.dpt[3,12]-fPlnk35*s.dpt[1,12]
    trqlnk6_5_3 = trqlnk6_4_3-fPlnk15*s.dpt[2,12]+fPlnk25*s.dpt[1,12]
    fSlnk15 = Flink5*(ROlnk10_220*e25+ROlnk10_320*e35+e15*C20)
    fSlnk25 = Flink5*(e25*C19+e35*S19)
    fSlnk35 = Flink5*(ROlnk10_820*e25+ROlnk10_920*e35+e15*S20)
    frclnk20_5_1 = -fSlnk15+frclnk20_4_1
    frclnk20_5_2 = -fSlnk25+frclnk20_4_2
    frclnk20_5_3 = -fSlnk35+frclnk20_4_3
    trqlnk20_5_1 = trqlnk20_4_1-fSlnk35*s.dpt[2,40]
    trqlnk20_5_2 = trqlnk20_4_2+fSlnk35*s.dpt[1,40]
    trqlnk20_5_3 = trqlnk20_4_3+fSlnk15*s.dpt[2,40]-fSlnk25*s.dpt[1,40]
    fPlnk16 = Flink6*e16
    fPlnk26 = Flink6*e26
    fPlnk36 = Flink6*e36
    frclnk6_6_1 = fPlnk16+frclnk6_5_1
    frclnk6_6_2 = fPlnk26+frclnk6_5_2
    frclnk6_6_3 = fPlnk36+frclnk6_5_3
    trqlnk6_6_1 = trqlnk6_5_1-fPlnk26*s.dpt[3,13]+fPlnk36*s.dpt[2,13]
    trqlnk6_6_2 = trqlnk6_5_2+fPlnk16*s.dpt[3,13]-fPlnk36*s.dpt[1,13]
    trqlnk6_6_3 = trqlnk6_5_3-fPlnk16*s.dpt[2,13]+fPlnk26*s.dpt[1,13]
    fSlnk16 = Flink6*(ROlnk12_220*e26+ROlnk12_320*e36+e16*C20)
    fSlnk26 = Flink6*(e26*C19+e36*S19)
    fSlnk36 = Flink6*(ROlnk12_820*e26+ROlnk12_920*e36+e16*S20)
    frclnk20_6_1 = -fSlnk16+frclnk20_5_1
    frclnk20_6_2 = -fSlnk26+frclnk20_5_2
    frclnk20_6_3 = -fSlnk36+frclnk20_5_3
    trqlnk20_6_1 = trqlnk20_5_1-fSlnk36*s.dpt[2,41]
    trqlnk20_6_2 = trqlnk20_5_2+fSlnk36*s.dpt[1,41]
    trqlnk20_6_3 = trqlnk20_5_3+fSlnk16*s.dpt[2,41]-fSlnk26*s.dpt[1,41]
    fPlnk17 = Flink7*e17
    fPlnk27 = Flink7*e27
    fPlnk37 = Flink7*e37
    frclnk6_7_1 = fPlnk17+frclnk6_6_1
    frclnk6_7_2 = fPlnk27+frclnk6_6_2
    frclnk6_7_3 = fPlnk37+frclnk6_6_3
    trqlnk6_7_1 = trqlnk6_6_1-fPlnk27*s.dpt[3,19]+fPlnk37*s.dpt[2,19]
    trqlnk6_7_2 = trqlnk6_6_2+fPlnk17*s.dpt[3,19]-fPlnk37*s.dpt[1,19]
    trqlnk6_7_3 = trqlnk6_6_3-fPlnk17*s.dpt[2,19]+fPlnk27*s.dpt[1,19]
    fSlnk17 = Flink7*(ROlnk14_220*e27+ROlnk14_320*e37+e17*C20)
    fSlnk27 = Flink7*(e27*C19+e37*S19)
    fSlnk37 = Flink7*(ROlnk14_820*e27+ROlnk14_920*e37+e17*S20)
    frclnk20_7_1 = -fSlnk17+frclnk20_6_1
    frclnk20_7_2 = -fSlnk27+frclnk20_6_2
    frclnk20_7_3 = -fSlnk37+frclnk20_6_3
    trqlnk20_7_1 = trqlnk20_6_1-fSlnk37*s.dpt[2,48]
    trqlnk20_7_2 = trqlnk20_6_2+fSlnk37*s.dpt[1,48]
    trqlnk20_7_3 = trqlnk20_6_3+fSlnk17*s.dpt[2,48]-fSlnk27*s.dpt[1,48]
    fPlnk18 = Flink8*e18
    fPlnk28 = Flink8*e28
    fPlnk38 = Flink8*e38
    frclnk6_8_1 = fPlnk18+frclnk6_7_1
    frclnk6_8_2 = fPlnk28+frclnk6_7_2
    frclnk6_8_3 = fPlnk38+frclnk6_7_3
    trqlnk6_8_1 = trqlnk6_7_1-fPlnk28*s.dpt[3,21]+fPlnk38*s.dpt[2,21]
    trqlnk6_8_2 = trqlnk6_7_2+fPlnk18*s.dpt[3,21]-fPlnk38*s.dpt[1,21]
    trqlnk6_8_3 = trqlnk6_7_3-fPlnk18*s.dpt[2,21]+fPlnk28*s.dpt[1,21]
    fSlnk18 = Flink8*(ROlnk16_220*e28+ROlnk16_320*e38+e18*C20)
    fSlnk28 = Flink8*(e28*C19+e38*S19)
    fSlnk38 = Flink8*(ROlnk16_820*e28+ROlnk16_920*e38+e18*S20)
    frclnk20_8_1 = -fSlnk18+frclnk20_7_1
    frclnk20_8_2 = -fSlnk28+frclnk20_7_2
    frclnk20_8_3 = -fSlnk38+frclnk20_7_3
    trqlnk20_8_1 = trqlnk20_7_1-fSlnk38*s.dpt[2,47]
    trqlnk20_8_2 = trqlnk20_7_2+fSlnk38*s.dpt[1,47]
    trqlnk20_8_3 = trqlnk20_7_3+fSlnk18*s.dpt[2,47]-fSlnk28*s.dpt[1,47]
    fPlnk19 = Flink9*e19
    fPlnk29 = Flink9*e29
    fPlnk39 = Flink9*e39
    frclnk6_9_1 = fPlnk19+frclnk6_8_1
    frclnk6_9_2 = fPlnk29+frclnk6_8_2
    frclnk6_9_3 = fPlnk39+frclnk6_8_3
    trqlnk6_9_1 = trqlnk6_8_1-fPlnk29*s.dpt[3,20]+fPlnk39*s.dpt[2,20]
    trqlnk6_9_2 = trqlnk6_8_2+fPlnk19*s.dpt[3,20]-fPlnk39*s.dpt[1,20]
    trqlnk6_9_3 = trqlnk6_8_3-fPlnk19*s.dpt[2,20]+fPlnk29*s.dpt[1,20]
    fSlnk19 = Flink9*(ROlnk18_220*e29+ROlnk18_320*e39+e19*C20)
    fSlnk29 = Flink9*(e29*C19+e39*S19)
    fSlnk39 = Flink9*(ROlnk18_820*e29+ROlnk18_920*e39+e19*S20)
    frclnk20_9_1 = -fSlnk19+frclnk20_8_1
    frclnk20_9_2 = -fSlnk29+frclnk20_8_2
    frclnk20_9_3 = -fSlnk39+frclnk20_8_3
    trqlnk20_9_1 = trqlnk20_8_1-fSlnk39*s.dpt[2,49]
    trqlnk20_9_2 = trqlnk20_8_2+fSlnk39*s.dpt[1,49]
    trqlnk20_9_3 = trqlnk20_8_3+fSlnk19*s.dpt[2,49]-fSlnk29*s.dpt[1,49]
    fPlnk110 = Flink10*(ROlnk19_220*e210+ROlnk19_320*e310+e110*C20)
    fPlnk210 = Flink10*(e210*C19+e310*S19)
    fPlnk310 = Flink10*(ROlnk19_820*e210+ROlnk19_920*e310+e110*S20)
    frclnk20_10_1 = fPlnk110+frclnk20_9_1
    frclnk20_10_2 = fPlnk210+frclnk20_9_2
    frclnk20_10_3 = fPlnk310+frclnk20_9_3
    trqlnk20_10_1 = trqlnk20_9_1+fPlnk310*s.dpt[2,50]
    trqlnk20_10_2 = trqlnk20_9_2-fPlnk310*s.dpt[1,50]
    trqlnk20_10_3 = trqlnk20_9_3-fPlnk110*s.dpt[2,50]+fPlnk210*s.dpt[1,50]
    fSlnk110 = Flink10*e110
    fSlnk210 = Flink10*e210
    fSlnk310 = Flink10*e310
    frclnk6_10_1 = -fSlnk110+frclnk6_9_1
    frclnk6_10_2 = -fSlnk210+frclnk6_9_2
    frclnk6_10_3 = -fSlnk310+frclnk6_9_3
    trqlnk6_10_1 = trqlnk6_9_1+fSlnk210*s.dpt[3,22]-fSlnk310*s.dpt[2,22]
    trqlnk6_10_2 = trqlnk6_9_2-fSlnk110*s.dpt[3,22]+fSlnk310*s.dpt[1,22]
    trqlnk6_10_3 = trqlnk6_9_3+fSlnk110*s.dpt[2,22]-fSlnk210*s.dpt[1,22]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_10_1
    frc[2,6] = s.frc[2,6]+frclnk6_10_2
    frc[3,6] = s.frc[3,6]+frclnk6_10_3
    trq[1,6] = s.trq[1,6]+trqlnk6_10_1
    trq[2,6] = s.trq[2,6]+trqlnk6_10_2
    trq[3,6] = s.trq[3,6]+trqlnk6_10_3
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


