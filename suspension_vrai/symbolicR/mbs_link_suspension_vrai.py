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
#	==> Generation Date: Mon Mar 24 14:14:28 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 32
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
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
 
# Augmented Joint Position Vectors

    Dz191 = q[19]+s.dpt[1,7]
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,18]*C8-s.dpt[3,18]*S8
    RLlnk2_32 = s.dpt[2,18]*S8+s.dpt[3,18]*C8
    POlnk2_22 = RLlnk2_22+s.dpt[2,3]
    ORlnk2_22 = -qd[8]*RLlnk2_32
    ORlnk2_32 = qd[8]*RLlnk2_22
    Plnk11 = -s.dpt[1,1]+s.dpt[1,3]
    Plnk21 = POlnk2_22-s.dpt[2,1]
    Plnk31 = RLlnk2_32-s.dpt[3,1]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk3_22 = s.dpt[2,24]*C14-s.dpt[3,24]*S14
    RLlnk3_32 = s.dpt[2,24]*S14+s.dpt[3,24]*C14
    POlnk3_22 = RLlnk3_22+s.dpt[2,6]
    ORlnk3_22 = -qd[14]*RLlnk3_32
    ORlnk3_32 = qd[14]*RLlnk3_22
    Plnk12 = s.dpt[1,4]-s.dpt[1,6]
    Plnk22 = -POlnk3_22+s.dpt[2,4]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+RLlnk3_32*RLlnk3_32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = -RLlnk3_32/Z2
    Zd2 = -ORlnk3_22*e22-ORlnk3_32*e32
    ROlnk5_123 = C22*C23
    ROlnk5_223 = S22*C23
    ROlnk5_723 = C22*S23
    ROlnk5_823 = S22*S23
    ROlnk5_424 = ROlnk5_723*S24-S22*C24
    ROlnk5_524 = ROlnk5_823*S24+C22*C24
    ROlnk5_624 = C23*S24
    ROlnk5_724 = ROlnk5_723*C24+S22*S24
    ROlnk5_824 = ROlnk5_823*C24-C22*S24
    ROlnk5_924 = C23*C24
    OMlnk5_15 = -qd[23]*S22
    OMlnk5_25 = qd[23]*C22
    OMlnk5_16 = OMlnk5_15+qd[24]*ROlnk5_123
    OMlnk5_26 = OMlnk5_25+qd[24]*ROlnk5_223
    OMlnk5_36 = qd[22]-qd[24]*S23
    RLlnk5_17 = ROlnk5_123*s.dpt[1,28]+ROlnk5_424*s.dpt[2,28]
    RLlnk5_27 = ROlnk5_223*s.dpt[1,28]+ROlnk5_524*s.dpt[2,28]
    RLlnk5_37 = ROlnk5_624*s.dpt[2,28]-s.dpt[1,28]*S23
    POlnk5_17 = RLlnk5_17+Dz191
    POlnk5_27 = q[20]+RLlnk5_27
    POlnk5_37 = q[21]+RLlnk5_37
    ORlnk5_17 = OMlnk5_26*RLlnk5_37-OMlnk5_36*RLlnk5_27
    ORlnk5_27 = -OMlnk5_16*RLlnk5_37+OMlnk5_36*RLlnk5_17
    ORlnk5_37 = OMlnk5_16*RLlnk5_27-OMlnk5_26*RLlnk5_17
    VIlnk5_17 = qd[19]+ORlnk5_17
    VIlnk5_27 = qd[20]+ORlnk5_27
    VIlnk5_37 = qd[21]+ORlnk5_37
    Plnk13 = -POlnk5_17+s.dpt[1,9]
    Plnk23 = -POlnk5_27+s.dpt[2,9]
    PPlnk3 = POlnk5_37*POlnk5_37+Plnk13*Plnk13+Plnk23*Plnk23
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = -POlnk5_37/Z3
    Zd3 = -VIlnk5_17*e13-VIlnk5_27*e23-VIlnk5_37*e33
    ROlnk7_123 = C22*C23
    ROlnk7_223 = S22*C23
    ROlnk7_723 = C22*S23
    ROlnk7_823 = S22*S23
    ROlnk7_424 = ROlnk7_723*S24-S22*C24
    ROlnk7_524 = ROlnk7_823*S24+C22*C24
    ROlnk7_624 = C23*S24
    ROlnk7_724 = ROlnk7_723*C24+S22*S24
    ROlnk7_824 = ROlnk7_823*C24-C22*S24
    ROlnk7_924 = C23*C24
    OMlnk7_15 = -qd[23]*S22
    OMlnk7_25 = qd[23]*C22
    OMlnk7_16 = OMlnk7_15+qd[24]*ROlnk7_123
    OMlnk7_26 = OMlnk7_25+qd[24]*ROlnk7_223
    OMlnk7_36 = qd[22]-qd[24]*S23
    RLlnk7_17 = ROlnk7_123*s.dpt[1,30]+ROlnk7_424*s.dpt[2,30]
    RLlnk7_27 = ROlnk7_223*s.dpt[1,30]+ROlnk7_524*s.dpt[2,30]
    RLlnk7_37 = ROlnk7_624*s.dpt[2,30]-s.dpt[1,30]*S23
    POlnk7_17 = RLlnk7_17+Dz191
    POlnk7_27 = q[20]+RLlnk7_27
    POlnk7_37 = q[21]+RLlnk7_37
    ORlnk7_17 = OMlnk7_26*RLlnk7_37-OMlnk7_36*RLlnk7_27
    ORlnk7_27 = -OMlnk7_16*RLlnk7_37+OMlnk7_36*RLlnk7_17
    ORlnk7_37 = OMlnk7_16*RLlnk7_27-OMlnk7_26*RLlnk7_17
    VIlnk7_17 = qd[19]+ORlnk7_17
    VIlnk7_27 = qd[20]+ORlnk7_27
    VIlnk7_37 = qd[21]+ORlnk7_37
    Plnk14 = -POlnk7_17+s.dpt[1,8]
    Plnk24 = -POlnk7_27+s.dpt[2,8]
    PPlnk4 = POlnk7_37*POlnk7_37+Plnk14*Plnk14+Plnk24*Plnk24
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = -POlnk7_37/Z4
    Zd4 = -VIlnk7_17*e14-VIlnk7_27*e24-VIlnk7_37*e34
    ROlnk10_123 = C22*C23
    ROlnk10_223 = S22*C23
    ROlnk10_723 = C22*S23
    ROlnk10_823 = S22*S23
    ROlnk10_424 = ROlnk10_723*S24-S22*C24
    ROlnk10_524 = ROlnk10_823*S24+C22*C24
    ROlnk10_624 = C23*S24
    ROlnk10_724 = ROlnk10_723*C24+S22*S24
    ROlnk10_824 = ROlnk10_823*C24-C22*S24
    ROlnk10_924 = C23*C24
    OMlnk10_15 = -qd[23]*S22
    OMlnk10_25 = qd[23]*C22
    OMlnk10_16 = OMlnk10_15+qd[24]*ROlnk10_123
    OMlnk10_26 = OMlnk10_25+qd[24]*ROlnk10_223
    OMlnk10_36 = qd[22]-qd[24]*S23
    RLlnk10_17 = ROlnk10_123*s.dpt[1,33]+ROlnk10_424*s.dpt[2,33]
    RLlnk10_27 = ROlnk10_223*s.dpt[1,33]+ROlnk10_524*s.dpt[2,33]
    RLlnk10_37 = ROlnk10_624*s.dpt[2,33]-s.dpt[1,33]*S23
    POlnk10_17 = RLlnk10_17+Dz191
    POlnk10_27 = q[20]+RLlnk10_27
    POlnk10_37 = q[21]+RLlnk10_37
    ORlnk10_17 = OMlnk10_26*RLlnk10_37-OMlnk10_36*RLlnk10_27
    ORlnk10_27 = -OMlnk10_16*RLlnk10_37+OMlnk10_36*RLlnk10_17
    ORlnk10_37 = OMlnk10_16*RLlnk10_27-OMlnk10_26*RLlnk10_17
    VIlnk10_17 = qd[19]+ORlnk10_17
    VIlnk10_27 = qd[20]+ORlnk10_27
    VIlnk10_37 = qd[21]+ORlnk10_37
    Plnk15 = POlnk10_17-s.dpt[1,11]
    PPlnk5 = POlnk10_27*POlnk10_27+POlnk10_37*POlnk10_37+Plnk15*Plnk15
    Z5 = sqrt(PPlnk5)
    e15 = Plnk15/Z5
    e25 = POlnk10_27/Z5
    e35 = POlnk10_37/Z5
    Zd5 = VIlnk10_17*e15+VIlnk10_27*e25+VIlnk10_37*e35
    ROlnk12_123 = C22*C23
    ROlnk12_223 = S22*C23
    ROlnk12_723 = C22*S23
    ROlnk12_823 = S22*S23
    ROlnk12_424 = ROlnk12_723*S24-S22*C24
    ROlnk12_524 = ROlnk12_823*S24+C22*C24
    ROlnk12_624 = C23*S24
    ROlnk12_724 = ROlnk12_723*C24+S22*S24
    ROlnk12_824 = ROlnk12_823*C24-C22*S24
    ROlnk12_924 = C23*C24
    OMlnk12_15 = -qd[23]*S22
    OMlnk12_25 = qd[23]*C22
    OMlnk12_16 = OMlnk12_15+qd[24]*ROlnk12_123
    OMlnk12_26 = OMlnk12_25+qd[24]*ROlnk12_223
    OMlnk12_36 = qd[22]-qd[24]*S23
    RLlnk12_17 = ROlnk12_123*s.dpt[1,31]+ROlnk12_424*s.dpt[2,31]
    RLlnk12_27 = ROlnk12_223*s.dpt[1,31]+ROlnk12_524*s.dpt[2,31]
    RLlnk12_37 = ROlnk12_624*s.dpt[2,31]-s.dpt[1,31]*S23
    POlnk12_17 = RLlnk12_17+Dz191
    POlnk12_27 = q[20]+RLlnk12_27
    POlnk12_37 = q[21]+RLlnk12_37
    ORlnk12_17 = OMlnk12_26*RLlnk12_37-OMlnk12_36*RLlnk12_27
    ORlnk12_27 = -OMlnk12_16*RLlnk12_37+OMlnk12_36*RLlnk12_17
    ORlnk12_37 = OMlnk12_16*RLlnk12_27-OMlnk12_26*RLlnk12_17
    VIlnk12_17 = qd[19]+ORlnk12_17
    VIlnk12_27 = qd[20]+ORlnk12_27
    VIlnk12_37 = qd[21]+ORlnk12_37
    Plnk16 = POlnk12_17-s.dpt[1,10]
    PPlnk6 = POlnk12_27*POlnk12_27+POlnk12_37*POlnk12_37+Plnk16*Plnk16
    Z6 = sqrt(PPlnk6)
    e16 = Plnk16/Z6
    e26 = POlnk12_27/Z6
    e36 = POlnk12_37/Z6
    Zd6 = VIlnk12_17*e16+VIlnk12_27*e26+VIlnk12_37*e36

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
    Flink5 = s.user_LinkForces(Z5,Zd5,s,tsim,5)
    Flink6 = s.user_LinkForces(Z6,Zd6,s,tsim,6)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*s.dpt[3,1]+fPlnk31*s.dpt[2,1]
    trqlnk6_1_2 = fPlnk11*s.dpt[3,1]-fPlnk31*s.dpt[1,1]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,1]+fPlnk21*s.dpt[1,1]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C8+e31*S8)
    fSlnk31 = Flink1*(-e21*S8+e31*C8)
    trqlnk8_1_1 = fSlnk21*s.dpt[3,18]-fSlnk31*s.dpt[2,18]
    trqlnk8_1_2 = -fSlnk11*s.dpt[3,18]
    trqlnk8_1_3 = fSlnk11*s.dpt[2,18]
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*(e22*C14+e32*S14)
    fPlnk32 = Flink2*(-e22*S14+e32*C14)
    trqlnk14_2_1 = -fPlnk22*s.dpt[3,24]+fPlnk32*s.dpt[2,24]
    trqlnk14_2_2 = fPlnk12*s.dpt[3,24]
    trqlnk14_2_3 = -fPlnk12*s.dpt[2,24]
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11-fSlnk12
    frclnk6_2_2 = fPlnk21-fSlnk22
    frclnk6_2_3 = fPlnk31-fSlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fSlnk32*s.dpt[2,4]
    trqlnk6_2_2 = trqlnk6_1_2+fSlnk32*s.dpt[1,4]
    trqlnk6_2_3 = trqlnk6_1_3+fSlnk12*s.dpt[2,4]-fSlnk22*s.dpt[1,4]
    fPlnk13 = Flink3*(ROlnk5_123*e13+ROlnk5_223*e23-e33*S23)
    fPlnk23 = Flink3*(ROlnk5_424*e13+ROlnk5_524*e23+ROlnk5_624*e33)
    fPlnk33 = Flink3*(ROlnk5_724*e13+ROlnk5_824*e23+ROlnk5_924*e33)
    trqlnk24_3_1 = fPlnk33*s.dpt[2,28]
    trqlnk24_3_2 = -fPlnk33*s.dpt[1,28]
    trqlnk24_3_3 = -fPlnk13*s.dpt[2,28]+fPlnk23*s.dpt[1,28]
    fSlnk13 = Flink3*e13
    fSlnk23 = Flink3*e23
    fSlnk33 = Flink3*e33
    frclnk6_3_1 = -fSlnk13+frclnk6_2_1
    frclnk6_3_2 = -fSlnk23+frclnk6_2_2
    frclnk6_3_3 = -fSlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fSlnk33*s.dpt[2,9]
    trqlnk6_3_2 = trqlnk6_2_2+fSlnk33*s.dpt[1,9]
    trqlnk6_3_3 = trqlnk6_2_3+fSlnk13*s.dpt[2,9]-fSlnk23*s.dpt[1,9]
    fPlnk14 = Flink4*(ROlnk7_123*e14+ROlnk7_223*e24-e34*S23)
    fPlnk24 = Flink4*(ROlnk7_424*e14+ROlnk7_524*e24+ROlnk7_624*e34)
    fPlnk34 = Flink4*(ROlnk7_724*e14+ROlnk7_824*e24+ROlnk7_924*e34)
    frclnk24_4_1 = fPlnk13+fPlnk14
    frclnk24_4_2 = fPlnk23+fPlnk24
    frclnk24_4_3 = fPlnk33+fPlnk34
    trqlnk24_4_1 = trqlnk24_3_1+fPlnk34*s.dpt[2,30]
    trqlnk24_4_2 = trqlnk24_3_2-fPlnk34*s.dpt[1,30]
    trqlnk24_4_3 = trqlnk24_3_3-fPlnk14*s.dpt[2,30]+fPlnk24*s.dpt[1,30]
    fSlnk14 = Flink4*e14
    fSlnk24 = Flink4*e24
    fSlnk34 = Flink4*e34
    frclnk6_4_1 = -fSlnk14+frclnk6_3_1
    frclnk6_4_2 = -fSlnk24+frclnk6_3_2
    frclnk6_4_3 = -fSlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fSlnk34*s.dpt[2,8]
    trqlnk6_4_2 = trqlnk6_3_2+fSlnk34*s.dpt[1,8]
    trqlnk6_4_3 = trqlnk6_3_3+fSlnk14*s.dpt[2,8]-fSlnk24*s.dpt[1,8]
    fPlnk15 = Flink5*e15
    fPlnk25 = Flink5*e25
    fPlnk35 = Flink5*e35
    frclnk6_5_1 = fPlnk15+frclnk6_4_1
    frclnk6_5_2 = fPlnk25+frclnk6_4_2
    frclnk6_5_3 = fPlnk35+frclnk6_4_3
    trqlnk6_5_2 = trqlnk6_4_2-fPlnk35*s.dpt[1,11]
    trqlnk6_5_3 = trqlnk6_4_3+fPlnk25*s.dpt[1,11]
    fSlnk15 = Flink5*(ROlnk10_123*e15+ROlnk10_223*e25-e35*S23)
    fSlnk25 = Flink5*(ROlnk10_424*e15+ROlnk10_524*e25+ROlnk10_624*e35)
    fSlnk35 = Flink5*(ROlnk10_724*e15+ROlnk10_824*e25+ROlnk10_924*e35)
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
    trqlnk6_6_2 = trqlnk6_5_2-fPlnk36*s.dpt[1,10]
    trqlnk6_6_3 = trqlnk6_5_3+fPlnk26*s.dpt[1,10]
    fSlnk16 = Flink6*(ROlnk12_123*e16+ROlnk12_223*e26-e36*S23)
    fSlnk26 = Flink6*(ROlnk12_424*e16+ROlnk12_524*e26+ROlnk12_624*e36)
    fSlnk36 = Flink6*(ROlnk12_724*e16+ROlnk12_824*e26+ROlnk12_924*e36)
    frclnk24_6_1 = -fSlnk16+frclnk24_5_1
    frclnk24_6_2 = -fSlnk26+frclnk24_5_2
    frclnk24_6_3 = -fSlnk36+frclnk24_5_3
    trqlnk24_6_1 = trqlnk24_5_1-fSlnk36*s.dpt[2,31]
    trqlnk24_6_2 = trqlnk24_5_2+fSlnk36*s.dpt[1,31]
    trqlnk24_6_3 = trqlnk24_5_3+fSlnk16*s.dpt[2,31]-fSlnk26*s.dpt[1,31]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_6_1
    frc[2,6] = s.frc[2,6]+frclnk6_6_2
    frc[3,6] = s.frc[3,6]+frclnk6_6_3
    trq[1,6] = s.trq[1,6]+trqlnk6_4_1
    trq[2,6] = s.trq[2,6]+trqlnk6_6_2
    trq[3,6] = s.trq[3,6]+trqlnk6_6_3
    frc[1,8] = s.frc[1,8]-fSlnk11
    frc[2,8] = s.frc[2,8]-fSlnk21
    frc[3,8] = s.frc[3,8]-fSlnk31
    trq[1,8] = s.trq[1,8]+trqlnk8_1_1
    trq[2,8] = s.trq[2,8]+trqlnk8_1_2
    trq[3,8] = s.trq[3,8]+trqlnk8_1_3
    frc[1,14] = s.frc[1,14]+fPlnk12
    frc[2,14] = s.frc[2,14]+fPlnk22
    frc[3,14] = s.frc[3,14]+fPlnk32
    trq[1,14] = s.trq[1,14]+trqlnk14_2_1
    trq[2,14] = s.trq[2,14]+trqlnk14_2_2
    trq[3,14] = s.trq[3,14]+trqlnk14_2_3
    frc[1,24] = s.frc[1,24]+frclnk24_6_1
    frc[2,24] = s.frc[2,24]+frclnk24_6_2
    frc[3,24] = s.frc[3,24]+frclnk24_6_3
    trq[1,24] = s.trq[1,24]+trqlnk24_6_1
    trq[2,24] = s.trq[2,24]+trqlnk24_6_2
    trq[3,24] = s.trq[3,24]+trqlnk24_6_3
 
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

# Number of continuation lines = 0


