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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    ALPHA24 = qdd[2]*C4+ALPHA33*S4
    ALPHA34 = -qdd[2]*S4+ALPHA33*C4
    OM15 = qd[4]*C5
    OM35 = qd[4]*S5
    OMp15 = -qd[4]*qd[5]*S5+qdd[4]*C5
    OMp35 = qd[4]*qd[5]*C5+qdd[4]*S5
    ALPHA15 = qdd[1]*C5-ALPHA34*S5
    ALPHA35 = qdd[1]*S5+ALPHA34*C5
    OM16 = qd[5]*S6+OM15*C6
    OM26 = qd[5]*C6-OM15*S6
    OM36 = qd[6]+OM35
    OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15)
    OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6])
    OMp36 = qdd[6]+OMp35
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BETA26 = BS26-OMp36
    BETA46 = BS26+OMp36
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA15*C6+ALPHA24*S6
    ALPHA26 = -ALPHA15*S6+ALPHA24*C6
    OM17 = OM16*C7+OM26*S7
    OM27 = -OM16*S7+OM26*C7
    OM37 = qd[7]+OM36
    OMp17 = C7*(OMp16+qd[7]*OM26)+S7*(OMp26-qd[7]*OM16)
    OMp27 = C7*(OMp26-qd[7]*OM16)-S7*(OMp16+qd[7]*OM26)
    OMp37 = qdd[7]+OMp36
    ALPHA17 = C7*(ALPHA16+BETA26*s.dpt[2,1]+BS16*s.dpt[1,1])+S7*(ALPHA26+BETA46*s.dpt[1,1]+BS56*s.dpt[2,1])
    ALPHA27 = C7*(ALPHA26+BETA46*s.dpt[1,1]+BS56*s.dpt[2,1])-S7*(ALPHA16+BETA26*s.dpt[2,1]+BS16*s.dpt[1,1])
    ALPHA37 = ALPHA35+BETA76*s.dpt[1,1]+BETA86*s.dpt[2,1]
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17)
    OMp28 = qdd[8]+OMp27
    OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37)
    ALPHA18 = ALPHA17*C8-ALPHA37*S8
    ALPHA38 = ALPHA17*S8+ALPHA37*C8
    OM19 = OM16*C9-OM36*S9
    OM29 = qd[9]+OM26
    OM39 = OM16*S9+OM36*C9
    OMp19 = C9*(OMp16-qd[9]*OM36)-S9*(OMp36+qd[9]*OM16)
    OMp29 = qdd[9]+OMp26
    OMp39 = C9*(OMp36+qd[9]*OM16)+S9*(OMp16-qd[9]*OM36)
    ALPHA19 = C9*(ALPHA16+BETA26*s.dpt[2,2]+BS16*s.dpt[1,2])-S9*(ALPHA35+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2])
    ALPHA29 = ALPHA26+BETA46*s.dpt[1,2]+BS56*s.dpt[2,2]
    ALPHA39 = C9*(ALPHA35+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2])+S9*(ALPHA16+BETA26*s.dpt[2,2]+BS16*s.dpt[1,2])
    OM110 = OM16*C10-OM36*S10
    OM210 = qd[10]+OM26
    OM310 = OM16*S10+OM36*C10
    OMp110 = C10*(OMp16-qd[10]*OM36)-S10*(OMp36+qd[10]*OM16)
    OMp210 = qdd[10]+OMp26
    OMp310 = C10*(OMp36+qd[10]*OM16)+S10*(OMp16-qd[10]*OM36)
    ALPHA110 = C10*(ALPHA16+BETA26*s.dpt[2,3]+BS16*s.dpt[1,3])-S10*(ALPHA35+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])
    ALPHA210 = ALPHA26+BETA46*s.dpt[1,3]+BS56*s.dpt[2,3]
    ALPHA310 = C10*(ALPHA35+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])+S10*(ALPHA16+BETA26*s.dpt[2,3]+BS16*s.dpt[1,3])
    OM111 = OM16*C11+OM26*S11
    OM211 = -OM16*S11+OM26*C11
    OM311 = qd[11]+OM36
    OMp111 = C11*(OMp16+qd[11]*OM26)+S11*(OMp26-qd[11]*OM16)
    OMp211 = C11*(OMp26-qd[11]*OM16)-S11*(OMp16+qd[11]*OM26)
    OMp311 = qdd[11]+OMp36
    ALPHA111 = C11*(ALPHA16+BETA26*s.dpt[2,4]+BS16*s.dpt[1,4])+S11*(ALPHA26+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])
    ALPHA211 = C11*(ALPHA26+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])-S11*(ALPHA16+BETA26*s.dpt[2,4]+BS16*s.dpt[1,4])
    ALPHA311 = ALPHA35+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4]
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111)
    OMp212 = qdd[12]+OMp211
    OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311)
    ALPHA112 = ALPHA111*C12-ALPHA311*S12
    ALPHA312 = ALPHA111*S12+ALPHA311*C12
 
# Backward Dynamics

    Fs112 = -s.frc[1,12]+s.m[12]*ALPHA112
    Fs212 = -s.frc[2,12]+s.m[12]*ALPHA211
    Fs312 = -s.frc[3,12]+s.m[12]*ALPHA312
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312
    Cq212 = -s.trq[2,12]+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312
    Fq111 = Fs112*C12+Fs312*S12
    Fq311 = -Fs112*S12+Fs312*C12
    Cq111 = Cq112*C12+Cq312*S12
    Cq311 = -Cq112*S12+Cq312*C12
    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA310
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310
    Fs19 = -s.frc[1,9]+s.m[9]*ALPHA19
    Fs29 = -s.frc[2,9]+s.m[9]*ALPHA29
    Fs39 = -s.frc[3,9]+s.m[9]*ALPHA39
    Cq19 = -s.trq[1,9]+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39
    Cq29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39
    Cq39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39
    Fs18 = -s.frc[1,8]+s.m[8]*ALPHA18
    Fs28 = -s.frc[2,8]+s.m[8]*ALPHA27
    Fs38 = -s.frc[3,8]+s.m[8]*ALPHA38
    Cq18 = -s.trq[1,8]+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38
    Cq28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38
    Fq17 = Fs18*C8+Fs38*S8
    Fq37 = -Fs18*S8+Fs38*C8
    Cq17 = Cq18*C8+Cq38*S8
    Cq37 = -Cq18*S8+Cq38*C8
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA16
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA26
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA35
    Fq16 = Fs16+Fq111*C11+Fq17*C7+Fs110*C10+Fs19*C9-Fs212*S11-Fs28*S7+Fs310*S10+Fs39*S9
    Fq26 = Fs210+Fs26+Fs29+Fq111*S11+Fq17*S7+Fs212*C11+Fs28*C7
    Fq36 = Fq311+Fq37+Fs36-Fs110*S10-Fs19*S9+Fs310*C10+Fs39*C9
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq110*C10+Cq111*C11+Cq17*C7+Cq19*C9-Cq212*S11-Cq28*S7+Cq310*S10+Cq39*S9+Fq311*s.dpt[2,4]+Fq37*s.dpt[2,1]+s.dpt[2,2]*(-Fs19*S9+Fs39*C9)+s.dpt[2,3]*(-Fs110*S10+Fs310*C10)
    Cq26 = -s.trq[2,6]+Cq210+Cq29+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq111*S11+Cq17*S7+Cq212*C11+Cq28*C7-Fq311*s.dpt[1,4]-Fq37*s.dpt[1,1]-s.dpt[1,2]*(-Fs19*S9+Fs39*C9)-s.dpt[1,3]*(-Fs110*S10+Fs310*C10)
    Cq36 = -s.trq[3,6]+Cq311+Cq37-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq110*S10-Cq19*S9+Cq310*C10+Cq39*C9+Fs210*s.dpt[1,3]+Fs29*s.dpt[1,2]+s.dpt[1,1]*(Fq17*S7+Fs28*C7)+s.dpt[1,4]*(Fq111*S11+Fs212*C11)-s.dpt[2,1]*(Fq17*C7-Fs28*S7)-s.dpt[2,2]*(Fs19*C9+Fs39*S9)-s.dpt[2,3]*(Fs110*C10+Fs310*S10)-s.dpt[2,4]*(Fq111*C11-Fs212*S11)
    Fq15 = Fq16*C6-Fq26*S6
    Fq25 = Fq16*S6+Fq26*C6
    Cq15 = Cq16*C6-Cq26*S6
    Cq25 = Cq16*S6+Cq26*C6
    Fq14 = Fq15*C5+Fq36*S5
    Fq34 = -Fq15*S5+Fq36*C5
    Cq14 = Cq15*C5+Cq36*S5
    Fq23 = Fq25*C4-Fq34*S4
    Fq33 = Fq25*S4+Fq34*C4
 
# Symbolic model output

    Qq[1] = Fq14
    Qq[2] = Fq23
    Qq[3] = Fq33
    Qq[4] = Cq14
    Qq[5] = Cq25
    Qq[6] = Cq36
    Qq[7] = Cq37
    Qq[8] = Cq28
    Qq[9] = Cq29
    Qq[10] = Cq210
    Qq[11] = Cq311
    Qq[12] = Cq212

# Number of continuation lines = 0


