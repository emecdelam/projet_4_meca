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
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = qdd[3]*S5+ALPHA24*C5
    ALPHA35 = qdd[3]*C5-ALPHA24*S5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6])
    OMp26 = qdd[6]+OMp25
    OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35)
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA26 = BS26-OMp36
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA14*C6-ALPHA35*S6
    ALPHA36 = ALPHA14*S6+ALPHA35*C6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[7]+OMp16
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,3]+BETA36*s.dpt[3,3]+BS16*s.dpt[1,3]
    ALPHA27 = C7*(ALPHA25+BETA46*s.dpt[1,3]+BETA66*s.dpt[3,3]+BS56*s.dpt[2,3])+S7*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3]+BS96*s.dpt[3,3])
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3]+BS96*s.dpt[3,3])-S7*(ALPHA25+BETA46*s.dpt[1,3]+BETA66*s.dpt[3,3]+BS56*s.dpt[2,3])
    OM18 = qd[8]+OM16
    OM28 = OM26*C8+OM36*S8
    OM38 = -OM26*S8+OM36*C8
    OMp18 = qdd[8]+OMp16
    OMp28 = C8*(OMp26+qd[8]*OM36)+S8*(OMp36-qd[8]*OM26)
    OMp38 = C8*(OMp36-qd[8]*OM26)-S8*(OMp26+qd[8]*OM36)
    BS28 = OM18*OM28
    BS58 = -OM18*OM18-OM38*OM38
    BS68 = OM28*OM38
    BETA28 = BS28-OMp38
    BETA88 = BS68+OMp18
    ALPHA18 = ALPHA16+BETA26*s.dpt[2,4]+BS16*s.dpt[1,4]
    ALPHA28 = C8*(ALPHA25+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])+S8*(ALPHA36+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4])
    ALPHA38 = C8*(ALPHA36+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4])-S8*(ALPHA25+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OMp19 = qdd[9]+OMp18
    OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28)
    OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38)
    ALPHA19 = ALPHA18+BETA28*s.dpt[2,24]
    ALPHA29 = C9*(ALPHA28+BS58*s.dpt[2,24])+S9*(ALPHA38+BETA88*s.dpt[2,24])
    ALPHA39 = C9*(ALPHA38+BETA88*s.dpt[2,24])-S9*(ALPHA28+BS58*s.dpt[2,24])
    OM110 = OM19*C10+OM29*S10
    OM210 = -OM19*S10+OM29*C10
    OM310 = qd[10]+OM39
    OMp110 = C10*(OMp19+qd[10]*OM29)+S10*(OMp29-qd[10]*OM19)
    OMp210 = C10*(OMp29-qd[10]*OM19)-S10*(OMp19+qd[10]*OM29)
    OMp310 = qdd[10]+OMp39
    BS210 = OM110*OM210
    BS310 = OM110*OM310
    BS510 = -OM110*OM110-OM310*OM310
    BS610 = OM210*OM310
    BS910 = -OM110*OM110-OM210*OM210
    BETA210 = BS210-OMp310
    BETA310 = BS310+OMp210
    BETA610 = BS610-OMp110
    BETA810 = BS610+OMp110
    ALPHA110 = ALPHA19*C10+ALPHA29*S10
    ALPHA210 = -ALPHA19*S10+ALPHA29*C10
    OM111 = qd[11]+OM110
    OM211 = OM210*C11+OM310*S11
    OM311 = -OM210*S11+OM310*C11
    OMp111 = qdd[11]+OMp110
    OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210)
    OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310)
    ALPHA111 = ALPHA110+BETA210*s.dpt[2,27]+BETA310*s.dpt[3,27]
    ALPHA211 = C11*(ALPHA210+BETA610*s.dpt[3,27]+BS510*s.dpt[2,27])+S11*(ALPHA39+BETA810*s.dpt[2,27]+BS910*s.dpt[3,27])
    ALPHA311 = C11*(ALPHA39+BETA810*s.dpt[2,27]+BS910*s.dpt[3,27])-S11*(ALPHA210+BETA610*s.dpt[3,27]+BS510*s.dpt[2,27])
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111)
    OMp212 = qdd[12]+OMp211
    OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311)
    ALPHA112 = ALPHA111*C12-ALPHA311*S12
    ALPHA312 = ALPHA111*S12+ALPHA311*C12
    OM113 = qd[13]+OM16
    OM213 = OM26*C13+OM36*S13
    OM313 = -OM26*S13+OM36*C13
    OMp113 = qdd[13]+OMp16
    OMp213 = C13*(OMp26+qd[13]*OM36)+S13*(OMp36-qd[13]*OM26)
    OMp313 = C13*(OMp36-qd[13]*OM26)-S13*(OMp26+qd[13]*OM36)
    ALPHA113 = ALPHA16+BETA26*s.dpt[2,6]+BETA36*s.dpt[3,6]+BS16*s.dpt[1,6]
    ALPHA213 = C13*(ALPHA25+BETA46*s.dpt[1,6]+BETA66*s.dpt[3,6]+BS56*s.dpt[2,6])+S13*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6]+BS96*s.dpt[3,6])
    ALPHA313 = C13*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6]+BS96*s.dpt[3,6])-S13*(ALPHA25+BETA46*s.dpt[1,6]+BETA66*s.dpt[3,6]+BS56*s.dpt[2,6])
    OM114 = qd[14]+OM16
    OM214 = OM26*C14+OM36*S14
    OM314 = -OM26*S14+OM36*C14
    OMp114 = qdd[14]+OMp16
    OMp214 = C14*(OMp26+qd[14]*OM36)+S14*(OMp36-qd[14]*OM26)
    OMp314 = C14*(OMp36-qd[14]*OM26)-S14*(OMp26+qd[14]*OM36)
    BS214 = OM114*OM214
    BS514 = -OM114*OM114-OM314*OM314
    BS614 = OM214*OM314
    BETA214 = BS214-OMp314
    BETA814 = BS614+OMp114
    ALPHA114 = ALPHA16+BETA26*s.dpt[2,7]+BS16*s.dpt[1,7]
    ALPHA214 = C14*(ALPHA25+BETA46*s.dpt[1,7]+BS56*s.dpt[2,7])+S14*(ALPHA36+BETA76*s.dpt[1,7]+BETA86*s.dpt[2,7])
    ALPHA314 = C14*(ALPHA36+BETA76*s.dpt[1,7]+BETA86*s.dpt[2,7])-S14*(ALPHA25+BETA46*s.dpt[1,7]+BS56*s.dpt[2,7])
    OM115 = qd[15]+OM114
    OM215 = OM214*C15+OM314*S15
    OM315 = -OM214*S15+OM314*C15
    OMp115 = qdd[15]+OMp114
    OMp215 = C15*(OMp214+qd[15]*OM314)+S15*(OMp314-qd[15]*OM214)
    OMp315 = C15*(OMp314-qd[15]*OM214)-S15*(OMp214+qd[15]*OM314)
    ALPHA115 = ALPHA114+BETA214*s.dpt[2,30]
    ALPHA215 = C15*(ALPHA214+BS514*s.dpt[2,30])+S15*(ALPHA314+BETA814*s.dpt[2,30])
    ALPHA315 = C15*(ALPHA314+BETA814*s.dpt[2,30])-S15*(ALPHA214+BS514*s.dpt[2,30])
    OM116 = OM115*C16+OM215*S16
    OM216 = -OM115*S16+OM215*C16
    OM316 = qd[16]+OM315
    OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115)
    OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215)
    OMp316 = qdd[16]+OMp315
    BS216 = OM116*OM216
    BS316 = OM116*OM316
    BS516 = -OM116*OM116-OM316*OM316
    BS616 = OM216*OM316
    BS916 = -OM116*OM116-OM216*OM216
    BETA216 = BS216-OMp316
    BETA316 = BS316+OMp216
    BETA616 = BS616-OMp116
    BETA816 = BS616+OMp116
    ALPHA116 = ALPHA115*C16+ALPHA215*S16
    ALPHA216 = -ALPHA115*S16+ALPHA215*C16
    OM117 = qd[17]+OM116
    OM217 = OM216*C17+OM316*S17
    OM317 = -OM216*S17+OM316*C17
    OMp117 = qdd[17]+OMp116
    OMp217 = C17*(OMp216+qd[17]*OM316)+S17*(OMp316-qd[17]*OM216)
    OMp317 = C17*(OMp316-qd[17]*OM216)-S17*(OMp216+qd[17]*OM316)
    ALPHA117 = ALPHA116+BETA216*s.dpt[2,33]+BETA316*s.dpt[3,33]
    ALPHA217 = C17*(ALPHA216+BETA616*s.dpt[3,33]+BS516*s.dpt[2,33])+S17*(ALPHA315+BETA816*s.dpt[2,33]+BS916*s.dpt[3,33])
    ALPHA317 = C17*(ALPHA315+BETA816*s.dpt[2,33]+BS916*s.dpt[3,33])-S17*(ALPHA216+BETA616*s.dpt[3,33]+BS516*s.dpt[2,33])
    OM118 = OM117*C18-OM317*S18
    OM218 = qd[18]+OM217
    OM318 = OM117*S18+OM317*C18
    OMp118 = C18*(OMp117-qd[18]*OM317)-S18*(OMp317+qd[18]*OM117)
    OMp218 = qdd[18]+OMp217
    OMp318 = C18*(OMp317+qd[18]*OM117)+S18*(OMp117-qd[18]*OM317)
    ALPHA118 = ALPHA117*C18-ALPHA317*S18
    ALPHA318 = ALPHA117*S18+ALPHA317*C18
    OM119 = OM16*C19-OM36*S19
    OM219 = qd[19]+OM26
    OM319 = OM16*S19+OM36*C19
    OMp119 = C19*(OMp16-qd[19]*OM36)-S19*(OMp36+qd[19]*OM16)
    OMp219 = qdd[19]+OMp26
    OMp319 = C19*(OMp36+qd[19]*OM16)+S19*(OMp16-qd[19]*OM36)
    ALPHA119 = C19*(ALPHA16+BS16*s.dpt[1,15])-S19*(ALPHA36+BETA76*s.dpt[1,15])
    ALPHA219 = ALPHA25+BETA46*s.dpt[1,15]
    ALPHA319 = C19*(ALPHA36+BETA76*s.dpt[1,15])+S19*(ALPHA16+BS16*s.dpt[1,15])
    OM120 = qd[20]+OM119
    OM220 = OM219*C20+OM319*S20
    OM320 = -OM219*S20+OM319*C20
    OMp120 = qdd[20]+OMp119
    OMp220 = C20*(OMp219+qd[20]*OM319)+S20*(OMp319-qd[20]*OM219)
    OMp320 = C20*(OMp319-qd[20]*OM219)-S20*(OMp219+qd[20]*OM319)
    BS220 = OM120*OM220
    BS520 = -OM120*OM120-OM320*OM320
    BS620 = OM220*OM320
    BETA220 = BS220-OMp320
    BETA820 = BS620+OMp120
    ALPHA220 = ALPHA219*C20+ALPHA319*S20
    ALPHA320 = -ALPHA219*S20+ALPHA319*C20
    OM121 = OM120*C21-OM320*S21
    OM221 = qd[21]+OM220
    OM321 = OM120*S21+OM320*C21
    OMp121 = C21*(OMp120-qd[21]*OM320)-S21*(OMp320+qd[21]*OM120)
    OMp221 = qdd[21]+OMp220
    OMp321 = C21*(OMp320+qd[21]*OM120)+S21*(OMp120-qd[21]*OM320)
    ALPHA121 = C21*(ALPHA119+BETA220*s.dpt[2,39])-S21*(ALPHA320+BETA820*s.dpt[2,39])
    ALPHA221 = ALPHA220+BS520*s.dpt[2,39]
    ALPHA321 = C21*(ALPHA320+BETA820*s.dpt[2,39])+S21*(ALPHA119+BETA220*s.dpt[2,39])
    OM122 = OM120*C22-OM320*S22
    OM222 = qd[22]+OM220
    OM322 = OM120*S22+OM320*C22
    OMp122 = C22*(OMp120-qd[22]*OM320)-S22*(OMp320+qd[22]*OM120)
    OMp222 = qdd[22]+OMp220
    OMp322 = C22*(OMp320+qd[22]*OM120)+S22*(OMp120-qd[22]*OM320)
    ALPHA122 = C22*(ALPHA119+BETA220*s.dpt[2,40])-S22*(ALPHA320+BETA820*s.dpt[2,40])
    ALPHA222 = ALPHA220+BS520*s.dpt[2,40]
    ALPHA322 = C22*(ALPHA320+BETA820*s.dpt[2,40])+S22*(ALPHA119+BETA220*s.dpt[2,40])
 
# Backward Dynamics

    Fs122 = -s.frc[1,22]+s.m[22]*ALPHA122
    Fs222 = -s.frc[2,22]+s.m[22]*ALPHA222
    Fs322 = -s.frc[3,22]+s.m[22]*ALPHA322
    Cq122 = -s.trq[1,22]+s.In[1,22]*OMp122-s.In[5,22]*OM222*OM322+s.In[9,22]*OM222*OM322
    Cq222 = -s.trq[2,22]+s.In[1,22]*OM122*OM322+s.In[5,22]*OMp222-s.In[9,22]*OM122*OM322
    Cq322 = -s.trq[3,22]-s.In[1,22]*OM122*OM222+s.In[5,22]*OM122*OM222+s.In[9,22]*OMp322
    Fs121 = -s.frc[1,21]+s.m[21]*ALPHA121
    Fs221 = -s.frc[2,21]+s.m[21]*ALPHA221
    Fs321 = -s.frc[3,21]+s.m[21]*ALPHA321
    Cq121 = -s.trq[1,21]+s.In[1,21]*OMp121-s.In[5,21]*OM221*OM321+s.In[9,21]*OM221*OM321
    Cq221 = -s.trq[2,21]+s.In[1,21]*OM121*OM321+s.In[5,21]*OMp221-s.In[9,21]*OM121*OM321
    Cq321 = -s.trq[3,21]-s.In[1,21]*OM121*OM221+s.In[5,21]*OM121*OM221+s.In[9,21]*OMp321
    Fs120 = -s.frc[1,20]+s.m[20]*ALPHA119
    Fs220 = -s.frc[2,20]+s.m[20]*ALPHA220
    Fs320 = -s.frc[3,20]+s.m[20]*ALPHA320
    Fq120 = Fs120+Fs121*C21+Fs122*C22+Fs321*S21+Fs322*S22
    Fq220 = Fs220+Fs221+Fs222
    Fq320 = Fs320-Fs121*S21-Fs122*S22+Fs321*C21+Fs322*C22
    Cq120 = -s.trq[1,20]+s.In[1,20]*OMp120-s.In[5,20]*OM220*OM320+s.In[9,20]*OM220*OM320+Cq121*C21+Cq122*C22+Cq321*S21+Cq322*S22+s.dpt[2,39]*(-Fs121*S21+Fs321*C21)+s.dpt[2,40]*(-Fs122*S22+Fs322*C22)
    Cq220 = -s.trq[2,20]+Cq221+Cq222+s.In[1,20]*OM120*OM320+s.In[5,20]*OMp220-s.In[9,20]*OM120*OM320
    Cq320 = -s.trq[3,20]-s.In[1,20]*OM120*OM220+s.In[5,20]*OM120*OM220+s.In[9,20]*OMp320-Cq121*S21-Cq122*S22+Cq321*C21+Cq322*C22-s.dpt[2,39]*(Fs121*C21+Fs321*S21)-s.dpt[2,40]*(Fs122*C22+Fs322*S22)
    Fq219 = Fq220*C20-Fq320*S20
    Fq319 = Fq220*S20+Fq320*C20
    Cq219 = Cq220*C20-Cq320*S20
    Cq319 = Cq220*S20+Cq320*C20
    Fs118 = -s.frc[1,18]+s.m[18]*ALPHA118
    Fs218 = -s.frc[2,18]+s.m[18]*ALPHA217
    Fs318 = -s.frc[3,18]+s.m[18]*ALPHA318
    Cq118 = -s.trq[1,18]+s.In[1,18]*OMp118-s.In[5,18]*OM218*OM318+s.In[9,18]*OM218*OM318
    Cq218 = -s.trq[2,18]+s.In[1,18]*OM118*OM318+s.In[5,18]*OMp218-s.In[9,18]*OM118*OM318
    Cq318 = -s.trq[3,18]-s.In[1,18]*OM118*OM218+s.In[5,18]*OM118*OM218+s.In[9,18]*OMp318
    Fq117 = Fs118*C18+Fs318*S18
    Fq317 = -Fs118*S18+Fs318*C18
    Cq117 = Cq118*C18+Cq318*S18
    Cq317 = -Cq118*S18+Cq318*C18
    Fs116 = -s.frc[1,16]+s.m[16]*ALPHA116
    Fs216 = -s.frc[2,16]+s.m[16]*ALPHA216
    Fs316 = -s.frc[3,16]+s.m[16]*ALPHA315
    Fq116 = Fq117+Fs116
    Fq216 = Fs216-Fq317*S17+Fs218*C17
    Fq316 = Fs316+Fq317*C17+Fs218*S17
    Cq116 = -s.trq[1,16]+Cq117+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316+s.dpt[2,33]*(Fq317*C17+Fs218*S17)-s.dpt[3,33]*(-Fq317*S17+Fs218*C17)
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316+Cq218*C17-Cq317*S17+Fq117*s.dpt[3,33]
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316+Cq218*S17+Cq317*C17-Fq117*s.dpt[2,33]
    Fq115 = Fq116*C16-Fq216*S16
    Fq215 = Fq116*S16+Fq216*C16
    Cq115 = Cq116*C16-Cq216*S16
    Cq215 = Cq116*S16+Cq216*C16
    Fs114 = -s.frc[1,14]+s.m[14]*ALPHA114
    Fs214 = -s.frc[2,14]+s.m[14]*ALPHA214
    Fs314 = -s.frc[3,14]+s.m[14]*ALPHA314
    Fq114 = Fq115+Fs114
    Fq214 = Fs214+Fq215*C15-Fq316*S15
    Fq314 = Fs314+Fq215*S15+Fq316*C15
    Cq114 = -s.trq[1,14]+Cq115+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314+s.dpt[2,30]*(Fq215*S15+Fq316*C15)
    Cq214 = -s.trq[2,14]+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314+Cq215*C15-Cq316*S15
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314+Cq215*S15+Cq316*C15-Fq115*s.dpt[2,30]
    Fs113 = -s.frc[1,13]+s.m[13]*ALPHA113
    Fs213 = -s.frc[2,13]+s.m[13]*ALPHA213
    Fs313 = -s.frc[3,13]+s.m[13]*ALPHA313
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313
    Cq213 = -s.trq[2,13]+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313
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
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA39
    Fq110 = Fq111+Fs110
    Fq210 = Fs210-Fq311*S11+Fs212*C11
    Fq310 = Fs310+Fq311*C11+Fs212*S11
    Cq110 = -s.trq[1,10]+Cq111+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+s.dpt[2,27]*(Fq311*C11+Fs212*S11)-s.dpt[3,27]*(-Fq311*S11+Fs212*C11)
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310+Cq212*C11-Cq311*S11+Fq111*s.dpt[3,27]
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310+Cq212*S11+Cq311*C11-Fq111*s.dpt[2,27]
    Fq19 = Fq110*C10-Fq210*S10
    Fq29 = Fq110*S10+Fq210*C10
    Cq19 = Cq110*C10-Cq210*S10
    Cq29 = Cq110*S10+Cq210*C10
    Fs18 = -s.frc[1,8]+s.m[8]*ALPHA18
    Fs28 = -s.frc[2,8]+s.m[8]*ALPHA28
    Fs38 = -s.frc[3,8]+s.m[8]*ALPHA38
    Fq18 = Fq19+Fs18
    Fq28 = Fs28+Fq29*C9-Fq310*S9
    Fq38 = Fs38+Fq29*S9+Fq310*C9
    Cq18 = -s.trq[1,8]+Cq19+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+s.dpt[2,24]*(Fq29*S9+Fq310*C9)
    Cq28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38+Cq29*C9-Cq310*S9
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38+Cq29*S9+Cq310*C9-Fq19*s.dpt[2,24]
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BS96*s.l[3,6])
    Fq16 = Fq114+Fq18+Fs113+Fs16+Fs17-s.frc[1,23]*C23-s.frc[1,24]*C24+s.frc[2,23]*S23+s.frc[2,24]*S24+Fq120*C19+Fq319*S19
    Fq26 = Fq219+Fs26-s.frc[1,23]*S23-s.frc[1,24]*S24-s.frc[2,23]*C23-s.frc[2,24]*C24+Fq214*C14+Fq28*C8-Fq314*S14-Fq38*S8+Fs213*C13+Fs27*C7-Fs313*S13-Fs37*S7
    Fq36 = -s.frc[3,23]-s.frc[3,24]+Fs36-Fq120*S19+Fq214*S14+Fq28*S8+Fq314*C14+Fq319*C19+Fq38*C8+Fs213*S13+Fs27*S7+Fs313*C13+Fs37*C7
    Cq16 = -s.trq[1,6]+Cq113+Cq114+Cq17+Cq18+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36-s.frc[3,23]*s.dpt[2,20]-s.frc[3,24]*s.dpt[2,21]-s.trq[1,23]*C23-s.trq[1,24]*C24+s.trq[2,23]*S23+s.trq[2,24]*S24+Cq120*C19+Cq319*S19-Fs26*s.l[3,6]+s.dpt[2,3]*(Fs27*S7+Fs37*C7)+s.dpt[2,4]*(Fq28*S8+Fq38*C8)+s.dpt[2,6]*(Fs213*S13+Fs313*C13)+s.dpt[2,7]*(Fq214*S14+Fq314*C14)-s.dpt[3,3]*(Fs27*C7-Fs37*S7)-s.dpt[3,6]*(Fs213*C13-Fs313*S13)
    Cq26 = -s.trq[2,6]+Cq219+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+s.frc[3,23]*s.dpt[1,20]+s.frc[3,24]*s.dpt[1,21]-s.trq[1,23]*S23-s.trq[1,24]*S24-s.trq[2,23]*C23-s.trq[2,24]*C24+Cq213*C13+Cq214*C14+Cq27*C7+Cq28*C8-Cq313*S13-Cq314*S14-Cq37*S7-Cq38*S8+Fs113*s.dpt[3,6]+Fs16*s.l[3,6]+Fs17*s.dpt[3,3]-s.dpt[1,15]*(-Fq120*S19+Fq319*C19)-s.dpt[1,3]*(Fs27*S7+Fs37*C7)-s.dpt[1,4]*(Fq28*S8+Fq38*C8)-s.dpt[1,6]*(Fs213*S13+Fs313*C13)-s.dpt[1,7]*(Fq214*S14+Fq314*C14)
    Cq36 = -s.trq[3,23]-s.trq[3,24]-s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq120*S19+Cq213*S13+Cq214*S14+Cq27*S7+Cq28*S8+Cq313*C13+Cq314*C14+Cq319*C19+Cq37*C7+Cq38*C8-Fq114*s.dpt[2,7]-Fq18*s.dpt[2,4]+Fq219*s.dpt[1,15]-Fs113*s.dpt[2,6]-Fs17*s.dpt[2,3]+s.dpt[1,20]*(-s.frc[1,23]*S23-s.frc[2,23]*C23)+s.dpt[1,21]*(-s.frc[1,24]*S24-s.frc[2,24]*C24)+s.dpt[1,3]*(Fs27*C7-Fs37*S7)+s.dpt[1,4]*(Fq28*C8-Fq38*S8)+s.dpt[1,6]*(Fs213*C13-Fs313*S13)+s.dpt[1,7]*(Fq214*C14-Fq314*S14)-s.dpt[2,20]*(-s.frc[1,23]*C23+s.frc[2,23]*S23)-s.dpt[2,21]*(-s.frc[1,24]*C24+s.frc[2,24]*S24)
    Fq15 = Fq16*C6+Fq36*S6
    Fq35 = -Fq16*S6+Fq36*C6
    Cq15 = Cq16*C6+Cq36*S6
    Cq35 = -Cq16*S6+Cq36*C6
    Fq24 = Fq26*C5-Fq35*S5
    Fq34 = Fq26*S5+Fq35*C5
    Cq34 = Cq26*S5+Cq35*C5
    Fq13 = Fq15*C4-Fq24*S4
    Fq23 = Fq15*S4+Fq24*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq15
    Qq[6] = Cq26
    Qq[7] = Cq17
    Qq[8] = Cq18
    Qq[9] = Cq19
    Qq[10] = Cq310
    Qq[11] = Cq111
    Qq[12] = Cq212
    Qq[13] = Cq113
    Qq[14] = Cq114
    Qq[15] = Cq115
    Qq[16] = Cq316
    Qq[17] = Cq117
    Qq[18] = Cq218
    Qq[19] = Cq219
    Qq[20] = Cq120
    Qq[21] = Cq221
    Qq[22] = Cq222
    Qq[23] = -s.trq[3,23]
    Qq[24] = -s.trq[3,24]

# Number of continuation lines = 0


