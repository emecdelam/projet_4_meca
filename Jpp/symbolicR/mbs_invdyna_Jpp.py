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

    ALPHA31 = qdd[1]-s.g[3]
    ALPHA14 = qdd[2]*S4+qdd[3]*C4
    ALPHA24 = qdd[2]*C4-qdd[3]*S4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = ALPHA24*C5+ALPHA31*S5
    ALPHA35 = -ALPHA24*S5+ALPHA31*C5
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
    BS27 = OM17*OM27
    BS57 = -OM17*OM17-OM37*OM37
    BS67 = OM27*OM37
    BETA27 = BS27-OMp37
    BETA87 = BS67+OMp17
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,2]+BS16*s.dpt[1,2]
    ALPHA27 = C7*(ALPHA25+BETA46*s.dpt[1,2]+BS56*s.dpt[2,2])+S7*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2])
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2])-S7*(ALPHA25+BETA46*s.dpt[1,2]+BS56*s.dpt[2,2])
    OM18 = qd[8]+OM17
    OM28 = OM27*C8+OM37*S8
    OM38 = -OM27*S8+OM37*C8
    OMp18 = qdd[8]+OMp17
    OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27)
    OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37)
    ALPHA18 = ALPHA17+BETA27*s.dpt[2,23]
    ALPHA28 = C8*(ALPHA27+BS57*s.dpt[2,23])+S8*(ALPHA37+BETA87*s.dpt[2,23])
    ALPHA38 = C8*(ALPHA37+BETA87*s.dpt[2,23])-S8*(ALPHA27+BS57*s.dpt[2,23])
    OM19 = OM18*C9+OM28*S9
    OM29 = -OM18*S9+OM28*C9
    OM39 = qd[9]+OM38
    OMp19 = C9*(OMp18+qd[9]*OM28)+S9*(OMp28-qd[9]*OM18)
    OMp29 = C9*(OMp28-qd[9]*OM18)-S9*(OMp18+qd[9]*OM28)
    OMp39 = qdd[9]+OMp38
    BS29 = OM19*OM29
    BS39 = OM19*OM39
    BS59 = -OM19*OM19-OM39*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA29 = BS29-OMp39
    BETA39 = BS39+OMp29
    BETA69 = BS69-OMp19
    BETA89 = BS69+OMp19
    ALPHA19 = ALPHA18*C9+ALPHA28*S9
    ALPHA29 = -ALPHA18*S9+ALPHA28*C9
    OM110 = qd[10]+OM19
    OM210 = OM29*C10+OM39*S10
    OM310 = -OM29*S10+OM39*C10
    OMp110 = qdd[10]+OMp19
    OMp210 = C10*(OMp29+qd[10]*OM39)+S10*(OMp39-qd[10]*OM29)
    OMp310 = C10*(OMp39-qd[10]*OM29)-S10*(OMp29+qd[10]*OM39)
    ALPHA110 = ALPHA19+BETA29*s.dpt[2,26]+BETA39*s.dpt[3,26]
    ALPHA210 = C10*(ALPHA29+BETA69*s.dpt[3,26]+BS59*s.dpt[2,26])+S10*(ALPHA38+BETA89*s.dpt[2,26]+BS99*s.dpt[3,26])
    ALPHA310 = C10*(ALPHA38+BETA89*s.dpt[2,26]+BS99*s.dpt[3,26])-S10*(ALPHA29+BETA69*s.dpt[3,26]+BS59*s.dpt[2,26])
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    ALPHA111 = ALPHA110*C11-ALPHA310*S11
    ALPHA311 = ALPHA110*S11+ALPHA310*C11
    OM112 = qd[12]+OM16
    OM212 = OM26*C12+OM36*S12
    OM312 = -OM26*S12+OM36*C12
    OMp112 = qdd[12]+OMp16
    OMp212 = C12*(OMp26+qd[12]*OM36)+S12*(OMp36-qd[12]*OM26)
    OMp312 = C12*(OMp36-qd[12]*OM26)-S12*(OMp26+qd[12]*OM36)
    ALPHA112 = ALPHA16+BETA26*s.dpt[2,3]+BETA36*s.dpt[3,3]+BS16*s.dpt[1,3]
    ALPHA212 = C12*(ALPHA25+BETA46*s.dpt[1,3]+BETA66*s.dpt[3,3]+BS56*s.dpt[2,3])+S12*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3]+BS96*s.dpt[3,3])
    ALPHA312 = C12*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3]+BS96*s.dpt[3,3])-S12*(ALPHA25+BETA46*s.dpt[1,3]+BETA66*s.dpt[3,3]+BS56*s.dpt[2,3])
    OM113 = qd[13]+OM16
    OM213 = OM26*C13+OM36*S13
    OM313 = -OM26*S13+OM36*C13
    OMp113 = qdd[13]+OMp16
    OMp213 = C13*(OMp26+qd[13]*OM36)+S13*(OMp36-qd[13]*OM26)
    OMp313 = C13*(OMp36-qd[13]*OM26)-S13*(OMp26+qd[13]*OM36)
    BS213 = OM113*OM213
    BS513 = -OM113*OM113-OM313*OM313
    BS613 = OM213*OM313
    BETA213 = BS213-OMp313
    BETA813 = BS613+OMp113
    ALPHA113 = ALPHA16+BETA26*s.dpt[2,5]+BS16*s.dpt[1,5]
    ALPHA213 = C13*(ALPHA25+BETA46*s.dpt[1,5]+BS56*s.dpt[2,5])+S13*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5])
    ALPHA313 = C13*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5])-S13*(ALPHA25+BETA46*s.dpt[1,5]+BS56*s.dpt[2,5])
    OM114 = qd[14]+OM113
    OM214 = OM213*C14+OM313*S14
    OM314 = -OM213*S14+OM313*C14
    OMp114 = qdd[14]+OMp113
    OMp214 = C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213)
    OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313)
    ALPHA114 = ALPHA113+BETA213*s.dpt[2,30]
    ALPHA214 = C14*(ALPHA213+BS513*s.dpt[2,30])+S14*(ALPHA313+BETA813*s.dpt[2,30])
    ALPHA314 = C14*(ALPHA313+BETA813*s.dpt[2,30])-S14*(ALPHA213+BS513*s.dpt[2,30])
    OM115 = OM114*C15+OM214*S15
    OM215 = -OM114*S15+OM214*C15
    OM315 = qd[15]+OM314
    OMp115 = C15*(OMp114+qd[15]*OM214)+S15*(OMp214-qd[15]*OM114)
    OMp215 = C15*(OMp214-qd[15]*OM114)-S15*(OMp114+qd[15]*OM214)
    OMp315 = qdd[15]+OMp314
    BS215 = OM115*OM215
    BS315 = OM115*OM315
    BS515 = -OM115*OM115-OM315*OM315
    BS615 = OM215*OM315
    BS915 = -OM115*OM115-OM215*OM215
    BETA215 = BS215-OMp315
    BETA315 = BS315+OMp215
    BETA615 = BS615-OMp115
    BETA815 = BS615+OMp115
    ALPHA115 = ALPHA114*C15+ALPHA214*S15
    ALPHA215 = -ALPHA114*S15+ALPHA214*C15
    OM116 = qd[16]+OM115
    OM216 = OM215*C16+OM315*S16
    OM316 = -OM215*S16+OM315*C16
    OMp116 = qdd[16]+OMp115
    OMp216 = C16*(OMp215+qd[16]*OM315)+S16*(OMp315-qd[16]*OM215)
    OMp316 = C16*(OMp315-qd[16]*OM215)-S16*(OMp215+qd[16]*OM315)
    ALPHA116 = ALPHA115+BETA215*s.dpt[2,33]+BETA315*s.dpt[3,33]
    ALPHA216 = C16*(ALPHA215+BETA615*s.dpt[3,33]+BS515*s.dpt[2,33])+S16*(ALPHA314+BETA815*s.dpt[2,33]+BS915*s.dpt[3,33])
    ALPHA316 = C16*(ALPHA314+BETA815*s.dpt[2,33]+BS915*s.dpt[3,33])-S16*(ALPHA215+BETA615*s.dpt[3,33]+BS515*s.dpt[2,33])
    OM117 = OM116*C17-OM316*S17
    OM217 = qd[17]+OM216
    OM317 = OM116*S17+OM316*C17
    OMp117 = C17*(OMp116-qd[17]*OM316)-S17*(OMp316+qd[17]*OM116)
    OMp217 = qdd[17]+OMp216
    OMp317 = C17*(OMp316+qd[17]*OM116)+S17*(OMp116-qd[17]*OM316)
    ALPHA117 = ALPHA116*C17-ALPHA316*S17
    ALPHA317 = ALPHA116*S17+ALPHA316*C17
    OM118 = qd[18]+OM16
    OM218 = OM26*C18+OM36*S18
    OM318 = -OM26*S18+OM36*C18
    OMp118 = qdd[18]+OMp16
    OMp218 = C18*(OMp26+qd[18]*OM36)+S18*(OMp36-qd[18]*OM26)
    OMp318 = C18*(OMp36-qd[18]*OM26)-S18*(OMp26+qd[18]*OM36)
    ALPHA118 = ALPHA16+BETA26*s.dpt[2,6]+BETA36*s.dpt[3,6]+BS16*s.dpt[1,6]
    ALPHA218 = C18*(ALPHA25+BETA46*s.dpt[1,6]+BETA66*s.dpt[3,6]+BS56*s.dpt[2,6])+S18*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6]+BS96*s.dpt[3,6])
    ALPHA318 = C18*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6]+BS96*s.dpt[3,6])-S18*(ALPHA25+BETA46*s.dpt[1,6]+BETA66*s.dpt[3,6]+BS56*s.dpt[2,6])
    OM119 = qd[19]+OM16
    OM219 = OM26*C19+OM36*S19
    OM319 = -OM26*S19+OM36*C19
    OMp119 = qdd[19]+OMp16
    OMp219 = C19*(OMp26+qd[19]*OM36)+S19*(OMp36-qd[19]*OM26)
    OMp319 = C19*(OMp36-qd[19]*OM26)-S19*(OMp26+qd[19]*OM36)
    ALPHA119 = ALPHA16+BS16*s.dpt[1,14]
    ALPHA219 = C19*(ALPHA25+BETA46*s.dpt[1,14])+S19*(ALPHA36+BETA76*s.dpt[1,14])
    ALPHA319 = C19*(ALPHA36+BETA76*s.dpt[1,14])-S19*(ALPHA25+BETA46*s.dpt[1,14])
    OM120 = OM119*C20-OM319*S20
    OM220 = qd[20]+OM219
    OM320 = OM119*S20+OM319*C20
    OMp120 = C20*(OMp119-qd[20]*OM319)-S20*(OMp319+qd[20]*OM119)
    OMp220 = qdd[20]+OMp219
    OMp320 = C20*(OMp319+qd[20]*OM119)+S20*(OMp119-qd[20]*OM319)
    BS220 = OM120*OM220
    BS520 = -OM120*OM120-OM320*OM320
    BS620 = OM220*OM320
    BETA220 = BS220-OMp320
    BETA820 = BS620+OMp120
    ALPHA120 = ALPHA119*C20-ALPHA319*S20
    ALPHA320 = ALPHA119*S20+ALPHA319*C20
    OM121 = OM120*C21-OM320*S21
    OM221 = qd[21]+OM220
    OM321 = OM120*S21+OM320*C21
    OMp121 = C21*(OMp120-qd[21]*OM320)-S21*(OMp320+qd[21]*OM120)
    OMp221 = qdd[21]+OMp220
    OMp321 = C21*(OMp320+qd[21]*OM120)+S21*(OMp120-qd[21]*OM320)
    ALPHA121 = C21*(ALPHA120+BETA220*s.dpt[2,43])-S21*(ALPHA320+BETA820*s.dpt[2,43])
    ALPHA221 = ALPHA219+BS520*s.dpt[2,43]
    ALPHA321 = C21*(ALPHA320+BETA820*s.dpt[2,43])+S21*(ALPHA120+BETA220*s.dpt[2,43])
    OM122 = OM120*C22-OM320*S22
    OM222 = qd[22]+OM220
    OM322 = OM120*S22+OM320*C22
    OMp122 = C22*(OMp120-qd[22]*OM320)-S22*(OMp320+qd[22]*OM120)
    OMp222 = qdd[22]+OMp220
    OMp322 = C22*(OMp320+qd[22]*OM120)+S22*(OMp120-qd[22]*OM320)
    ALPHA122 = C22*(ALPHA120+BETA220*s.dpt[2,44])-S22*(ALPHA320+BETA820*s.dpt[2,44])
    ALPHA222 = ALPHA219+BS520*s.dpt[2,44]
    ALPHA322 = C22*(ALPHA320+BETA820*s.dpt[2,44])+S22*(ALPHA120+BETA220*s.dpt[2,44])
 
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
    Fs120 = -s.frc[1,20]+s.m[20]*ALPHA120
    Fs220 = -s.frc[2,20]+s.m[20]*ALPHA219
    Fs320 = -s.frc[3,20]+s.m[20]*ALPHA320
    Fq120 = Fs120+Fs121*C21+Fs122*C22+Fs321*S21+Fs322*S22
    Fq220 = Fs220+Fs221+Fs222
    Fq320 = Fs320-Fs121*S21-Fs122*S22+Fs321*C21+Fs322*C22
    Cq120 = -s.trq[1,20]+s.In[1,20]*OMp120-s.In[5,20]*OM220*OM320+s.In[9,20]*OM220*OM320+Cq121*C21+Cq122*C22+Cq321*S21+Cq322*S22+s.dpt[2,43]*(-Fs121*S21+Fs321*C21)+s.dpt[2,44]*(-Fs122*S22+Fs322*C22)
    Cq220 = -s.trq[2,20]+Cq221+Cq222+s.In[1,20]*OM120*OM320+s.In[5,20]*OMp220-s.In[9,20]*OM120*OM320
    Cq320 = -s.trq[3,20]-s.In[1,20]*OM120*OM220+s.In[5,20]*OM120*OM220+s.In[9,20]*OMp320-Cq121*S21-Cq122*S22+Cq321*C21+Cq322*C22-s.dpt[2,43]*(Fs121*C21+Fs321*S21)-s.dpt[2,44]*(Fs122*C22+Fs322*S22)
    Fq119 = Fq120*C20+Fq320*S20
    Fq319 = -Fq120*S20+Fq320*C20
    Cq119 = Cq120*C20+Cq320*S20
    Cq319 = -Cq120*S20+Cq320*C20
    Fs118 = -s.frc[1,18]+s.m[18]*ALPHA118
    Fs218 = -s.frc[2,18]+s.m[18]*ALPHA218
    Fs318 = -s.frc[3,18]+s.m[18]*ALPHA318
    Cq118 = -s.trq[1,18]+s.In[1,18]*OMp118-s.In[5,18]*OM218*OM318+s.In[9,18]*OM218*OM318
    Cq218 = -s.trq[2,18]+s.In[1,18]*OM118*OM318+s.In[5,18]*OMp218-s.In[9,18]*OM118*OM318
    Cq318 = -s.trq[3,18]-s.In[1,18]*OM118*OM218+s.In[5,18]*OM118*OM218+s.In[9,18]*OMp318
    Fs117 = -s.frc[1,17]+s.m[17]*ALPHA117
    Fs217 = -s.frc[2,17]+s.m[17]*ALPHA216
    Fs317 = -s.frc[3,17]+s.m[17]*ALPHA317
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp117-s.In[5,17]*OM217*OM317+s.In[9,17]*OM217*OM317
    Cq217 = -s.trq[2,17]+s.In[1,17]*OM117*OM317+s.In[5,17]*OMp217-s.In[9,17]*OM117*OM317
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM117*OM217+s.In[5,17]*OM117*OM217+s.In[9,17]*OMp317
    Fq116 = Fs117*C17+Fs317*S17
    Fq316 = -Fs117*S17+Fs317*C17
    Cq116 = Cq117*C17+Cq317*S17
    Cq316 = -Cq117*S17+Cq317*C17
    Fs115 = -s.frc[1,15]+s.m[15]*ALPHA115
    Fs215 = -s.frc[2,15]+s.m[15]*ALPHA215
    Fs315 = -s.frc[3,15]+s.m[15]*ALPHA314
    Fq115 = Fq116+Fs115
    Fq215 = Fs215-Fq316*S16+Fs217*C16
    Fq315 = Fs315+Fq316*C16+Fs217*S16
    Cq115 = -s.trq[1,15]+Cq116+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315+s.dpt[2,33]*(Fq316*C16+Fs217*S16)-s.dpt[3,33]*(-Fq316*S16+Fs217*C16)
    Cq215 = -s.trq[2,15]+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315+Cq217*C16-Cq316*S16+Fq116*s.dpt[3,33]
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315+Cq217*S16+Cq316*C16-Fq116*s.dpt[2,33]
    Fq114 = Fq115*C15-Fq215*S15
    Fq214 = Fq115*S15+Fq215*C15
    Cq114 = Cq115*C15-Cq215*S15
    Cq214 = Cq115*S15+Cq215*C15
    Fs113 = -s.frc[1,13]+s.m[13]*ALPHA113
    Fs213 = -s.frc[2,13]+s.m[13]*ALPHA213
    Fs313 = -s.frc[3,13]+s.m[13]*ALPHA313
    Fq113 = Fq114+Fs113
    Fq213 = Fs213+Fq214*C14-Fq315*S14
    Fq313 = Fs313+Fq214*S14+Fq315*C14
    Cq113 = -s.trq[1,13]+Cq114+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+s.dpt[2,30]*(Fq214*S14+Fq315*C14)
    Cq213 = -s.trq[2,13]+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313+Cq214*C14-Cq315*S14
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313+Cq214*S14+Cq315*C14-Fq114*s.dpt[2,30]
    Fs112 = -s.frc[1,12]+s.m[12]*ALPHA112
    Fs212 = -s.frc[2,12]+s.m[12]*ALPHA212
    Fs312 = -s.frc[3,12]+s.m[12]*ALPHA312
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312
    Cq212 = -s.trq[2,12]+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312
    Fs111 = -s.frc[1,11]+s.m[11]*ALPHA111
    Fs211 = -s.frc[2,11]+s.m[11]*ALPHA210
    Fs311 = -s.frc[3,11]+s.m[11]*ALPHA311
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311
    Fq110 = Fs111*C11+Fs311*S11
    Fq310 = -Fs111*S11+Fs311*C11
    Cq110 = Cq111*C11+Cq311*S11
    Cq310 = -Cq111*S11+Cq311*C11
    Fs19 = -s.frc[1,9]+s.m[9]*ALPHA19
    Fs29 = -s.frc[2,9]+s.m[9]*ALPHA29
    Fs39 = -s.frc[3,9]+s.m[9]*ALPHA38
    Fq19 = Fq110+Fs19
    Fq29 = Fs29-Fq310*S10+Fs211*C10
    Fq39 = Fs39+Fq310*C10+Fs211*S10
    Cq19 = -s.trq[1,9]+Cq110+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39+s.dpt[2,26]*(Fq310*C10+Fs211*S10)-s.dpt[3,26]*(-Fq310*S10+Fs211*C10)
    Cq29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39+Cq211*C10-Cq310*S10+Fq110*s.dpt[3,26]
    Cq39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39+Cq211*S10+Cq310*C10-Fq110*s.dpt[2,26]
    Fq18 = Fq19*C9-Fq29*S9
    Fq28 = Fq19*S9+Fq29*C9
    Cq18 = Cq19*C9-Cq29*S9
    Cq28 = Cq19*S9+Cq29*C9
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Fq17 = Fq18+Fs17
    Fq27 = Fs27+Fq28*C8-Fq39*S8
    Fq37 = Fs37+Fq28*S8+Fq39*C8
    Cq17 = -s.trq[1,7]+Cq18+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+s.dpt[2,23]*(Fq28*S8+Fq39*C8)
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37+Cq28*C8-Cq39*S8
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37+Cq28*S8+Cq39*C8-Fq18*s.dpt[2,23]
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA16
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA25
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA36
    Fq16 = Fq113+Fq119+Fq17+Fs112+Fs118+Fs16-s.frc[1,23]*C23-s.frc[1,24]*C24+s.frc[2,23]*S23+s.frc[2,24]*S24
    Fq26 = Fs26-s.frc[1,23]*S23-s.frc[1,24]*S24-s.frc[2,23]*C23-s.frc[2,24]*C24+Fq213*C13+Fq220*C19+Fq27*C7-Fq313*S13-Fq319*S19-Fq37*S7+Fs212*C12+Fs218*C18-Fs312*S12-Fs318*S18
    Fq36 = -s.frc[3,23]-s.frc[3,24]+Fs36+Fq213*S13+Fq220*S19+Fq27*S7+Fq313*C13+Fq319*C19+Fq37*C7+Fs212*S12+Fs218*S18+Fs312*C12+Fs318*C18
    Cq16 = -s.trq[1,6]+Cq112+Cq113+Cq118+Cq119+Cq17-s.In[5,6]*OM26*OM36-s.frc[3,23]*s.dpt[2,17]-s.frc[3,24]*s.dpt[2,18]-s.trq[1,23]*C23-s.trq[1,24]*C24+s.trq[2,23]*S23+s.trq[2,24]*S24+s.dpt[2,2]*(Fq27*S7+Fq37*C7)+s.dpt[2,3]*(Fs212*S12+Fs312*C12)+s.dpt[2,5]*(Fq213*S13+Fq313*C13)+s.dpt[2,6]*(Fs218*S18+Fs318*C18)-s.dpt[3,3]*(Fs212*C12-Fs312*S12)-s.dpt[3,6]*(Fs218*C18-Fs318*S18)
    Cq26 = -s.trq[2,6]+s.In[5,6]*OMp26+s.frc[3,23]*s.dpt[1,17]+s.frc[3,24]*s.dpt[1,18]-s.trq[1,23]*S23-s.trq[1,24]*S24-s.trq[2,23]*C23-s.trq[2,24]*C24+Cq212*C12+Cq213*C13+Cq218*C18+Cq220*C19+Cq27*C7-Cq312*S12-Cq313*S13-Cq318*S18-Cq319*S19-Cq37*S7+Fs112*s.dpt[3,3]+Fs118*s.dpt[3,6]-s.dpt[1,14]*(Fq220*S19+Fq319*C19)-s.dpt[1,2]*(Fq27*S7+Fq37*C7)-s.dpt[1,3]*(Fs212*S12+Fs312*C12)-s.dpt[1,5]*(Fq213*S13+Fq313*C13)-s.dpt[1,6]*(Fs218*S18+Fs318*C18)
    Cq36 = -s.trq[3,23]-s.trq[3,24]-s.trq[3,6]+s.In[5,6]*OM16*OM26+Cq212*S12+Cq213*S13+Cq218*S18+Cq220*S19+Cq27*S7+Cq312*C12+Cq313*C13+Cq318*C18+Cq319*C19+Cq37*C7-Fq113*s.dpt[2,5]-Fq17*s.dpt[2,2]-Fs112*s.dpt[2,3]-Fs118*s.dpt[2,6]+s.dpt[1,14]*(Fq220*C19-Fq319*S19)+s.dpt[1,17]*(-s.frc[1,23]*S23-s.frc[2,23]*C23)+s.dpt[1,18]*(-s.frc[1,24]*S24-s.frc[2,24]*C24)+s.dpt[1,2]*(Fq27*C7-Fq37*S7)+s.dpt[1,3]*(Fs212*C12-Fs312*S12)+s.dpt[1,5]*(Fq213*C13-Fq313*S13)+s.dpt[1,6]*(Fs218*C18-Fs318*S18)-s.dpt[2,17]*(-s.frc[1,23]*C23+s.frc[2,23]*S23)-s.dpt[2,18]*(-s.frc[1,24]*C24+s.frc[2,24]*S24)
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

    Qq[1] = Fq34
    Qq[2] = Fq23
    Qq[3] = Fq13
    Qq[4] = Cq34
    Qq[5] = Cq15
    Qq[6] = Cq26
    Qq[7] = Cq17
    Qq[8] = Cq18
    Qq[9] = Cq39
    Qq[10] = Cq110
    Qq[11] = Cq211
    Qq[12] = Cq112
    Qq[13] = Cq113
    Qq[14] = Cq114
    Qq[15] = Cq315
    Qq[16] = Cq116
    Qq[17] = Cq217
    Qq[18] = Cq118
    Qq[19] = Cq119
    Qq[20] = Cq220
    Qq[21] = Cq221
    Qq[22] = Cq222
    Qq[23] = -s.trq[3,23]
    Qq[24] = -s.trq[3,24]

# Number of continuation lines = 0


