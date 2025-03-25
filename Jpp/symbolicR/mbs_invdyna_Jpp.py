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
#	==> Generation Date: Tue Mar 25 22:12:43 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 30
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
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
 
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
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,1]+BS16*s.dpt[1,1]
    ALPHA27 = C7*(ALPHA25+BETA46*s.dpt[1,1]+BS56*s.dpt[2,1])+S7*(ALPHA36+BETA76*s.dpt[1,1]+BETA86*s.dpt[2,1])
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,1]+BETA86*s.dpt[2,1])-S7*(ALPHA25+BETA46*s.dpt[1,1]+BS56*s.dpt[2,1])
    OM18 = qd[8]+OM17
    OM28 = OM27*C8+OM37*S8
    OM38 = -OM27*S8+OM37*C8
    OMp18 = qdd[8]+OMp17
    OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27)
    OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37)
    ALPHA18 = ALPHA17+BETA27*s.dpt[2,22]
    ALPHA28 = C8*(ALPHA27+BS57*s.dpt[2,22])+S8*(ALPHA37+BETA87*s.dpt[2,22])
    ALPHA38 = C8*(ALPHA37+BETA87*s.dpt[2,22])-S8*(ALPHA27+BS57*s.dpt[2,22])
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
    ALPHA110 = ALPHA19+BETA29*s.dpt[2,25]+BETA39*s.dpt[3,25]
    ALPHA210 = C10*(ALPHA29+BETA69*s.dpt[3,25]+BS59*s.dpt[2,25])+S10*(ALPHA38+BETA89*s.dpt[2,25]+BS99*s.dpt[3,25])
    ALPHA310 = C10*(ALPHA38+BETA89*s.dpt[2,25]+BS99*s.dpt[3,25])-S10*(ALPHA29+BETA69*s.dpt[3,25]+BS59*s.dpt[2,25])
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
    ALPHA112 = ALPHA16+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2]
    ALPHA212 = C12*(ALPHA25+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S12*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA312 = C12*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S12*(ALPHA25+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])
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
    ALPHA113 = ALPHA16+BETA26*s.dpt[2,4]+BS16*s.dpt[1,4]
    ALPHA213 = C13*(ALPHA25+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])+S13*(ALPHA36+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4])
    ALPHA313 = C13*(ALPHA36+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4])-S13*(ALPHA25+BETA46*s.dpt[1,4]+BS56*s.dpt[2,4])
    OM114 = qd[14]+OM113
    OM214 = OM213*C14+OM313*S14
    OM314 = -OM213*S14+OM313*C14
    OMp114 = qdd[14]+OMp113
    OMp214 = C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213)
    OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313)
    ALPHA114 = ALPHA113+BETA213*s.dpt[2,29]
    ALPHA214 = C14*(ALPHA213+BS513*s.dpt[2,29])+S14*(ALPHA313+BETA813*s.dpt[2,29])
    ALPHA314 = C14*(ALPHA313+BETA813*s.dpt[2,29])-S14*(ALPHA213+BS513*s.dpt[2,29])
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
    ALPHA116 = ALPHA115+BETA215*s.dpt[2,32]+BETA315*s.dpt[3,32]
    ALPHA216 = C16*(ALPHA215+BETA615*s.dpt[3,32]+BS515*s.dpt[2,32])+S16*(ALPHA314+BETA815*s.dpt[2,32]+BS915*s.dpt[3,32])
    ALPHA316 = C16*(ALPHA314+BETA815*s.dpt[2,32]+BS915*s.dpt[3,32])-S16*(ALPHA215+BETA615*s.dpt[3,32]+BS515*s.dpt[2,32])
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
    ALPHA118 = ALPHA16+BETA26*s.dpt[2,5]+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5]
    ALPHA218 = C18*(ALPHA25+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])+S18*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])
    ALPHA318 = C18*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])-S18*(ALPHA25+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])
    BS119 = -OM26*OM26-OM36*OM36
    BS219 = OM16*OM26
    BS319 = OM16*OM36
    BETA419 = BS219+OMp36
    BETA719 = BS319-OMp26
    ALPHA119 = ALPHA16+q[19]*BETA36+(2.0)*qd[19]*OM26+BS16*s.dpt[1,13]
    ALPHA219 = ALPHA25+q[19]*BETA66-(2.0)*qd[19]*OM16+BETA46*s.dpt[1,13]
    ALPHA319 = qdd[19]+ALPHA36+q[19]*BS96+BETA76*s.dpt[1,13]
    BS220 = OM16*OM26
    BS520 = -OM16*OM16-OM36*OM36
    BS620 = OM26*OM36
    BETA220 = BS220-OMp36
    BETA820 = BS620+OMp16
    ALPHA120 = qdd[20]+ALPHA119+q[20]*BS119
    ALPHA220 = ALPHA219+q[20]*BETA419+(2.0)*qd[20]*OM36
    ALPHA320 = ALPHA319+q[20]*BETA719-(2.0)*qd[20]*OM26
    ALPHA121 = ALPHA120+q[21]*BETA220-(2.0)*qd[21]*OM36
    ALPHA221 = qdd[21]+ALPHA220+q[21]*BS520
    ALPHA321 = ALPHA320+q[21]*BETA820+(2.0)*qd[21]*OM16
    OM122 = qd[22]+OM16
    OM222 = OM26*C22+OM36*S22
    OM322 = -OM26*S22+OM36*C22
    OMp122 = qdd[22]+OMp16
    OMp222 = C22*(OMp26+qd[22]*OM36)+S22*(OMp36-qd[22]*OM26)
    OMp322 = C22*(OMp36-qd[22]*OM26)-S22*(OMp26+qd[22]*OM36)
    ALPHA222 = ALPHA221*C22+ALPHA321*S22
    ALPHA322 = -ALPHA221*S22+ALPHA321*C22
    OM123 = OM122*C23-OM322*S23
    OM223 = qd[23]+OM222
    OM323 = OM122*S23+OM322*C23
    OMp123 = C23*(OMp122-qd[23]*OM322)-S23*(OMp322+qd[23]*OM122)
    OMp223 = qdd[23]+OMp222
    OMp323 = C23*(OMp322+qd[23]*OM122)+S23*(OMp122-qd[23]*OM322)
    ALPHA123 = ALPHA121*C23-ALPHA322*S23
    ALPHA323 = ALPHA121*S23+ALPHA322*C23
    OM124 = OM123*C24+OM223*S24
    OM224 = -OM123*S24+OM223*C24
    OM324 = qd[24]+OM323
    OMp124 = C24*(OMp123+qd[24]*OM223)+S24*(OMp223-qd[24]*OM123)
    OMp224 = C24*(OMp223-qd[24]*OM123)-S24*(OMp123+qd[24]*OM223)
    OMp324 = qdd[24]+OMp323
    BS224 = OM124*OM224
    BS324 = OM124*OM324
    BS524 = -OM124*OM124-OM324*OM324
    BS624 = OM224*OM324
    BS924 = -OM124*OM124-OM224*OM224
    BETA224 = BS224-OMp324
    BETA324 = BS324+OMp224
    BETA624 = BS624-OMp124
    BETA824 = BS624+OMp124
    ALPHA124 = ALPHA123*C24+ALPHA222*S24
    ALPHA224 = -ALPHA123*S24+ALPHA222*C24
    OM125 = OM124*C25-OM324*S25
    OM225 = qd[25]+OM224
    OM325 = OM124*S25+OM324*C25
    OMp125 = C25*(OMp124-qd[25]*OM324)-S25*(OMp324+qd[25]*OM124)
    OMp225 = qdd[25]+OMp224
    OMp325 = C25*(OMp324+qd[25]*OM124)+S25*(OMp124-qd[25]*OM324)
    ALPHA125 = C25*(ALPHA124+BETA224*s.dpt[2,38]+BETA324*s.dpt[3,38])-S25*(ALPHA323+BETA824*s.dpt[2,38]+BS924*s.dpt[3,38])
    ALPHA225 = ALPHA224+BETA624*s.dpt[3,38]+BS524*s.dpt[2,38]
    ALPHA325 = C25*(ALPHA323+BETA824*s.dpt[2,38]+BS924*s.dpt[3,38])+S25*(ALPHA124+BETA224*s.dpt[2,38]+BETA324*s.dpt[3,38])
    OM126 = OM124*C26-OM324*S26
    OM226 = qd[26]+OM224
    OM326 = OM124*S26+OM324*C26
    OMp126 = C26*(OMp124-qd[26]*OM324)-S26*(OMp324+qd[26]*OM124)
    OMp226 = qdd[26]+OMp224
    OMp326 = C26*(OMp324+qd[26]*OM124)+S26*(OMp124-qd[26]*OM324)
    ALPHA126 = C26*(ALPHA124+BETA224*s.dpt[2,41]+BETA324*s.dpt[3,41])-S26*(ALPHA323+BETA824*s.dpt[2,41]+BS924*s.dpt[3,41])
    ALPHA226 = ALPHA224+BETA624*s.dpt[3,41]+BS524*s.dpt[2,41]
    ALPHA326 = C26*(ALPHA323+BETA824*s.dpt[2,41]+BS924*s.dpt[3,41])+S26*(ALPHA124+BETA224*s.dpt[2,41]+BETA324*s.dpt[3,41])
    OM127 = OM124*C27-OM324*S27
    OM227 = qd[27]+OM224
    OM327 = OM124*S27+OM324*C27
    OMp127 = C27*(OMp124-qd[27]*OM324)-S27*(OMp324+qd[27]*OM124)
    OMp227 = qdd[27]+OMp224
    OMp327 = C27*(OMp324+qd[27]*OM124)+S27*(OMp124-qd[27]*OM324)
    ALPHA127 = C27*(ALPHA124+BETA224*s.dpt[2,42])-S27*(ALPHA323+BETA824*s.dpt[2,42])
    ALPHA227 = ALPHA224+BS524*s.dpt[2,42]
    ALPHA327 = C27*(ALPHA323+BETA824*s.dpt[2,42])+S27*(ALPHA124+BETA224*s.dpt[2,42])
    OM128 = OM124*C28-OM324*S28
    OM228 = qd[28]+OM224
    OM328 = OM124*S28+OM324*C28
    OMp128 = C28*(OMp124-qd[28]*OM324)-S28*(OMp324+qd[28]*OM124)
    OMp228 = qdd[28]+OMp224
    OMp328 = C28*(OMp324+qd[28]*OM124)+S28*(OMp124-qd[28]*OM324)
    ALPHA128 = C28*(ALPHA124+BETA224*s.dpt[2,43])-S28*(ALPHA323+BETA824*s.dpt[2,43])
    ALPHA228 = ALPHA224+BS524*s.dpt[2,43]
    ALPHA328 = C28*(ALPHA323+BETA824*s.dpt[2,43])+S28*(ALPHA124+BETA224*s.dpt[2,43])
 
# Backward Dynamics

    Fs128 = -s.frc[1,28]+s.m[28]*ALPHA128
    Fs228 = -s.frc[2,28]+s.m[28]*ALPHA228
    Fs328 = -s.frc[3,28]+s.m[28]*ALPHA328
    Cq128 = -s.trq[1,28]+s.In[1,28]*OMp128-s.In[5,28]*OM228*OM328+s.In[9,28]*OM228*OM328
    Cq228 = -s.trq[2,28]+s.In[1,28]*OM128*OM328+s.In[5,28]*OMp228-s.In[9,28]*OM128*OM328
    Cq328 = -s.trq[3,28]-s.In[1,28]*OM128*OM228+s.In[5,28]*OM128*OM228+s.In[9,28]*OMp328
    Fs127 = -s.frc[1,27]+s.m[27]*ALPHA127
    Fs227 = -s.frc[2,27]+s.m[27]*ALPHA227
    Fs327 = -s.frc[3,27]+s.m[27]*ALPHA327
    Cq127 = -s.trq[1,27]+s.In[1,27]*OMp127-s.In[5,27]*OM227*OM327+s.In[9,27]*OM227*OM327
    Cq227 = -s.trq[2,27]+s.In[1,27]*OM127*OM327+s.In[5,27]*OMp227-s.In[9,27]*OM127*OM327
    Cq327 = -s.trq[3,27]-s.In[1,27]*OM127*OM227+s.In[5,27]*OM127*OM227+s.In[9,27]*OMp327
    Fs126 = -s.frc[1,26]+s.m[26]*ALPHA126
    Fs226 = -s.frc[2,26]+s.m[26]*ALPHA226
    Fs326 = -s.frc[3,26]+s.m[26]*ALPHA326
    Cq126 = -s.trq[1,26]+s.In[1,26]*OMp126-s.In[5,26]*OM226*OM326+s.In[9,26]*OM226*OM326
    Cq226 = -s.trq[2,26]+s.In[1,26]*OM126*OM326+s.In[5,26]*OMp226-s.In[9,26]*OM126*OM326
    Cq326 = -s.trq[3,26]-s.In[1,26]*OM126*OM226+s.In[5,26]*OM126*OM226+s.In[9,26]*OMp326
    Fs125 = -s.frc[1,25]+s.m[25]*ALPHA125
    Fs225 = -s.frc[2,25]+s.m[25]*ALPHA225
    Fs325 = -s.frc[3,25]+s.m[25]*ALPHA325
    Cq125 = -s.trq[1,25]+s.In[1,25]*OMp125-s.In[5,25]*OM225*OM325+s.In[9,25]*OM225*OM325
    Cq225 = -s.trq[2,25]+s.In[1,25]*OM125*OM325+s.In[5,25]*OMp225-s.In[9,25]*OM125*OM325
    Cq325 = -s.trq[3,25]-s.In[1,25]*OM125*OM225+s.In[5,25]*OM125*OM225+s.In[9,25]*OMp325
    Fs124 = -s.frc[1,24]+s.m[24]*ALPHA124
    Fs224 = -s.frc[2,24]+s.m[24]*ALPHA224
    Fs324 = -s.frc[3,24]+s.m[24]*ALPHA323
    Fq124 = Fs124+Fs125*C25+Fs126*C26+Fs127*C27+Fs128*C28+Fs325*S25+Fs326*S26+Fs327*S27+Fs328*S28
    Fq224 = Fs224+Fs225+Fs226+Fs227+Fs228
    Fq324 = Fs324-Fs125*S25-Fs126*S26-Fs127*S27-Fs128*S28+Fs325*C25+Fs326*C26+Fs327*C27+Fs328*C28
    Cq124 = -s.trq[1,24]+s.In[1,24]*OMp124-s.In[5,24]*OM224*OM324+s.In[9,24]*OM224*OM324+Cq125*C25+Cq126*C26+Cq127*C27+Cq128*C28+Cq325*S25+Cq326*S26+Cq327*S27+Cq328*S28-Fs225*s.dpt[3,38]-Fs226*s.dpt[3,41]+s.dpt[2,38]*(-Fs125*S25+Fs325*C25)+s.dpt[2,41]*(-Fs126*S26+Fs326*C26)+s.dpt[2,42]*(-Fs127*S27+Fs327*C27)+s.dpt[2,43]*(-Fs128*S28+Fs328*C28)
    Cq224 = -s.trq[2,24]+Cq225+Cq226+Cq227+Cq228+s.In[1,24]*OM124*OM324+s.In[5,24]*OMp224-s.In[9,24]*OM124*OM324+s.dpt[3,38]*(Fs125*C25+Fs325*S25)+s.dpt[3,41]*(Fs126*C26+Fs326*S26)
    Cq324 = -s.trq[3,24]-s.In[1,24]*OM124*OM224+s.In[5,24]*OM124*OM224+s.In[9,24]*OMp324-Cq125*S25-Cq126*S26-Cq127*S27-Cq128*S28+Cq325*C25+Cq326*C26+Cq327*C27+Cq328*C28-s.dpt[2,38]*(Fs125*C25+Fs325*S25)-s.dpt[2,41]*(Fs126*C26+Fs326*S26)-s.dpt[2,42]*(Fs127*C27+Fs327*S27)-s.dpt[2,43]*(Fs128*C28+Fs328*S28)
    Fq123 = Fq124*C24-Fq224*S24
    Fq223 = Fq124*S24+Fq224*C24
    Cq123 = Cq124*C24-Cq224*S24
    Cq223 = Cq124*S24+Cq224*C24
    Fq122 = Fq123*C23+Fq324*S23
    Fq322 = -Fq123*S23+Fq324*C23
    Cq122 = Cq123*C23+Cq324*S23
    Cq322 = -Cq123*S23+Cq324*C23
    Fq221 = Fq223*C22-Fq322*S22
    Fq321 = Fq223*S22+Fq322*C22
    Cq221 = Cq223*C22-Cq322*S22
    Cq321 = Cq223*S22+Cq322*C22
    Cq120 = Cq122+q[21]*Fq321
    Cq320 = Cq321-q[21]*Fq122
    Cq219 = Cq221-q[20]*Fq321
    Cq319 = Cq320+q[20]*Fq221
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
    Cq115 = -s.trq[1,15]+Cq116+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315+s.dpt[2,32]*(Fq316*C16+Fs217*S16)-s.dpt[3,32]*(-Fq316*S16+Fs217*C16)
    Cq215 = -s.trq[2,15]+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315+Cq217*C16-Cq316*S16+Fq116*s.dpt[3,32]
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315+Cq217*S16+Cq316*C16-Fq116*s.dpt[2,32]
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
    Cq113 = -s.trq[1,13]+Cq114+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+s.dpt[2,29]*(Fq214*S14+Fq315*C14)
    Cq213 = -s.trq[2,13]+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313+Cq214*C14-Cq315*S14
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313+Cq214*S14+Cq315*C14-Fq114*s.dpt[2,29]
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
    Cq19 = -s.trq[1,9]+Cq110+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39+s.dpt[2,25]*(Fq310*C10+Fs211*S10)-s.dpt[3,25]*(-Fq310*S10+Fs211*C10)
    Cq29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39+Cq211*C10-Cq310*S10+Fq110*s.dpt[3,25]
    Cq39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39+Cq211*S10+Cq310*C10-Fq110*s.dpt[2,25]
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
    Cq17 = -s.trq[1,7]+Cq18+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+s.dpt[2,22]*(Fq28*S8+Fq39*C8)
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37+Cq28*C8-Cq39*S8
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37+Cq28*S8+Cq39*C8-Fq18*s.dpt[2,22]
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA16
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA25
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA36
    Fq16 = Fq113+Fq122+Fq17+Fs112+Fs118+Fs16-s.frc[1,29]*C29-s.frc[1,30]*C30+s.frc[2,29]*S29+s.frc[2,30]*S30
    Fq26 = Fq221+Fs26-s.frc[1,29]*S29-s.frc[1,30]*S30-s.frc[2,29]*C29-s.frc[2,30]*C30+Fq213*C13+Fq27*C7-Fq313*S13-Fq37*S7+Fs212*C12+Fs218*C18-Fs312*S12-Fs318*S18
    Fq36 = -s.frc[3,29]-s.frc[3,30]+Fq321+Fs36+Fq213*S13+Fq27*S7+Fq313*C13+Fq37*C7+Fs212*S12+Fs218*S18+Fs312*C12+Fs318*C18
    Cq16 = -s.trq[1,6]+Cq112+Cq113+Cq118+Cq120+Cq17-q[19]*Fq221-s.In[5,6]*OM26*OM36-s.frc[3,29]*s.dpt[2,16]-s.frc[3,30]*s.dpt[2,17]-s.trq[1,29]*C29-s.trq[1,30]*C30+s.trq[2,29]*S29+s.trq[2,30]*S30+s.dpt[2,1]*(Fq27*S7+Fq37*C7)+s.dpt[2,2]*(Fs212*S12+Fs312*C12)+s.dpt[2,4]*(Fq213*S13+Fq313*C13)+s.dpt[2,5]*(Fs218*S18+Fs318*C18)-s.dpt[3,2]*(Fs212*C12-Fs312*S12)-s.dpt[3,5]*(Fs218*C18-Fs318*S18)
    Cq26 = -s.trq[2,6]+Cq219+q[19]*Fq122+s.In[5,6]*OMp26+s.frc[3,29]*s.dpt[1,16]+s.frc[3,30]*s.dpt[1,17]-s.trq[1,29]*S29-s.trq[1,30]*S30-s.trq[2,29]*C29-s.trq[2,30]*C30+Cq212*C12+Cq213*C13+Cq218*C18+Cq27*C7-Cq312*S12-Cq313*S13-Cq318*S18-Cq37*S7-Fq321*s.dpt[1,13]+Fs112*s.dpt[3,2]+Fs118*s.dpt[3,5]-s.dpt[1,1]*(Fq27*S7+Fq37*C7)-s.dpt[1,2]*(Fs212*S12+Fs312*C12)-s.dpt[1,4]*(Fq213*S13+Fq313*C13)-s.dpt[1,5]*(Fs218*S18+Fs318*C18)
    Cq36 = -s.trq[3,29]-s.trq[3,30]-s.trq[3,6]+Cq319+s.In[5,6]*OM16*OM26+Cq212*S12+Cq213*S13+Cq218*S18+Cq27*S7+Cq312*C12+Cq313*C13+Cq318*C18+Cq37*C7-Fq113*s.dpt[2,4]-Fq17*s.dpt[2,1]+Fq221*s.dpt[1,13]-Fs112*s.dpt[2,2]-Fs118*s.dpt[2,5]+s.dpt[1,16]*(-s.frc[1,29]*S29-s.frc[2,29]*C29)+s.dpt[1,17]*(-s.frc[1,30]*S30-s.frc[2,30]*C30)+s.dpt[1,1]*(Fq27*C7-Fq37*S7)+s.dpt[1,2]*(Fs212*C12-Fs312*S12)+s.dpt[1,4]*(Fq213*C13-Fq313*S13)+s.dpt[1,5]*(Fs218*C18-Fs318*S18)-s.dpt[2,16]*(-s.frc[1,29]*C29+s.frc[2,29]*S29)-s.dpt[2,17]*(-s.frc[1,30]*C30+s.frc[2,30]*S30)
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
    Qq[19] = Fq321
    Qq[20] = Fq122
    Qq[21] = Fq221
    Qq[22] = Cq122
    Qq[23] = Cq223
    Qq[24] = Cq324
    Qq[25] = Cq225
    Qq[26] = Cq226
    Qq[27] = Cq227
    Qq[28] = Cq228
    Qq[29] = -s.trq[3,29]
    Qq[30] = -s.trq[3,30]

# Number of continuation lines = 0


