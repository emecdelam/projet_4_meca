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
    S31 = sin(q[31])
    C31 = cos(q[31])
    S32 = sin(q[32])
    C32 = cos(q[32])
 
# Augmented Joint Position Vectors

    Dz191 = q[19]+s.dpt[1,7]
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM15 = -qd[4]*S5
    OM35 = qd[4]*C5
    OMp15 = -qd[4]*qd[5]*C5-qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA15 = -qdd[3]*S5+ALPHA14*C5
    ALPHA35 = qdd[3]*C5+ALPHA14*S5
    OM16 = qd[6]+OM15
    OM26 = qd[5]*C6+OM35*S6
    OM36 = -qd[5]*S6+OM35*C6
    OMp16 = qdd[6]+OMp15
    OMp26 = C6*(qdd[5]+qd[6]*OM35)+S6*(OMp35-qd[5]*qd[6])
    OMp36 = C6*(OMp35-qd[5]*qd[6])-S6*(qdd[5]+qd[6]*OM35)
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
    ALPHA26 = ALPHA24*C6+ALPHA35*S6
    ALPHA36 = -ALPHA24*S6+ALPHA35*C6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[7]+OMp16
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    ALPHA17 = ALPHA15+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2]
    ALPHA27 = C7*(ALPHA26+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S7*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S7*(ALPHA26+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])
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
    ALPHA18 = ALPHA15+BETA26*s.dpt[2,3]+BS16*s.dpt[1,3]
    ALPHA28 = C8*(ALPHA26+BETA46*s.dpt[1,3]+BS56*s.dpt[2,3])+S8*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])
    ALPHA38 = C8*(ALPHA36+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])-S8*(ALPHA26+BETA46*s.dpt[1,3]+BS56*s.dpt[2,3])
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OMp19 = qdd[9]+OMp18
    OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28)
    OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38)
    ALPHA19 = ALPHA18+BETA28*s.dpt[2,17]
    ALPHA29 = C9*(ALPHA28+BS58*s.dpt[2,17])+S9*(ALPHA38+BETA88*s.dpt[2,17])
    ALPHA39 = C9*(ALPHA38+BETA88*s.dpt[2,17])-S9*(ALPHA28+BS58*s.dpt[2,17])
    OM110 = OM19*C10+OM29*S10
    OM210 = -OM19*S10+OM29*C10
    OM310 = qd[10]+OM39
    OMp110 = C10*(OMp19+qd[10]*OM29)+S10*(OMp29-qd[10]*OM19)
    OMp210 = C10*(OMp29-qd[10]*OM19)-S10*(OMp19+qd[10]*OM29)
    OMp310 = qdd[10]+OMp39
    BS210 = OM110*OM210
    BS510 = -OM110*OM110-OM310*OM310
    BS610 = OM210*OM310
    BETA210 = BS210-OMp310
    BETA810 = BS610+OMp110
    ALPHA110 = ALPHA19*C10+ALPHA29*S10
    ALPHA210 = -ALPHA19*S10+ALPHA29*C10
    OM111 = qd[11]+OM110
    OM211 = OM210*C11+OM310*S11
    OM311 = -OM210*S11+OM310*C11
    OMp111 = qdd[11]+OMp110
    OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210)
    OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310)
    ALPHA111 = ALPHA110+BETA210*s.dpt[2,20]
    ALPHA211 = C11*(ALPHA210+BS510*s.dpt[2,20])+S11*(ALPHA39+BETA810*s.dpt[2,20])
    ALPHA311 = C11*(ALPHA39+BETA810*s.dpt[2,20])-S11*(ALPHA210+BS510*s.dpt[2,20])
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
    ALPHA113 = ALPHA15+BETA26*s.dpt[2,5]+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5]
    ALPHA213 = C13*(ALPHA26+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])+S13*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])
    ALPHA313 = C13*(ALPHA36+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])-S13*(ALPHA26+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])
    OM114 = qd[14]+OM16
    OM214 = OM26*C14+OM36*S14
    OM314 = -OM26*S14+OM36*C14
    OMp114 = qdd[14]+OMp16
    OMp214 = C14*(OMp26+qd[14]*OM36)+S14*(OMp36-qd[14]*OM26)
    OMp314 = C14*(OMp36-qd[14]*OM26)-S14*(OMp26+qd[14]*OM36)
    ALPHA114 = ALPHA15+BETA26*s.dpt[2,6]+BS16*s.dpt[1,6]
    ALPHA214 = C14*(ALPHA26+BETA46*s.dpt[1,6]+BS56*s.dpt[2,6])+S14*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6])
    ALPHA314 = C14*(ALPHA36+BETA76*s.dpt[1,6]+BETA86*s.dpt[2,6])-S14*(ALPHA26+BETA46*s.dpt[1,6]+BS56*s.dpt[2,6])
    OM115 = qd[15]+OM114
    OM215 = OM214*C15+OM314*S15
    OM315 = -OM214*S15+OM314*C15
    OMp115 = qdd[15]+OMp114
    OMp215 = C15*(OMp214+qd[15]*OM314)+S15*(OMp314-qd[15]*OM214)
    OMp315 = C15*(OMp314-qd[15]*OM214)-S15*(OMp214+qd[15]*OM314)
    OM116 = OM115*C16+OM215*S16
    OM216 = -OM115*S16+OM215*C16
    OM316 = qd[16]+OM315
    OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115)
    OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215)
    OMp316 = qdd[16]+OMp315
    OM117 = qd[17]+OM116
    OM217 = OM216*C17+OM316*S17
    OM317 = -OM216*S17+OM316*C17
    OMp117 = qdd[17]+OMp116
    OMp217 = C17*(OMp216+qd[17]*OM316)+S17*(OMp316-qd[17]*OM216)
    OMp317 = C17*(OMp316-qd[17]*OM216)-S17*(OMp216+qd[17]*OM316)
    OM118 = OM117*C18-OM317*S18
    OM218 = qd[18]+OM217
    OM318 = OM117*S18+OM317*C18
    OMp118 = C18*(OMp117-qd[18]*OM317)-S18*(OMp317+qd[18]*OM117)
    OMp218 = qdd[18]+OMp217
    OMp318 = C18*(OMp317+qd[18]*OM117)+S18*(OMp117-qd[18]*OM317)
    BS219 = OM16*OM26
    BS519 = -OM16*OM16-OM36*OM36
    BS619 = OM26*OM36
    BETA219 = BS219-OMp36
    BETA819 = BS619+OMp16
    ALPHA119 = qdd[19]+ALPHA15+BS16*Dz191
    ALPHA219 = ALPHA26+(2.0)*qd[19]*OM36+BETA46*Dz191
    ALPHA319 = ALPHA36-(2.0)*qd[19]*OM26+BETA76*Dz191
    BS320 = OM16*OM36
    BS620 = OM26*OM36
    BS920 = -OM16*OM16-OM26*OM26
    BETA320 = BS320+OMp26
    BETA620 = BS620-OMp16
    ALPHA120 = ALPHA119+q[20]*BETA219-(2.0)*qd[20]*OM36
    ALPHA220 = qdd[20]+ALPHA219+q[20]*BS519
    ALPHA320 = ALPHA319+q[20]*BETA819+(2.0)*qd[20]*OM16
    ALPHA121 = ALPHA120+q[21]*BETA320+(2.0)*qd[21]*OM26
    ALPHA221 = ALPHA220+q[21]*BETA620-(2.0)*qd[21]*OM16
    ALPHA321 = qdd[21]+ALPHA320+q[21]*BS920
    OM122 = OM16*C22+OM26*S22
    OM222 = -OM16*S22+OM26*C22
    OM322 = qd[22]+OM36
    OMp122 = C22*(OMp16+qd[22]*OM26)+S22*(OMp26-qd[22]*OM16)
    OMp222 = C22*(OMp26-qd[22]*OM16)-S22*(OMp16+qd[22]*OM26)
    OMp322 = qdd[22]+OMp36
    ALPHA122 = ALPHA121*C22+ALPHA221*S22
    ALPHA222 = -ALPHA121*S22+ALPHA221*C22
    OM123 = OM122*C23-OM322*S23
    OM223 = qd[23]+OM222
    OM323 = OM122*S23+OM322*C23
    OMp123 = C23*(OMp122-qd[23]*OM322)-S23*(OMp322+qd[23]*OM122)
    OMp223 = qdd[23]+OMp222
    OMp323 = C23*(OMp322+qd[23]*OM122)+S23*(OMp122-qd[23]*OM322)
    ALPHA123 = ALPHA122*C23-ALPHA321*S23
    ALPHA323 = ALPHA122*S23+ALPHA321*C23
    OM124 = qd[24]+OM123
    OM224 = OM223*C24+OM323*S24
    OM324 = -OM223*S24+OM323*C24
    OMp124 = qdd[24]+OMp123
    OMp224 = C24*(OMp223+qd[24]*OM323)+S24*(OMp323-qd[24]*OM223)
    OMp324 = C24*(OMp323-qd[24]*OM223)-S24*(OMp223+qd[24]*OM323)
    BS224 = OM124*OM224
    BS324 = OM124*OM324
    BS524 = -OM124*OM124-OM324*OM324
    BS624 = OM224*OM324
    BS924 = -OM124*OM124-OM224*OM224
    BETA224 = BS224-OMp324
    BETA324 = BS324+OMp224
    BETA624 = BS624-OMp124
    BETA824 = BS624+OMp124
    ALPHA224 = ALPHA222*C24+ALPHA323*S24
    ALPHA324 = -ALPHA222*S24+ALPHA323*C24
    OM125 = OM124*C25-OM324*S25
    OM225 = qd[25]+OM224
    OM325 = OM124*S25+OM324*C25
    OMp125 = C25*(OMp124-qd[25]*OM324)-S25*(OMp324+qd[25]*OM124)
    OMp225 = qdd[25]+OMp224
    OMp325 = C25*(OMp324+qd[25]*OM124)+S25*(OMp124-qd[25]*OM324)
    ALPHA125 = C25*(ALPHA123+BETA224*s.dpt[2,29])-S25*(ALPHA324+BETA824*s.dpt[2,29])
    ALPHA225 = ALPHA224+BS524*s.dpt[2,29]
    ALPHA325 = C25*(ALPHA324+BETA824*s.dpt[2,29])+S25*(ALPHA123+BETA224*s.dpt[2,29])
    OM126 = OM124*C26-OM324*S26
    OM226 = qd[26]+OM224
    OM326 = OM124*S26+OM324*C26
    OMp126 = C26*(OMp124-qd[26]*OM324)-S26*(OMp324+qd[26]*OM124)
    OMp226 = qdd[26]+OMp224
    OMp326 = C26*(OMp324+qd[26]*OM124)+S26*(OMp124-qd[26]*OM324)
    ALPHA126 = C26*(ALPHA123+BETA224*s.dpt[2,32])-S26*(ALPHA324+BETA824*s.dpt[2,32])
    ALPHA226 = ALPHA224+BS524*s.dpt[2,32]
    ALPHA326 = C26*(ALPHA324+BETA824*s.dpt[2,32])+S26*(ALPHA123+BETA224*s.dpt[2,32])
    OM127 = OM124*C27-OM324*S27
    OM227 = qd[27]+OM224
    OM327 = OM124*S27+OM324*C27
    OMp127 = C27*(OMp124-qd[27]*OM324)-S27*(OMp324+qd[27]*OM124)
    OMp227 = qdd[27]+OMp224
    OMp327 = C27*(OMp324+qd[27]*OM124)+S27*(OMp124-qd[27]*OM324)
    ALPHA127 = C27*(ALPHA123+BETA324*s.dpt[3,34])-S27*(ALPHA324+BS924*s.dpt[3,34])
    ALPHA227 = ALPHA224+BETA624*s.dpt[3,34]
    ALPHA327 = C27*(ALPHA324+BS924*s.dpt[3,34])+S27*(ALPHA123+BETA324*s.dpt[3,34])
    OM128 = OM124*C28+OM224*S28
    OM228 = -OM124*S28+OM224*C28
    OM328 = qd[28]+OM324
    OMp128 = C28*(OMp124+qd[28]*OM224)+S28*(OMp224-qd[28]*OM124)
    OMp228 = C28*(OMp224-qd[28]*OM124)-S28*(OMp124+qd[28]*OM224)
    OMp328 = qdd[28]+OMp324
    ALPHA128 = C28*(ALPHA123+BETA324*s.dpt[3,35])+S28*(ALPHA224+BETA624*s.dpt[3,35])
    ALPHA228 = C28*(ALPHA224+BETA624*s.dpt[3,35])-S28*(ALPHA123+BETA324*s.dpt[3,35])
    ALPHA328 = ALPHA324+BS924*s.dpt[3,35]
    OM129 = OM128*C29-OM328*S29
    OM229 = qd[29]+OM228
    OM329 = OM128*S29+OM328*C29
    OMp129 = C29*(OMp128-qd[29]*OM328)-S29*(OMp328+qd[29]*OM128)
    OMp229 = qdd[29]+OMp228
    OMp329 = C29*(OMp328+qd[29]*OM128)+S29*(OMp128-qd[29]*OM328)
    ALPHA129 = ALPHA128*C29-ALPHA328*S29
    ALPHA329 = ALPHA128*S29+ALPHA328*C29
    OM130 = OM124*C30+OM224*S30
    OM230 = -OM124*S30+OM224*C30
    OM330 = qd[30]+OM324
    OMp130 = C30*(OMp124+qd[30]*OM224)+S30*(OMp224-qd[30]*OM124)
    OMp230 = C30*(OMp224-qd[30]*OM124)-S30*(OMp124+qd[30]*OM224)
    OMp330 = qdd[30]+OMp324
    ALPHA130 = C30*(ALPHA123+BETA324*s.dpt[3,36])+S30*(ALPHA224+BETA624*s.dpt[3,36])
    ALPHA230 = C30*(ALPHA224+BETA624*s.dpt[3,36])-S30*(ALPHA123+BETA324*s.dpt[3,36])
    ALPHA330 = ALPHA324+BS924*s.dpt[3,36]
    OM131 = OM130*C31-OM330*S31
    OM231 = qd[31]+OM230
    OM331 = OM130*S31+OM330*C31
    OMp131 = C31*(OMp130-qd[31]*OM330)-S31*(OMp330+qd[31]*OM130)
    OMp231 = qdd[31]+OMp230
    OMp331 = C31*(OMp330+qd[31]*OM130)+S31*(OMp130-qd[31]*OM330)
    ALPHA131 = ALPHA130*C31-ALPHA330*S31
    ALPHA331 = ALPHA130*S31+ALPHA330*C31
    OM132 = OM124*C32-OM324*S32
    OM232 = qd[32]+OM224
    OM332 = OM124*S32+OM324*C32
    OMp132 = C32*(OMp124-qd[32]*OM324)-S32*(OMp324+qd[32]*OM124)
    OMp232 = qdd[32]+OMp224
    OMp332 = C32*(OMp324+qd[32]*OM124)+S32*(OMp124-qd[32]*OM324)
    ALPHA132 = C32*(ALPHA123+BETA324*s.dpt[3,37])-S32*(ALPHA324+BS924*s.dpt[3,37])
    ALPHA232 = ALPHA224+BETA624*s.dpt[3,37]
    ALPHA332 = C32*(ALPHA324+BS924*s.dpt[3,37])+S32*(ALPHA123+BETA324*s.dpt[3,37])
 
# Backward Dynamics

    Fs132 = -s.frc[1,32]+s.m[32]*ALPHA132
    Fs232 = -s.frc[2,32]+s.m[32]*ALPHA232
    Fs332 = -s.frc[3,32]+s.m[32]*ALPHA332
    Cq132 = -s.trq[1,32]+s.In[1,32]*OMp132-s.In[5,32]*OM232*OM332+s.In[9,32]*OM232*OM332
    Cq232 = -s.trq[2,32]+s.In[1,32]*OM132*OM332+s.In[5,32]*OMp232-s.In[9,32]*OM132*OM332
    Cq332 = -s.trq[3,32]-s.In[1,32]*OM132*OM232+s.In[5,32]*OM132*OM232+s.In[9,32]*OMp332
    Fs131 = -s.frc[1,31]+s.m[31]*ALPHA131
    Fs231 = -s.frc[2,31]+s.m[31]*ALPHA230
    Fs331 = -s.frc[3,31]+s.m[31]*ALPHA331
    Cq131 = -s.trq[1,31]+s.In[1,31]*OMp131-s.In[5,31]*OM231*OM331+s.In[9,31]*OM231*OM331
    Cq231 = -s.trq[2,31]+s.In[1,31]*OM131*OM331+s.In[5,31]*OMp231-s.In[9,31]*OM131*OM331
    Cq331 = -s.trq[3,31]-s.In[1,31]*OM131*OM231+s.In[5,31]*OM131*OM231+s.In[9,31]*OMp331
    Fq130 = Fs131*C31+Fs331*S31
    Fq330 = -Fs131*S31+Fs331*C31
    Cq130 = Cq131*C31+Cq331*S31
    Cq330 = -Cq131*S31+Cq331*C31
    Fs129 = -s.frc[1,29]+s.m[29]*ALPHA129
    Fs229 = -s.frc[2,29]+s.m[29]*ALPHA228
    Fs329 = -s.frc[3,29]+s.m[29]*ALPHA329
    Cq129 = -s.trq[1,29]+s.In[1,29]*OMp129-s.In[5,29]*OM229*OM329+s.In[9,29]*OM229*OM329
    Cq229 = -s.trq[2,29]+s.In[1,29]*OM129*OM329+s.In[5,29]*OMp229-s.In[9,29]*OM129*OM329
    Cq329 = -s.trq[3,29]-s.In[1,29]*OM129*OM229+s.In[5,29]*OM129*OM229+s.In[9,29]*OMp329
    Fq128 = Fs129*C29+Fs329*S29
    Fq328 = -Fs129*S29+Fs329*C29
    Cq128 = Cq129*C29+Cq329*S29
    Cq328 = -Cq129*S29+Cq329*C29
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
    Fq124 = -s.frc[1,24]+Fq128*C28+Fq130*C30+Fs125*C25+Fs126*C26+Fs127*C27+Fs132*C32-Fs229*S28-Fs231*S30+Fs325*S25+Fs326*S26+Fs327*S27+Fs332*S32
    Fq224 = -s.frc[2,24]+Fs225+Fs226+Fs227+Fs232+Fq128*S28+Fq130*S30+Fs229*C28+Fs231*C30
    Fq324 = -s.frc[3,24]+Fq328+Fq330-Fs125*S25-Fs126*S26-Fs127*S27-Fs132*S32+Fs325*C25+Fs326*C26+Fs327*C27+Fs332*C32
    Cq124 = -s.trq[1,24]+s.In[1,24]*OMp124-s.In[5,24]*OM224*OM324+s.In[9,24]*OM224*OM324+Cq125*C25+Cq126*C26+Cq127*C27+Cq128*C28+Cq130*C30+Cq132*C32-Cq229*S28-Cq231*S30+Cq325*S25+Cq326*S26+Cq327*S27+Cq332*S32-Fs227*s.dpt[3,34]-Fs232*s.dpt[3,37]+s.dpt[2,29]*(-Fs125*S25+Fs325*C25)+s.dpt[2,32]*(-Fs126*S26+Fs326*C26)-s.dpt[3,35]*(Fq128*S28+Fs229*C28)-s.dpt[3,36]*(Fq130*S30+Fs231*C30)
    Cq224 = -s.trq[2,24]+Cq225+Cq226+Cq227+Cq232+s.In[1,24]*OM124*OM324+s.In[5,24]*OMp224-s.In[9,24]*OM124*OM324+Cq128*S28+Cq130*S30+Cq229*C28+Cq231*C30+s.dpt[3,34]*(Fs127*C27+Fs327*S27)+s.dpt[3,35]*(Fq128*C28-Fs229*S28)+s.dpt[3,36]*(Fq130*C30-Fs231*S30)+s.dpt[3,37]*(Fs132*C32+Fs332*S32)
    Cq324 = -s.trq[3,24]+Cq328+Cq330-s.In[1,24]*OM124*OM224+s.In[5,24]*OM124*OM224+s.In[9,24]*OMp324-Cq125*S25-Cq126*S26-Cq127*S27-Cq132*S32+Cq325*C25+Cq326*C26+Cq327*C27+Cq332*C32-s.dpt[2,29]*(Fs125*C25+Fs325*S25)-s.dpt[2,32]*(Fs126*C26+Fs326*S26)
    Fq223 = Fq224*C24-Fq324*S24
    Fq323 = Fq224*S24+Fq324*C24
    Cq223 = Cq224*C24-Cq324*S24
    Cq323 = Cq224*S24+Cq324*C24
    Fq122 = Fq124*C23+Fq323*S23
    Fq322 = -Fq124*S23+Fq323*C23
    Cq122 = Cq124*C23+Cq323*S23
    Cq322 = -Cq124*S23+Cq323*C23
    Fq121 = Fq122*C22-Fq223*S22
    Fq221 = Fq122*S22+Fq223*C22
    Cq121 = Cq122*C22-Cq223*S22
    Cq221 = Cq122*S22+Cq223*C22
    Cq120 = Cq121-q[21]*Fq221
    Cq220 = Cq221+q[21]*Fq121
    Cq119 = Cq120+q[20]*Fq322
    Cq319 = Cq322-q[20]*Fq121
    Cq118 = -s.trq[1,18]+s.In[1,18]*OMp118-s.In[5,18]*OM218*OM318+s.In[9,18]*OM218*OM318
    Cq218 = -s.trq[2,18]+s.In[1,18]*OM118*OM318+s.In[5,18]*OMp218-s.In[9,18]*OM118*OM318
    Cq318 = -s.trq[3,18]-s.In[1,18]*OM118*OM218+s.In[5,18]*OM118*OM218+s.In[9,18]*OMp318
    Fq117 = -s.frc[1,18]*C18-s.frc[3,18]*S18
    Fq317 = s.frc[1,18]*S18-s.frc[3,18]*C18
    Cq117 = Cq118*C18+Cq318*S18
    Cq317 = -Cq118*S18+Cq318*C18
    Fq116 = -s.frc[1,16]+Fq117
    Fq216 = -s.frc[2,16]-s.frc[2,18]*C17-Fq317*S17
    Fq316 = -s.frc[3,16]-s.frc[2,18]*S17+Fq317*C17
    Cq116 = -s.trq[1,16]+Cq117+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316+s.dpt[2,26]*(-s.frc[2,18]*S17+Fq317*C17)
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316+Cq218*C17-Cq317*S17
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316+Cq218*S17+Cq317*C17-Fq117*s.dpt[2,26]
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
    Cq114 = -s.trq[1,14]+Cq115+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314+s.dpt[2,23]*(Fq215*S15+Fq316*C15)
    Cq214 = -s.trq[2,14]+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314+Cq215*C15-Cq316*S15
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314+Cq215*S15+Cq316*C15-Fq115*s.dpt[2,23]
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
    Cq110 = -s.trq[1,10]+Cq111+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+s.dpt[2,20]*(Fq311*C11+Fs212*S11)
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310+Cq212*C11-Cq311*S11
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310+Cq212*S11+Cq311*C11-Fq111*s.dpt[2,20]
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
    Cq18 = -s.trq[1,8]+Cq19+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+s.dpt[2,17]*(Fq29*S9+Fq310*C9)
    Cq28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38+Cq29*C9-Cq310*S9
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38+Cq29*S9+Cq310*C9-Fq19*s.dpt[2,17]
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA15
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA26
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA36
    Fq16 = Fq114+Fq121+Fq18+Fs113+Fs16+Fs17
    Fq26 = Fq221+Fs26+Fq214*C14+Fq28*C8-Fq314*S14-Fq38*S8+Fs213*C13+Fs27*C7-Fs313*S13-Fs37*S7
    Fq36 = Fq322+Fs36+Fq214*S14+Fq28*S8+Fq314*C14+Fq38*C8+Fs213*S13+Fs27*S7+Fs313*C13+Fs37*C7
    Cq16 = -s.trq[1,6]+Cq113+Cq114+Cq119+Cq17+Cq18+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+s.dpt[2,2]*(Fs27*S7+Fs37*C7)+s.dpt[2,3]*(Fq28*S8+Fq38*C8)+s.dpt[2,5]*(Fs213*S13+Fs313*C13)+s.dpt[2,6]*(Fq214*S14+Fq314*C14)-s.dpt[3,2]*(Fs27*C7-Fs37*S7)-s.dpt[3,5]*(Fs213*C13-Fs313*S13)
    Cq26 = -s.trq[2,6]+Cq220+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq213*C13+Cq214*C14+Cq27*C7+Cq28*C8-Cq313*S13-Cq314*S14-Cq37*S7-Cq38*S8-Fq322*Dz191+Fs113*s.dpt[3,5]+Fs17*s.dpt[3,2]-s.dpt[1,2]*(Fs27*S7+Fs37*C7)-s.dpt[1,3]*(Fq28*S8+Fq38*C8)-s.dpt[1,5]*(Fs213*S13+Fs313*C13)-s.dpt[1,6]*(Fq214*S14+Fq314*C14)
    Cq36 = -s.trq[3,6]+Cq319-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36+Cq213*S13+Cq214*S14+Cq27*S7+Cq28*S8+Cq313*C13+Cq314*C14+Cq37*C7+Cq38*C8-Fq114*s.dpt[2,6]-Fq18*s.dpt[2,3]+Fq221*Dz191-Fs113*s.dpt[2,5]-Fs17*s.dpt[2,2]+s.dpt[1,2]*(Fs27*C7-Fs37*S7)+s.dpt[1,3]*(Fq28*C8-Fq38*S8)+s.dpt[1,5]*(Fs213*C13-Fs313*S13)+s.dpt[1,6]*(Fq214*C14-Fq314*S14)
    Fq25 = Fq26*C6-Fq36*S6
    Fq35 = Fq26*S6+Fq36*C6
    Cq25 = Cq26*C6-Cq36*S6
    Cq35 = Cq26*S6+Cq36*C6
    Fq14 = Fq16*C5+Fq35*S5
    Fq34 = -Fq16*S5+Fq35*C5
    Cq34 = -Cq16*S5+Cq35*C5
    Fq13 = Fq14*C4-Fq25*S4
    Fq23 = Fq14*S4+Fq25*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq25
    Qq[6] = Cq16
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
    Qq[19] = Fq121
    Qq[20] = Fq221
    Qq[21] = Fq322
    Qq[22] = Cq322
    Qq[23] = Cq223
    Qq[24] = Cq124
    Qq[25] = Cq225
    Qq[26] = Cq226
    Qq[27] = Cq227
    Qq[28] = Cq328
    Qq[29] = Cq229
    Qq[30] = Cq330
    Qq[31] = Cq231
    Qq[32] = Cq232

# Number of continuation lines = 0


