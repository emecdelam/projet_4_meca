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
#	==> Generation Date: Thu Mar 20 14:58:16 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
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

    ALPHA24 = qdd[2]*C4+qdd[3]*S4
    ALPHA34 = -qdd[2]*S4+qdd[3]*C4
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
    BS96 = -OM16*OM16-OM26*OM26
    BETA26 = BS26-OMp36
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA15*C6+ALPHA24*S6
    ALPHA26 = -ALPHA15*S6+ALPHA24*C6
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2]
    ALPHA27 = C7*(ALPHA26+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S7*(ALPHA35+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA37 = C7*(ALPHA35+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S7*(ALPHA26+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])
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
    ALPHA18 = ALPHA16+BETA26*s.dpt[2,3]+BS16*s.dpt[1,3]
    ALPHA28 = C8*(ALPHA26+BETA46*s.dpt[1,3]+BS56*s.dpt[2,3])+S8*(ALPHA35+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])
    ALPHA38 = C8*(ALPHA35+BETA76*s.dpt[1,3]+BETA86*s.dpt[2,3])-S8*(ALPHA26+BETA46*s.dpt[1,3]+BS56*s.dpt[2,3])
    OM19 = OM18*C9-OM38*S9
    OM29 = qd[9]+OM28
    OM39 = OM18*S9+OM38*C9
    OMp19 = C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18)
    OMp29 = qdd[9]+OMp28
    OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38)
    BS29 = OM19*OM29
    BS39 = OM19*OM39
    BS59 = -OM19*OM19-OM39*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA29 = BS29-OMp39
    BETA39 = BS39+OMp29
    BETA69 = BS69-OMp19
    BETA89 = BS69+OMp19
    ALPHA19 = C9*(ALPHA18+BETA28*s.dpt[2,8])-S9*(ALPHA38+BETA88*s.dpt[2,8])
    ALPHA29 = ALPHA28+BS58*s.dpt[2,8]
    ALPHA39 = C9*(ALPHA38+BETA88*s.dpt[2,8])+S9*(ALPHA18+BETA28*s.dpt[2,8])
    ALPHA110 = C10*(ALPHA19+BETA29*s.dpt[2,11]+BETA39*s.dpt[3,11])-S10*(ALPHA39+BETA89*s.dpt[2,11]+BS99*s.dpt[3,11])
    ALPHA210 = ALPHA29+BETA69*s.dpt[3,11]+BS59*s.dpt[2,11]
    ALPHA310 = C10*(ALPHA39+BETA89*s.dpt[2,11]+BS99*s.dpt[3,11])+S10*(ALPHA19+BETA29*s.dpt[2,11]+BETA39*s.dpt[3,11])
 
# Backward Dynamics

    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA310
    Fs19 = -s.frc[1,9]+s.m[9]*ALPHA19
    Fs29 = -s.frc[2,9]+s.m[9]*ALPHA29
    Fs39 = -s.frc[3,9]+s.m[9]*ALPHA39
    Fq19 = Fs19+Fs110*C10+Fs310*S10
    Fq29 = Fs210+Fs29
    Fq39 = Fs39-Fs110*S10+Fs310*C10
    Cq19 = -s.trq[1,9]-s.trq[1,10]*C10-s.trq[3,10]*S10-Fs210*s.dpt[3,11]+s.dpt[2,11]*(-Fs110*S10+Fs310*C10)
    Cq29 = -s.trq[2,10]-s.trq[2,9]+s.dpt[3,11]*(Fs110*C10+Fs310*S10)
    Cq39 = -s.trq[3,9]+s.trq[1,10]*S10-s.trq[3,10]*C10-s.dpt[2,11]*(Fs110*C10+Fs310*S10)
    Fs18 = -s.frc[1,8]+s.m[8]*ALPHA18
    Fs28 = -s.frc[2,8]+s.m[8]*ALPHA28
    Fs38 = -s.frc[3,8]+s.m[8]*ALPHA38
    Fq18 = Fs18+Fq19*C9+Fq39*S9
    Fq28 = Fq29+Fs28
    Fq38 = Fs38-Fq19*S9+Fq39*C9
    Cq18 = -s.trq[1,8]+Cq19*C9+Cq39*S9+s.dpt[2,8]*(-Fq19*S9+Fq39*C9)
    Cq28 = -s.trq[2,8]+Cq29
    Cq38 = -s.trq[3,8]-Cq19*S9+Cq39*C9-s.dpt[2,8]*(Fq19*C9+Fq39*S9)
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA16
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA26
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA35
    Fq16 = -s.frc[1,11]-s.frc[1,12]+Fq18+Fs16+Fs17
    Fq26 = Fs26-s.frc[2,11]*C11-s.frc[2,12]*C12+s.frc[3,11]*S11+s.frc[3,12]*S12+Fq28*C8-Fq38*S8+Fs27*C7-Fs37*S7
    Fq36 = Fs36-s.frc[2,11]*S11-s.frc[2,12]*S12-s.frc[3,11]*C11-s.frc[3,12]*C12+Fq28*S8+Fq38*C8+Fs27*S7+Fs37*C7
    Cq16 = -s.trq[1,11]-s.trq[1,12]-s.trq[1,6]-s.trq[1,7]+Cq18+s.dpt[2,2]*(Fs27*S7+Fs37*C7)+s.dpt[2,3]*(Fq28*S8+Fq38*C8)+s.dpt[2,5]*(-s.frc[2,11]*S11-s.frc[3,11]*C11)+s.dpt[2,6]*(-s.frc[2,12]*S12-s.frc[3,12]*C12)-s.dpt[3,2]*(Fs27*C7-Fs37*S7)-s.dpt[3,5]*(-s.frc[2,11]*C11+s.frc[3,11]*S11)
    Cq26 = -s.trq[2,6]-s.frc[1,11]*s.dpt[3,5]-s.trq[2,11]*C11-s.trq[2,12]*C12-s.trq[2,7]*C7+s.trq[3,11]*S11+s.trq[3,12]*S12+s.trq[3,7]*S7+Cq28*C8-Cq38*S8+Fs17*s.dpt[3,2]-s.dpt[1,2]*(Fs27*S7+Fs37*C7)-s.dpt[1,3]*(Fq28*S8+Fq38*C8)-s.dpt[1,5]*(-s.frc[2,11]*S11-s.frc[3,11]*C11)-s.dpt[1,6]*(-s.frc[2,12]*S12-s.frc[3,12]*C12)
    Cq36 = -s.trq[3,6]+s.frc[1,11]*s.dpt[2,5]+s.frc[1,12]*s.dpt[2,6]-s.trq[2,11]*S11-s.trq[2,12]*S12-s.trq[2,7]*S7-s.trq[3,11]*C11-s.trq[3,12]*C12-s.trq[3,7]*C7+Cq28*S8+Cq38*C8-Fq18*s.dpt[2,3]-Fs17*s.dpt[2,2]+s.dpt[1,2]*(Fs27*C7-Fs37*S7)+s.dpt[1,3]*(Fq28*C8-Fq38*S8)+s.dpt[1,5]*(-s.frc[2,11]*C11+s.frc[3,11]*S11)+s.dpt[1,6]*(-s.frc[2,12]*C12+s.frc[3,12]*S12)
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
    Qq[7] = -s.trq[1,7]
    Qq[8] = Cq18
    Qq[9] = Cq29
    Qq[10] = -s.trq[2,10]
    Qq[11] = -s.trq[1,11]
    Qq[12] = -s.trq[1,12]

# Number of continuation lines = 0


