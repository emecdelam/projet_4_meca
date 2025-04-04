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
#	==> Generation Date: Mon Mar 24 13:26:35 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: MON_LIV
#
#	==> Number of joints: 12
#
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
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
    S6p7 = C6*S7+S6*C7
    C6p7 = C6*C7-S6*S7
    S11p6 = C11*S6+S11*C6
    C11p6 = C11*C6-S11*S6
 
# Forward Kinematics

    AF24 = -s.g[3]*S4
    AF34 = -s.g[3]*C4
    OM15 = qd[4]*C5
    OM35 = qd[4]*S5
    OA15 = -qd[4]*qd[5]*S5
    OA35 = qd[4]*qd[5]*C5
    AF15 = -AF34*S5
    AF35 = AF34*C5
    AM15_2 = S4*S5
    AM35_2 = -S4*C5
    AM15_3 = -C4*S5
    AM35_3 = C4*C5
    OM16 = qd[5]*S6+OM15*C6
    OM26 = qd[5]*C6-OM15*S6
    OM36 = qd[6]+OM35
    OA16 = -qd[6]*OM15*S6+C6*(OA15+qd[5]*qd[6])
    OA26 = -qd[6]*OM15*C6-S6*(OA15+qd[5]*qd[6])
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BEF26 = BS26-OA35
    BEF46 = BS26+OA35
    BEF76 = BS36-OA26
    BEF86 = BS66+OA16
    AF16 = AF15*C6+AF24*S6
    AF26 = -AF15*S6+AF24*C6
    AM16_1 = C5*C6
    AM26_1 = -C5*S6
    AM16_2 = AM15_2*C6+C4*S6
    AM26_2 = -AM15_2*S6+C4*C6
    AM16_3 = AM15_3*C6+S4*S6
    AM26_3 = -AM15_3*S6+S4*C6
    OB16_4 = C5*C6
    OB26_4 = -C5*S6
    OM17 = OM16*C7+OM26*S7
    OM27 = -OM16*S7+OM26*C7
    OM37 = qd[7]+OM36
    OA17 = C7*(OA16+qd[7]*OM26)+S7*(OA26-qd[7]*OM16)
    OA27 = C7*(OA26-qd[7]*OM16)-S7*(OA16+qd[7]*OM26)
    AF17 = C7*(AF16+BEF26*s.dpt[2,1]+BS16*s.dpt[1,1])+S7*(AF26+BEF46*s.dpt[1,1]+BS56*s.dpt[2,1])
    AF27 = C7*(AF26+BEF46*s.dpt[1,1]+BS56*s.dpt[2,1])-S7*(AF16+BEF26*s.dpt[2,1]+BS16*s.dpt[1,1])
    AF37 = AF35+BEF76*s.dpt[1,1]+BEF86*s.dpt[2,1]
    AM17_1 = AM16_1*C7+AM26_1*S7
    AM27_1 = -AM16_1*S7+AM26_1*C7
    AM17_2 = AM16_2*C7+AM26_2*S7
    AM27_2 = -AM16_2*S7+AM26_2*C7
    AM17_3 = AM16_3*C7+AM26_3*S7
    AM27_3 = -AM16_3*S7+AM26_3*C7
    OB17_4 = OB16_4*C7+OB26_4*S7
    OB27_4 = -OB16_4*S7+OB26_4*C7
    AM17_4 = s.dpt[1,1]*S5*S7-s.dpt[2,1]*S5*C7
    AM27_4 = s.dpt[1,1]*S5*C7+s.dpt[2,1]*S5*S7
    AM37_4 = OB16_4*s.dpt[2,1]-OB26_4*s.dpt[1,1]
    AM37_5 = -s.dpt[1,1]*C6+s.dpt[2,1]*S6
    AM17_6 = s.dpt[1,1]*S7-s.dpt[2,1]*C7
    AM27_6 = s.dpt[1,1]*C7+s.dpt[2,1]*S7
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OA18 = C8*(OA17-qd[8]*OM37)-S8*(OA35+qd[8]*OM17)
    OA38 = C8*(OA35+qd[8]*OM17)+S8*(OA17-qd[8]*OM37)
    AF18 = AF17*C8-AF37*S8
    AF38 = AF17*S8+AF37*C8
    AM18_1 = AM17_1*C8-S5*S8
    AM38_1 = AM17_1*S8+S5*C8
    AM18_2 = AM17_2*C8-AM35_2*S8
    AM38_2 = AM17_2*S8+AM35_2*C8
    AM18_3 = AM17_3*C8-AM35_3*S8
    AM38_3 = AM17_3*S8+AM35_3*C8
    OB18_4 = OB17_4*C8-S5*S8
    OB38_4 = OB17_4*S8+S5*C8
    AM18_4 = AM17_4*C8-AM37_4*S8
    AM38_4 = AM17_4*S8+AM37_4*C8
    OB18_5 = S6p7*C8
    OB38_5 = S6p7*S8
    AM18_5 = -AM37_5*S8
    AM38_5 = AM37_5*C8
    AM18_6 = AM17_6*C8
    AM38_6 = AM17_6*S8
    OM19 = OM16*C9-OM36*S9
    OM29 = qd[9]+OM26
    OM39 = OM16*S9+OM36*C9
    OA19 = C9*(OA16-qd[9]*OM36)-S9*(OA35+qd[9]*OM16)
    OA39 = C9*(OA35+qd[9]*OM16)+S9*(OA16-qd[9]*OM36)
    AF19 = C9*(AF16+BEF26*s.dpt[2,2]+BS16*s.dpt[1,2])-S9*(AF35+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2])
    AF29 = AF26+BEF46*s.dpt[1,2]+BS56*s.dpt[2,2]
    AF39 = C9*(AF35+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2])+S9*(AF16+BEF26*s.dpt[2,2]+BS16*s.dpt[1,2])
    AM19_1 = AM16_1*C9-S5*S9
    AM39_1 = AM16_1*S9+S5*C9
    AM19_2 = AM16_2*C9-AM35_2*S9
    AM39_2 = AM16_2*S9+AM35_2*C9
    AM19_3 = AM16_3*C9-AM35_3*S9
    AM39_3 = AM16_3*S9+AM35_3*C9
    OB19_4 = OB16_4*C9-S5*S9
    OB39_4 = OB16_4*S9+S5*C9
    AM19_4 = -s.dpt[2,2]*S5*C9-S9*(OB16_4*s.dpt[2,2]-OB26_4*s.dpt[1,2])
    AM29_4 = s.dpt[1,2]*S5
    AM39_4 = -s.dpt[2,2]*S5*S9+C9*(OB16_4*s.dpt[2,2]-OB26_4*s.dpt[1,2])
    OB19_5 = S6*C9
    OB39_5 = S6*S9
    AM19_5 = -S9*(-s.dpt[1,2]*C6+s.dpt[2,2]*S6)
    AM39_5 = C9*(-s.dpt[1,2]*C6+s.dpt[2,2]*S6)
    AM19_6 = -s.dpt[2,2]*C9
    AM39_6 = -s.dpt[2,2]*S9
    OM110 = OM16*C10-OM36*S10
    OM210 = qd[10]+OM26
    OM310 = OM16*S10+OM36*C10
    OA110 = C10*(OA16-qd[10]*OM36)-S10*(OA35+qd[10]*OM16)
    OA310 = C10*(OA35+qd[10]*OM16)+S10*(OA16-qd[10]*OM36)
    AF110 = C10*(AF16+BEF26*s.dpt[2,3]+BS16*s.dpt[1,3])-S10*(AF35+BEF76*s.dpt[1,3]+BEF86*s.dpt[2,3])
    AF210 = AF26+BEF46*s.dpt[1,3]+BS56*s.dpt[2,3]
    AF310 = C10*(AF35+BEF76*s.dpt[1,3]+BEF86*s.dpt[2,3])+S10*(AF16+BEF26*s.dpt[2,3]+BS16*s.dpt[1,3])
    AM110_1 = AM16_1*C10-S10*S5
    AM310_1 = AM16_1*S10+C10*S5
    AM110_2 = AM16_2*C10-AM35_2*S10
    AM310_2 = AM16_2*S10+AM35_2*C10
    AM110_3 = AM16_3*C10-AM35_3*S10
    AM310_3 = AM16_3*S10+AM35_3*C10
    OB110_4 = OB16_4*C10-S10*S5
    OB310_4 = OB16_4*S10+C10*S5
    AM110_4 = -s.dpt[2,3]*C10*S5-S10*(OB16_4*s.dpt[2,3]-OB26_4*s.dpt[1,3])
    AM210_4 = s.dpt[1,3]*S5
    AM310_4 = -s.dpt[2,3]*S10*S5+C10*(OB16_4*s.dpt[2,3]-OB26_4*s.dpt[1,3])
    OB110_5 = C10*S6
    OB310_5 = S10*S6
    AM110_5 = -S10*(-s.dpt[1,3]*C6+s.dpt[2,3]*S6)
    AM310_5 = C10*(-s.dpt[1,3]*C6+s.dpt[2,3]*S6)
    AM110_6 = -s.dpt[2,3]*C10
    AM310_6 = -s.dpt[2,3]*S10
    OM111 = OM16*C11+OM26*S11
    OM211 = -OM16*S11+OM26*C11
    OM311 = qd[11]+OM36
    OA111 = C11*(OA16+qd[11]*OM26)+S11*(OA26-qd[11]*OM16)
    OA211 = C11*(OA26-qd[11]*OM16)-S11*(OA16+qd[11]*OM26)
    AF111 = C11*(AF16+BEF26*s.dpt[2,4]+BS16*s.dpt[1,4])+S11*(AF26+BEF46*s.dpt[1,4]+BS56*s.dpt[2,4])
    AF211 = C11*(AF26+BEF46*s.dpt[1,4]+BS56*s.dpt[2,4])-S11*(AF16+BEF26*s.dpt[2,4]+BS16*s.dpt[1,4])
    AF311 = AF35+BEF76*s.dpt[1,4]+BEF86*s.dpt[2,4]
    AM111_1 = AM16_1*C11+AM26_1*S11
    AM211_1 = -AM16_1*S11+AM26_1*C11
    AM111_2 = AM16_2*C11+AM26_2*S11
    AM211_2 = -AM16_2*S11+AM26_2*C11
    AM111_3 = AM16_3*C11+AM26_3*S11
    AM211_3 = -AM16_3*S11+AM26_3*C11
    OB111_4 = OB16_4*C11+OB26_4*S11
    OB211_4 = -OB16_4*S11+OB26_4*C11
    AM111_4 = s.dpt[1,4]*S11*S5-s.dpt[2,4]*C11*S5
    AM211_4 = s.dpt[1,4]*C11*S5+s.dpt[2,4]*S11*S5
    AM311_4 = OB16_4*s.dpt[2,4]-OB26_4*s.dpt[1,4]
    AM311_5 = -s.dpt[1,4]*C6+s.dpt[2,4]*S6
    AM111_6 = s.dpt[1,4]*S11-s.dpt[2,4]*C11
    AM211_6 = s.dpt[1,4]*C11+s.dpt[2,4]*S11
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OA112 = C12*(OA111-qd[12]*OM311)-S12*(OA35+qd[12]*OM111)
    OA312 = C12*(OA35+qd[12]*OM111)+S12*(OA111-qd[12]*OM311)
    AF112 = AF111*C12-AF311*S12
    AF312 = AF111*S12+AF311*C12
    AM112_1 = AM111_1*C12-S12*S5
    AM312_1 = AM111_1*S12+C12*S5
    AM112_2 = AM111_2*C12-AM35_2*S12
    AM312_2 = AM111_2*S12+AM35_2*C12
    AM112_3 = AM111_3*C12-AM35_3*S12
    AM312_3 = AM111_3*S12+AM35_3*C12
    OB112_4 = OB111_4*C12-S12*S5
    OB312_4 = OB111_4*S12+C12*S5
    AM112_4 = AM111_4*C12-AM311_4*S12
    AM312_4 = AM111_4*S12+AM311_4*C12
    OB112_5 = S11p6*C12
    OB312_5 = S11p6*S12
    AM112_5 = -AM311_5*S12
    AM312_5 = AM311_5*C12
    AM112_6 = AM111_6*C12
    AM312_6 = AM111_6*S12
 
# Backward Dynamics

    FA112 = -s.frc[1,12]+s.m[12]*AF112
    FA212 = -s.frc[2,12]+s.m[12]*AF211
    FA312 = -s.frc[3,12]+s.m[12]*AF312
    CF112 = -s.trq[1,12]+s.In[1,12]*OA112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312
    CF212 = -s.trq[2,12]+s.In[1,12]*OM112*OM312+s.In[5,12]*OA211-s.In[9,12]*OM112*OM312
    CF312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OA312
    FB112_1 = s.m[12]*AM112_1
    FB212_1 = s.m[12]*AM211_1
    FB312_1 = s.m[12]*AM312_1
    FB112_2 = s.m[12]*AM112_2
    FB212_2 = s.m[12]*AM211_2
    FB312_2 = s.m[12]*AM312_2
    FB112_3 = s.m[12]*AM112_3
    FB212_3 = s.m[12]*AM211_3
    FB312_3 = s.m[12]*AM312_3
    FB112_4 = s.m[12]*AM112_4
    FB212_4 = s.m[12]*AM211_4
    FB312_4 = s.m[12]*AM312_4
    CM112_4 = s.In[1,12]*OB112_4
    CM212_4 = s.In[5,12]*OB211_4
    CM312_4 = s.In[9,12]*OB312_4
    FB112_5 = s.m[12]*AM112_5
    FB312_5 = s.m[12]*AM312_5
    CM112_5 = s.In[1,12]*OB112_5
    CM212_5 = s.In[5,12]*C11p6
    CM312_5 = s.In[9,12]*OB312_5
    FB112_6 = s.m[12]*AM112_6
    FB212_6 = s.m[12]*AM211_6
    FB312_6 = s.m[12]*AM312_6
    CM112_6 = -s.In[1,12]*S12
    CM312_6 = s.In[9,12]*C12
    CM112_11 = -s.In[1,12]*S12
    CM312_11 = s.In[9,12]*C12
    FF111 = FA112*C12+FA312*S12
    FF311 = -FA112*S12+FA312*C12
    CF111 = CF112*C12+CF312*S12
    CF311 = -CF112*S12+CF312*C12
    FM111_1 = FB112_1*C12+FB312_1*S12
    FM311_1 = -FB112_1*S12+FB312_1*C12
    FM111_2 = FB112_2*C12+FB312_2*S12
    FM311_2 = -FB112_2*S12+FB312_2*C12
    FM111_3 = FB112_3*C12+FB312_3*S12
    FM311_3 = -FB112_3*S12+FB312_3*C12
    FM111_4 = FB112_4*C12+FB312_4*S12
    FM311_4 = -FB112_4*S12+FB312_4*C12
    CM111_4 = CM112_4*C12+CM312_4*S12
    CM311_4 = -CM112_4*S12+CM312_4*C12
    FM111_5 = FB112_5*C12+FB312_5*S12
    FM311_5 = -FB112_5*S12+FB312_5*C12
    CM111_5 = CM112_5*C12+CM312_5*S12
    CM311_5 = -CM112_5*S12+CM312_5*C12
    FM111_6 = FB112_6*C12+FB312_6*S12
    CM311_6 = -CM112_6*S12+CM312_6*C12
    CM311_11 = -CM112_11*S12+CM312_11*C12
    FA110 = -s.frc[1,10]+s.m[10]*AF110
    FA210 = -s.frc[2,10]+s.m[10]*AF210
    FA310 = -s.frc[3,10]+s.m[10]*AF310
    CF110 = -s.trq[1,10]+s.In[1,10]*OA110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    CF210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OA26-s.In[9,10]*OM110*OM310
    CF310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OA310
    FB110_1 = s.m[10]*AM110_1
    FB210_1 = s.m[10]*AM26_1
    FB310_1 = s.m[10]*AM310_1
    FB110_2 = s.m[10]*AM110_2
    FB210_2 = s.m[10]*AM26_2
    FB310_2 = s.m[10]*AM310_2
    FB110_3 = s.m[10]*AM110_3
    FB210_3 = s.m[10]*AM26_3
    FB310_3 = s.m[10]*AM310_3
    FB110_4 = s.m[10]*AM110_4
    FB210_4 = s.m[10]*AM210_4
    FB310_4 = s.m[10]*AM310_4
    CM110_4 = s.In[1,10]*OB110_4
    CM210_4 = s.In[5,10]*OB26_4
    CM310_4 = s.In[9,10]*OB310_4
    FB110_5 = s.m[10]*AM110_5
    FB310_5 = s.m[10]*AM310_5
    CM110_5 = s.In[1,10]*OB110_5
    CM210_5 = s.In[5,10]*C6
    CM310_5 = s.In[9,10]*OB310_5
    FB110_6 = s.m[10]*AM110_6
    FB210_6 = s.m[10]*s.dpt[1,3]
    FB310_6 = s.m[10]*AM310_6
    CM110_6 = -s.In[1,10]*S10
    CM310_6 = s.In[9,10]*C10
    FA19 = -s.frc[1,9]+s.m[9]*AF19
    FA29 = -s.frc[2,9]+s.m[9]*AF29
    FA39 = -s.frc[3,9]+s.m[9]*AF39
    CF19 = -s.trq[1,9]+s.In[1,9]*OA19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39
    CF29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OA26-s.In[9,9]*OM19*OM39
    CF39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OA39
    FB19_1 = s.m[9]*AM19_1
    FB29_1 = s.m[9]*AM26_1
    FB39_1 = s.m[9]*AM39_1
    FB19_2 = s.m[9]*AM19_2
    FB29_2 = s.m[9]*AM26_2
    FB39_2 = s.m[9]*AM39_2
    FB19_3 = s.m[9]*AM19_3
    FB29_3 = s.m[9]*AM26_3
    FB39_3 = s.m[9]*AM39_3
    FB19_4 = s.m[9]*AM19_4
    FB29_4 = s.m[9]*AM29_4
    FB39_4 = s.m[9]*AM39_4
    CM19_4 = s.In[1,9]*OB19_4
    CM29_4 = s.In[5,9]*OB26_4
    CM39_4 = s.In[9,9]*OB39_4
    FB19_5 = s.m[9]*AM19_5
    FB39_5 = s.m[9]*AM39_5
    CM19_5 = s.In[1,9]*OB19_5
    CM29_5 = s.In[5,9]*C6
    CM39_5 = s.In[9,9]*OB39_5
    FB19_6 = s.m[9]*AM19_6
    FB29_6 = s.m[9]*s.dpt[1,2]
    FB39_6 = s.m[9]*AM39_6
    CM19_6 = -s.In[1,9]*S9
    CM39_6 = s.In[9,9]*C9
    FA18 = -s.frc[1,8]+s.m[8]*AF18
    FA28 = -s.frc[2,8]+s.m[8]*AF27
    FA38 = -s.frc[3,8]+s.m[8]*AF38
    CF18 = -s.trq[1,8]+s.In[1,8]*OA18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38
    CF28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OA27-s.In[9,8]*OM18*OM38
    CF38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OA38
    FB18_1 = s.m[8]*AM18_1
    FB28_1 = s.m[8]*AM27_1
    FB38_1 = s.m[8]*AM38_1
    FB18_2 = s.m[8]*AM18_2
    FB28_2 = s.m[8]*AM27_2
    FB38_2 = s.m[8]*AM38_2
    FB18_3 = s.m[8]*AM18_3
    FB28_3 = s.m[8]*AM27_3
    FB38_3 = s.m[8]*AM38_3
    FB18_4 = s.m[8]*AM18_4
    FB28_4 = s.m[8]*AM27_4
    FB38_4 = s.m[8]*AM38_4
    CM18_4 = s.In[1,8]*OB18_4
    CM28_4 = s.In[5,8]*OB27_4
    CM38_4 = s.In[9,8]*OB38_4
    FB18_5 = s.m[8]*AM18_5
    FB38_5 = s.m[8]*AM38_5
    CM18_5 = s.In[1,8]*OB18_5
    CM28_5 = s.In[5,8]*C6p7
    CM38_5 = s.In[9,8]*OB38_5
    FB18_6 = s.m[8]*AM18_6
    FB28_6 = s.m[8]*AM27_6
    FB38_6 = s.m[8]*AM38_6
    CM18_6 = -s.In[1,8]*S8
    CM38_6 = s.In[9,8]*C8
    CM18_7 = -s.In[1,8]*S8
    CM38_7 = s.In[9,8]*C8
    FF17 = FA18*C8+FA38*S8
    FF37 = -FA18*S8+FA38*C8
    CF17 = CF18*C8+CF38*S8
    CF37 = -CF18*S8+CF38*C8
    FM17_1 = FB18_1*C8+FB38_1*S8
    FM37_1 = -FB18_1*S8+FB38_1*C8
    FM17_2 = FB18_2*C8+FB38_2*S8
    FM37_2 = -FB18_2*S8+FB38_2*C8
    FM17_3 = FB18_3*C8+FB38_3*S8
    FM37_3 = -FB18_3*S8+FB38_3*C8
    FM17_4 = FB18_4*C8+FB38_4*S8
    FM37_4 = -FB18_4*S8+FB38_4*C8
    CM17_4 = CM18_4*C8+CM38_4*S8
    CM37_4 = -CM18_4*S8+CM38_4*C8
    FM17_5 = FB18_5*C8+FB38_5*S8
    FM37_5 = -FB18_5*S8+FB38_5*C8
    CM17_5 = CM18_5*C8+CM38_5*S8
    CM37_5 = -CM18_5*S8+CM38_5*C8
    FM17_6 = FB18_6*C8+FB38_6*S8
    CM37_6 = -CM18_6*S8+CM38_6*C8
    CM37_7 = -CM18_7*S8+CM38_7*C8
    FA16 = -s.frc[1,6]+s.m[6]*AF16
    FA26 = -s.frc[2,6]+s.m[6]*AF26
    FA36 = -s.frc[3,6]+s.m[6]*AF35
    FF16 = FA16+FA110*C10+FA19*C9-FA212*S11-FA28*S7+FA310*S10+FA39*S9+FF111*C11+FF17*C7
    FF26 = FA210+FA26+FA29+FA212*C11+FA28*C7+FF111*S11+FF17*S7
    FF36 = FA36+FF311+FF37-FA110*S10-FA19*S9+FA310*C10+FA39*C9
    CF16 = -s.trq[1,6]+s.In[1,6]*OA16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+CF110*C10+CF111*C11+CF17*C7+CF19*C9-CF212*S11-CF28*S7+CF310*S10+CF39*S9+FF311*s.dpt[2,4]+FF37*s.dpt[2,1]+s.dpt[2,2]*(-FA19*S9+FA39*C9)+s.dpt[2,3]*(-FA110*S10+FA310*C10)
    CF26 = -s.trq[2,6]+CF210+CF29+s.In[1,6]*OM16*OM36+s.In[5,6]*OA26-s.In[9,6]*OM16*OM36+CF111*S11+CF17*S7+CF212*C11+CF28*C7-FF311*s.dpt[1,4]-FF37*s.dpt[1,1]-s.dpt[1,2]*(-FA19*S9+FA39*C9)-s.dpt[1,3]*(-FA110*S10+FA310*C10)
    CF36 = -s.trq[3,6]+CF311+CF37-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OA35-CF110*S10-CF19*S9+CF310*C10+CF39*C9+FA210*s.dpt[1,3]+FA29*s.dpt[1,2]+s.dpt[1,1]*(FA28*C7+FF17*S7)+s.dpt[1,4]*(FA212*C11+FF111*S11)-s.dpt[2,1]*(-FA28*S7+FF17*C7)-s.dpt[2,2]*(FA19*C9+FA39*S9)-s.dpt[2,3]*(FA110*C10+FA310*S10)-s.dpt[2,4]*(-FA212*S11+FF111*C11)
    FB16_1 = s.m[6]*AM16_1
    FB26_1 = s.m[6]*AM26_1
    FB36_1 = s.m[6]*S5
    FM16_1 = FB16_1+FB110_1*C10+FB19_1*C9-FB212_1*S11-FB28_1*S7+FB310_1*S10+FB39_1*S9+FM111_1*C11+FM17_1*C7
    FM26_1 = FB210_1+FB26_1+FB29_1+FB212_1*C11+FB28_1*C7+FM111_1*S11+FM17_1*S7
    FM36_1 = FB36_1+FM311_1+FM37_1-FB110_1*S10-FB19_1*S9+FB310_1*C10+FB39_1*C9
    CM16_1 = FM311_1*s.dpt[2,4]+FM37_1*s.dpt[2,1]+s.dpt[2,2]*(-FB19_1*S9+FB39_1*C9)+s.dpt[2,3]*(-FB110_1*S10+FB310_1*C10)
    CM26_1 = -FM311_1*s.dpt[1,4]-FM37_1*s.dpt[1,1]-s.dpt[1,2]*(-FB19_1*S9+FB39_1*C9)-s.dpt[1,3]*(-FB110_1*S10+FB310_1*C10)
    CM36_1 = FB210_1*s.dpt[1,3]+FB29_1*s.dpt[1,2]+s.dpt[1,1]*(FB28_1*C7+FM17_1*S7)+s.dpt[1,4]*(FB212_1*C11+FM111_1*S11)-s.dpt[2,1]*(-FB28_1*S7+FM17_1*C7)-s.dpt[2,2]*(FB19_1*C9+FB39_1*S9)-s.dpt[2,3]*(FB110_1*C10+FB310_1*S10)-s.dpt[2,4]*(-FB212_1*S11+FM111_1*C11)
    FB16_2 = s.m[6]*AM16_2
    FB26_2 = s.m[6]*AM26_2
    FB36_2 = s.m[6]*AM35_2
    FM16_2 = FB16_2+FB110_2*C10+FB19_2*C9-FB212_2*S11-FB28_2*S7+FB310_2*S10+FB39_2*S9+FM111_2*C11+FM17_2*C7
    FM26_2 = FB210_2+FB26_2+FB29_2+FB212_2*C11+FB28_2*C7+FM111_2*S11+FM17_2*S7
    FM36_2 = FB36_2+FM311_2+FM37_2-FB110_2*S10-FB19_2*S9+FB310_2*C10+FB39_2*C9
    CM16_2 = FM311_2*s.dpt[2,4]+FM37_2*s.dpt[2,1]+s.dpt[2,2]*(-FB19_2*S9+FB39_2*C9)+s.dpt[2,3]*(-FB110_2*S10+FB310_2*C10)
    CM26_2 = -FM311_2*s.dpt[1,4]-FM37_2*s.dpt[1,1]-s.dpt[1,2]*(-FB19_2*S9+FB39_2*C9)-s.dpt[1,3]*(-FB110_2*S10+FB310_2*C10)
    CM36_2 = FB210_2*s.dpt[1,3]+FB29_2*s.dpt[1,2]+s.dpt[1,1]*(FB28_2*C7+FM17_2*S7)+s.dpt[1,4]*(FB212_2*C11+FM111_2*S11)-s.dpt[2,1]*(-FB28_2*S7+FM17_2*C7)-s.dpt[2,2]*(FB19_2*C9+FB39_2*S9)-s.dpt[2,3]*(FB110_2*C10+FB310_2*S10)-s.dpt[2,4]*(-FB212_2*S11+FM111_2*C11)
    FB16_3 = s.m[6]*AM16_3
    FB26_3 = s.m[6]*AM26_3
    FB36_3 = s.m[6]*AM35_3
    FM16_3 = FB16_3+FB110_3*C10+FB19_3*C9-FB212_3*S11-FB28_3*S7+FB310_3*S10+FB39_3*S9+FM111_3*C11+FM17_3*C7
    FM26_3 = FB210_3+FB26_3+FB29_3+FB212_3*C11+FB28_3*C7+FM111_3*S11+FM17_3*S7
    FM36_3 = FB36_3+FM311_3+FM37_3-FB110_3*S10-FB19_3*S9+FB310_3*C10+FB39_3*C9
    CM16_3 = FM311_3*s.dpt[2,4]+FM37_3*s.dpt[2,1]+s.dpt[2,2]*(-FB19_3*S9+FB39_3*C9)+s.dpt[2,3]*(-FB110_3*S10+FB310_3*C10)
    CM26_3 = -FM311_3*s.dpt[1,4]-FM37_3*s.dpt[1,1]-s.dpt[1,2]*(-FB19_3*S9+FB39_3*C9)-s.dpt[1,3]*(-FB110_3*S10+FB310_3*C10)
    CM36_3 = FB210_3*s.dpt[1,3]+FB29_3*s.dpt[1,2]+s.dpt[1,1]*(FB28_3*C7+FM17_3*S7)+s.dpt[1,4]*(FB212_3*C11+FM111_3*S11)-s.dpt[2,1]*(-FB28_3*S7+FM17_3*C7)-s.dpt[2,2]*(FB19_3*C9+FB39_3*S9)-s.dpt[2,3]*(FB110_3*C10+FB310_3*S10)-s.dpt[2,4]*(-FB212_3*S11+FM111_3*C11)
    CM16_4 = s.In[1,6]*OB16_4+CM110_4*C10+CM111_4*C11+CM17_4*C7+CM19_4*C9-CM212_4*S11-CM28_4*S7+CM310_4*S10+CM39_4*S9+FM311_4*s.dpt[2,4]+FM37_4*s.dpt[2,1]+s.dpt[2,2]*(-FB19_4*S9+FB39_4*C9)+s.dpt[2,3]*(-FB110_4*S10+FB310_4*C10)
    CM26_4 = CM210_4+CM29_4+s.In[5,6]*OB26_4+CM111_4*S11+CM17_4*S7+CM212_4*C11+CM28_4*C7-FM311_4*s.dpt[1,4]-FM37_4*s.dpt[1,1]-s.dpt[1,2]*(-FB19_4*S9+FB39_4*C9)-s.dpt[1,3]*(-FB110_4*S10+FB310_4*C10)
    CM36_4 = CM311_4+CM37_4+s.In[9,6]*S5-CM110_4*S10-CM19_4*S9+CM310_4*C10+CM39_4*C9+FB210_4*s.dpt[1,3]+FB29_4*s.dpt[1,2]+s.dpt[1,1]*(FB28_4*C7+FM17_4*S7)+s.dpt[1,4]*(FB212_4*C11+FM111_4*S11)-s.dpt[2,1]*(-FB28_4*S7+FM17_4*C7)-s.dpt[2,2]*(FB19_4*C9+FB39_4*S9)-s.dpt[2,3]*(FB110_4*C10+FB310_4*S10)-s.dpt[2,4]*(-FB212_4*S11+FM111_4*C11)
    CM16_5 = s.In[1,6]*S6+CM110_5*C10+CM111_5*C11+CM17_5*C7+CM19_5*C9-CM212_5*S11-CM28_5*S7+CM310_5*S10+CM39_5*S9+FM311_5*s.dpt[2,4]+FM37_5*s.dpt[2,1]+s.dpt[2,2]*(-FB19_5*S9+FB39_5*C9)+s.dpt[2,3]*(-FB110_5*S10+FB310_5*C10)
    CM26_5 = CM210_5+CM29_5+s.In[5,6]*C6+CM111_5*S11+CM17_5*S7+CM212_5*C11+CM28_5*C7-FM311_5*s.dpt[1,4]-FM37_5*s.dpt[1,1]-s.dpt[1,2]*(-FB19_5*S9+FB39_5*C9)-s.dpt[1,3]*(-FB110_5*S10+FB310_5*C10)
    CM36_5 = CM311_5+CM37_5-CM110_5*S10-CM19_5*S9+CM310_5*C10+CM39_5*C9+FM111_5*s.dpt[1,4]*S11-FM111_5*s.dpt[2,4]*C11+FM17_5*s.dpt[1,1]*S7-FM17_5*s.dpt[2,1]*C7-s.dpt[2,2]*(FB19_5*C9+FB39_5*S9)-s.dpt[2,3]*(FB110_5*C10+FB310_5*S10)
    CM36_6 = s.In[9,6]+CM311_6+CM37_6-CM110_6*S10-CM19_6*S9+CM310_6*C10+CM39_6*C9+FB210_6*s.dpt[1,3]+FB29_6*s.dpt[1,2]+s.dpt[1,1]*(FB28_6*C7+FM17_6*S7)+s.dpt[1,4]*(FB212_6*C11+FM111_6*S11)-s.dpt[2,1]*(-FB28_6*S7+FM17_6*C7)-s.dpt[2,2]*(FB19_6*C9+FB39_6*S9)-s.dpt[2,3]*(FB110_6*C10+FB310_6*S10)-s.dpt[2,4]*(-FB212_6*S11+FM111_6*C11)
    FF15 = FF16*C6-FF26*S6
    FF25 = FF16*S6+FF26*C6
    CF15 = CF16*C6-CF26*S6
    CF25 = CF16*S6+CF26*C6
    FM15_1 = FM16_1*C6-FM26_1*S6
    FM25_1 = FM16_1*S6+FM26_1*C6
    CM15_1 = CM16_1*C6-CM26_1*S6
    CM25_1 = CM16_1*S6+CM26_1*C6
    FM15_2 = FM16_2*C6-FM26_2*S6
    FM25_2 = FM16_2*S6+FM26_2*C6
    CM15_2 = CM16_2*C6-CM26_2*S6
    CM25_2 = CM16_2*S6+CM26_2*C6
    FM15_3 = FM16_3*C6-FM26_3*S6
    FM25_3 = FM16_3*S6+FM26_3*C6
    CM15_3 = CM16_3*C6-CM26_3*S6
    CM25_3 = CM16_3*S6+CM26_3*C6
    CM15_4 = CM16_4*C6-CM26_4*S6
    CM25_4 = CM16_4*S6+CM26_4*C6
    CM25_5 = CM16_5*S6+CM26_5*C6
    FF14 = FF15*C5+FF36*S5
    FF34 = -FF15*S5+FF36*C5
    CF14 = CF15*C5+CF36*S5
    FM14_1 = FM15_1*C5+FM36_1*S5
    FM34_1 = -FM15_1*S5+FM36_1*C5
    CM14_1 = CM15_1*C5+CM36_1*S5
    FM34_2 = -FM15_2*S5+FM36_2*C5
    CM14_2 = CM15_2*C5+CM36_2*S5
    FM34_3 = -FM15_3*S5+FM36_3*C5
    CM14_3 = CM15_3*C5+CM36_3*S5
    CM14_4 = CM15_4*C5+CM36_4*S5
    FF23 = FF25*C4-FF34*S4
    FF33 = FF25*S4+FF34*C4
    FM23_1 = FM25_1*C4-FM34_1*S4
    FM33_1 = FM25_1*S4+FM34_1*C4
    FM23_2 = FM25_2*C4-FM34_2*S4
    FM33_2 = FM25_2*S4+FM34_2*C4
    FM33_3 = FM25_3*S4+FM34_3*C4
 
# Symbolic model output

    c[1] = FF14
    c[2] = FF23
    c[3] = FF33
    c[4] = CF14
    c[5] = CF25
    c[6] = CF36
    c[7] = CF37
    c[8] = CF28
    c[9] = CF29
    c[10] = CF210
    c[11] = CF311
    c[12] = CF212
    M[1,1] = FM14_1
    M[1,2] = FM23_1
    M[1,3] = FM33_1
    M[1,4] = CM14_1
    M[1,5] = CM25_1
    M[1,6] = CM36_1
    M[2,1] = FM23_1
    M[2,2] = FM23_2
    M[2,3] = FM33_2
    M[2,4] = CM14_2
    M[2,5] = CM25_2
    M[2,6] = CM36_2
    M[3,1] = FM33_1
    M[3,2] = FM33_2
    M[3,3] = FM33_3
    M[3,4] = CM14_3
    M[3,5] = CM25_3
    M[3,6] = CM36_3
    M[4,1] = CM14_1
    M[4,2] = CM14_2
    M[4,3] = CM14_3
    M[4,4] = CM14_4
    M[4,5] = CM25_4
    M[4,6] = CM36_4
    M[4,7] = CM37_4
    M[4,8] = CM28_4
    M[4,9] = CM29_4
    M[4,10] = CM210_4
    M[4,11] = CM311_4
    M[4,12] = CM212_4
    M[5,1] = CM25_1
    M[5,2] = CM25_2
    M[5,3] = CM25_3
    M[5,4] = CM25_4
    M[5,5] = CM25_5
    M[5,6] = CM36_5
    M[5,7] = CM37_5
    M[5,8] = CM28_5
    M[5,9] = CM29_5
    M[5,10] = CM210_5
    M[5,11] = CM311_5
    M[5,12] = CM212_5
    M[6,1] = CM36_1
    M[6,2] = CM36_2
    M[6,3] = CM36_3
    M[6,4] = CM36_4
    M[6,5] = CM36_5
    M[6,6] = CM36_6
    M[6,7] = CM37_6
    M[6,11] = CM311_6
    M[7,4] = CM37_4
    M[7,5] = CM37_5
    M[7,6] = CM37_6
    M[7,7] = CM37_7
    M[8,4] = CM28_4
    M[8,5] = CM28_5
    M[8,8] = s.In[5,8]
    M[9,4] = CM29_4
    M[9,5] = CM29_5
    M[9,9] = s.In[5,9]
    M[10,4] = CM210_4
    M[10,5] = CM210_5
    M[10,10] = s.In[5,10]
    M[11,4] = CM311_4
    M[11,5] = CM311_5
    M[11,6] = CM311_6
    M[11,11] = CM311_11
    M[12,4] = CM212_4
    M[12,5] = CM212_5
    M[12,12] = s.In[5,12]

# Number of continuation lines = 0


