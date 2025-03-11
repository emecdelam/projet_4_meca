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
#	==> Generation Date: Tue Mar 11 10:18:23 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Caisse_a_savon
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
    C6p7 = C6*C7-S6*S7
    S6p7 = C6*S7+S6*C7
    C6p8 = C6*C8-S6*S8
    S6p8 = C6*S8+S6*C8
 
# Forward Kinematics

    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OA25 = qd[4]*qd[5]*C5
    OA35 = -qd[4]*qd[5]*S5
    AF25 = -s.g[3]*S5
    AF35 = -s.g[3]*C5
    AM25_1 = -S4*C5
    AM35_1 = S4*S5
    AM25_2 = C4*C5
    AM35_2 = -C4*S5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OA16 = -qd[6]*OM35*C6-S6*(OA35+qd[5]*qd[6])
    OA36 = -qd[6]*OM35*S6+C6*(OA35+qd[5]*qd[6])
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BEF26 = BS26-OA36
    BEF46 = BS26+OA36
    BEF76 = BS36-OA25
    BEF86 = BS66+OA16
    AF16 = -AF35*S6
    AF36 = AF35*C6
    AM16_1 = -AM35_1*S6+C4*C6
    AM36_1 = AM35_1*C6+C4*S6
    AM16_2 = -AM35_2*S6+S4*C6
    AM36_2 = AM35_2*C6+S4*S6
    AM16_3 = -C5*S6
    AM36_3 = C5*C6
    OB16_4 = -C5*S6
    OB36_4 = C5*C6
    OM17 = OM16*C7-OM36*S7
    OM27 = qd[7]+OM26
    OM37 = OM16*S7+OM36*C7
    OA17 = C7*(OA16-qd[7]*OM36)-S7*(OA36+qd[7]*OM16)
    OA37 = C7*(OA36+qd[7]*OM16)+S7*(OA16-qd[7]*OM36)
    AF17 = C7*(AF16+BEF26*s.dpt[2,1]+BS16*s.dpt[1,1])-S7*(AF36+BEF76*s.dpt[1,1]+BEF86*s.dpt[2,1])
    AF27 = AF25+BEF46*s.dpt[1,1]+BS56*s.dpt[2,1]
    AF37 = C7*(AF36+BEF76*s.dpt[1,1]+BEF86*s.dpt[2,1])+S7*(AF16+BEF26*s.dpt[2,1]+BS16*s.dpt[1,1])
    AM17_1 = AM16_1*C7-AM36_1*S7
    AM37_1 = AM16_1*S7+AM36_1*C7
    AM17_2 = AM16_2*C7-AM36_2*S7
    AM37_2 = AM16_2*S7+AM36_2*C7
    AM17_3 = AM16_3*C7-AM36_3*S7
    AM37_3 = AM16_3*S7+AM36_3*C7
    OB17_4 = OB16_4*C7-OB36_4*S7
    OB37_4 = OB16_4*S7+OB36_4*C7
    AM17_4 = -OB36_4*s.dpt[2,1]*C7-S7*(OB16_4*s.dpt[2,1]-s.dpt[1,1]*S5)
    AM27_4 = OB36_4*s.dpt[1,1]
    AM37_4 = -OB36_4*s.dpt[2,1]*S7+C7*(OB16_4*s.dpt[2,1]-s.dpt[1,1]*S5)
    AM17_5 = -s.dpt[2,1]*C6*S7-s.dpt[2,1]*S6*C7
    AM27_5 = s.dpt[1,1]*S6
    AM37_5 = s.dpt[2,1]*C6*C7-s.dpt[2,1]*S6*S7
    AM17_6 = s.dpt[1,1]*S7
    AM37_6 = -s.dpt[1,1]*C7
    OM18 = OM16*C8-OM36*S8
    OM28 = qd[8]+OM26
    OM38 = OM16*S8+OM36*C8
    OA18 = C8*(OA16-qd[8]*OM36)-S8*(OA36+qd[8]*OM16)
    OA38 = C8*(OA36+qd[8]*OM16)+S8*(OA16-qd[8]*OM36)
    AF18 = C8*(AF16+BEF26*s.dpt[2,2]+BS16*s.dpt[1,2])-S8*(AF36+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2])
    AF28 = AF25+BEF46*s.dpt[1,2]+BS56*s.dpt[2,2]
    AF38 = C8*(AF36+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2])+S8*(AF16+BEF26*s.dpt[2,2]+BS16*s.dpt[1,2])
    AM18_1 = AM16_1*C8-AM36_1*S8
    AM38_1 = AM16_1*S8+AM36_1*C8
    AM18_2 = AM16_2*C8-AM36_2*S8
    AM38_2 = AM16_2*S8+AM36_2*C8
    AM18_3 = AM16_3*C8-AM36_3*S8
    AM38_3 = AM16_3*S8+AM36_3*C8
    OB18_4 = OB16_4*C8-OB36_4*S8
    OB38_4 = OB16_4*S8+OB36_4*C8
    AM18_4 = -OB36_4*s.dpt[2,2]*C8-S8*(OB16_4*s.dpt[2,2]-s.dpt[1,2]*S5)
    AM28_4 = OB36_4*s.dpt[1,2]
    AM38_4 = -OB36_4*s.dpt[2,2]*S8+C8*(OB16_4*s.dpt[2,2]-s.dpt[1,2]*S5)
    AM18_5 = -s.dpt[2,2]*C6*S8-s.dpt[2,2]*S6*C8
    AM28_5 = s.dpt[1,2]*S6
    AM38_5 = s.dpt[2,2]*C6*C8-s.dpt[2,2]*S6*S8
    AM18_6 = s.dpt[1,2]*S8
    AM38_6 = -s.dpt[1,2]*C8
    OM19 = OM16*C9+OM26*S9
    OM29 = -OM16*S9+OM26*C9
    OM39 = qd[9]+OM36
    OA19 = C9*(OA16+qd[9]*OM26)+S9*(OA25-qd[9]*OM16)
    OA29 = C9*(OA25-qd[9]*OM16)-S9*(OA16+qd[9]*OM26)
    AF19 = C9*(AF16+BEF26*s.dpt[2,3]+BS16*s.dpt[1,3])+S9*(AF25+BEF46*s.dpt[1,3]+BS56*s.dpt[2,3])
    AF29 = C9*(AF25+BEF46*s.dpt[1,3]+BS56*s.dpt[2,3])-S9*(AF16+BEF26*s.dpt[2,3]+BS16*s.dpt[1,3])
    AF39 = AF36+BEF76*s.dpt[1,3]+BEF86*s.dpt[2,3]
    AM19_1 = AM16_1*C9+AM25_1*S9
    AM29_1 = -AM16_1*S9+AM25_1*C9
    AM19_2 = AM16_2*C9+AM25_2*S9
    AM29_2 = -AM16_2*S9+AM25_2*C9
    AM19_3 = AM16_3*C9+S5*S9
    AM29_3 = -AM16_3*S9+S5*C9
    OB19_4 = OB16_4*C9+S5*S9
    OB29_4 = -OB16_4*S9+S5*C9
    AM19_4 = OB36_4*s.dpt[1,3]*S9-OB36_4*s.dpt[2,3]*C9
    AM29_4 = OB36_4*s.dpt[1,3]*C9+OB36_4*s.dpt[2,3]*S9
    AM39_4 = OB16_4*s.dpt[2,3]-s.dpt[1,3]*S5
    OB19_5 = C6*C9
    OB29_5 = -C6*S9
    AM19_5 = s.dpt[1,3]*S6*S9-s.dpt[2,3]*S6*C9
    AM29_5 = s.dpt[1,3]*S6*C9+s.dpt[2,3]*S6*S9
    AM39_5 = s.dpt[2,3]*C6
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+OM29
    OM310 = OM19*S10+OM39*C10
    OA110 = C10*(OA19-qd[10]*OM39)-S10*(OA36+qd[10]*OM19)
    OA310 = C10*(OA36+qd[10]*OM19)+S10*(OA19-qd[10]*OM39)
    AF110 = AF19*C10-AF39*S10
    AF310 = AF19*S10+AF39*C10
    AM110_1 = AM19_1*C10-AM36_1*S10
    AM310_1 = AM19_1*S10+AM36_1*C10
    AM110_2 = AM19_2*C10-AM36_2*S10
    AM310_2 = AM19_2*S10+AM36_2*C10
    AM110_3 = AM19_3*C10-AM36_3*S10
    AM310_3 = AM19_3*S10+AM36_3*C10
    OB110_4 = OB19_4*C10-OB36_4*S10
    OB310_4 = OB19_4*S10+OB36_4*C10
    AM110_4 = AM19_4*C10-AM39_4*S10
    AM310_4 = AM19_4*S10+AM39_4*C10
    OB110_5 = OB19_5*C10-S10*S6
    OB310_5 = OB19_5*S10+C10*S6
    AM110_5 = AM19_5*C10-AM39_5*S10
    AM310_5 = AM19_5*S10+AM39_5*C10
    OB110_6 = C10*S9
    OB310_6 = S10*S9
    AM110_6 = s.dpt[1,3]*S10
    AM310_6 = -s.dpt[1,3]*C10
    OM111 = OM16*C11+OM26*S11
    OM211 = -OM16*S11+OM26*C11
    OM311 = qd[11]+OM36
    OA111 = C11*(OA16+qd[11]*OM26)+S11*(OA25-qd[11]*OM16)
    OA211 = C11*(OA25-qd[11]*OM16)-S11*(OA16+qd[11]*OM26)
    AF111 = C11*(AF16+BEF26*s.dpt[2,4]+BS16*s.dpt[1,4])+S11*(AF25+BEF46*s.dpt[1,4]+BS56*s.dpt[2,4])
    AF211 = C11*(AF25+BEF46*s.dpt[1,4]+BS56*s.dpt[2,4])-S11*(AF16+BEF26*s.dpt[2,4]+BS16*s.dpt[1,4])
    AF311 = AF36+BEF76*s.dpt[1,4]+BEF86*s.dpt[2,4]
    AM111_1 = AM16_1*C11+AM25_1*S11
    AM211_1 = -AM16_1*S11+AM25_1*C11
    AM111_2 = AM16_2*C11+AM25_2*S11
    AM211_2 = -AM16_2*S11+AM25_2*C11
    AM111_3 = AM16_3*C11+S11*S5
    AM211_3 = -AM16_3*S11+C11*S5
    OB111_4 = OB16_4*C11+S11*S5
    OB211_4 = -OB16_4*S11+C11*S5
    AM111_4 = OB36_4*s.dpt[1,4]*S11-OB36_4*s.dpt[2,4]*C11
    AM211_4 = OB36_4*s.dpt[1,4]*C11+OB36_4*s.dpt[2,4]*S11
    AM311_4 = OB16_4*s.dpt[2,4]-s.dpt[1,4]*S5
    OB111_5 = C11*C6
    OB211_5 = -S11*C6
    AM111_5 = s.dpt[1,4]*S11*S6-s.dpt[2,4]*C11*S6
    AM211_5 = s.dpt[1,4]*C11*S6+s.dpt[2,4]*S11*S6
    AM311_5 = s.dpt[2,4]*C6
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OA112 = C12*(OA111-qd[12]*OM311)-S12*(OA36+qd[12]*OM111)
    OA312 = C12*(OA36+qd[12]*OM111)+S12*(OA111-qd[12]*OM311)
    AF112 = AF111*C12-AF311*S12
    AF312 = AF111*S12+AF311*C12
    AM112_1 = AM111_1*C12-AM36_1*S12
    AM312_1 = AM111_1*S12+AM36_1*C12
    AM112_2 = AM111_2*C12-AM36_2*S12
    AM312_2 = AM111_2*S12+AM36_2*C12
    AM112_3 = AM111_3*C12-AM36_3*S12
    AM312_3 = AM111_3*S12+AM36_3*C12
    OB112_4 = OB111_4*C12-OB36_4*S12
    OB312_4 = OB111_4*S12+OB36_4*C12
    AM112_4 = AM111_4*C12-AM311_4*S12
    AM312_4 = AM111_4*S12+AM311_4*C12
    OB112_5 = OB111_5*C12-S12*S6
    OB312_5 = OB111_5*S12+C12*S6
    AM112_5 = AM111_5*C12-AM311_5*S12
    AM312_5 = AM111_5*S12+AM311_5*C12
    OB112_6 = S11*C12
    OB312_6 = S11*S12
    AM112_6 = s.dpt[1,4]*S12
    AM312_6 = -s.dpt[1,4]*C12
 
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
    FB212_5 = s.m[12]*AM211_5
    FB312_5 = s.m[12]*AM312_5
    CM112_5 = s.In[1,12]*OB112_5
    CM212_5 = s.In[5,12]*OB211_5
    CM312_5 = s.In[9,12]*OB312_5
    FB112_6 = s.m[12]*AM112_6
    FB312_6 = s.m[12]*AM312_6
    CM112_6 = s.In[1,12]*OB112_6
    CM212_6 = s.In[5,12]*C11
    CM312_6 = s.In[9,12]*OB312_6
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
    FM311_6 = -FB112_6*S12+FB312_6*C12
    CM111_6 = CM112_6*C12+CM312_6*S12
    CM311_6 = -CM112_6*S12+CM312_6*C12
    CM311_11 = -CM112_11*S12+CM312_11*C12
    FA110 = -s.frc[1,10]+s.m[10]*AF110
    FA210 = -s.frc[2,10]+s.m[10]*AF29
    FA310 = -s.frc[3,10]+s.m[10]*AF310
    CF110 = -s.trq[1,10]+s.In[1,10]*OA110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    CF210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OA29-s.In[9,10]*OM110*OM310
    CF310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OA310
    FB110_1 = s.m[10]*AM110_1
    FB210_1 = s.m[10]*AM29_1
    FB310_1 = s.m[10]*AM310_1
    FB110_2 = s.m[10]*AM110_2
    FB210_2 = s.m[10]*AM29_2
    FB310_2 = s.m[10]*AM310_2
    FB110_3 = s.m[10]*AM110_3
    FB210_3 = s.m[10]*AM29_3
    FB310_3 = s.m[10]*AM310_3
    FB110_4 = s.m[10]*AM110_4
    FB210_4 = s.m[10]*AM29_4
    FB310_4 = s.m[10]*AM310_4
    CM110_4 = s.In[1,10]*OB110_4
    CM210_4 = s.In[5,10]*OB29_4
    CM310_4 = s.In[9,10]*OB310_4
    FB110_5 = s.m[10]*AM110_5
    FB210_5 = s.m[10]*AM29_5
    FB310_5 = s.m[10]*AM310_5
    CM110_5 = s.In[1,10]*OB110_5
    CM210_5 = s.In[5,10]*OB29_5
    CM310_5 = s.In[9,10]*OB310_5
    FB110_6 = s.m[10]*AM110_6
    FB310_6 = s.m[10]*AM310_6
    CM110_6 = s.In[1,10]*OB110_6
    CM210_6 = s.In[5,10]*C9
    CM310_6 = s.In[9,10]*OB310_6
    CM110_9 = -s.In[1,10]*S10
    CM310_9 = s.In[9,10]*C10
    FF19 = FA110*C10+FA310*S10
    FF39 = -FA110*S10+FA310*C10
    CF19 = CF110*C10+CF310*S10
    CF39 = -CF110*S10+CF310*C10
    FM19_1 = FB110_1*C10+FB310_1*S10
    FM39_1 = -FB110_1*S10+FB310_1*C10
    FM19_2 = FB110_2*C10+FB310_2*S10
    FM39_2 = -FB110_2*S10+FB310_2*C10
    FM19_3 = FB110_3*C10+FB310_3*S10
    FM39_3 = -FB110_3*S10+FB310_3*C10
    FM19_4 = FB110_4*C10+FB310_4*S10
    FM39_4 = -FB110_4*S10+FB310_4*C10
    CM19_4 = CM110_4*C10+CM310_4*S10
    CM39_4 = -CM110_4*S10+CM310_4*C10
    FM19_5 = FB110_5*C10+FB310_5*S10
    FM39_5 = -FB110_5*S10+FB310_5*C10
    CM19_5 = CM110_5*C10+CM310_5*S10
    CM39_5 = -CM110_5*S10+CM310_5*C10
    FM39_6 = -FB110_6*S10+FB310_6*C10
    CM19_6 = CM110_6*C10+CM310_6*S10
    CM39_6 = -CM110_6*S10+CM310_6*C10
    CM39_9 = -CM110_9*S10+CM310_9*C10
    FA18 = -s.frc[1,8]+s.m[8]*AF18
    FA28 = -s.frc[2,8]+s.m[8]*AF28
    FA38 = -s.frc[3,8]+s.m[8]*AF38
    CF18 = -s.trq[1,8]+s.In[1,8]*OA18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38
    CF28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OA25-s.In[9,8]*OM18*OM38
    CF38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OA38
    FB18_1 = s.m[8]*AM18_1
    FB28_1 = s.m[8]*AM25_1
    FB38_1 = s.m[8]*AM38_1
    FB18_2 = s.m[8]*AM18_2
    FB28_2 = s.m[8]*AM25_2
    FB38_2 = s.m[8]*AM38_2
    FB18_3 = s.m[8]*AM18_3
    FB28_3 = s.m[8]*S5
    FB38_3 = s.m[8]*AM38_3
    FB18_4 = s.m[8]*AM18_4
    FB28_4 = s.m[8]*AM28_4
    FB38_4 = s.m[8]*AM38_4
    CM18_4 = s.In[1,8]*OB18_4
    CM28_4 = s.In[5,8]*S5
    CM38_4 = s.In[9,8]*OB38_4
    FB18_5 = s.m[8]*AM18_5
    FB28_5 = s.m[8]*AM28_5
    FB38_5 = s.m[8]*AM38_5
    CM18_5 = s.In[1,8]*C6p8
    CM38_5 = s.In[9,8]*S6p8
    FB18_6 = s.m[8]*AM18_6
    FB38_6 = s.m[8]*AM38_6
    FA17 = -s.frc[1,7]+s.m[7]*AF17
    FA27 = -s.frc[2,7]+s.m[7]*AF27
    FA37 = -s.frc[3,7]+s.m[7]*AF37
    CF17 = -s.trq[1,7]+s.In[1,7]*OA17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37
    CF27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OA25-s.In[9,7]*OM17*OM37
    CF37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OA37
    FB17_1 = s.m[7]*AM17_1
    FB27_1 = s.m[7]*AM25_1
    FB37_1 = s.m[7]*AM37_1
    FB17_2 = s.m[7]*AM17_2
    FB27_2 = s.m[7]*AM25_2
    FB37_2 = s.m[7]*AM37_2
    FB17_3 = s.m[7]*AM17_3
    FB27_3 = s.m[7]*S5
    FB37_3 = s.m[7]*AM37_3
    FB17_4 = s.m[7]*AM17_4
    FB27_4 = s.m[7]*AM27_4
    FB37_4 = s.m[7]*AM37_4
    CM17_4 = s.In[1,7]*OB17_4
    CM27_4 = s.In[5,7]*S5
    CM37_4 = s.In[9,7]*OB37_4
    FB17_5 = s.m[7]*AM17_5
    FB27_5 = s.m[7]*AM27_5
    FB37_5 = s.m[7]*AM37_5
    CM17_5 = s.In[1,7]*C6p7
    CM37_5 = s.In[9,7]*S6p7
    FB17_6 = s.m[7]*AM17_6
    FB37_6 = s.m[7]*AM37_6
    FA16 = -s.frc[1,6]+s.m[6]*AF16
    FA26 = -s.frc[2,6]+s.m[6]*AF25
    FA36 = -s.frc[3,6]+s.m[6]*AF36
    FF16 = FA16+FA17*C7+FA18*C8-FA210*S9-FA212*S11+FA37*S7+FA38*S8+FF111*C11+FF19*C9
    FF26 = FA26+FA27+FA28+FA210*C9+FA212*C11+FF111*S11+FF19*S9
    FF36 = FA36+FF311+FF39-FA17*S7-FA18*S8+FA37*C7+FA38*C8
    CF16 = -s.trq[1,6]+s.In[1,6]*OA16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+CF111*C11+CF17*C7+CF18*C8+CF19*C9-CF210*S9-CF212*S11+CF37*S7+CF38*S8+FF311*s.dpt[2,4]+FF39*s.dpt[2,3]+s.dpt[2,1]*(-FA17*S7+FA37*C7)+s.dpt[2,2]*(-FA18*S8+FA38*C8)
    CF26 = -s.trq[2,6]+CF27+CF28+s.In[1,6]*OM16*OM36+s.In[5,6]*OA25-s.In[9,6]*OM16*OM36+CF111*S11+CF19*S9+CF210*C9+CF212*C11-FF311*s.dpt[1,4]-FF39*s.dpt[1,3]-s.dpt[1,1]*(-FA17*S7+FA37*C7)-s.dpt[1,2]*(-FA18*S8+FA38*C8)
    CF36 = -s.trq[3,6]+CF311+CF39-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OA36-CF17*S7-CF18*S8+CF37*C7+CF38*C8+FA27*s.dpt[1,1]+FA28*s.dpt[1,2]+s.dpt[1,3]*(FA210*C9+FF19*S9)+s.dpt[1,4]*(FA212*C11+FF111*S11)-s.dpt[2,1]*(FA17*C7+FA37*S7)-s.dpt[2,2]*(FA18*C8+FA38*S8)-s.dpt[2,3]*(-FA210*S9+FF19*C9)-s.dpt[2,4]*(-FA212*S11+FF111*C11)
    FB16_1 = s.m[6]*AM16_1
    FB26_1 = s.m[6]*AM25_1
    FB36_1 = s.m[6]*AM36_1
    FM16_1 = FB16_1+FB17_1*C7+FB18_1*C8-FB210_1*S9-FB212_1*S11+FB37_1*S7+FB38_1*S8+FM111_1*C11+FM19_1*C9
    FM26_1 = FB26_1+FB27_1+FB28_1+FB210_1*C9+FB212_1*C11+FM111_1*S11+FM19_1*S9
    FM36_1 = FB36_1+FM311_1+FM39_1-FB17_1*S7-FB18_1*S8+FB37_1*C7+FB38_1*C8
    CM16_1 = FM311_1*s.dpt[2,4]+FM39_1*s.dpt[2,3]+s.dpt[2,1]*(-FB17_1*S7+FB37_1*C7)+s.dpt[2,2]*(-FB18_1*S8+FB38_1*C8)
    CM26_1 = -FM311_1*s.dpt[1,4]-FM39_1*s.dpt[1,3]-s.dpt[1,1]*(-FB17_1*S7+FB37_1*C7)-s.dpt[1,2]*(-FB18_1*S8+FB38_1*C8)
    CM36_1 = FB27_1*s.dpt[1,1]+FB28_1*s.dpt[1,2]+s.dpt[1,3]*(FB210_1*C9+FM19_1*S9)+s.dpt[1,4]*(FB212_1*C11+FM111_1*S11)-s.dpt[2,1]*(FB17_1*C7+FB37_1*S7)-s.dpt[2,2]*(FB18_1*C8+FB38_1*S8)-s.dpt[2,3]*(-FB210_1*S9+FM19_1*C9)-s.dpt[2,4]*(-FB212_1*S11+FM111_1*C11)
    FB16_2 = s.m[6]*AM16_2
    FB26_2 = s.m[6]*AM25_2
    FB36_2 = s.m[6]*AM36_2
    FM16_2 = FB16_2+FB17_2*C7+FB18_2*C8-FB210_2*S9-FB212_2*S11+FB37_2*S7+FB38_2*S8+FM111_2*C11+FM19_2*C9
    FM26_2 = FB26_2+FB27_2+FB28_2+FB210_2*C9+FB212_2*C11+FM111_2*S11+FM19_2*S9
    FM36_2 = FB36_2+FM311_2+FM39_2-FB17_2*S7-FB18_2*S8+FB37_2*C7+FB38_2*C8
    CM16_2 = FM311_2*s.dpt[2,4]+FM39_2*s.dpt[2,3]+s.dpt[2,1]*(-FB17_2*S7+FB37_2*C7)+s.dpt[2,2]*(-FB18_2*S8+FB38_2*C8)
    CM26_2 = -FM311_2*s.dpt[1,4]-FM39_2*s.dpt[1,3]-s.dpt[1,1]*(-FB17_2*S7+FB37_2*C7)-s.dpt[1,2]*(-FB18_2*S8+FB38_2*C8)
    CM36_2 = FB27_2*s.dpt[1,1]+FB28_2*s.dpt[1,2]+s.dpt[1,3]*(FB210_2*C9+FM19_2*S9)+s.dpt[1,4]*(FB212_2*C11+FM111_2*S11)-s.dpt[2,1]*(FB17_2*C7+FB37_2*S7)-s.dpt[2,2]*(FB18_2*C8+FB38_2*S8)-s.dpt[2,3]*(-FB210_2*S9+FM19_2*C9)-s.dpt[2,4]*(-FB212_2*S11+FM111_2*C11)
    FB16_3 = s.m[6]*AM16_3
    FB26_3 = s.m[6]*S5
    FB36_3 = s.m[6]*AM36_3
    FM16_3 = FB16_3+FB17_3*C7+FB18_3*C8-FB210_3*S9-FB212_3*S11+FB37_3*S7+FB38_3*S8+FM111_3*C11+FM19_3*C9
    FM26_3 = FB26_3+FB27_3+FB28_3+FB210_3*C9+FB212_3*C11+FM111_3*S11+FM19_3*S9
    FM36_3 = FB36_3+FM311_3+FM39_3-FB17_3*S7-FB18_3*S8+FB37_3*C7+FB38_3*C8
    CM16_3 = FM311_3*s.dpt[2,4]+FM39_3*s.dpt[2,3]+s.dpt[2,1]*(-FB17_3*S7+FB37_3*C7)+s.dpt[2,2]*(-FB18_3*S8+FB38_3*C8)
    CM26_3 = -FM311_3*s.dpt[1,4]-FM39_3*s.dpt[1,3]-s.dpt[1,1]*(-FB17_3*S7+FB37_3*C7)-s.dpt[1,2]*(-FB18_3*S8+FB38_3*C8)
    CM36_3 = FB27_3*s.dpt[1,1]+FB28_3*s.dpt[1,2]+s.dpt[1,3]*(FB210_3*C9+FM19_3*S9)+s.dpt[1,4]*(FB212_3*C11+FM111_3*S11)-s.dpt[2,1]*(FB17_3*C7+FB37_3*S7)-s.dpt[2,2]*(FB18_3*C8+FB38_3*S8)-s.dpt[2,3]*(-FB210_3*S9+FM19_3*C9)-s.dpt[2,4]*(-FB212_3*S11+FM111_3*C11)
    CM16_4 = s.In[1,6]*OB16_4+CM111_4*C11+CM17_4*C7+CM18_4*C8+CM19_4*C9-CM210_4*S9-CM212_4*S11+CM37_4*S7+CM38_4*S8+FM311_4*s.dpt[2,4]+FM39_4*s.dpt[2,3]+s.dpt[2,1]*(-FB17_4*S7+FB37_4*C7)+s.dpt[2,2]*(-FB18_4*S8+FB38_4*C8)
    CM26_4 = CM27_4+CM28_4+s.In[5,6]*S5+CM111_4*S11+CM19_4*S9+CM210_4*C9+CM212_4*C11-FM311_4*s.dpt[1,4]-FM39_4*s.dpt[1,3]-s.dpt[1,1]*(-FB17_4*S7+FB37_4*C7)-s.dpt[1,2]*(-FB18_4*S8+FB38_4*C8)
    CM36_4 = CM311_4+CM39_4+s.In[9,6]*OB36_4-CM17_4*S7-CM18_4*S8+CM37_4*C7+CM38_4*C8+FB27_4*s.dpt[1,1]+FB28_4*s.dpt[1,2]+s.dpt[1,3]*(FB210_4*C9+FM19_4*S9)+s.dpt[1,4]*(FB212_4*C11+FM111_4*S11)-s.dpt[2,1]*(FB17_4*C7+FB37_4*S7)-s.dpt[2,2]*(FB18_4*C8+FB38_4*S8)-s.dpt[2,3]*(-FB210_4*S9+FM19_4*C9)-s.dpt[2,4]*(-FB212_4*S11+FM111_4*C11)
    CM16_5 = s.In[1,6]*C6+CM111_5*C11+CM17_5*C7+CM18_5*C8+CM19_5*C9-CM210_5*S9-CM212_5*S11+CM37_5*S7+CM38_5*S8+FM311_5*s.dpt[2,4]+FM39_5*s.dpt[2,3]+s.dpt[2,1]*(-FB17_5*S7+FB37_5*C7)+s.dpt[2,2]*(-FB18_5*S8+FB38_5*C8)
    CM26_5 = CM111_5*S11+CM19_5*S9+CM210_5*C9+CM212_5*C11-FM311_5*s.dpt[1,4]-FM39_5*s.dpt[1,3]-s.dpt[1,1]*(-FB17_5*S7+FB37_5*C7)-s.dpt[1,2]*(-FB18_5*S8+FB38_5*C8)
    CM36_5 = CM311_5+CM39_5+s.In[9,6]*S6-CM17_5*S7-CM18_5*S8+CM37_5*C7+CM38_5*C8+FB27_5*s.dpt[1,1]+FB28_5*s.dpt[1,2]+s.dpt[1,3]*(FB210_5*C9+FM19_5*S9)+s.dpt[1,4]*(FB212_5*C11+FM111_5*S11)-s.dpt[2,1]*(FB17_5*C7+FB37_5*S7)-s.dpt[2,2]*(FB18_5*C8+FB38_5*S8)-s.dpt[2,3]*(-FB210_5*S9+FM19_5*C9)-s.dpt[2,4]*(-FB212_5*S11+FM111_5*C11)
    CM26_6 = s.In[5,6]+s.In[5,7]+s.In[5,8]+CM111_6*S11+CM19_6*S9+CM210_6*C9+CM212_6*C11-FM311_6*s.dpt[1,4]-FM39_6*s.dpt[1,3]-s.dpt[1,1]*(-FB17_6*S7+FB37_6*C7)-s.dpt[1,2]*(-FB18_6*S8+FB38_6*C8)
    FF15 = FF16*C6+FF36*S6
    FF35 = -FF16*S6+FF36*C6
    CF15 = CF16*C6+CF36*S6
    CF35 = -CF16*S6+CF36*C6
    FM15_1 = FM16_1*C6+FM36_1*S6
    FM35_1 = -FM16_1*S6+FM36_1*C6
    CM15_1 = CM16_1*C6+CM36_1*S6
    CM35_1 = -CM16_1*S6+CM36_1*C6
    FM15_2 = FM16_2*C6+FM36_2*S6
    FM35_2 = -FM16_2*S6+FM36_2*C6
    CM15_2 = CM16_2*C6+CM36_2*S6
    CM35_2 = -CM16_2*S6+CM36_2*C6
    FM35_3 = -FM16_3*S6+FM36_3*C6
    CM15_3 = CM16_3*C6+CM36_3*S6
    CM35_3 = -CM16_3*S6+CM36_3*C6
    CM15_4 = CM16_4*C6+CM36_4*S6
    CM35_4 = -CM16_4*S6+CM36_4*C6
    CM15_5 = CM16_5*C6+CM36_5*S6
    FF24 = FF26*C5-FF35*S5
    FF34 = FF26*S5+FF35*C5
    CF34 = CF26*S5+CF35*C5
    FM24_1 = FM26_1*C5-FM35_1*S5
    FM34_1 = FM26_1*S5+FM35_1*C5
    CM34_1 = CM26_1*S5+CM35_1*C5
    FM24_2 = FM26_2*C5-FM35_2*S5
    FM34_2 = FM26_2*S5+FM35_2*C5
    CM34_2 = CM26_2*S5+CM35_2*C5
    FM34_3 = FM26_3*S5+FM35_3*C5
    CM34_3 = CM26_3*S5+CM35_3*C5
    CM34_4 = CM26_4*S5+CM35_4*C5
    FF13 = FF15*C4-FF24*S4
    FF23 = FF15*S4+FF24*C4
    FM13_1 = FM15_1*C4-FM24_1*S4
    FM23_1 = FM15_1*S4+FM24_1*C4
    FM23_2 = FM15_2*S4+FM24_2*C4
 
# Symbolic model output

    c[1] = FF13
    c[2] = FF23
    c[3] = FF34
    c[4] = CF34
    c[5] = CF15
    c[6] = CF26
    c[7] = CF27
    c[8] = CF28
    c[9] = CF39
    c[10] = CF210
    c[11] = CF311
    c[12] = CF212
    M[1,1] = FM13_1
    M[1,2] = FM23_1
    M[1,3] = FM34_1
    M[1,4] = CM34_1
    M[1,5] = CM15_1
    M[1,6] = CM26_1
    M[2,1] = FM23_1
    M[2,2] = FM23_2
    M[2,3] = FM34_2
    M[2,4] = CM34_2
    M[2,5] = CM15_2
    M[2,6] = CM26_2
    M[3,1] = FM34_1
    M[3,2] = FM34_2
    M[3,3] = FM34_3
    M[3,4] = CM34_3
    M[3,5] = CM15_3
    M[3,6] = CM26_3
    M[4,1] = CM34_1
    M[4,2] = CM34_2
    M[4,3] = CM34_3
    M[4,4] = CM34_4
    M[4,5] = CM15_4
    M[4,6] = CM26_4
    M[4,7] = CM27_4
    M[4,8] = CM28_4
    M[4,9] = CM39_4
    M[4,10] = CM210_4
    M[4,11] = CM311_4
    M[4,12] = CM212_4
    M[5,1] = CM15_1
    M[5,2] = CM15_2
    M[5,3] = CM15_3
    M[5,4] = CM15_4
    M[5,5] = CM15_5
    M[5,6] = CM26_5
    M[5,9] = CM39_5
    M[5,10] = CM210_5
    M[5,11] = CM311_5
    M[5,12] = CM212_5
    M[6,1] = CM26_1
    M[6,2] = CM26_2
    M[6,3] = CM26_3
    M[6,4] = CM26_4
    M[6,5] = CM26_5
    M[6,6] = CM26_6
    M[6,7] = s.In[5,7]
    M[6,8] = s.In[5,8]
    M[6,9] = CM39_6
    M[6,10] = CM210_6
    M[6,11] = CM311_6
    M[6,12] = CM212_6
    M[7,4] = CM27_4
    M[7,6] = s.In[5,7]
    M[7,7] = s.In[5,7]
    M[8,4] = CM28_4
    M[8,6] = s.In[5,8]
    M[8,8] = s.In[5,8]
    M[9,4] = CM39_4
    M[9,5] = CM39_5
    M[9,6] = CM39_6
    M[9,9] = CM39_9
    M[10,4] = CM210_4
    M[10,5] = CM210_5
    M[10,6] = CM210_6
    M[10,10] = s.In[5,10]
    M[11,4] = CM311_4
    M[11,5] = CM311_5
    M[11,6] = CM311_6
    M[11,11] = CM311_11
    M[12,4] = CM212_4
    M[12,5] = CM212_5
    M[12,6] = CM212_6
    M[12,12] = s.In[5,12]

# Number of continuation lines = 0


