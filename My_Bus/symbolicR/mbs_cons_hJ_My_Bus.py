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
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    ROlp1_82 = -C8*S9-S8*C9
    ROlp1_92 = C8*C9-S8*S9
    RLlp1_22 = s.dpt[2,24]*C8
    RLlp1_32 = s.dpt[2,24]*S8
    POlp1_22 = RLlp1_22+s.dpt[2,4]
    RLlp1_24 = ROlp1_82*s.dpt[3,26]
    RLlp1_34 = ROlp1_92*s.dpt[3,26]
    POlp1_24 = POlp1_22+RLlp1_24
    POlp1_34 = RLlp1_32+RLlp1_34
    JTlp1_24_1 = -RLlp1_32-RLlp1_34
    JTlp1_34_1 = RLlp1_22+RLlp1_24
    JTlp1_14_3 = -RLlp1_24*ROlp1_92+RLlp1_34*ROlp1_82
    RLlp2_22 = s.dpt[2,23]*C7
    RLlp2_32 = s.dpt[2,23]*S7
    POlp2_22 = RLlp2_22+s.dpt[2,3]
    POlp2_32 = RLlp2_32+s.dpt[3,3]
    h_1 = -s.dpt[1,3]+s.dpt[1,4]
    h_2 = POlp1_24-POlp2_22
    h_3 = POlp1_34-POlp2_32
    RLlp3_22 = s.dpt[2,29]*C13
    RLlp3_32 = s.dpt[2,29]*S13
    POlp3_22 = RLlp3_22+s.dpt[2,6]
    POlp3_32 = RLlp3_32+s.dpt[3,6]
    ROlp4_82 = -C14*S15-S14*C15
    ROlp4_92 = C14*C15-S14*S15
    RLlp4_22 = s.dpt[2,30]*C14
    RLlp4_32 = s.dpt[2,30]*S14
    POlp4_22 = RLlp4_22+s.dpt[2,7]
    RLlp4_24 = ROlp4_82*s.dpt[3,32]
    RLlp4_34 = ROlp4_92*s.dpt[3,32]
    POlp4_24 = POlp4_22+RLlp4_24
    POlp4_34 = RLlp4_32+RLlp4_34
    JTlp4_24_1 = -RLlp4_32-RLlp4_34
    JTlp4_34_1 = RLlp4_22+RLlp4_24
    JTlp4_14_3 = -RLlp4_24*ROlp4_92+RLlp4_34*ROlp4_82
    h_4 = s.dpt[1,6]-s.dpt[1,7]
    h_5 = POlp3_22-POlp4_24
    h_6 = POlp3_32-POlp4_34
    RLlp5_12 = s.dpt[1,49]*C23-s.dpt[2,49]*S23
    RLlp5_22 = s.dpt[1,49]*S23+s.dpt[2,49]*C23
    POlp5_12 = RLlp5_12+s.dpt[1,20]
    POlp5_22 = RLlp5_22+s.dpt[2,20]
    RLlp6_12 = s.dpt[1,51]*C24-s.dpt[2,51]*S24
    RLlp6_22 = s.dpt[1,51]*S24+s.dpt[2,51]*C24
    POlp6_12 = RLlp6_12+s.dpt[1,21]
    POlp6_22 = RLlp6_22+s.dpt[2,21]
    Plp11 = POlp5_12-POlp6_12
    Plp21 = POlp5_22-POlp6_22
    P2lp1 = Plp11*Plp11+Plp21*Plp21
    l2rod1 = s.lrod[1]*s.lrod[1]
    h_7 = (0.50)*(P2lp1-l2rod1)
    Jac_7_23 = -Plp11*RLlp5_22+Plp21*RLlp5_12
    Jac_7_24 = Plp11*RLlp6_22-Plp21*RLlp6_12
    ROlp7_52 = C8*C9-S8*S9
    ROlp7_62 = C8*S9+S8*C9
    ROlp7_82 = -C8*S9-S8*C9
    ROlp7_92 = C8*C9-S8*S9
    ROlp7_23 = ROlp7_52*S10
    ROlp7_33 = ROlp7_62*S10
    RLlp7_22 = s.dpt[2,24]*C8
    RLlp7_32 = s.dpt[2,24]*S8
    POlp7_22 = RLlp7_22+s.dpt[2,4]
    RLlp7_14 = s.dpt[1,28]*C10
    RLlp7_24 = ROlp7_23*s.dpt[1,28]
    RLlp7_34 = ROlp7_33*s.dpt[1,28]
    POlp7_14 = RLlp7_14+s.dpt[1,4]
    POlp7_24 = POlp7_22+RLlp7_24
    POlp7_34 = RLlp7_32+RLlp7_34
    JTlp7_24_1 = -RLlp7_32-RLlp7_34
    JTlp7_34_1 = RLlp7_22+RLlp7_24
    JTlp7_14_3 = -RLlp7_24*ROlp7_92+RLlp7_34*ROlp7_82
    JTlp7_24_3 = RLlp7_14*ROlp7_92
    JTlp7_34_3 = -RLlp7_14*ROlp7_82
    RLlp8_12 = s.dpt[1,50]*C23-s.dpt[2,50]*S23
    RLlp8_22 = s.dpt[1,50]*S23+s.dpt[2,50]*C23
    POlp8_12 = RLlp8_12+s.dpt[1,20]
    POlp8_22 = RLlp8_22+s.dpt[2,20]
    Plp12 = POlp7_14-POlp8_12
    Plp22 = POlp7_24-POlp8_22
    P2lp2 = POlp7_34*POlp7_34+Plp12*Plp12+Plp22*Plp22
    l2rod2 = s.lrod[2]*s.lrod[2]
    h_8 = (0.50)*(P2lp2-l2rod2)
    Jac_8_8 = JTlp7_24_1*Plp22+JTlp7_34_1*POlp7_34
    Jac_8_9 = POlp7_34*RLlp7_24-Plp22*RLlp7_34
    Jac_8_10 = JTlp7_14_3*Plp12+JTlp7_24_3*Plp22+JTlp7_34_3*POlp7_34
    Jac_8_23 = Plp12*RLlp8_22-Plp22*RLlp8_12
    RLlp9_12 = s.dpt[1,52]*C24-s.dpt[2,52]*S24
    RLlp9_22 = s.dpt[1,52]*S24+s.dpt[2,52]*C24
    POlp9_12 = RLlp9_12+s.dpt[1,21]
    POlp9_22 = RLlp9_22+s.dpt[2,21]
    ROlp10_52 = C14*C15-S14*S15
    ROlp10_62 = C14*S15+S14*C15
    ROlp10_82 = -C14*S15-S14*C15
    ROlp10_92 = C14*C15-S14*S15
    ROlp10_23 = ROlp10_52*S16
    ROlp10_33 = ROlp10_62*S16
    RLlp10_22 = s.dpt[2,30]*C14
    RLlp10_32 = s.dpt[2,30]*S14
    POlp10_22 = RLlp10_22+s.dpt[2,7]
    RLlp10_14 = s.dpt[1,34]*C16
    RLlp10_24 = ROlp10_23*s.dpt[1,34]
    RLlp10_34 = ROlp10_33*s.dpt[1,34]
    POlp10_14 = RLlp10_14+s.dpt[1,7]
    POlp10_24 = POlp10_22+RLlp10_24
    POlp10_34 = RLlp10_32+RLlp10_34
    JTlp10_24_1 = -RLlp10_32-RLlp10_34
    JTlp10_34_1 = RLlp10_22+RLlp10_24
    JTlp10_14_3 = -RLlp10_24*ROlp10_92+RLlp10_34*ROlp10_82
    JTlp10_24_3 = RLlp10_14*ROlp10_92
    JTlp10_34_3 = -RLlp10_14*ROlp10_82
    Plp13 = -POlp10_14+POlp9_12
    Plp23 = -POlp10_24+POlp9_22
    P2lp3 = POlp10_34*POlp10_34+Plp13*Plp13+Plp23*Plp23
    l2rod3 = s.lrod[3]*s.lrod[3]
    h_9 = (0.50)*(P2lp3-l2rod3)
    Jac_9_24 = -Plp13*RLlp9_22+Plp23*RLlp9_12
    Jac_9_14 = -JTlp10_24_1*Plp23+JTlp10_34_1*POlp10_34
    Jac_9_15 = POlp10_34*RLlp10_24+Plp23*RLlp10_34
    Jac_9_16 = -JTlp10_14_3*Plp13-JTlp10_24_3*Plp23+JTlp10_34_3*POlp10_34
    ROlp11_42 = S19*S20
    ROlp11_62 = C19*S20
    RLlp11_13 = ROlp11_42*s.dpt[2,35]
    RLlp11_23 = s.dpt[2,35]*C20
    RLlp11_33 = ROlp11_62*s.dpt[2,35]
    POlp11_13 = RLlp11_13+s.dpt[1,15]
    JTlp11_13_2 = RLlp11_23*S19
    JTlp11_23_2 = -RLlp11_13*S19-RLlp11_33*C19
    JTlp11_33_2 = RLlp11_23*C19
    Plp14 = POlp11_13-s.dpt[1,12]
    Plp24 = RLlp11_23-s.dpt[2,12]
    P2lp4 = Plp14*Plp14+Plp24*Plp24+RLlp11_33*RLlp11_33
    l2rod4 = s.lrod[4]*s.lrod[4]
    h_10 = (0.50)*(P2lp4-l2rod4)
    Jac_10_19 = Plp14*RLlp11_33-RLlp11_13*RLlp11_33
    Jac_10_20 = JTlp11_13_2*Plp14+JTlp11_23_2*Plp24+JTlp11_33_2*RLlp11_33
    ROlp13_42 = S19*S20
    ROlp13_62 = C19*S20
    RLlp13_13 = ROlp13_42*s.dpt[2,37]
    RLlp13_23 = s.dpt[2,37]*C20
    RLlp13_33 = ROlp13_62*s.dpt[2,37]
    POlp13_13 = RLlp13_13+s.dpt[1,15]
    JTlp13_13_2 = RLlp13_23*S19
    JTlp13_23_2 = -RLlp13_13*S19-RLlp13_33*C19
    JTlp13_33_2 = RLlp13_23*C19
    Plp15 = POlp13_13-s.dpt[1,14]
    Plp25 = RLlp13_23-s.dpt[2,14]
    P2lp5 = Plp15*Plp15+Plp25*Plp25+RLlp13_33*RLlp13_33
    l2rod5 = s.lrod[5]*s.lrod[5]
    h_11 = (0.50)*(P2lp5-l2rod5)
    Jac_11_19 = Plp15*RLlp13_33-RLlp13_13*RLlp13_33
    Jac_11_20 = JTlp13_13_2*Plp15+JTlp13_23_2*Plp25+JTlp13_33_2*RLlp13_33
    ROlp16_42 = S19*S20
    ROlp16_62 = C19*S20
    ROlp16_72 = S19*C20
    ROlp16_92 = C19*C20
    RLlp16_13 = ROlp16_42*s.dpt[2,38]+ROlp16_72*s.dpt[3,38]
    RLlp16_23 = s.dpt[2,38]*C20-s.dpt[3,38]*S20
    RLlp16_33 = ROlp16_62*s.dpt[2,38]+ROlp16_92*s.dpt[3,38]
    POlp16_13 = RLlp16_13+s.dpt[1,15]
    JTlp16_13_2 = RLlp16_23*S19
    JTlp16_23_2 = -RLlp16_13*S19-RLlp16_33*C19
    JTlp16_33_2 = RLlp16_23*C19
    Plp16 = -POlp16_13+s.dpt[1,13]
    Plp26 = -RLlp16_23+s.dpt[2,13]
    Plp36 = -RLlp16_33+s.dpt[3,13]
    P2lp6 = Plp16*Plp16+Plp26*Plp26+Plp36*Plp36
    l2rod6 = s.lrod[6]*s.lrod[6]
    h_12 = (0.50)*(P2lp6-l2rod6)
    Jac_12_19 = -Plp16*RLlp16_33+Plp36*RLlp16_13
    Jac_12_20 = -JTlp16_13_2*Plp16-JTlp16_23_2*Plp26-JTlp16_33_2*Plp36
    ROlp18_42 = S19*S20
    ROlp18_62 = C19*S20
    ROlp18_72 = S19*C20
    ROlp18_92 = C19*C20
    RLlp18_13 = ROlp18_42*s.dpt[2,36]+ROlp18_72*s.dpt[3,36]
    RLlp18_23 = s.dpt[2,36]*C20-s.dpt[3,36]*S20
    RLlp18_33 = ROlp18_62*s.dpt[2,36]+ROlp18_92*s.dpt[3,36]
    POlp18_13 = RLlp18_13+s.dpt[1,15]
    JTlp18_13_2 = RLlp18_23*S19
    JTlp18_23_2 = -RLlp18_13*S19-RLlp18_33*C19
    JTlp18_33_2 = RLlp18_23*C19
    Plp17 = -POlp18_13+s.dpt[1,11]
    Plp27 = -RLlp18_23+s.dpt[2,11]
    Plp37 = -RLlp18_33+s.dpt[3,11]
    P2lp7 = Plp17*Plp17+Plp27*Plp27+Plp37*Plp37
    l2rod7 = s.lrod[7]*s.lrod[7]
    h_13 = (0.50)*(P2lp7-l2rod7)
    Jac_13_19 = -Plp17*RLlp18_33+Plp37*RLlp18_13
    Jac_13_20 = -JTlp18_13_2*Plp17-JTlp18_23_2*Plp27-JTlp18_33_2*Plp37
    h[1] = h_1
    h[2] = h_2
    h[3] = h_3
    h[4] = h_4
    h[5] = h_5
    h[6] = h_6
    h[7] = h_7
    h[8] = h_8
    h[9] = h_9
    h[10] = h_10
    h[11] = h_11
    h[12] = h_12
    h[13] = h_13
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = 0
    Jac[1,8] = 0
    Jac[1,9] = 0
    Jac[1,10] = JTlp1_14_3
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = 0
    Jac[1,17] = 0
    Jac[1,18] = 0
    Jac[1,19] = 0
    Jac[1,20] = 0
    Jac[1,21] = 0
    Jac[1,22] = 0
    Jac[1,23] = 0
    Jac[1,24] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = RLlp2_32
    Jac[2,8] = JTlp1_24_1
    Jac[2,9] = -RLlp1_34
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = 0
    Jac[2,18] = 0
    Jac[2,19] = 0
    Jac[2,20] = 0
    Jac[2,21] = 0
    Jac[2,22] = 0
    Jac[2,23] = 0
    Jac[2,24] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = -RLlp2_22
    Jac[3,8] = JTlp1_34_1
    Jac[3,9] = RLlp1_24
    Jac[3,10] = 0
    Jac[3,11] = 0
    Jac[3,12] = 0
    Jac[3,13] = 0
    Jac[3,14] = 0
    Jac[3,15] = 0
    Jac[3,16] = 0
    Jac[3,17] = 0
    Jac[3,18] = 0
    Jac[3,19] = 0
    Jac[3,20] = 0
    Jac[3,21] = 0
    Jac[3,22] = 0
    Jac[3,23] = 0
    Jac[3,24] = 0
    Jac[4,1] = 0
    Jac[4,2] = 0
    Jac[4,3] = 0
    Jac[4,4] = 0
    Jac[4,5] = 0
    Jac[4,6] = 0
    Jac[4,7] = 0
    Jac[4,8] = 0
    Jac[4,9] = 0
    Jac[4,10] = 0
    Jac[4,11] = 0
    Jac[4,12] = 0
    Jac[4,13] = 0
    Jac[4,14] = 0
    Jac[4,15] = 0
    Jac[4,16] = -JTlp4_14_3
    Jac[4,17] = 0
    Jac[4,18] = 0
    Jac[4,19] = 0
    Jac[4,20] = 0
    Jac[4,21] = 0
    Jac[4,22] = 0
    Jac[4,23] = 0
    Jac[4,24] = 0
    Jac[5,1] = 0
    Jac[5,2] = 0
    Jac[5,3] = 0
    Jac[5,4] = 0
    Jac[5,5] = 0
    Jac[5,6] = 0
    Jac[5,7] = 0
    Jac[5,8] = 0
    Jac[5,9] = 0
    Jac[5,10] = 0
    Jac[5,11] = 0
    Jac[5,12] = 0
    Jac[5,13] = -RLlp3_32
    Jac[5,14] = -JTlp4_24_1
    Jac[5,15] = RLlp4_34
    Jac[5,16] = 0
    Jac[5,17] = 0
    Jac[5,18] = 0
    Jac[5,19] = 0
    Jac[5,20] = 0
    Jac[5,21] = 0
    Jac[5,22] = 0
    Jac[5,23] = 0
    Jac[5,24] = 0
    Jac[6,1] = 0
    Jac[6,2] = 0
    Jac[6,3] = 0
    Jac[6,4] = 0
    Jac[6,5] = 0
    Jac[6,6] = 0
    Jac[6,7] = 0
    Jac[6,8] = 0
    Jac[6,9] = 0
    Jac[6,10] = 0
    Jac[6,11] = 0
    Jac[6,12] = 0
    Jac[6,13] = RLlp3_22
    Jac[6,14] = -JTlp4_34_1
    Jac[6,15] = -RLlp4_24
    Jac[6,16] = 0
    Jac[6,17] = 0
    Jac[6,18] = 0
    Jac[6,19] = 0
    Jac[6,20] = 0
    Jac[6,21] = 0
    Jac[6,22] = 0
    Jac[6,23] = 0
    Jac[6,24] = 0
    Jac[7,1] = 0
    Jac[7,2] = 0
    Jac[7,3] = 0
    Jac[7,4] = 0
    Jac[7,5] = 0
    Jac[7,6] = 0
    Jac[7,7] = 0
    Jac[7,8] = 0
    Jac[7,9] = 0
    Jac[7,10] = 0
    Jac[7,11] = 0
    Jac[7,12] = 0
    Jac[7,13] = 0
    Jac[7,14] = 0
    Jac[7,15] = 0
    Jac[7,16] = 0
    Jac[7,17] = 0
    Jac[7,18] = 0
    Jac[7,19] = 0
    Jac[7,20] = 0
    Jac[7,21] = 0
    Jac[7,22] = 0
    Jac[7,23] = Jac_7_23
    Jac[7,24] = Jac_7_24
    Jac[8,1] = 0
    Jac[8,2] = 0
    Jac[8,3] = 0
    Jac[8,4] = 0
    Jac[8,5] = 0
    Jac[8,6] = 0
    Jac[8,7] = 0
    Jac[8,8] = Jac_8_8
    Jac[8,9] = Jac_8_9
    Jac[8,10] = Jac_8_10
    Jac[8,11] = 0
    Jac[8,12] = 0
    Jac[8,13] = 0
    Jac[8,14] = 0
    Jac[8,15] = 0
    Jac[8,16] = 0
    Jac[8,17] = 0
    Jac[8,18] = 0
    Jac[8,19] = 0
    Jac[8,20] = 0
    Jac[8,21] = 0
    Jac[8,22] = 0
    Jac[8,23] = Jac_8_23
    Jac[8,24] = 0
    Jac[9,1] = 0
    Jac[9,2] = 0
    Jac[9,3] = 0
    Jac[9,4] = 0
    Jac[9,5] = 0
    Jac[9,6] = 0
    Jac[9,7] = 0
    Jac[9,8] = 0
    Jac[9,9] = 0
    Jac[9,10] = 0
    Jac[9,11] = 0
    Jac[9,12] = 0
    Jac[9,13] = 0
    Jac[9,14] = Jac_9_14
    Jac[9,15] = Jac_9_15
    Jac[9,16] = Jac_9_16
    Jac[9,17] = 0
    Jac[9,18] = 0
    Jac[9,19] = 0
    Jac[9,20] = 0
    Jac[9,21] = 0
    Jac[9,22] = 0
    Jac[9,23] = 0
    Jac[9,24] = Jac_9_24
    Jac[10,1] = 0
    Jac[10,2] = 0
    Jac[10,3] = 0
    Jac[10,4] = 0
    Jac[10,5] = 0
    Jac[10,6] = 0
    Jac[10,7] = 0
    Jac[10,8] = 0
    Jac[10,9] = 0
    Jac[10,10] = 0
    Jac[10,11] = 0
    Jac[10,12] = 0
    Jac[10,13] = 0
    Jac[10,14] = 0
    Jac[10,15] = 0
    Jac[10,16] = 0
    Jac[10,17] = 0
    Jac[10,18] = 0
    Jac[10,19] = Jac_10_19
    Jac[10,20] = Jac_10_20
    Jac[10,21] = 0
    Jac[10,22] = 0
    Jac[10,23] = 0
    Jac[10,24] = 0
    Jac[11,1] = 0
    Jac[11,2] = 0
    Jac[11,3] = 0
    Jac[11,4] = 0
    Jac[11,5] = 0
    Jac[11,6] = 0
    Jac[11,7] = 0
    Jac[11,8] = 0
    Jac[11,9] = 0
    Jac[11,10] = 0
    Jac[11,11] = 0
    Jac[11,12] = 0
    Jac[11,13] = 0
    Jac[11,14] = 0
    Jac[11,15] = 0
    Jac[11,16] = 0
    Jac[11,17] = 0
    Jac[11,18] = 0
    Jac[11,19] = Jac_11_19
    Jac[11,20] = Jac_11_20
    Jac[11,21] = 0
    Jac[11,22] = 0
    Jac[11,23] = 0
    Jac[11,24] = 0
    Jac[12,1] = 0
    Jac[12,2] = 0
    Jac[12,3] = 0
    Jac[12,4] = 0
    Jac[12,5] = 0
    Jac[12,6] = 0
    Jac[12,7] = 0
    Jac[12,8] = 0
    Jac[12,9] = 0
    Jac[12,10] = 0
    Jac[12,11] = 0
    Jac[12,12] = 0
    Jac[12,13] = 0
    Jac[12,14] = 0
    Jac[12,15] = 0
    Jac[12,16] = 0
    Jac[12,17] = 0
    Jac[12,18] = 0
    Jac[12,19] = Jac_12_19
    Jac[12,20] = Jac_12_20
    Jac[12,21] = 0
    Jac[12,22] = 0
    Jac[12,23] = 0
    Jac[12,24] = 0
    Jac[13,1] = 0
    Jac[13,2] = 0
    Jac[13,3] = 0
    Jac[13,4] = 0
    Jac[13,5] = 0
    Jac[13,6] = 0
    Jac[13,7] = 0
    Jac[13,8] = 0
    Jac[13,9] = 0
    Jac[13,10] = 0
    Jac[13,11] = 0
    Jac[13,12] = 0
    Jac[13,13] = 0
    Jac[13,14] = 0
    Jac[13,15] = 0
    Jac[13,16] = 0
    Jac[13,17] = 0
    Jac[13,18] = 0
    Jac[13,19] = Jac_13_19
    Jac[13,20] = Jac_13_20
    Jac[13,21] = 0
    Jac[13,22] = 0
    Jac[13,23] = 0
    Jac[13,24] = 0

# Number of continuation lines = 0


