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
#	==> Generation Date: Mon Mar 24 11:43:11 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Jpp
#
#	==> Number of joints: 26
#
#	==> Function: F27 - Link Forces (3D)
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos
from numpy import zeros

def link3D(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    frc = s.frc
    trq = s.trq


# Number of continuation lines = 0

    print("ERROR : Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. ")
    print("        Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting. ")
    print("        Error raised in mbs_link3D.")
    s.flag_stop = 1

