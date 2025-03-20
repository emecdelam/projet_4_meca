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
#	==> Function: F19 - External Forces
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos
from numpy import zeros

def extforces(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    qdd = s.qdd
    frc = s.frc
    trq = s.trq


# Number of continuation lines = 0

    print("ERROR : Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. ")
    print("        Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting. ")
    print("        Error raised in mbs_extforces.")
    s.flag_stop = 1

