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
#	==> Generation Date: Thu Mar 20 15:02:22 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: suspension_vrai
#
#	==> Number of joints: 13
#
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt

# Number of continuation lines = 0

  print("ERROR : Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. ")
  print("        Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting. ")
  print("        Error raised in mbs_sensor.")
  s.flag_stop = 1

