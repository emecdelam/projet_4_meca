# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_DrivenJoints(mbs_data, tsim):
    """Set the values of the driven joints directly in the MbsData structure.

    The position, velocity and acceleration of the driven joints must be set in
    the attributes mbs_data.q, mbs_data.qd and mbs_data.qdd .

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """
    id_j = mbs_data.joint_id["Joint_10"]
    omega = 0.24
    # Example: joint 5 under constant acceleration with non-zero initial
    #          coordinate (mbs_data.q0) and velocity (mbs_data.qd0).
    mbs_data.qdd[id_j] = 0
    mbs_data.qd[id_j]  = 0
    mbs_data.q[id_j]   = omega
    id_i = mbs_data.joint_id["Joint_14"]
    # Example: joint 5 under constant acceleration with non-zero initial
    #          coordinate (mbs_data.q0) and velocity (mbs_data.qd0).
    mbs_data.qdd[id_i] = 0
    mbs_data.qd[id_i]  = 0
    mbs_data.q[id_i]   = -omega
    
    
    id_k = mbs_data.joint_id["Joint_22"]

    mbs_data.qdd[id_k] = 0
    mbs_data.qd[id_k]  = 10
    mbs_data.q[id_k]   = tsim * 10

    id_l = mbs_data.joint_id["Joint_23"]

    mbs_data.qdd[id_l] = 0
    mbs_data.qd[id_l]  = 10
    mbs_data.q[id_l]   = tsim * 10
    
    
    
    id_m = mbs_data.joint_id["Joint_26"]

    mbs_data.qdd[id_m] = 0
    mbs_data.qd[id_m]  = 0.07
    mbs_data.q[id_m]   = tsim * 0.07

    id_n = mbs_data.joint_id["Joint_27"]

    mbs_data.qdd[id_n] = 0
    mbs_data.qd[id_n]  = 0.07
    mbs_data.q[id_n]   = tsim * 0.07
    # Example: joint 5 under constant acceleration with non-zero initial
    #          coordinate (mbs_data.q0) and velocity (mbs_data.qd0).
    # mbs_data.qdd[5] = 2
    # mbs_data.qd[5]  = mbs_data.qd0[5] + mbs_data.qdd[5]*tsim
    # mbs_data.q[5]   = mbs_data.q0[5]  + mbs_data.qd0[5]*tsim + 0.5 * mbs_data.qdd[5]*tsim*tsim

    return
