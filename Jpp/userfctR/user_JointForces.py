# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    mbs_data.Qq[1:] = 0.

    if tsim < 2:

        id_j = mbs_data.joint_id["Joint_22"]
        mbs_data.Qq[id_j] = 100
    
        id_j = mbs_data.joint_id["Joint_23"]
        mbs_data.Qq[id_j] = 100

    if tsim > 2:

        id_j = mbs_data.joint_id["Joint_26"]
        mbs_data.Qq[id_j] = -mbs_data.qd[id_j] * 1000
        
        id_j = mbs_data.joint_id["Joint_27"]
        mbs_data.Qq[id_j] = -mbs_data.qd[id_j] * 1000

    # Example: damping in joint number 5
    # D = 0.5 # N/(m/s)
    # mbs_data.Qq[5] = -D * mbs_data.qd[5]

    return
