# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """
    
    from json import load as j_load
    with open('constants.json', 'r') as file:       
        dic = j_load(file)
        
    L0sd = dic['L0sd']
    Ksd = dic['Ksd']
    Dsd = dic['Dsd']
    L0s = dic['L0s']
    Ks = dic['Ks']
    Dd = dic['Dd']

    if identity in (mbs_data.link_id['Link_0'], mbs_data.link_id['Link_1']):
        Flink = Ksd * (Z - L0sd) + Dsd * Zd
    elif identity in (mbs_data.link_id['Link_2'], mbs_data.link_id['Link_3'], mbs_data.link_id['Link_4'], mbs_data.link_id['Link_5']):
        Flink = Ks * (Z - L0s)
    elif identity in (mbs_data.link_id['Link_8'], mbs_data.link_id['Link_9'], mbs_data.link_id['Link_10'], mbs_data.link_id['Link_11']):
        Flink = Dd * Zd
    else:
        Flink = 0.0

    return Flink