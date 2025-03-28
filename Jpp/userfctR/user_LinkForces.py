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
    import json
    with open('constants.json', 'r') as file:
        dic = json.load(file)
    Flink = 0.0
    
    L0sd = dic["L0sd"]
    Ksd = dic["Ksd"]
    Dsd = dic["Dsd"]
    Ks = dic["Ks"]
    L0s = dic["L0s"]
    if (identity == mbs_data.link_id['Link_0'] or 
        identity == mbs_data.link_id['Link_1']):
        Flink = Ksd*(Z-L0sd) + Dsd * Zd

    if (identity == mbs_data.link_id['Link_2'] or 
        identity == mbs_data.link_id['Link_3'] or
        identity == mbs_data.link_id['Link_4'] or
        identity == mbs_data.link_id['Link_5']):
        L0 = 0.5
        K = 5000
        D = 120
        Flink = Ks*(Z-L0s)
    if identity == mbs_data.link_id['Link_6']:
        L0 = 0.5
        K = 1000000
        Flink = K*(Z-L0)
    if identity == mbs_data.link_id['Link_7']:
        L0 = 0.5
        K = 1000000
        Flink = K*(Z-L0)

    if identity == mbs_data.link_id['Link_8']:
        L0 = 0.5
        D = 120
        Flink = D * Zd
    if identity == mbs_data.link_id['Link_9']:
        L0 = 0.5
        D = 120
        Flink = D * Zd
    if identity == mbs_data.link_id['Link_10']:
        L0 = 0.5
        D = 120
        Flink = D * Zd
    if identity == mbs_data.link_id['Link_11']:
        L0 = 0.5
        D = 120
        Flink = D * Zd
    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)

    return Flink
