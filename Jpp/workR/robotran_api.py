#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.MBsysPy.eu
Contact : info@MBsysPy.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy
    from MBsysPy import MbsResult, MbsData
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

import os
import parameter_manager as param
import coreutils as utils
from typing import Optional, Union
import matplotlib.pyplot as plt
import matplotlib.figure as fig




def loader(path: str) -> MbsData:
    """Loads the .mbs file at the specified path

    Args:
        path (str): the path to the .mbs file

    Raises:
        SystemExit: if the file is not found

    Returns:
        MbsData: the loaded data from the .mbs file
    """
    if os.path.isfile(path):
        mbs_data = MBsysPy.MbsData(path)
        return mbs_data
    utils.log("Wrong path to load the project", utils.Level.CRITICAL)
    exit(1)

def _partionning(mbs_data: MbsData) -> None:
    """Generate the coordiante partionning

    Args:
        mbs_data (MbsData): the data for the user model
    """
    mbs_data.process = 1
    mbs_part = MBsysPy.MbsPart(mbs_data)
    mbs_part.set_options(rowperm=1, verbose=1)
    mbs_part.run()
    del mbs_part

def equilibrium(mbs_data: MbsData) -> None:
    """Tries to find the equilibrium for the system

    Note:
        The `loader()` and `partionning()`functions must be called first to properly initialize `mbs_data`

    Args:
        mbs_data (MbsData): the data for the user model
    """
    _partionning(mbs_data)
    mbs_data.process = 2
    mbs_equil = MBsysPy.MbsEquil(mbs_data)
    mbs_equil.set_options(save_anim = 1, senstol = 1e-6, compute_uxd = 0, resfilename = "equil1")
    mbs_equil.run()
    mbs_equil.print_equil()
    del mbs_equil

def modal(mbs_data: MbsData) -> None:
    """Tries to perform the modal analysis to get the eigen values

    Note:
        The `loader()` and `partionning()`functions must be called first to properly initialize `mbs_data`

    Args:
        mbs_data (MbsData): the data for the user model
    """
    _partionning(mbs_data)
    mbs_data.process = 4
    mbs_modal = MBsysPy.MbsModal(mbs_data)
    mbs_modal.set_options(save_result = 1, save_anim = 1, mode_ampl = 0.2)
    mbs_modal.run()
    del mbs_modal
def dir_dyn(mbs_data: MbsData) -> Union[MbsResult, float, float]:
    """Tries to perform the direct dynamics analysis

    Note:
        The `loader()` and `partionning()`functions must be called first to properly initialize `mbs_data`

    Args:
        mbs_data (MbsData): the data for the user model

    Returns:
        MbsResult: the result class containing position, speed, accelaration
    """
    _partionning(mbs_data)
    mbs_data.process = 3
    mbs_dirdyn = MBsysPy.MbsDirdyn(mbs_data)
    mbs_dirdyn.set_options(dt0=1e-3, tf=5.0, save2file=1)
    results = mbs_dirdyn.run()
    return results, mbs_dirdyn.get_options('t0'), mbs_dirdyn.get_options('tf')

def plot_initializer(xlabel: str, ylabel: str, plt_name: str) -> fig.FigureBase.gca:
    """Initialize the plot with the specified parameters

    Args:
        xlabel (str): the label in x
        ylabel (str): the label in y
        plt_name (str): the name of the figure/ plot

    Returns:
        fig.FigureBase.gca: the matplotlib figure
    """
    fig = plt.figure(num=plt_name)
    axis = fig.gca()
    axis.grid(True)
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)

    return axis

def plot_finalizer(axis: fig.FigureBase.gca, times:tuple, ylim: tuple = None):
    """Finalizes the plot

    Args:
        axis (fig.FigureBase.gca): the matplotlib gca plot
        times (tuple): the t0 and tf on x
        ylim (tuple, optional): the limit in y. Defaults to None.
    """
    axis.set_xlim(left=times[0], right=times[1])
    if ylim != None:
        axis.set_ylim(bottom=ylim[0], top=ylim[1])
    axis.legend(fontsize=16)
    