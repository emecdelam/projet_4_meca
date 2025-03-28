#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple program to automate analysis on multiple parameters
"""
import robotran_api as api
import coreutils as utils
import parameter_manager as par
from rich.progress import Progress
from itertools import chain
import matplotlib.pyplot as plt


# Constants
params_path = 'constants.json'
parameters: dict = {
    "L0sd": 0.625,
    "Ksd": 600000,
    "Dsd": 80000,
    "L0s": 0.3,
    "Ks": 50000,
    "Dd": 120,
    "Kext": 1600000.0,
    "Dext": 150.0,
    "Fz": 1000000
}
mbs_path = '../dataR/Jpp.mbs'




def dict_str(dic : dict) -> str:
    res = str(dic).removeprefix("{").removesuffix("}")
    res = res.replace("'", "").replace(":", "=")
    return res

# Initialization
par.parameters_init(params_path)
par.append_parameter(params_path ,parameters)
mbs_data = api.loader(mbs_path)

# Configuration
names = ["Analyse de la position pour plusieur Kext",
         "Analyse de la position pour plusieur Dext", 
         "Analyse de la position pour plusieur poids passager"]
utils.log(f"Configuring names : {names}", utils.Level.INFO)

labels = [('Temps [s]', 'Hauteur du corps [m]') for i in range(2)]
utils.log(f"Configuring labels : {labels}", utils.Level.INFO)

params = [[{'Kext': 3000.0}, {'Kext': 1000.0},{'Kext': 500.0}],
          [{'Kext':2000.0,'Dext': 300.0}, {'Dext': 100.0}, {'Dext' : 150.0}],
          [{'Fz':2000000, 'Fz':3000000,'Fz':500000}]]
utils.log(f"Configuring params : {params}", utils.Level.INFO)

legends = [[f'Position du chariot avec {dict_str(i)}' for i in j] for j in params]
utils.log(f"Configuring legends : {legends}", utils.Level.INFO)

ylims = [None for i in labels]
utils.log(f"Configuring ylims : {ylims}", utils.Level.INFO)


# Main loop
with Progress() as progress:
    task_plot = progress.add_task("[blue]Generating plots...", total=len(labels))
    taks_dyn = progress.add_task("[green]Running dynamic analysis...", total=len(list(chain.from_iterable(params))))
    for plot in range(len(labels)):
        try:
            utils.log("Starting new plot", utils.Level.INFO)
            axis = api.plot_initializer(labels[plot][0],
                                labels[plot][1],
                                names[plot]
                                )
            t0 = 0.0
            tf = 5.0        
            for curve in range(len(params[plot])):
                try:
                    utils.log("Starting new dynamic simulation", utils.Level.INFO)
                    par.append_parameters(params_path,[params[plot][curve]])
                    results, nt0, ntf = api.dir_dyn(mbs_data)
                    axis.plot(results.q[:, 0], results.q[:, 4], label=legends[plot][curve])
                    progress.update(taks_dyn, advance=1)
                    t0 = min(t0, nt0)
                    tf = max(tf, ntf)
                except Exception as e:
                    utils.log(f"Failed to get the curve due to {e}", utils.Level.ERROR)
            api.plot_finalizer(axis, (t0, tf), ylims[plot])
            plt.savefig(names[plot])
            progress.update(task_plot, advance=1)
        except Exception as e:
            utils.log(f"Failed to plot due to {e}", utils.Level.ERROR)
    