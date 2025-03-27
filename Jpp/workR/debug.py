#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple program to perform direct dynamics
"""
import robotran_api as api
import coreutils as utils
import parameter_manager as par
import matplotlib.pyplot as plt

params_path = 'constants.json'
parameters: dict = {
    'Kext': 2000.0,
    'Dext': 150.0
}
mbs_path = '../dataR/Jpp.mbs'

par.parameters_init(params_path)
par.append_parameter(params_path ,parameters)
mbs_data = api.loader(mbs_path)

axis = api.plot_initializer('Temps [s]', 
                            'Hauteur [m]',
                            'Example plot'    
                            )
par.append_parameters(params_path,[parameters])
results, t0, tf = api.dir_dyn(mbs_data)
axis.plot(results.q[:, 0], results.q[:, 1], label="Position")
api.plot_finalizer(axis, (t0, tf), (0, max(results.q[:, 1])))
plt.show()