import subprocess
import coreutils
import numpy as np
from typing import List, Tuple
import os
import os.path

def plot(figure_name: str, data: str, axis_x: str, axis_y: str, filename: str, ylim: float = None):
    main_path = "../workR/main.py"
    if not os.path.isfile(main_path):
        coreutils.log('The path to the main.py file is incorrect', coreutils.Level.CRITICAL)
    process = ['python3', main_path, data, axis_x, axis_y, filename]
    if ylim != None:
        process.append(ylim)
    subprocess.run(process)

plot('test', r"axis.plot(results.q[:, 0], results.q[:, 1], label='test')", "x", "y", "plot.png")