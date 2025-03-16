# Structure
.
├── README.md
├── animationR
│   └── dirdyn_q.anim           <- The animation file
├── dataR
│   └── MON_LIV.mbs             <- The mbs file
├── debug.sh                    <- Executable to start a simple direct dynamic analysis for debugging
├── pyproject.toml              <- Configuration file for uv
├── resultsR/                   <- Folder for the results
├── start.sh                    <- Executable to start the brain.py in the correct folder
├── symbolicR                   <- Mbs files (do not touch)
├── userfctR
│   ├── ...
│   ├── user_ExtForces.py       <- External forces file
│   └── ...
├── uv.lock                     <- uv file for packaging
└── workR
    ├── README.md               <- You are here
    ├── brain.py                <- The core of the automation, the program to automate mutliple simulations
    ├── constants.json          <- All physics constants as parameters
    ├── coreutils.py            <- Utility functions
    ├── debug.py                <- Simple file to run basic simulation
    ├── parameter_manager.py    <- Functions to manage constants.json
    ├── robotran_api.py         <- The old main.py file, now a way to run simulations
    ├── tgc_bakker_contact.py   <- File to compute contact force
    └── tgc_car_kine_wheel.py   <- File to compute contact force