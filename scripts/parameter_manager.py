import json
import os
import coreutils
from typing import List, Dict

def path_check(path: str) -> bool:
    if os.path.isfile(path):
        return False
    coreutils.log(f"Cannot read data at path : {path}", coreutils.Level.CRITICAL)
    return True


def write_data(path: str, new_data: Dict) -> None:
    with open(path, "w") as file:
        json.dump(new_data, file, indent=4)

def read_data(path: str) -> dict:
    if path_check(path):
        return
    with open(path, "r") as file:
        return json.load(file)

def append_parameters(path: str, parameters: List[Dict]) -> None:
    """Append by receiving a list of dictionnaries"""
    if path_check(path):
        return
    if type(parameters) == dict:
        coreutils.log("Trying to append multiple parameters that aren't inside a list, to append multiple parameters use : [{'a':1},{'b':0}], defaulting to single parameter use", coreutils.Level.WARNING)
        parameters = [parameters]

    data = read_data(path)
    for parameter in parameters:
        data.append(parameter)
    write_data(path, data)

def append_paremeter(path: str, parameter: Dict) -> None:
    """Append by receiving a dictionnary"""
    append_parameters(path, [parameter])

def parameters_init(path: str) -> None:
    """Initialises empty json file"""
    if path_check(path):
        coreutils.log("Trying to initialize a file that already exists, skipping", coreutils.Level.INFO)
        return
    with open(path, "w") as file:
        json.dump({}, file, indent=4)
    coreutils.log(f"Successfully create parameters file at : {path}", coreutils.Level.SUCCESS)