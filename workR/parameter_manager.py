#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple program to manage multiple parameters
"""
import json
import os
import coreutils as utils
from typing import List, Dict

def _path_check(path: str) -> bool:
    if os.path.isfile(path):
        return False
    return True


def _write_data(path: str, new_data: Dict) -> None:
    with open(path, "w") as file:
        json.dump(new_data, file, indent=4)

def _read_data(path: str) -> dict:
    if _path_check(path):
        utils.log(f"Cannot read data at path : {path}", utils.Level.CRITICAL)
        return
    with open(path, "r") as file:
        return json.load(file)

def append_parameters(path: str, parameters: List[Dict]) -> None:
    """Append by receiving a list of dictionnaries"""
    if _path_check(path):
        return
    if type(parameters) == dict:
        utils.log("Trying to append multiple parameters that aren't inside a list, to append multiple parameters use : [{'a':1},{'b':0}], defaulting to single parameter use", utils.Level.WARNING)
        parameters = [parameters]

    data = _read_data(path)
    for parameter in parameters:
        for key in parameter.keys():
            data[key] = parameter[key]
    _write_data(path, data)

def append_parameter(path: str, parameter: Dict) -> None:
    """Append by receiving a dictionnary"""
    append_parameters(path, [parameter])

def parameters_init(path: str) -> None:
    """Initialises empty json file"""
    if not _path_check(path):
        # utils.log(f"Trying to initialize a file that already exists at path : {path}, skipping", utils.Level.INFO)
        return
    with open(path, "w") as file:
        json.dump({}, file, indent=4)
    utils.log(f"Successfully create parameters file at : {path}", utils.Level.SUCCESS)