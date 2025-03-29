#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple program containing useful functions for colored logging with caller tracking.
"""
from enum import Enum
from datetime import datetime, UTC
from typing import Optional
from inspect import currentframe, getframeinfo
from os import path


class Colors:
    """
    Colors class providing ANSI color codes for terminal output formatting.
    """
    # Reset
    RESET = '\033[0m'
    
    # Styles
    BOLD = '\033[01m'
    DISABLE = '\033[02m'
    UNDERLINE = '\033[04m'
    REVERSE = '\033[07m'
    STRIKETHROUGH = '\033[09m'
    INVISIBLE = '\033[08m'
    
    # Foreground colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    ORANGE = '\033[33m'
    BLUE = '\033[34m'
    PURPLE = '\033[35m'
    CYAN = '\033[36m'
    LIGHTGREY = '\033[37m'
    DARKGREY = '\033[90m'
    LIGHTRED = '\033[91m'
    LIGHTGREEN = '\033[92m'
    YELLOW = '\033[93m'
    LIGHTBLUE = '\033[94m'
    PINK = '\033[95m'
    LIGHTCYAN = '\033[96m'
    
    # Background colors
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_ORANGE = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_PURPLE = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_LIGHTGREY = '\033[47m'


class Level(Enum):
    """Enumeration of log levels with proper string representation."""
    DEBUG = 'DEBUG'
    INFO = 'INFO'
    WARNING = 'WARNING'
    ERROR = 'ERROR'
    CRITICAL = 'CRITICAL'
    SUCCESS = 'SUCCESS'
    
    def __str__(self) -> str:
        return self.value


def get_caller_info() -> str:
    """
    Get information about where the log function was called.
    
    Returns:
        String with file path, line number and function name
    """
    try:
        # Get the frame 2 levels up (skip get_caller_info and log function)
        caller_frame = currentframe().f_back.f_back
        
        if not caller_frame:
            return "unknown location"
        
        frame_info = getframeinfo(caller_frame)
        
        file_path = path.abspath(frame_info.filename)
        line_number = frame_info.lineno
        function_name = frame_info.function

        return f"{file_path}:{line_number} in {function_name}()"
    except Exception as e:
        return f"unknown location (error: {str(e)})"


def log(message_content: str, level: Level, text_color: Optional[str] = "") -> None:
    """
    Log a message with the specified level and color formatting.
    
    Args:
        message_content: The message to be logged
        level: The logging level (from Level enum)
        text_color: Optional additional text color to apply
    """
    level_colors = {
        Level.CRITICAL: Colors.BLACK + Colors.BG_ORANGE,
        Level.ERROR: Colors.BLACK + Colors.BG_RED,
        Level.WARNING: Colors.YELLOW,
        Level.SUCCESS: Colors.GREEN,
        Level.INFO: Colors.BLUE,
    }
    
    color = text_color + level_colors.get(level, "") if level in level_colors else text_color
    
    timestamp = datetime.now(UTC).strftime("%Y-%m-%d %H:%M:%S")
    caller_info = get_caller_info()
    formatted_message = f'[{timestamp.ljust(19)}] [{str(level).ljust(8)}] {message_content}'
    
    if color:
        print(f'{color}{formatted_message}{Colors.RESET} {Colors.DARKGREY}[{caller_info}]{Colors.RESET}')
    else:
        print(f'{formatted_message} {Colors.DARKGREY}[{caller_info}]{Colors.RESET}')


__all__ = ['Colors', 'Level', 'log']
__version__ = '1.0.0'
__author__ = 'emecdelam'