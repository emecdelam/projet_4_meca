from dataclasses import dataclass
from datetime import datetime
from typing import Optional


class Colors:
    '''Colors class:reset all colors with colors.reset; two
    sub classes fg for foreground
    and bg for background; use as colors.subclass.colorname.
    i.e. colors.fg.red or colors.bg.greenalso, the generic bold, disable,
    underline, reverse, strike through,
    and invisible work with the main class i.e. colors.bold'''
    reset = '\033[0m'
    bold = '\033[01m'
    disable = '\033[02m'
    underline = '\033[04m'
    reverse = '\033[07m'
    strikethrough = '\033[09m'
    invisible = '\033[08m'
    black = '\033[30m'
    red = '\033[31m'
    green = '\033[32m'
    orange = '\033[33m'
    blue = '\033[34m'
    purple = '\033[35m'
    cyan = '\033[36m'
    lightgrey = '\033[37m'
    darkgrey = '\033[90m'
    lightred = '\033[91m'
    lightgreen = '\033[92m'
    yellow = '\033[93m'
    lightblue = '\033[94m'
    pink = '\033[95m'
    lightcyan = '\033[96m'

    class bg:
        black = '\033[40m'
        red = '\033[41m'
        green = '\033[42m'
        orange = '\033[43m'
        blue = '\033[44m'
        purple = '\033[45m'
        cyan = '\033[46m'
        lightgrey = '\033[47m'


@dataclass
class Level:
    DEBUG: str = 'DEBUG'
    INFO: str = 'INFO'
    WARNING: str = 'WARNING'
    ERROR: str = 'ERROR'
    CRITICAL: str = 'CRITICAL'
    SUCCESS: str = 'SUCCESS'

def log(message_content: str, level: str, text_color: Optional[str] = "") -> None:

    match level:
        case Level.CRITICAL:
            text_color += Colors.black + Colors.bg.orange
        case Level.ERROR:
            text_color += Colors.black + Colors.bg.red
        case Level.WARNING:
            text_color = Colors.yellow
        case Level.SUCCESS:
            text_color += Colors.green
        case Level.INFO:
            text_color += Colors.blue
        case _:
            pass

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    if text_color:
        print(f'{text_color}[{timestamp.ljust(19)}] [{str(level).ljust(8)}] {message_content}{Colors.reset}')
    else:
        print(f'[{timestamp.ljust(19)}] [{str(level).ljust(8)}] {message_content}{Colors.reset}')