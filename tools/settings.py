"""
mbed SDK
Copyright (c) 2016 ARM Limited

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from os import getenv
from os.path import join, abspath, dirname, exists
import logging

ROOT = abspath(join(dirname(__file__), ".."))


##############################################################################
# Toolchains and Build System Settings
##############################################################################
BUILD_DIR = abspath(join(ROOT, ".build"))

# ARM Compiler 5
ARM_PATH = "C:/Keil_v5/ARM/ARMCC"

# GCC ARM
GCC_ARM_PATH = ""

# GCC CodeRed
GCC_CR_PATH = "C:/code_red/RedSuite_4.2.0_349/redsuite/Tools/bin"

# IAR
IAR_PATH = "C:/Program Files (x86)/IAR Systems/Embedded Workbench 7.3/arm"

# Goanna static analyser. Please overload it in mbed_settings.py
GOANNA_PATH = "c:/Program Files (x86)/RedLizards/Goanna Central 3.2.3/bin"

# cppcheck path (command) and output message format
CPPCHECK_CMD = ["cppcheck", "--enable=all"]
CPPCHECK_MSG_FORMAT = ["--template=[{severity}] {file}@{line}: {id}:{message}"]

BUILD_OPTIONS = []

# mbed.org username
MBED_ORG_USER = ""

CLI_COLOR_MAP = {
    "warning": "yellow",
    "error"  : "red"
}

##############################################################################
# User Settings (file)
##############################################################################
try:
    # Allow to overwrite the default settings without the need to edit the
    # settings file stored in the repository
    from mbed_settings import *
except ImportError:
    pass


##############################################################################
# User Settings (env vars)
##############################################################################
_ENV_PATHS = ['ARM_PATH', 'GCC_ARM_PATH', 'GCC_CR_PATH', 'IAR_PATH']

for _n in _ENV_PATHS:
    if getenv('MBED_'+_n):
        if exists(getenv('MBED_'+_n)):
            globals()[_n] = getenv('MBED_'+_n)
        else:
            print "WARNING: MBED_%s set as environment variable but doesn't exist" % _n


##############################################################################
# Test System Settings
##############################################################################
SERVER_PORT = 59432
SERVER_ADDRESS = "10.2.200.94"
LOCALHOST = "10.2.200.94"

MUTs = {
    "1" : {"mcu": "LPC1768",
        "port":"COM41", "disk":'E:\\',
        "peripherals": ["TMP102", "digital_loop", "port_loop", "analog_loop", "SD"]
    },
    "2": {"mcu": "LPC11U24",
        "port":"COM42", "disk":'F:\\',
        "peripherals":  ["TMP102", "digital_loop", "port_loop", "SD"]
    },
    "3" : {"mcu": "KL25Z",
        "port":"COM43", "disk":'G:\\',
        "peripherals":  ["TMP102", "digital_loop", "port_loop", "analog_loop", "SD"]
    },
}
