# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


import sys  # Python version
from ctypes import *


def out(text, newLine=True):
    sys.stdout.write(text)
    if newLine:
        print("")


def is_python3():
    return sys.version_info[0] == 3


def get_c_string(py_string):
    if is_python3():
        return c_char_p(py_string.encode("UTF-8"))
    else:
        return c_char_p(py_string)


def get_py_string(c_string):
    return str(c_string)


def is_string(py_string):
    if is_python3():
        return isinstance(py_string, str)
    else:
        return isinstance(py_string, basestring)


def list_to_c_array_of_doubles(py_list):
    return (c_double * len(py_list))(*py_list)


def list_to_c_array_of_of_ints(py_list):
    int_list = [int(x) for x in py_list]
    return (c_int * len(int_list))(*int_list)


def c_array_to_list(CArray, size):
    return [CArray[i] for i in range(size)]


# http://stackoverflow.com/questions/952914/making-a-flat-list-out-of-list-of-lists-in-python
#def flattenList(l):
#    return [item for sublist in l for item in sublist]

def flatten_list(l):
    result = []
    for item in l:
        if isinstance(item, list):
            result += flatten_list(item)
        else:
            result.append(item)
    return result


def get_architecture():
    arch = "x86"
    if sizeof(c_void_p) == 8:
        arch = "x86-64"
    return arch
