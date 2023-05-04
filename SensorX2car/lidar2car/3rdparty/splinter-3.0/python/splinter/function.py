# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


from . import splinter
from .utilities import *
import pandas as pd
import numpy as np


class Function:
    def __init__(self):
        self._handle = None
        self._num_variables = None

    def eval(self, x):
        x = self._transform_input(x)

        num_points = len(x) // self._num_variables
        res = splinter._call(splinter._get_handle().splinter_bspline_eval_row_major, self._handle, (c_double * len(x))(*x), len(x))

        return c_array_to_list(res, num_points)

    def eval_jacobian(self, x):
        x = self._transform_input(x)

        num_points = len(x) // self._num_variables
        jac = splinter._call(splinter._get_handle().splinter_bspline_eval_jacobian_row_major, self._handle, (c_double * len(x))(*x), len(x))

        # Convert from ctypes array to Python list of lists
        # jacobians is a list of the jacobians in all evaluated points
        jacobians = []
        for i in range(num_points):
            jacobians.append([])
            for j in range(self._num_variables):
                jacobians[i].append(jac[i * self._num_variables + j])
        return jacobians

    def eval_hessian(self, x):
        x = self._transform_input(x)

        num_points = len(x) // self._num_variables
        hes = splinter._call(splinter._get_handle().splinter_bspline_eval_hessian_row_major, self._handle, (c_double * len(x))(*x), len(x))

        # Convert from ctypes array to Python list of list of lists
        # hessians is a list of the hessians in all points
        hessians = []
        for i in range(num_points):
            hessians.append([])
            for j in range(self._num_variables):
                hessians[i].append([])
                for k in range(self._num_variables):
                    hessians[i][j].append(hes[i * self._num_variables * self._num_variables + j * self._num_variables + k])
        return hessians

    def get_num_variables(self):
        return splinter._call(splinter._get_handle().splinter_bspline_get_num_variables, self._handle)

    def save(self, filename):
        splinter._call(splinter._get_handle().splinter_bspline_save, self._handle, get_c_string(filename))

    def _transform_input(self, x):
        if isinstance(x, np.ndarray):
            x = x.tolist()

        if not isinstance(x, list):
            x = [x]

        # See if x is on the form [[x0,x1],[x2,x3]]
        # if not we assume it to be on the form
        # [x0, x1, x2, x3]
        if isinstance(x[0], list):
            x = flatten_list(x)

        return x


    def __del__(self):
        if self._handle is not None:
            splinter._call(splinter._get_handle().splinter_bspline_delete, self._handle)
        self._handle = None
