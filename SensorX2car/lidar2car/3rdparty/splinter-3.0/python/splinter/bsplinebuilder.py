# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


from . import splinter
from ctypes import *
from .bspline import BSpline
from .datatable import DataTable


class BSplineBuilder:

    class Smoothing:
        NONE, IDENTITY, PSPLINE = range(3)

        @staticmethod
        def is_valid(value):
            return value in range(3)

    class KnotSpacing:
        AS_SAMPLED, EQUIDISTANT, EXPERIMENTAL = range(3)

        @staticmethod
        def is_valid(value):
            return value in range(3)

    def __init__(self, x, y, degree=3, smoothing=Smoothing.NONE, alpha=0.1, knot_spacing=KnotSpacing.AS_SAMPLED, num_basis_functions=int(1e6)):
        self._handle = None  # Handle for referencing the c side of this object
        self._datatable = DataTable(x, y)
        self._num_basis_functions = [10 ** 3] * self._datatable.get_num_variables()

        self._degrees = None
        self._alpha = None
        self._smoothing = None
        self._knot_spacing = None
        self._num_basis_functions = None

        self._handle = splinter._call(splinter._get_handle().splinter_bspline_builder_init, self._datatable._get_handle())
        self.degree(degree)
        self.set_alpha(alpha)
        self.smoothing(smoothing)
        self.knot_spacing(knot_spacing)
        self.num_basis_functions(num_basis_functions)

    def degree(self, degrees):
        # If the value is a single number, make it a list of numVariables length
        if not isinstance(degrees, list):
            degrees = [degrees] * self._datatable.get_num_variables()

        if len(degrees) != self._datatable.get_num_variables():
            raise ValueError("BSplineBuilder:degree: Inconsistent number of degrees.")

        valid_degrees = range(0, 6)
        for deg in degrees:
            if deg not in valid_degrees:
                raise ValueError("BSplineBuilder:degree: Invalid degree: " + str(deg))

        self._degrees = degrees

        splinter._call(splinter._get_handle().splinter_bspline_builder_set_degree, self._handle, (c_int * len(self._degrees))(*self._degrees), len(self._degrees))
        return self

    def set_alpha(self, new_alpha):
        if new_alpha < 0:
            raise ValueError("BSplineBuilder:set_alpha: alpha must be non-negative.")

        self._alpha = new_alpha

        splinter._call(splinter._get_handle().splinter_bspline_builder_set_alpha, self._handle, self._alpha)
        return self

    def smoothing(self, smoothing):
        if not BSplineBuilder.Smoothing.is_valid(smoothing):
            raise ValueError("BSplineBuilder::smoothing: Invalid smoothing: " + str(smoothing))

        self._smoothing = smoothing

        splinter._call(splinter._get_handle().splinter_bspline_builder_set_smoothing, self._handle, self._smoothing)
        return self

    def knot_spacing(self, knot_spacing):
        if not BSplineBuilder.KnotSpacing.is_valid(knot_spacing):
            raise ValueError("BSplineBuilder::knot_spacing: Invalid knotspacing: " + str(knot_spacing))

        self._knot_spacing = knot_spacing

        splinter._call(splinter._get_handle().splinter_bspline_builder_set_knot_spacing, self._handle, self._knot_spacing)
        return self

    def num_basis_functions(self, num_basis_functions):
        # If the value is a single number, make it a list of num_variables length
        if not isinstance(num_basis_functions, list):
            num_basis_functions = [num_basis_functions] * self._datatable.get_num_variables()

        if len(num_basis_functions) != self._datatable.get_num_variables():
            raise ValueError("BSplineBuilder:num_basis_functions: Inconsistent number of degrees.")

        for num_basis_function in num_basis_functions:
            if not isinstance(num_basis_function, int):
                raise ValueError(
                    "BSplineBuilder:num_basis_functions: Invalid number of basis functions (must be integer): " + str(
                        num_basis_function))

        self._num_basis_functions = num_basis_functions

        splinter._call(splinter._get_handle().splinter_bspline_builder_set_num_basis_functions, self._handle,
                       (c_int * len(self._num_basis_functions))(*self._num_basis_functions),
                       len(self._num_basis_functions))
        return self

    # Returns a handle to the created internal BSpline object
    def build(self):
        bspline_handle = splinter._call(splinter._get_handle().splinter_bspline_builder_build, self._handle)

        return BSpline(bspline_handle)

    def __del__(self):
        if self._handle is not None:
            splinter._call(splinter._get_handle().splinter_bspline_builder_delete, self._handle)
        self._handle = None
