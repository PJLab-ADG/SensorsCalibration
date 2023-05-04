# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


from .datatable import DataTable
from .bspline import BSpline
from .bsplinebuilder import BSplineBuilder

from .splinter import load

try:
    load()
except Exception as e:
    print(e)

__all__ = [
    "splinter",
    "datatable",
    "bspline",
    "bsplinebuilder"
]

splinter.BSplineBuilder = bsplinebuilder.BSplineBuilder
