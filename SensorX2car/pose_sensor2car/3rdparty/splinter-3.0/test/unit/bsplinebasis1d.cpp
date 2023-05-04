/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <bsplinebasis1d.h>

using namespace SPLINTER;

#define COMMON_TAGS "[unit][bsplinebasis1d]"
#define COMMON_TEXT " unit test"

TEST_CASE("supportHack" COMMON_TEXT, COMMON_TAGS)
{
    std::vector<double> knots = {1, 1, 1, 2.1, 3.1, 4, 4, 4};
    BSplineBasis1D bb(knots, 2);

    double x = 4;
    bb.supportHack(x);
    REQUIRE(x < 4);
}

