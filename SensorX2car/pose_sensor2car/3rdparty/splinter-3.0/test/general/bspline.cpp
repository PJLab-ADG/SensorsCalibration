/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <bsplinetestingutilities.h>

using namespace SPLINTER;

#define COMMON_TAGS "[general][bspline]"
#define COMMON_TEXT " subdivision test"

TEST_CASE("BSpline recursive subdivision" COMMON_TEXT, COMMON_TAGS "[subdivision]")
{
    // TODO: The current code for comparing BSplines require identical bounds which fails in this test.
    //REQUIRE(runRecursiveDomainReductionTest());
}

TEST_CASE("BSpline knot insertion" COMMON_TEXT, COMMON_TAGS "[knotinsertion]")
{
    REQUIRE(testKnotInsertion());
}