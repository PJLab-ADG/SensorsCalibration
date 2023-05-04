/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <knots.h>

using namespace SPLINTER;

#define COMMON_TAGS "[unit][knots]"
#define COMMON_TEXT " unit test"


TEST_CASE("isKnotVectorRegular" COMMON_TEXT, COMMON_TAGS)
{
    std::vector<double> knots0 = {1, 2.1, 3.1, 4};
    std::vector<double> knots1 = {1, 1, 2.1, 3.1, 4, 4};
    std::vector<double> knots2 = {1, 1, 1, 2.1, 3.1, 4, 4, 4};
    std::vector<double> knots3 = {1, 1, 1, 1, 2.1, 3.1, 4, 4, 4, 4};
    std::vector<double> knots4 = {1, 1, 2.1, 2.0, 4, 4};

    REQUIRE(isKnotVectorRegular(knots0, 0));
    REQUIRE(isKnotVectorRegular(knots1, 1));
    REQUIRE(isKnotVectorRegular(knots2, 2));
    REQUIRE(isKnotVectorRegular(knots3, 3));
    REQUIRE(isKnotVectorRegular(knots3, 4));
    REQUIRE(!isKnotVectorRegular(knots4, 1));
}

TEST_CASE("isKnotVectorClamped" COMMON_TEXT, COMMON_TAGS)
{
    std::vector<double> knots1 = {1, 1, 2.1, 3.1, 4, 4};
    std::vector<double> knots2 = {1, 1, 1, 2.1, 3.1, 4, 4, 4};
    std::vector<double> knots3 = {1, 1, 1, 1, 2.1, 3.1, 4, 4, 4, 4};

    REQUIRE(isKnotVectorClamped(knots1, 1));
    REQUIRE(isKnotVectorClamped(knots2, 2));
    REQUIRE(isKnotVectorClamped(knots3, 3));
}

TEST_CASE("isKnotVectorRefinement" COMMON_TEXT, COMMON_TAGS)
{
    std::vector<double> knots1 = {1, 1, 1, 2.1, 3.1, 4, 4, 4};
    std::vector<double> knots2 = {1, 1, 1, 2.1, 2.5, 3.1, 4, 4, 4};

    REQUIRE(isKnotVectorRefinement(knots1, knots2));
}
