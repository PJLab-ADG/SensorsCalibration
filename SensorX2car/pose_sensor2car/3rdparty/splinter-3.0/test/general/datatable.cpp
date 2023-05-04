/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <testingutilities.h>
#include <datatable.h>

using namespace SPLINTER;


#define COMMON_TAGS "[general][datatable]"


TEST_CASE("DataTable set operations", COMMON_TAGS) {
    int dim = 2;
    auto func = getTestFunction(dim, 2);

    auto start = std::vector<double>(dim);
    auto end = std::vector<double>(dim);
    auto points = std::vector<unsigned int>(dim);

    /* Grid:
     *   0 1 2 3 4 5 6 7 8 9
     * 0 x x x x x x x x x x
     * 1 x x x x x x x x x x
     * 2 x x x x x x x x x x
     * 3 x x x x x x x x x x
     * 4 x x x x x x x x x x
     * 5 o o o o o o o o o o
     * 6 o o o o o o o o o o
     * 7 o o o o o o o o o o
     * 8 o o o o o o o o o o
     * 9 o o o o o o o o o o
     *
     * x: samples in table1
     * o: samples in table2
     */

    start.at(0) = 0.0;
    start.at(1) = 0.0;
    end.at(0) = 9.0;
    end.at(1) = 4.0;
    points.at(0) = 10;
    points.at(1) = 5;
    // range1 is from 0.0,0.0 to 9.0,4.0, 50 points total
    auto range1 = linspace(start, end, points);
    auto table1 = sample(func, range1);

    start.at(0) = 0.0;
    start.at(1) = 5.0;
    end.at(0) = 9.0;
    end.at(1) = 9.0;
    points.at(0) = 10;
    points.at(1) = 5;
    // range2 is from 0.0,5.0 to 9.0,9.0, 50 points total
    auto range2 = linspace(start, end, points);
    auto table2 = sample(func, range2);

    start.at(0) = 0.0;
    start.at(1) = 0.0;
    end.at(0) = 9.0;
    end.at(1) = 9.0;
    points.at(0) = 10;
    points.at(1) = 10;
    // range3 is from 0.0,0.0 to 9.0,9.0, 100 points total
    auto range3 = linspace(start, end, points);
    auto table3 = sample(func, range3);

    // Summation
    CHECK(table1 + table2 == table3);
    CHECK(table2 + table1 == table3);

    // Subtraction
    CHECK(table3 - table2 == table1);
    CHECK(table3 - table1 == table2);

    // A table subtracted from itself should yield a table with no samples
    DataTable zeroSampleTable;
    CHECK(table3 - table3 == zeroSampleTable);
    CHECK(table3 - table2 - table1 == zeroSampleTable);

    // A table added to itself should be equal to itself (disregarding allowing duplicates)
    auto table4 = table3 + table3;
    CHECK(table4 == table3);
}
