/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <datatable.h>
#include "testingutilities.h"

using namespace SPLINTER;


#define COMMON_TAGS "[serialization][datatable]"


TEST_CASE("DataTable can be saved and loaded", COMMON_TAGS)
{
    DataTable table;
    const char *fileName = "test.datatable";

    SECTION("DataTable with 0 samples") {
        table.save(fileName);
        DataTable loadedTable(fileName);

        REQUIRE(table == loadedTable);
    }

    SECTION("DataTable with samples from f_1_1")
    {
        auto testFunc = getTestFunction(1, 1);
        auto dim = testFunc->getNumVariables();
        auto points = linspace(dim, std::pow(1000, 1.0/dim));
        table = sample(testFunc, points);

        table.save(fileName);
        DataTable loadedTable(fileName);

        REQUIRE(table == loadedTable);
    }

    SECTION("DataTable with samples from f_2_1")
    {
        auto testFunc = getTestFunction(2, 1);
        auto dim = testFunc->getNumVariables();
        auto points = linspace(dim, std::pow(1000, 1.0/dim));
        table = sample(testFunc, points);

        table.save(fileName);
        DataTable loadedTable(fileName);

        REQUIRE(table == loadedTable);
    }

    remove(fileName);
}
