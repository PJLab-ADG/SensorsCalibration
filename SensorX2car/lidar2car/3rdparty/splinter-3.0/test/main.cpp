/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#define CATCH_CONFIG_RUNNER
#include <Catch.h>
#include <testfunction.h>
#include <testfunctions.h>

int main(int argc, char *argv[])
{
    setupTestFunctions();

    int result = Catch::Session().run(argc, argv);

    tearDownTestFunctions();

    return result;
}
