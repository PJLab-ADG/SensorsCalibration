/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "cinterface/cinterface.h"
#include "cinterface/utilities.h"

extern "C"
{

int splinter_get_error()
{
    int temp = SPLINTER::splinter_last_func_call_error;
    SPLINTER::splinter_last_func_call_error = 0;
    return temp;
}

const char *splinter_get_error_string()
{
    return SPLINTER::splinter_error_string;
}

} // extern "C"
