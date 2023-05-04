/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_BSPLINETESTINGUTILITIES_H
#define SPLINTER_BSPLINETESTINGUTILITIES_H

#include <bsplinebuilder.h>
#include <datatable.h>

namespace SPLINTER
{

DataTable sampleTestFunction();

/*
 * Test knot insertion
 */
bool testKnotInsertion();

/*
 * Methods for B-spline domain reduction testing
 */
bool domainReductionTest(BSpline &bs, const BSpline &bs_orig);
bool runRecursiveDomainReductionTest();

} // namespace SPLINTER

#endif //SPLINTER_BSPLINETESTINGUTILITIES_H
