/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_KNOTS_H
#define SPLINTER_KNOTS_H

#include <vector>

namespace SPLINTER
{

// Knot vector related
bool isKnotVectorRegular(const std::vector<double> &knots, unsigned int degree);
bool isKnotVectorClamped(const std::vector<double> &knots, unsigned int degree);
bool isKnotVectorRefinement(const std::vector<double> &knots, const std::vector<double> &refinedKnots);

} // namespace SPLINTER

#endif // SPLINTER_KNOTS_H
