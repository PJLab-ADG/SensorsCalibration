/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <knots.h>
#include <algorithm>

namespace SPLINTER
{

bool isKnotVectorRegular(const std::vector<double> &knots, unsigned int degree)
{
    // Check size
    if (knots.size() < 2 * (degree + 1))
        return false;

    // Check order
    if (!std::is_sorted(knots.begin(), knots.end()))
        return false;

    // Check multiplicity of knots
    for (std::vector<double>::const_iterator it = knots.begin(); it != knots.end(); ++it)
    {
        if (count(knots.begin(), knots.end(), *it) > degree + 1)
            return false;
    }

    return true;
}

bool isKnotVectorClamped(const std::vector<double> &knots, unsigned int degree)
{
    // Check multiplicity of first knot
    if (std::count(knots.begin(), knots.begin() + degree + 1, knots.front()) != degree + 1)
        return false;

    // Check multiplicity of last knot
    if (std::count(knots.end() - degree - 1, knots.end(), knots.back()) != degree + 1)
        return false;

    return true;
}

bool isKnotVectorRefinement(const std::vector<double> &knots, const std::vector<double> &refinedKnots)
{
    // Check size
    if (refinedKnots.size() < knots.size())
        return false;

    // Check that each element in knots occurs at least as many times in refinedKnots
    for (std::vector<double>::const_iterator it = knots.begin() ; it != knots.end(); ++it)
    {
        int m_tau = count(knots.begin(), knots.end(), *it);
        int m_t = count(refinedKnots.begin(), refinedKnots.end(), *it);
        if (m_t < m_tau) return false;
    }

    // Check that range is not changed
    if (knots.front() != refinedKnots.front()) return false;
    if (knots.back() != refinedKnots.back()) return false;

    return true;
}

} // namespace SPLINTER
