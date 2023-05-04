/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <testfunction.h>
#include <utilities.h>
#include <cmath> // std::ceil

namespace SPLINTER
{

TestFunction::TestFunction(std::function<double (const std::vector<double> &)> f,
                           size_t numVariables,
                           std::string functionString)
        : Function(numVariables),
          functionString(functionString),
          constDegree(false),
          powers(DenseMatrix::Zero(0, 0)),
          f(f)
{
}

TestFunction::TestFunction(std::function<double (const std::vector<double> &)> f, size_t numVariables,
                           std::string functionString,  DenseMatrix powers)
        : Function(numVariables),
          functionString(functionString),
          constDegree(true),
          powers(powers),
          f(f)
{
}

TestFunction::~TestFunction()
{
}

double TestFunction::eval(DenseVector x) const
{
    return f(denseVectorToVector(x));
}

std::vector<unsigned int> TestFunction::getConstDegreeInt() const
{
    auto intDegrees = std::vector<unsigned int>(powers.rows(), 0);

    auto maxCoeffs = powers.rowwise().maxCoeff();
    for (size_t i = 0; i < powers.rows(); ++i)
    {
        intDegrees.at(i) = (unsigned int) std::ceil(maxCoeffs(i));
    }

    return intDegrees;
}

double TestFunction::getMaxDegree() const
{
    double maxDegree = 0.0;
    for (auto deg : getConstDegreeInt())
    {
        if (deg > maxDegree)
        {
            maxDegree = deg;
        }
    }
    return maxDegree;
}

} // namespace SPLINTER
