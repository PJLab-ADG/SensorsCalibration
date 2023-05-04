/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <function.h>
#include "utilities.h"

namespace SPLINTER
{

double Function::eval(const std::vector<double> &x) const
{
    auto denseX = vectorToDenseVector(x);

    return eval(denseX);
}

std::vector<double> Function::evalJacobian(const std::vector<double> &x) const
{
    auto denseX = vectorToDenseVector(x);

    return denseVectorToVector(evalJacobian(denseX));
}

std::vector<std::vector<double>> Function::evalHessian(const std::vector<double> &x) const
{
    auto denseX = vectorToDenseVector(x);

    return denseMatrixToVectorVector(secondOrderCentralDifference(denseX));
}

std::vector<double> Function::centralDifference(const std::vector<double> &x) const
{
    auto denseX = vectorToDenseVector(x);

    auto dx = centralDifference(denseX);

    return denseVectorToVector(dx);
}

std::vector<std::vector<double>> Function::secondOrderCentralDifference(const std::vector<double> &x) const
{
    auto denseX = vectorToDenseVector(x);

    DenseMatrix ddx = secondOrderCentralDifference(denseX);

    return denseMatrixToVectorVector(ddx);
}

DenseMatrix Function::evalJacobian(DenseVector x) const
{
    return centralDifference(x);
}

DenseMatrix Function::evalHessian(DenseVector x) const
{
    auto vec = denseVectorToVector(x);

    auto hessian = evalHessian(vec);

    return vectorVectorToDenseMatrix(hessian);
}

DenseMatrix Function::centralDifference(DenseVector x) const
{
    DenseMatrix dx(1, x.size());

    double h = 1e-6; // perturbation step size
    double hForward = 0.5*h;
    double hBackward = 0.5*h;

    for (unsigned int i = 0; i < getNumVariables(); ++i)
    {
        DenseVector xForward(x);
        xForward(i) = xForward(i) + hForward;

        DenseVector xBackward(x);
        xBackward(i) = xBackward(i) - hBackward;

        double yForward = eval(xForward);
        double yBackward = eval(xBackward);

        dx(i) = (yForward - yBackward)/(hBackward + hForward);
    }

    return dx;
}

DenseMatrix Function::secondOrderCentralDifference(DenseVector x) const
{
    DenseMatrix ddx(getNumVariables(), getNumVariables());

    double h = 1e-6; // perturbation step size
    double hForward = 0.5*h;
    double hBackward = 0.5*h;

    for (size_t i = 0; i < getNumVariables(); ++i)
    {
        for (size_t j = 0; j < getNumVariables(); ++j)
        {
            DenseVector x0(x);
            DenseVector x1(x);
            DenseVector x2(x);
            DenseVector x3(x);

            x0(i) = x0(i) + hForward;
            x0(j) = x0(j) + hForward;

            x1(i) = x1(i) - hBackward;
            x1(j) = x1(j) + hForward;

            x2(i) = x2(i) + hForward;
            x2(j) = x2(j) - hBackward;

            x3(i) = x3(i) - hBackward;
            x3(j) = x3(j) - hBackward;

            ddx(i, j) = (eval(x0) - eval(x1) - eval(x2) + eval(x3)) / (h * h);
        }
    }

    return ddx;
}

} // namespace SPLINTER