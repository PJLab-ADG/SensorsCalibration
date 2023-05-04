/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_BSPLINEBUILDER_H
#define SPLINTER_BSPLINEBUILDER_H

#include "datatable.h"
#include "bspline.h"

namespace SPLINTER
{

// B-spline smoothing
enum class BSpline::Smoothing
{
    NONE,       // No smoothing
    IDENTITY,   // Regularization term alpha*c'*I*c is added to OLS objective
    PSPLINE     // Smoothing term alpha*Delta(c,2) is added to OLS objective
};

// B-spline knot spacing
/*
 * To be added:
 * AS_SAMPLED_NOT_CLAMPED   // Place knots close to sample points. Without clamps.
 * EQUIDISTANT_NOT_CLAMPED  // Equidistant knots without clamps.
 */
enum class BSpline::KnotSpacing
{
    AS_SAMPLED,     // Mimic spacing of sample points (moving average). With clamps (p+1 multiplicity of end knots).
    EQUIDISTANT,    // Equidistant knots. With clamps (p+1 multiplicity of end knots).
    EXPERIMENTAL    // Experimental knot spacing (for testing purposes).
};

// B-spline builder class
class SPLINTER_API BSpline::Builder
{
public:
    Builder(const DataTable &data);

    Builder& alpha(double alpha)
    {
        if (alpha < 0)
            throw Exception("BSpline::Builder::alpha: alpha must be non-negative.");

        _alpha = alpha;
        return *this;
    }

    // Set build options

    Builder& degree(unsigned int degree)
    {
        _degrees = getBSplineDegrees(_data.getNumVariables(), degree);
        return *this;
    }

    Builder& degree(std::vector<unsigned int> degrees)
    {
        if (degrees.size() != _data.getNumVariables())
            throw Exception("BSpline::Builder: Inconsistent length on degree vector.");
        _degrees = degrees;
        return *this;
    }

    Builder& numBasisFunctions(unsigned int numBasisFunctions)
    {
        _numBasisFunctions = std::vector<unsigned int>(_data.getNumVariables(), numBasisFunctions);
        return *this;
    }

    Builder& numBasisFunctions(std::vector<unsigned int> numBasisFunctions)
    {
        if (numBasisFunctions.size() != _data.getNumVariables())
            throw Exception("BSpline::Builder: Inconsistent length on numBasisFunctions vector.");
        _numBasisFunctions = numBasisFunctions;
        return *this;
    }

    Builder& knotSpacing(KnotSpacing knotSpacing)
    {
        _knotSpacing = knotSpacing;
        return *this;
    }

    Builder& smoothing(Smoothing smoothing)
    {
        _smoothing = smoothing;
        return *this;
    }

    // Build B-spline
    BSpline build() const;

private:
    Builder();

    std::vector<unsigned int> getBSplineDegrees(unsigned int numVariables, unsigned int degree)
    {
        if (degree > 5)
            throw Exception("BSpline::Builder: Only degrees in range [0, 5] are supported.");
        return std::vector<unsigned int>(numVariables, degree);
    }

    // Control point computations
    DenseVector computeCoefficients(const BSpline &bspline) const;
    DenseVector computeBSplineCoefficients(const BSpline &bspline) const;
    SparseMatrix computeBasisFunctionMatrix(const BSpline &bspline) const;
    DenseVector getSamplePointValues() const;
    // P-spline control point calculation
    SparseMatrix getSecondOrderFiniteDifferenceMatrix(const BSpline &bspline) const;

    // Computing knots
    std::vector<std::vector<double>> computeKnotVectors() const;
    std::vector<double> computeKnotVector(const std::vector<double> &values, unsigned int degree, unsigned int numBasisFunctions) const;
    std::vector<double> knotVectorMovingAverage(const std::vector<double> &values, unsigned int degree) const;
    std::vector<double> knotVectorEquidistant(const std::vector<double> &values, unsigned int degree, unsigned int numBasisFunctions) const;
    std::vector<double> knotVectorBuckets(const std::vector<double> &values, unsigned int degree, unsigned int maxSegments = 10) const;

    // Auxiliary
    std::vector<double> extractUniqueSorted(const std::vector<double> &values) const;

    // Member variables
    DataTable _data;
    std::vector<unsigned int> _degrees;
    std::vector<unsigned int> _numBasisFunctions;
    KnotSpacing _knotSpacing;
    Smoothing _smoothing;
    double _alpha;
};

} // namespace SPLINTER

#endif // SPLINTER_BSPLINEBUILDER_H
