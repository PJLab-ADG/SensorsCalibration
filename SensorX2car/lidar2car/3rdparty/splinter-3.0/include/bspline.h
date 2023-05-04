/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_BSPLINE_H
#define SPLINTER_BSPLINE_H

#include "function.h"
#include "bsplinebasis.h"

namespace SPLINTER
{

/**
 * Class that implements the multivariate tensor product B-spline
 */
class SPLINTER_API BSpline : public Function
{
public:
    /**
     * Builder class for construction by regression
     * Implemented in BSplineBuilder.*
     */
    class Builder;
    enum class Smoothing;
    enum class KnotSpacing;

    BSpline(unsigned int numVariables);

    /**
     * Construct B-spline from knot vectors, coefficients, and basis degrees
     */
    BSpline(std::vector< std::vector<double> > knotVectors, std::vector<unsigned int> basisDegrees);
    BSpline(std::vector<double> coefficients, std::vector< std::vector<double> > knotVectors, std::vector<unsigned int> basisDegrees);
    BSpline(DenseVector coefficients, std::vector< std::vector<double> > knotVectors, std::vector<unsigned int> basisDegrees);

    /**
     * Construct B-spline from file
     */
    BSpline(const char *fileName);
    BSpline(const std::string &fileName);

    virtual BSpline* clone() const { return new BSpline(*this); }

    /**
     * Evaluation of B-spline
     */

    // Avoid name hiding
    using Function::eval;
    using Function::evalJacobian;
    using Function::evalHessian;

    // Evaluation of B-spline
    double eval(DenseVector x) const override;
    DenseMatrix evalJacobian(DenseVector x) const override;
    DenseMatrix evalHessian(DenseVector x) const override;

    // Evaluation of B-spline basis functions
    SparseVector evalBasis(DenseVector x) const;
    SparseMatrix evalBasisJacobian(DenseVector x) const;

    /**
     * Getters
     */
    DenseVector getCoefficients()
    {
        return coefficients;
    }

    unsigned int getNumCoefficients() const
    {
        return coefficients.size();
    }

    unsigned int getNumControlPoints() const
    {
        return coefficients.size();
    }

    std::vector<unsigned int> getNumBasisFunctionsPerVariable() const;

    unsigned int getNumBasisFunctions() const
    {
        return basis.getNumBasisFunctions();
    }

    DenseMatrix getControlPoints() const;
    std::vector< std::vector<double>> getKnotVectors() const;
    std::vector<unsigned int> getBasisDegrees() const;
    std::vector<double> getDomainUpperBound() const;
    std::vector<double> getDomainLowerBound() const;

    /**
     * Setters
     */
    void setCoefficients(const DenseVector &coefficients);
    void setControlPoints(const DenseMatrix &controlPoints);
    void checkControlPoints() const;

    // Linear transformation of control points (B-spline has affine invariance)
    void updateControlPoints(const DenseMatrix &A);

    // Reduce support of B-spline
    void reduceSupport(std::vector<double> lb, std::vector<double> ub, bool doRegularizeKnotVectors = true);

    // Perform global knot refinement
    void globalKnotRefinement(); // All knots in one shabang

    // Perform a local knot refinement at x
    void localKnotRefinement(DenseVector x);

    // Decompose B-spline to Bezier form
    void decomposeToBezierForm();

    // Insert a knot until desired knot multiplicity is obtained
    void insertKnots(double tau, unsigned int dim, unsigned int multiplicity = 1);

    void save(const std::string &fileName) const override;

    std::string getDescription() const override;

protected:
    BSpline();

    BSplineBasis basis;

    /*
     * The control point matrix is P = (knotaverages, coefficients) in R^(m x n),
     * where m = numBasisFunctions and n = numVariables + 1. Each row in P is a control point.
     */
    DenseVector coefficients;
    DenseMatrix knotaverages;

    // Control point computations
    DenseMatrix computeKnotAverages() const;

private:
    // Domain reduction
    void regularizeKnotVectors(std::vector<double> &lb, std::vector<double> &ub);
    bool removeUnsupportedBasisFunctions(std::vector<double> &lb, std::vector<double> &ub);

    // Helper functions
    bool pointInDomain(DenseVector x) const;

    void load(const std::string &fileName) override;

    friend class Serializer;
    friend bool operator==(const BSpline &lhs, const BSpline &rhs);
};

} // namespace SPLINTER

#endif // SPLINTER_BSPLINE_H
