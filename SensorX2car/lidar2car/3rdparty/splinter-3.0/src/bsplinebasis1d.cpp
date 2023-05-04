/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <bsplinebasis1d.h>
#include <knots.h>
#include <algorithm>
#include <utilities.h>
#include <iostream>

namespace SPLINTER
{

BSplineBasis1D::BSplineBasis1D()
{
}

BSplineBasis1D::BSplineBasis1D(const std::vector<double> &knots, unsigned int degree)
    : degree(degree),
      knots(knots),
      targetNumBasisfunctions((degree+1)+2*degree+1) // Minimum p+1
{
//    if (degree <= 0)
//        throw Exception("BSplineBasis1D::BSplineBasis1D: Cannot create B-spline basis functions of degree <= 0.");

    if (!isKnotVectorRegular(knots, degree))
        throw Exception("BSplineBasis1D::BSplineBasis1D: Knot vector is not regular.");
}

SparseVector BSplineBasis1D::eval(double x) const
{
    SparseVector values(getNumBasisFunctions());

    if (!insideSupport(x))
        return values;

    supportHack(x);

    std::vector<int> indexSupported = indexSupportedBasisfunctions(x);

    values.reserve(indexSupported.size());

    // Evaluate nonzero basis functions
    for (auto it = indexSupported.begin(); it != indexSupported.end(); ++it)
    {
        double val = deBoorCox(x, *it, degree);
        if (fabs(val) > 1e-12)
            values.insert(*it) = val;
    }

    // Alternative evaluation using basis matrix
//    int knotIndex = indexHalfopenInterval(x); // knot index

//    SparseMatrix basisvalues2 = buildBsplineMatrix(x, knotIndex, 1);
//    for (int i = 2; i <= basisDegree; i++)
//    {
//        SparseMatrix Ri = buildBsplineMatrix(x, knotIndex, i);
//        basisvalues2 = basisvalues2*Ri;
//    }
//    basisvalues2.makeCompressed();

//    assert(basisvalues2.rows() == 1);
//    assert(basisvalues2.cols() == basisDegree + 1);

    return values;
}

SparseVector BSplineBasis1D::evalDerivative(double x, int r) const
{
    // Evaluate rth derivative of basis functions at x
    // Returns vector [D^(r)B_(u-p,p)(x) ... D^(r)B_(u,p)(x)]
    // where u is the knot index and p is the degree
    int p = degree;

    // Continuity requirement
    //assert(p > r);
    if (p <= r)
    {
        // Return zero-gradient
        SparseVector DB(getNumBasisFunctions());
        return DB;
    }

    // Check for knot multiplicity here!

    supportHack(x);

    int knotIndex = indexHalfopenInterval(x);

    // Algorithm 3.18 from Lyche and Moerken (2011)
    SparseMatrix B(1,1);
    B.insert(0,0) = 1;

    for (int i = 1; i <= p-r; i++)
    {
        SparseMatrix R = buildBasisMatrix(x, knotIndex, i);
        B = B*R;
    }

    for (int i = p-r+1; i <= p; i++)
    {
        SparseMatrix DR = buildBasisMatrix(x, knotIndex, i, true);
        B = B*DR;
    }
    double factorial = std::tgamma(p+1)/std::tgamma(p-r+1);
    B = B*factorial;

    if (B.cols() != p+1)
        throw Exception("BSplineBasis1D::evalDerivative: Wrong number of columns of B matrix.");

    // From row vector to extended column vector
    SparseVector DB(getNumBasisFunctions());
    DB.reserve(p+1);
    int i = knotIndex-p; // First insertion index
    for (int k = 0; k < B.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(B,k); it; ++it)
    {
        DB.insert(i+it.col()) = it.value();
    }

    return DB;
}

// Old implementation of first derivative of basis functions
SparseVector BSplineBasis1D::evalFirstDerivative(double x) const
{
    SparseVector values(getNumBasisFunctions());

    supportHack(x);

    std::vector<int> supportedBasisFunctions = indexSupportedBasisfunctions(x);

    for (int i : supportedBasisFunctions)
    {
        // Differentiate basis function
        // Equation 3.35 in Lyche & Moerken (2011)
        double b1 = deBoorCox(x, i, degree-1);
        double b2 = deBoorCox(x, i+1, degree-1);

        double t11 = knots.at(i);
        double t12 = knots.at(i+degree);
        double t21 = knots.at(i+1);
        double t22 = knots.at(i+degree+1);

        (t12 == t11) ? b1 = 0 : b1 = b1/(t12-t11);
        (t22 == t21) ? b2 = 0 : b2 = b2/(t22-t21);

        values.insert(i) = degree*(b1 - b2);
    }

    return values;
}

// Used to evaluate basis functions - alternative to the recursive deBoorCox
SparseMatrix BSplineBasis1D::buildBasisMatrix(double x, unsigned int u, unsigned int k, bool diff) const
{
    /* Build B-spline Matrix
     * R_k in R^(k,k+1)
     * or, if diff = true, the differentiated basis matrix
     * DR_k in R^(k,k+1)
     */

    if (!(k >= 1 && k <= getBasisDegree()))
    {
        throw Exception("BSplineBasis1D::buildBasisMatrix: Incorrect input paramaters!");
    }

//    assert(u >= basisDegree + 1);
//    assert(u < ks.size() - basisDegree);

    unsigned int rows = k;
    unsigned int cols = k+1;
    SparseMatrix R(rows, cols);
    R.reserve(Eigen::VectorXi::Constant(cols, 2));

    for (unsigned int i = 0; i < rows; i++)
    {
        double dk = knots.at(u+1+i) - knots.at(u+1+i-k);
        if (dk == 0)
        {
            continue;
        }
        else
        {
            if (diff)
            {
                // Insert diagonal element
                R.insert(i,i) = -1/dk;

                // Insert super-diagonal element
                R.insert(i,i+1) = 1/dk;
            }
            else
            {
                // Insert diagonal element
                double a = (knots.at(u+1+i) - x)/dk;
                if (a != 0)
                    R.insert(i,i) = a;

                // Insert super-diagonal element
                double b = (x - knots.at(u+1+i-k))/dk;
                if (b != 0)
                    R.insert(i,i+1) = b;
            }
        }
    }

    R.makeCompressed();

    return R;
}

double BSplineBasis1D::deBoorCox(double x, int i, int k) const
{
    if (k == 0)
    {
        if (inHalfopenInterval(x, knots.at(i), knots.at(i+1)))
            return 1;
        else
            return 0;
    }
    else
    {
        double s1,s2,r1,r2;

        s1 = deBoorCoxCoeff(x, knots.at(i),   knots.at(i+k));
        s2 = deBoorCoxCoeff(x, knots.at(i+1), knots.at(i+k+1));

        r1 = deBoorCox(x, i,   k-1);
        r2 = deBoorCox(x, i+1, k-1);

        return s1*r1 + (1-s2)*r2;
    }
}

double BSplineBasis1D::deBoorCoxCoeff(double x, double x_min, double x_max) const
{
    if (x_min < x_max && x_min <= x && x <= x_max)
        return (x - x_min)/(x_max - x_min);
    return 0;
}

// Insert knots and compute knot insertion matrix (to update control points)
SparseMatrix BSplineBasis1D::insertKnots(double tau, unsigned int multiplicity)
{
    if (!insideSupport(tau))
        throw Exception("BSplineBasis1D::insertKnots: Cannot insert knot outside domain!");

    if (knotMultiplicity(tau) + multiplicity > degree + 1)
        throw Exception("BSplineBasis1D::insertKnots: Knot multiplicity is too high!");

    // New knot vector
    int index = indexHalfopenInterval(tau);

    std::vector<double> extKnots = knots;
    for (unsigned int i = 0; i < multiplicity; i++)
        extKnots.insert(extKnots.begin()+index+1, tau);

    if (!isKnotVectorRegular(extKnots, degree))
        throw Exception("BSplineBasis1D::insertKnots: New knot vector is not regular!");

    // Return knot insertion matrix
    SparseMatrix A = buildKnotInsertionMatrix(extKnots);

    // Update knots
    knots = extKnots;

    return A;
}

SparseMatrix BSplineBasis1D::refineKnots()
{
    // Build refine knot vector
    std::vector<double> refinedKnots = knots;

    unsigned int targetNumKnots = targetNumBasisfunctions + degree + 1;
    while (refinedKnots.size() < targetNumKnots)
    {
        int index = indexLongestInterval(refinedKnots);
        double newKnot = (refinedKnots.at(index) + refinedKnots.at(index+1))/2.0;
        refinedKnots.insert(std::lower_bound(refinedKnots.begin(), refinedKnots.end(), newKnot), newKnot);
    }

    if (!isKnotVectorRegular(refinedKnots, degree))
        throw Exception("BSplineBasis1D::refineKnots: New knot vector is not regular!");

    if (!isKnotVectorRefinement(knots, refinedKnots))
        throw Exception("BSplineBasis1D::refineKnots: New knot vector is not a proper refinement!");

    // Return knot insertion matrix
    SparseMatrix A = buildKnotInsertionMatrix(refinedKnots);

    // Update knots
    knots = refinedKnots;

    return A;
}

SparseMatrix BSplineBasis1D::refineKnotsLocally(double x)
{
    if (!insideSupport(x))
        throw Exception("BSplineBasis1D::refineKnotsLocally: Cannot refine outside support!");

    if (getNumBasisFunctions() >= getNumBasisFunctionsTarget()
            || assertNear(knots.front(), knots.back()))
    {
        unsigned int n = getNumBasisFunctions();
        DenseMatrix A = DenseMatrix::Identity(n,n);
        return A.sparseView();
    }

    // Refined knot vector
    std::vector<double> refinedKnots = knots;

    auto upper = std::lower_bound(refinedKnots.begin(), refinedKnots.end(), x);

    // Check left boundary
    if (upper == refinedKnots.begin())
        std::advance(upper, degree+1);

    // Get previous iterator
    auto lower = std::prev(upper);

    // Do not insert if upper and lower bounding knot are close
    if (assertNear(*upper, *lower))
    {
        unsigned int n = getNumBasisFunctions();
        DenseMatrix A = DenseMatrix::Identity(n,n);
        return A.sparseView();
    }

    // Insert knot at x
    double insertVal = x;

    // Adjust x if it is on or close to a knot
    if (knotMultiplicity(x) > 0
            || assertNear(*upper, x, 1e-6, 1e-6)
            || assertNear(*lower, x, 1e-6, 1e-6))
    {
        insertVal = (*upper + *lower)/2.0;
    }

    // Insert new knot
    refinedKnots.insert(upper, insertVal);

    if (!isKnotVectorRegular(refinedKnots, degree))
        throw Exception("BSplineBasis1D::refineKnotsLocally: New knot vector is not regular!");

    if (!isKnotVectorRefinement(knots, refinedKnots))
        throw Exception("BSplineBasis1D::refineKnotsLocally: New knot vector is not a proper refinement!");

    // Build knot insertion matrix
    SparseMatrix A = buildKnotInsertionMatrix(refinedKnots);

    // Update knots
    knots = refinedKnots;

    return A;
}

SparseMatrix BSplineBasis1D::decomposeToBezierForm()
{
    // Build refine knot vector
    std::vector<double> refinedKnots = knots;

    // Start at first knot and add knots until all knots have multiplicity degree + 1
    std::vector<double>::iterator knoti = refinedKnots.begin();
    while (knoti != refinedKnots.end())
    {
        // Insert new knots
        int mult = degree + 1 - knotMultiplicity(*knoti);
        if (mult > 0)
        {
            std::vector<double> newKnots(mult, *knoti);
            refinedKnots.insert(knoti, newKnots.begin(), newKnots.end());
        }

        // Advance to next knot
        knoti = std::upper_bound(refinedKnots.begin(), refinedKnots.end(), *knoti);
    }

    if (!isKnotVectorRegular(refinedKnots, degree))
        throw Exception("BSplineBasis1D::refineKnots: New knot vector is not regular!");

    if (!isKnotVectorRefinement(knots, refinedKnots))
        throw Exception("BSplineBasis1D::refineKnots: New knot vector is not a proper refinement!");

    // Return knot insertion matrix
    SparseMatrix A = buildKnotInsertionMatrix(refinedKnots);

    // Update knots
    knots = refinedKnots;

    return A;
}

SparseMatrix BSplineBasis1D::buildKnotInsertionMatrix(const std::vector<double> &refinedKnots) const
{
    if (!isKnotVectorRegular(refinedKnots, degree))
        throw Exception("BSplineBasis1D::buildKnotInsertionMatrix: New knot vector is not regular!");

    if (!isKnotVectorRefinement(knots, refinedKnots))
        throw Exception("BSplineBasis1D::buildKnotInsertionMatrix: New knot vector is not a proper refinement!");

    std::vector<double> knotsAug = refinedKnots;
    unsigned int n = knots.size() - degree - 1;
    unsigned int m = knotsAug.size() - degree - 1;

    SparseMatrix A(m, n);
    //A.resize(m,n);
    A.reserve(Eigen::VectorXi::Constant(n, degree + 1));

    // Build A row-by-row
    for (unsigned int i = 0; i < m; i++)
    {
        int u = indexHalfopenInterval(knotsAug.at(i));

        SparseMatrix R(1,1);
        R.insert(0,0) = 1;

        // For p > 0
        for (unsigned int j = 1; j <= degree; j++)
        {
            SparseMatrix Ri = buildBasisMatrix(knotsAug.at(i + j), u, j);
            R = R*Ri;
        }

        // Size check
        if (R.rows() != 1 || R.cols() != (int)degree + 1)
        {
            throw Exception("BSplineBasis1D::buildKnotInsertionMatrix: Incorrect matrix dimensions!");
        }

        // Insert row values
        int j = u - degree; // First insertion index
        for (int k = 0; k < R.outerSize(); ++k)
        for (SparseMatrix::InnerIterator it(R, k); it; ++it)
        {
            A.insert(i, j + it.col()) = it.value();
        }
    }

    A.makeCompressed();

    return A;
}

/*
 * The B-spline domain is the half-open domain [ knots.first(), knots.end() ).
 * The hack checks if x is at the right boundary (if x = knots.end()), if so,
 * a small number is subtracted from x, moving x into the half-open domain.
 */
void BSplineBasis1D::supportHack(double &x) const
{
    if (x == knots.back())
        x = std::nextafter(x, std::numeric_limits<double>::lowest());
}

/*
 * Finds index i such that knots.at(i) <= x < knots.at(i+1).
 * Returns false if x is outside support.
 */
int BSplineBasis1D::indexHalfopenInterval(double x) const
{
    if (x < knots.front() || x > knots.back())
        throw Exception("BSplineBasis1D::indexHalfopenInterval: x outside knot interval!");

    // Find first knot that is larger than x
    std::vector<double>::const_iterator it = std::upper_bound(knots.begin(), knots.end(), x);

    // Return index
    int index = it - knots.begin();
    return index - 1;
}

SparseMatrix BSplineBasis1D::reduceSupport(double lb, double ub)
{
    // Check bounds
    if (lb < knots.front() || ub > knots.back())
        throw Exception("BSplineBasis1D::reduceSupport: Cannot increase support!");

    unsigned int k = degree + 1;

    int index_lower = indexSupportedBasisfunctions(lb).front();
    int index_upper = indexSupportedBasisfunctions(ub).back();

    // Check lower bound index
    if (k != knotMultiplicity(knots.at(index_lower)))
    {
        int suggested_index = index_lower - 1;
        if (0 <= suggested_index)
        {
            index_lower = suggested_index;
        }
        else
        {
            throw Exception("BSplineBasis1D::reduceSupport: Suggested index is negative!");
        }
    }

    // Check upper bound index
    if (knotMultiplicity(ub) == k && knots.at(index_upper) == ub)
    {
        index_upper -= k;
    }

    // New knot vector
    std::vector<double> si;
    si.insert(si.begin(), knots.begin()+index_lower, knots.begin()+index_upper+k+1);

    // Construct selection matrix A
    int numOld = knots.size()-k; // Current number of basis functions
    int numNew = si.size()-k; // Number of basis functions after update

    if (numOld < numNew)
        throw Exception("BSplineBasis1D::reduceSupport: Number of basis functions is increased instead of reduced!");

    DenseMatrix Ad = DenseMatrix::Zero(numOld, numNew);
    Ad.block(index_lower, 0, numNew, numNew) = DenseMatrix::Identity(numNew, numNew);
    SparseMatrix A = Ad.sparseView();

    // Update knots
    knots = si;

    return A;
}

double BSplineBasis1D::getKnotValue(unsigned int index) const
{
    return knots.at(index);
}

unsigned int BSplineBasis1D::knotMultiplicity(double tau) const
{
    return std::count(knots.begin(), knots.end(), tau);
}

bool BSplineBasis1D::inHalfopenInterval(double x, double x_min, double x_max) const
{
    return (x_min <= x) && (x < x_max);
}

bool BSplineBasis1D::insideSupport(double x) const
{
    return (knots.front() <= x) && (x <= knots.back());
}

unsigned int BSplineBasis1D::getNumBasisFunctions() const
{
    return knots.size() - (degree + 1);
}

unsigned int BSplineBasis1D::getNumBasisFunctionsTarget() const
{
    return targetNumBasisfunctions;
}

// Return indices of supporting basis functions at x
std::vector<int> BSplineBasis1D::indexSupportedBasisfunctions(double x) const
{
    std::vector<int> ret;
    if (insideSupport(x))
    {
        int last = indexHalfopenInterval(x);
        if (last < 0)
        {
            // NOTE: can this happen?
            last = knots.size() - 1 - (degree + 1);
        }
        int first = std::max((int)(last - degree), 0);
        for (int i = first; i <= last; i++)
        {
            ret.push_back(i);
        }
    }
    return ret;
}

unsigned int BSplineBasis1D::indexLongestInterval() const
{
    return indexLongestInterval(knots);
}

unsigned int BSplineBasis1D::indexLongestInterval(const std::vector<double> &vec) const
{
    double longest = 0;
    double interval = 0;
    unsigned int index = 0;

    for (unsigned int i = 0; i < vec.size() - 1; i++)
    {
        interval = vec.at(i+1) - vec.at(i);
        if (longest < interval)
        {
            longest = interval;
            index = i;
        }
    }
    return index;
}

} // namespace SPLINTER
