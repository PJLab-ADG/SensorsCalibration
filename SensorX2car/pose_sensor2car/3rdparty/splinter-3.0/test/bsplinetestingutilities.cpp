/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <bsplinetestingutilities.h>
#include <testingutilities.h>
#include <utilities.h>

namespace SPLINTER
{

DataTable sampleTestFunction()
{
    DataTable samples;

    // Sample function
    auto x0_vec = linspace(0,2,20);
    auto x1_vec = linspace(0,2,20);
    DenseVector x(2);
    double y;

    for (auto x0 : x0_vec)
    {
        for (auto x1 : x1_vec)
        {
            // Sample function at x
            x(0) = x0;
            x(1) = x1;
            y = sixHumpCamelBack(x);

            // Store sample
            samples.addSample(x,y);
        }
    }

    return samples;
}

bool testKnotInsertion()
{
    DataTable samples = sampleTestFunction();

    // Build B-splines that interpolate the samples
    BSpline bspline1 = BSpline::Builder(samples).degree(1).build();
    BSpline bspline2 = BSpline::Builder(samples).degree(2).build();
    BSpline bspline3 = BSpline::Builder(samples).degree(3).build();

    BSpline bspline1_copy(bspline1);
    BSpline bspline2_copy(bspline2);
    BSpline bspline3_copy(bspline3);

    bspline1.insertKnots(0.83, 0);
    bspline1.insertKnots(1.37, 1);
    bspline2.insertKnots(0.83, 1);
    bspline2.insertKnots(1.37, 0);
    bspline3.insertKnots(0.83, 1);
    bspline3.insertKnots(1.37, 1);

    // Sample function
    auto x0_vec = linspace(0, 2, 200);
    auto x1_vec = linspace(0, 2, 200);
    DenseVector x(2);

    for (auto x0 : x0_vec)
    {
        for (auto x1 : x1_vec)
        {
            // Sample function at x
            x(0) = x0;
            x(1) = x1;

            double y1 = bspline1.eval(x);
            double y1_copy = bspline1_copy.eval(x);

            if (!assertNear(y1, y1_copy, 1e-10))
                return false;

            double y2 = bspline2.eval(x);
            double y2_copy = bspline2_copy.eval(x);

            if (!assertNear(y2, y2_copy, 1e-10))
                return false;

            double y3 = bspline3.eval(x);
            double y3_copy = bspline3_copy.eval(x);

            if (!assertNear(y3, y3_copy, 1e-10))
                return false;
        }
    }

    return true;
}

bool domainReductionTest(BSpline &bs, const BSpline &bs_orig)
{
    if (bs.getNumVariables() != 2 || bs_orig.getNumVariables() != 2)
        return false;

    // Check for error
    if (!compareBSplines(bs, bs_orig))
        return false;

    auto lb = bs.getDomainLowerBound();
    auto ub = bs.getDomainUpperBound();

    bool flag = false;
    unsigned int index = 0;
    for (; index < lb.size(); index++)
    {
        if (ub.at(index)-lb.at(index) > 1e-1)
        {
            flag = true;
            break;
        }
    }

    if (flag)
    {
        auto split = (ub.at(index) + lb.at(index))/2;

        auto lb2 = lb;
        auto ub2 = ub; ub2.at(index) = split;
        BSpline bs2(bs);
        bs2.reduceSupport(lb2, ub2);

        auto lb3 = lb; lb3.at(index) = split;
        auto ub3 = ub;
        BSpline bs3(bs);
        bs3.reduceSupport(lb3, ub3);

        return (domainReductionTest(bs2, bs_orig) && domainReductionTest(bs3, bs_orig));
    }

    return true;
}

bool runRecursiveDomainReductionTest()
{
    // Create new DataTable to manage samples
    DataTable samples = sampleTestFunction();

    // Build B-splines that interpolate the samples
    BSpline bspline1 = BSpline::Builder(samples).degree(1).build();
    BSpline bspline2 = BSpline::Builder(samples).degree(2).build();
    BSpline bspline3 = BSpline::Builder(samples).degree(3).build();
    BSpline bspline4 = BSpline::Builder(samples).degree(4).build();

    if (!domainReductionTest(bspline1, bspline1))
        return false;
    if (!domainReductionTest(bspline2, bspline2))
        return false;
    if (!domainReductionTest(bspline3, bspline3))
        return false;
    if (!domainReductionTest(bspline4, bspline4))
        return false;

    return true;
}

//double kroneckerTestFunction(DenseVector x)
//{
////    assert(x.rows() == dim);
//
//    double y = 1 + (.1 + 0.5*x(0) - x(0)*x(0) - 0.33*x(0)*x(0)*x(0))*(.1 + 0.5*x(1) - x(1)*x(1) - 2*x(1)*x(1)*x(1))*(.1 - 0.5*x(2) + x(2)*x(2) + 2*x(2)*x(2)*x(2))*(.1 - 0.5*x(3) + x(3)*x(3) + 2*x(3)*x(3)*x(3));
//    return y;
//}
//
///*
// * This example was used to test the speed difference
// * of Eigens Kronecker product and the custom-made
// * myKroneckerProduct() when refining a B-spline.
// * Note that the Kronecker product function call
// * have to be manually switched between test runs.
// */
//void kroneckerTest()
//{
//    std::vector<double> lb = {-1,-1,-1,-1};
//    std::vector<double> ub = {2,2,2,2};
//
//    DataTable table;
//    int n = 6;
//    for (int i0 = 0; i0 < n; i0++)
//    {
//        for (int i1 = 0; i1 < n; i1++)
//        {
//            for (int i2 = 0; i2 < n; i2++)
//            {
//                for (int i3 = 0; i3 < n; i3++)
//                {
//                    double x0 = lb.at(0) + i0*(ub.at(0)-lb.at(0))/(n-1);
//                    double x1 = lb.at(1) + i1*(ub.at(1)-lb.at(1))/(n-1);
//                    double x2 = lb.at(2) + i2*(ub.at(2)-lb.at(2))/(n-1);
//                    double x3 = lb.at(3) + i3*(ub.at(3)-lb.at(3))/(n-1);
//
//                    DenseVector xv(4);
//                    xv(0) = x0;
//                    xv(1) = x1;
//                    xv(2) = x2;
//                    xv(3) = x3;
//
//                    double y = kroneckerTestFunction(xv);
//                    table.addSample(xv,y);
//                }
//            }
//        }
//    }
//
//    cout << "Creating large 4-D B-spline" << endl;
//    BSpline bs(table, BSplineType::CUBIC);
//    BSpline bs2(bs);
//
//    //Timer timer;
//
//    std::vector<double> lb2 = {-.5,-.5,-.5,-.5};
//    std::vector<double> ub2 = {1,1,1,1};
//
//    cout << "Reducing domain" << endl;
//    //timer.start();
//    bs.reduceSupport(lb2,ub2);
//    //timer.stop();
//    // Old insertion method: 538, 536
//    // New insertion method: > 25000!
//    //cout << "Time (ms): " << timer.getMilliSeconds() << endl;
//
//    cout << "Doing error check!" << endl;
//    for (double i0 = lb2.at(0); i0 <= ub2.at(0); i0 += 0.1)
//    {
//        for (double i1 = lb2.at(1); i1 <= ub2.at(1); i1 += 0.1)
//        {
//            for (double i2 = lb2.at(2); i2 <= ub2.at(2); i2 += 0.1)
//            {
//                for (double i3 = lb2.at(3); i3 <= ub2.at(3); i3 += 0.1)
//                {
//                    DenseVector x(4);
//                    x(0) = i0;
//                    x(1) = i1;
//                    x(2) = i2;
//                    x(3) = i3;
//
//                    auto y1 = bs.eval(x);
//                    auto y2 = bs2.eval(x);
//
//                    auto dy = y1-y2;
//                    //dymax(0) += 1e-12;
//
//                    double eps = std::numeric_limits<double>::epsilon();
//                    eps = 1e-14;
//                    if (dy > eps || dy < -eps)
//                    {
//                        cout << "Error detected: " << dy << endl;
//                        exit(1);
//                    }
//
//                }
//            }
//        }
//    }
//    cout << "Error check ran successfully!" << endl;
//
//    // Check knot vectors after domain reduction
////    std::vector< std::vector<double> > knots2 = bs2.getKnotVectors();
////    std::vector< std::vector<double> > knots = bs.getKnotVectors();
//
////    cout << "Original knots: " << endl;
////    cout << "---------------------" << endl;
////    printVector(knots2);
//
////    cout << "New knots: " << endl;
////    cout << "---------------------" << endl;
////    printVector(knots);
//
//}
//
//void localRefinementTest()
//{
////    DenseMatrix coeffs = DenseMatrix::Ones(1,4);
////    std::vector<std::vector<double>> knots = {{1,1,2,3,4,4}};
//
//    DenseMatrix coeffs = DenseMatrix::Ones(1,2);
//    std::vector<std::vector<double>> knots = {{1,1,1.000000001,1.000000001}};
//
//    std::vector<unsigned int> degs = {1};
//    BSpline bs(coeffs, knots, degs);
//
//    /*
//     * x = 1 => knot inserted at 1.5
//     * x = 2 => knot inserted at 1.5
//     * x = 2.0001 => knot inserted at 2.5
//     * x = 2.01 => knot inserted at 2.001
//     * x = 4 => knot inserted at 3.5
//     */
//    DenseVector x(1); x(0) = 1;
//    bs.localKnotRefinement(x);
//
//    auto knots2 = bs.getKnotVectors();
//
//    for (auto k : knots2.at(0))
//        cout << k << ", ";
//}
//

} // namespace SPLINTER