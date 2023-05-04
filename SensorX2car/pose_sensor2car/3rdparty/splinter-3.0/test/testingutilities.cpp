/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "testingutilities.h"
#include <utilities.h>
#include <testfunctions.h>
#include <Catch.h>
#include <iostream>
#include <bsplinebuilder.h>

using namespace std;

namespace SPLINTER
{

// See https://en.wikipedia.org/wiki/Relative_change_and_difference#Formulae
double getError(double exactVal, double approxVal)
{
    double maxAbsVal = std::max(std::abs(exactVal), std::abs(approxVal));

    // Both are ~0
    if(maxAbsVal < 1e-14)
    {
        return 0.0;
    }

    double absError = std::abs(exactVal - approxVal);

    return std::min(absError, absError / maxAbsVal);
}

// Checks if a is within margin of b
bool equalsWithinRange(double a, double b, double margin)
{
    return b - margin <= a && a <= b + margin;
}

bool compareFunctions(const Function &f1, const Function &f2)
{
    int dim = f1.getNumVariables();
    auto start = std::vector<double>(dim);
    auto end = std::vector<double>(dim);
    auto numPoints = std::vector<unsigned int>(dim);

    // Try to avoid testing at "nice" values,
    // as that is likely where the function was sampled.
    for(int i = 0; i < dim; i++) {
        start.at(i) = -10.0;
        end.at(i) = 10.0;
        numPoints.at(i) = 10;
    }

    auto points = linspace(start, end, numPoints);

    return compareFunctions(f1, f2, points);
}

bool compareFunctions(const Function &exact, const Function &approx, const std::vector<std::vector<double>> &points)
{
    // Max value of the norms of function/jacobian/hessian errors
    const double one_norm_epsilon = 0.1;
    const double two_norm_epsilon = 0.1;
    const double inf_norm_epsilon = 0.1;

    return compareFunctions(exact, approx, points, one_norm_epsilon, two_norm_epsilon, inf_norm_epsilon);
}

bool compareFunctions(const Function &exact, const Function &approx, const std::vector<std::vector<double>> &points, double one_norm_epsilon, double two_norm_epsilon, double inf_norm_epsilon)
{
    bool equal = true;

    REQUIRE(exact.getNumVariables() == approx.getNumVariables());

    DenseVector normOneValVec(points.size());
    DenseVector normTwoValVec(points.size());
    DenseVector normInfValVec(points.size());

    DenseVector normOneJacVec(points.size());
    DenseVector normTwoJacVec(points.size());
    DenseVector normInfJacVec(points.size());

    DenseVector normOneHesVec(points.size());
    DenseVector normTwoHesVec(points.size());
    DenseVector normInfHesVec(points.size());

    int i = 0;
    for (auto &point : points) {
        DenseVector x = vecToDense(point);

//        INFO("Evaluation point: " << pretty_print(x));

        /*SECTION("Function approximates the value within tolerance")*/
        {
            DenseMatrix exactValue(1,1);
            exactValue(0,0) = exact.eval(x);
            DenseMatrix approxValue(1,1);
            approxValue(0,0) = approx.eval(x);
            DenseMatrix error = exactValue - approxValue;

//            INFO("Exact value:");
//            INFO(exactValue);
//            INFO("Approximated value:");
//            INFO(approxValue);
//            INFO("Exact - approx:");
//            INFO(error);

            normOneValVec(i) = getOneNorm(error);
            normTwoValVec(i) = getTwoNorm(error);
            normInfValVec(i) = getInfNorm(error);

//            REQUIRE(oneNorm(error) <= one_norm_epsilon);
//            REQUIRE(twoNorm(error) <= two_norm_epsilon);
//            REQUIRE(maxNorm(error) <= inf_norm_epsilon);
        }


        /*SECTION("Function approximates the Jacobian within tolerance")*/
        {
            auto exactJacobian = exact.evalJacobian(x);
            auto approxJacobian = approx.evalJacobian(x);
            auto errorJacobian = exactJacobian - approxJacobian;

            normOneJacVec(i) = getOneNorm(errorJacobian);
            normTwoJacVec(i) = getTwoNorm(errorJacobian);
            normInfJacVec(i) = getInfNorm(errorJacobian);
//            INFO("Exact Jacobian:");
//            INFO(exactJacobian);
//            INFO("Approximated Jacobian:");
//            INFO(approxJacobian);
//            INFO("Exact - Approx: ");
//            INFO(errorJacobian);
//
//            REQUIRE(oneNorm(errorJacobian) <= one_norm_epsilon);
//            REQUIRE(twoNorm(errorJacobian) <= two_norm_epsilon);
//            REQUIRE(maxNorm(errorJacobian) <= inf_norm_epsilon);
        }


        /*SECTION("Function approximates the Hessian within tolerance")*/
        {
            auto exactHessian = exact.evalHessian(x);
            auto approxHessian = approx.evalHessian(x);
            auto errorHessian = exactHessian - approxHessian;

            normOneHesVec(i) = getOneNorm(errorHessian);
            normTwoHesVec(i) = getTwoNorm(errorHessian);
            normInfHesVec(i) = getInfNorm(errorHessian);

//            INFO("x: ");
//            INFO(x);
//            INFO("Exact Hessian:");
//            INFO(exactHessian);
//            INFO("Approximated Hessian:");
//            INFO(approxHessian);
//            INFO("Exact - Approx: ");
//            INFO(errorHessian);

//            CHECK(getOneNorm(errorHessian) <= one_norm_epsilon);
//            CHECK(getTwoNorm(errorHessian) <= two_norm_epsilon);
//            CHECK(getInfNorm(errorHessian) <= inf_norm_epsilon);
        }

        i++;
    }

    DenseVector valNorms(3);
    valNorms(0) = getOneNorm(normOneValVec);
    valNorms(1) = getTwoNorm(normTwoValVec);
    valNorms(2) = getInfNorm(normInfValVec);

    DenseVector jacNorms(3);
    jacNorms(0) = getOneNorm(normOneJacVec);
    jacNorms(1) = getTwoNorm(normTwoJacVec);
    jacNorms(2) = getInfNorm(normInfJacVec);

    DenseVector hesNorms(3);
    hesNorms(0) = getOneNorm(normOneHesVec);
    hesNorms(1) = getTwoNorm(normTwoHesVec);
    hesNorms(2) = getInfNorm(normInfHesVec);

    if(valNorms(0) / points.size() > one_norm_epsilon) {
        INFO("1-norm function value (\"avg\"): " << valNorms(0) / points.size());
        equal = false;
    }
    if(valNorms(1) > two_norm_epsilon) {
        INFO("2-norm function value: " << valNorms(1));
        equal = false;
    }
    if(valNorms(2) > inf_norm_epsilon) {
        INFO("inf-norm function value: " << valNorms(2));
        equal = false;
    }

    if(jacNorms(0) / points.size() > one_norm_epsilon) {
        INFO("1-norm jacobian value (\"avg\"): " << jacNorms(0) / points.size());
        equal = false;
    }
    if(jacNorms(1) > two_norm_epsilon) {
        INFO("2-norm jacobian value: " << jacNorms(1));
        equal = false;
    }
    if(jacNorms(2) > inf_norm_epsilon) {
        INFO("inf-norm jacobian value: " << jacNorms(2));
        equal = false;
    }

    if(hesNorms(0) / points.size() > one_norm_epsilon) {
        INFO("1-norm hessian value (\"avg\"): " << hesNorms(0) / points.size());
        equal = false;
    }
    if(hesNorms(1) > two_norm_epsilon) {
        INFO("2-norm hessian value: " << hesNorms(1));
        equal = false;
    }
    if(hesNorms(2) > inf_norm_epsilon) {
        INFO("inf-norm hessian value: " << hesNorms(2));
        equal = false;
    }

    return equal;
}

void compareFunctionValue(std::vector<TestFunction *> funcs,
                          std::function<Function *(const DataTable &table)> approx_gen_func,
                          size_t numSamplePoints, size_t numEvalPoints,
                          double one_eps, double two_eps, double inf_eps)
{
    for(auto &exact : funcs)
    {
        compareFunctionValue(exact, approx_gen_func, numSamplePoints, numEvalPoints, one_eps, two_eps, inf_eps);
    }
}
void compareFunctionValue(TestFunction *exact,
                          std::function<Function *(const DataTable &table)> approx_gen_func,
                          size_t numSamplePoints, size_t numEvalPoints,
                          double one_eps, double two_eps, double inf_eps)
{
    auto dim = exact->getNumVariables();

    auto samplePoints = linspace(dim, -5, 5, std::pow(numSamplePoints, 1.0/dim));
    auto evalPoints = linspace(dim, -5, 5, std::pow(numEvalPoints, 1.0/dim));

    DataTable table = sample(exact, samplePoints);

    Function *approx = approx_gen_func(table);

    INFO("Approximant: " << approx->getDescription());
    INFO("Function: " << exact->getFunctionStr());

    DenseVector errorVec(evalPoints.size());

    double maxError = 0.0;
    DenseVector maxErrorPoint = vecToDense(evalPoints.at(0));

    int i = 0;
    for (auto &point : evalPoints)
    {
        DenseVector x = vecToDense(point);

        double exactValue = exact->eval(x);
        double approxValue = approx->eval(x);
        double error = getError(exactValue, approxValue);

        if(error > maxError)
        {
            maxError = error;
            maxErrorPoint = x;
        }

        errorVec(i) = error;

        i++;
    }

    DenseVector norms(3);
    norms(0) = getOneNorm(errorVec);
    norms(1) = getTwoNorm(errorVec);
    norms(2) = getInfNorm(errorVec);

    INFO(std::setw(16) << std::left << "1-norm (\"avg\"):" << std::setw(16) << std::right << norms(0) / evalPoints.size() << " <= " << one_eps);
    INFO(std::setw(16) << std::left << "2-norm:"           << std::setw(16) << std::right << norms(1) << " <= " << two_eps);
    INFO(std::setw(16) << std::left << "Inf-norm:"         << std::setw(16) << std::right << norms(2) << " <= " << inf_eps);


    // Print out the point with the largest error
    std::string maxErrorPointStr("(");
    for(size_t i = 0; i < (size_t) maxErrorPoint.size(); ++i)
    {
        if(i != 0)
        {
            maxErrorPointStr.append(", ");
        }
        maxErrorPointStr.append(std::to_string(maxErrorPoint(i)));
    }
    maxErrorPointStr.append(")");
    INFO("");
    INFO(std::setw(16) << std::left << "Max error:"        << std::setw(16) << std::right << maxError);
    INFO(" at " << maxErrorPointStr);
    INFO(std::setw(16) << std::left << "Exact value:"      << std::setw(16) << std::right << exact->eval(maxErrorPoint));
    INFO(std::setw(16) << std::left << "Approx value:"     << std::setw(16) << std::right << approx->eval(maxErrorPoint));

    CHECK(norms(0) / evalPoints.size() <= one_eps);
    /*if(norms(0) / evalPoints.size() > one_eps*//* || norms(1) > two_eps || norms(2) > inf_eps*//*) {
        CHECK(false);
    }*/

    delete approx;
}

/* Compares the jacobian of the approximant to the central difference of the approximant function value */
void compareJacobianValue(TestFunction *exact,
                          std::function<Function *(const DataTable &table)> approx_gen_func,
                          size_t numSamplePoints, size_t numEvalPoints,
                          double one_eps, double two_eps, double inf_eps)
{
    auto dim = exact->getNumVariables();

    auto samplePoints = linspace(dim, -5, 5, std::pow(numSamplePoints, 1.0/dim));
    auto evalPoints = linspace(dim, -4.95, 4.95, std::pow(numEvalPoints, 1.0/dim));

    DataTable table = sample(exact, samplePoints);

    Function *approx = approx_gen_func(table);

    INFO("Approximant: " << approx->getDescription());
    INFO("Function: " << exact->getFunctionStr());

    DenseVector oneNormVec(evalPoints.size());
    DenseVector twoNormVec(evalPoints.size());
    DenseVector infNormVec(evalPoints.size());

    double maxOneNormError = 0.0;
    double maxTwoNormError = 0.0;
    double maxInfNormError = 0.0;

    DenseVector maxOneNormErrorPoint(dim);
    maxOneNormErrorPoint.fill(0.0);
    DenseVector maxTwoNormErrorPoint(dim);
    maxTwoNormErrorPoint.fill(0.0);
    DenseVector maxInfNormErrorPoint(dim);
    maxInfNormErrorPoint.fill(0.0);

    int i = 0;
    for (auto &point : evalPoints)
    {
        DenseVector x = vecToDense(point);

        // Compare the central difference to the approximated jacobian
        DenseMatrix exactValue = approx->centralDifference(x);
        DenseMatrix approxValue = approx->evalJacobian(x);

        DenseVector error = DenseVector::Zero(exactValue.cols());
        for (size_t j = 0; j < (size_t) error.size(); ++j)
        {
            error(j) = getError(exactValue(j), approxValue(j));
        }

        oneNormVec(i) = getOneNorm(error) / error.size(); // "Average"
        twoNormVec(i) = getTwoNorm(error);
        infNormVec(i) = getInfNorm(error);

        if (oneNormVec(i) > maxOneNormError)
        {
            maxOneNormError = oneNormVec(i);
            maxOneNormErrorPoint = x;
        }
        if (twoNormVec(i) > maxTwoNormError)
        {
            maxTwoNormError = twoNormVec(i);
            maxTwoNormErrorPoint = x;
        }
        if (infNormVec(i) > maxInfNormError)
        {
            maxInfNormError = infNormVec(i);
            maxInfNormErrorPoint = x;
        }

        i++;
    }

    DenseVector norms(3);
    norms(0) = getOneNorm(oneNormVec);
    norms(1) = getTwoNorm(twoNormVec);
    norms(2) = getInfNorm(infNormVec);

    INFO(std::setw(16) << std::left << "1-norm (\"avg\"):" << std::setw(16) << std::right << norms(0) / evalPoints.size() << " <= " << one_eps);
    INFO(std::setw(16) << std::left << "2-norm:"           << std::setw(16) << std::right << norms(1) << " <= " << two_eps);
    INFO(std::setw(16) << std::left << "Inf-norm:"         << std::setw(16) << std::right << norms(2) << " <= " << inf_eps);


    auto getDenseAsStrOneLine = [](const DenseMatrix &x) {
        std::string denseAsStrOneLine("(");
        for(size_t i = 0; i < (size_t) x.size(); ++i)
        {
            if(i != 0)
            {
                denseAsStrOneLine.append(", ");
            }
            denseAsStrOneLine.append(std::to_string(x(i)));
        }
        denseAsStrOneLine.append(")");
        return denseAsStrOneLine;
    };

    // Print out the points with the largest errors
    INFO("");
    INFO("Max errors:");
    INFO("");
    INFO(std::setw(16) << std::left << "1-norm:"           << std::setw(32) << std::right << maxOneNormError);
    INFO(" at " << getDenseAsStrOneLine(maxOneNormErrorPoint));
    INFO(std::setw(16) << std::left << "Approx value:"      << std::setw(32) << std::right << getDenseAsStrOneLine(approx->evalJacobian(maxOneNormErrorPoint)));
    INFO(std::setw(16) << std::left << "Central difference:"     << std::setw(32) << std::right << getDenseAsStrOneLine(approx->centralDifference(maxOneNormErrorPoint)));

    INFO("");
    INFO(std::setw(16) << std::left << "2-norm:"           << std::setw(32) << std::right << maxTwoNormError);
    INFO(" at " << getDenseAsStrOneLine(maxTwoNormErrorPoint));
    INFO(std::setw(16) << std::left << "Approx value:"      << std::setw(32) << std::right << getDenseAsStrOneLine(approx->evalJacobian(maxTwoNormErrorPoint)));
    INFO(std::setw(16) << std::left << "Central difference:"     << std::setw(32) << std::right << getDenseAsStrOneLine(approx->centralDifference(maxTwoNormErrorPoint)));

    INFO("");
    INFO(std::setw(16) << std::left << "Inf-norm:"         << std::setw(32) << std::right << maxInfNormError);
    INFO(" at " << getDenseAsStrOneLine(maxInfNormErrorPoint));
    INFO(std::setw(16) << std::left << "Approx value:"      << std::setw(32) << std::right << getDenseAsStrOneLine(approx->evalJacobian(maxInfNormErrorPoint)));
    INFO(std::setw(16) << std::left << "Central difference:"     << std::setw(32) << std::right << getDenseAsStrOneLine(approx->centralDifference(maxInfNormErrorPoint)));

    CHECK(norms(2) <= inf_eps);
    //CHECK(norms(0) / evalPoints.size() <= one_eps);
    /*if(norms(0) / evalPoints.size() > one_eps || norms(1) > two_eps || norms(2) > inf_eps) {
        CHECK(false);
    }*/

    delete approx;
}

void checkHessianSymmetry(TestFunction *exact,
                          std::function<Function *(const DataTable &table)> approx_gen_func,
                          size_t numSamplePoints, size_t numEvalPoints)
{
    auto dim = exact->getNumVariables();

    auto samplePoints = linspace(dim, -5, 5, std::pow(numSamplePoints, 1.0/dim));
    auto evalPoints = linspace(dim, -4.95, 4.95, std::pow(numEvalPoints, 1.0/dim));

    DataTable table = sample(exact, samplePoints);

    Function *approx = approx_gen_func(table);

    INFO("Approximant: " << approx->getDescription());
    INFO("Function: " << exact->getFunctionStr());

    bool allSymmetric = true;

    DenseVector x(dim);
    for (auto &point : evalPoints)
    {
        x = vecToDense(point);

        if(!isSymmetricHessian(*approx, x))
        {
            allSymmetric = false;
            break;
        }
    }

    std::string x_str;
    for(size_t i = 0; i < (size_t) x.size(); ++i)
    {
        if(i != 0)
        {
            x_str.append(", ");
        }
        x_str.append(to_string(x(i)));
    }
    INFO("Approximated hessian at " << x_str << ":");
    INFO(approx->evalHessian(x));
    CHECK(allSymmetric);

    delete approx;
}

bool compareBSplines(const BSpline &left, const BSpline &right)
{
    auto left_lb = left.getDomainLowerBound();
    auto left_ub = left.getDomainUpperBound();
    auto right_lb = right.getDomainLowerBound();
    auto right_ub = right.getDomainUpperBound();

    REQUIRE(left_lb.size() == left_ub.size());
    REQUIRE(left_ub.size() == right_lb.size());
    REQUIRE(right_lb.size() == right_ub.size());

    int dim = left_lb.size();

    auto points = linspace(dim);

    for(int i = 0; i < dim; i++) {
        REQUIRE(left_lb.at(i) == right_lb.at(i));
        REQUIRE(left_ub.at(i) == right_ub.at(i));
    }

    return compareFunctions(left, right, points);

//    auto x0_vec = linspace(lb.at(0), ub.at(0), 10);
//    auto x1_vec = linspace(lb.at(1), ub.at(1), 10);
//
//    DenseVector x(2);
//    for (auto x0 : x0_vec)
//    {
//        for (auto x1 : x1_vec)
//        {
//            x(0) = x0;
//            x(1) = x1;
//
//            double yb = bs.eval(x);
//            double yb_orig = bs_orig.eval(x);
//            if (std::abs(yb-yb_orig) > 1e-8)
//            {
//                cout << yb << endl;
//                cout << yb_orig << endl;
//                return false;
//            }
//        }
//    }
//
//    return true;
}


DataTable sample(const Function &func, std::vector<std::vector<double>> &points) {
    return sample(&func, points);
}

DataTable sample(const Function *func, std::vector<std::vector<double>> &points)
{
    DataTable table;

    for(auto &point : points) {
        DenseVector x = vecToDense(point);
        table.addSample(point, func->eval(x));
    }

    return table;
}



double sixHumpCamelBack(DenseVector x)
{
    assert(x.rows() == 2);
    return (4 - 2.1*x(0)*x(0) + (1/3.)*x(0)*x(0)*x(0)*x(0))*x(0)*x(0) + x(0)*x(1) + (-4 + 4*x(1)*x(1))*x(1)*x(1);
}

double getOneNorm(const DenseMatrix &m)
{
    return m.lpNorm<1>();
    double norm = 0.0;
    for(int i = 0; i < m.rows(); i++) {
        for(int j = 0; j < m.cols(); j++) {
            norm += std::abs(m(i, j));
        }
    }

    return norm;
}

double getTwoNorm(const DenseMatrix &m)
{
    return m.lpNorm<2>();
    double norm = 0.0;
    for(int i = 0; i < m.rows(); i++) {
        for(int j = 0; j < m.cols(); j++) {
            norm += std::pow(m(i, j), 2);
        }
    }

    norm = std::sqrt(norm);

    return norm;
}

double getInfNorm(const DenseMatrix &m)
{
    return m.lpNorm<Eigen::Infinity>();
    double norm = 0.0;
    for(int i = 0; i < m.rows(); i++) {
        for(int j = 0; j < m.cols(); j++) {
            norm = std::max(std::abs(m(i, j)), norm);
        }
    }

    return norm;
}

double log(double base, double x)
{
    return std::log(x) / std::log(base);
}

// Enumerates all permutations
std::vector<std::vector<double>> linspace(std::vector<double> start, std::vector<double> end, std::vector<unsigned int> points)
{
    assert(start.size() == end.size() && end.size() == points.size());

    size_t nDims = start.size();
    size_t numPoints = 1;
    for(size_t i = 0; i < nDims; i++) {
        numPoints *= points.at(i);
    }

#ifndef NDEBUG
    if(numPoints > 10000) {
        std::cout << "Warning: Enumerating " << numPoints << " points." << std::endl;
    }
#endif // ifndef NDEBUG

    auto result = std::vector<std::vector<double>>(numPoints);

    auto dx = std::vector<double>(nDims);
    auto it = std::vector<unsigned int>(nDims);
    auto cur = std::vector<double>(nDims);

    for(unsigned int i = 0; i < nDims; i++) {
        cur.at(i) = start.at(i);
        it.at(i) = 0;
        dx.at(i) = (end.at(i) - start.at(i)) / (points.at(i) - 1);
    }

    size_t curDim = 0;
    size_t i = 0;
    // Add the start vector
    result.at(i++) = std::vector<double>(start);
    while(true) {
        curDim = 0;
        while(curDim < nDims && it.at(curDim)+1 >= points.at(curDim)) {
            it.at(curDim) = 0;
            cur.at(curDim) = start.at(curDim) + dx.at(curDim) * it.at(curDim);
            curDim++;
        }
        // If we're unable to find a number that can be increased,
        // it means were at the highest number
        // (for 4 digits in decimal that is 9999)
        if(curDim >= nDims) {
            break;
        }

        it.at(curDim)++;
        cur.at(curDim) = start.at(curDim) + dx.at(curDim) * it.at(curDim);

        result.at(i++) = std::vector<double>(cur);
    }

    assert(i == numPoints);

    return result;
}

std::vector<std::vector<double>> linspace(int dim, double start, double end, unsigned int points)
{
    auto startVec = std::vector<double>(dim);
    auto endVec = std::vector<double>(dim);
    auto pointsVec = std::vector<unsigned int>(dim);

    for(int i = 0; i < dim; i++) {
        startVec.at(i) = start;
        endVec.at(i) = end;
        pointsVec.at(i) = points;
    }

    return linspace(startVec, endVec, pointsVec);
}

std::vector<std::vector<double>> linspace(int dim)
{
    // Total number of points to evaluate
    // Will be distributed evenly per dimension
    const int totSamples = 10000;

    // Take the dim'th root to get number of samples per dimension
    unsigned int pointsPerDim = std::pow(totSamples, 1.0/dim);

    // Clamp so that pointsPerDim >= 2
    if(pointsPerDim < 2) {
        pointsPerDim = 2;
    }

    return linspace(dim, pointsPerDim);
}

std::vector<std::vector<double>> linspace(int dim, unsigned int pointsPerDim)
{
    auto start = std::vector<double>(dim);
    auto end = std::vector<double>(dim);
    auto numPoints = std::vector<unsigned int>(dim);


    for(int i = 0; i < dim; i++) {

        start.at(i) = -10.0;
        end.at(i) = 10.0;
        numPoints.at(i) = pointsPerDim;
    }

    return linspace(start, end, numPoints);
}


std::vector<double> denseToVec(const DenseVector &dense)
{
    auto vec = std::vector<double>(dense.size());
    for(int i = 0; i < (int) dense.size(); i++) {
        vec.at(i) = dense(i);
    }

    return vec;
}

DenseVector vecToDense(const std::vector<double> &vec)
{
    DenseVector dense(vec.size());
    for(int i = 0; i < (int) vec.size(); i++) {
        dense(i) = vec.at(i);
    }

    return dense;
}

std::string pretty_print(const DenseVector &denseVec)
{
    std::string str("[");
    for(int i = 0; i < denseVec.rows(); i++) {
        str += to_string(denseVec(i));
        if(i + 1 < denseVec.rows()) {
            str += "; ";
        }
    }
    str += "]";

    return str;
}


TestFunction *getTestFunction(int numVariables, int degree)
{
    return testFunctions.at(numVariables).at(degree);
}

std::vector<TestFunction *> getTestFunctionsOfDegree(int degree)
{
    auto testFuncs = std::vector<TestFunction *>();
    for(int i = 1; i < (int) testFunctions.size(); ++i) {
        if(degree < (int) testFunctions.at(i).size()) {
            testFuncs.push_back(testFunctions.at(i).at(degree));
        }
    }
    return testFuncs;
}

std::vector<TestFunction *> getTestFunctionWithNumVariables(int numVariables)
{
    return testFunctions.at(numVariables);
}

std::vector<TestFunction *> getPolynomialFunctions()
{
    auto testFuncs = std::vector<TestFunction *>();
    for(int i = 1; i < (int) testFunctions.size(); ++i) {
        for(int j = 0; j < (int) testFunctions.at(i).size(); ++j) {
            testFuncs.push_back(testFunctions.at(i).at(j));
        }
    }
    return testFuncs;
}

std::vector<TestFunction *> getNastyTestFunctions()
{
    return testFunctions.at(0);
}

DenseMatrix getErrorNorms(const Function *exact, const Function *approx, const std::vector<std::vector<double>> &points)
{
    assert(exact->getNumVariables() == approx->getNumVariables());

    DenseVector normOneValVec(points.size());
    DenseVector normTwoValVec(points.size());
    DenseVector normInfValVec(points.size());

    DenseVector normOneJacVec(points.size());
    DenseVector normTwoJacVec(points.size());
    DenseVector normInfJacVec(points.size());

    DenseVector normOneHesVec(points.size());
    DenseVector normTwoHesVec(points.size());
    DenseVector normInfHesVec(points.size());

    int i = 0;
    for (auto &point : points) {
        DenseVector x = vecToDense(point);

        {
            DenseMatrix exactValue(1,1);
            exactValue(0,0) = exact->eval(x);
            DenseMatrix approxValue(1,1);
            approxValue(0,0) = approx->eval(x);
            DenseMatrix error = exactValue - approxValue;

            normOneValVec(i) = getOneNorm(error);
            normTwoValVec(i) = getTwoNorm(error);
            normInfValVec(i) = getInfNorm(error);
        }

        {
            auto exactJacobian = exact->evalJacobian(x);
            auto approxJacobian = approx->evalJacobian(x);
            auto errorJacobian = exactJacobian - approxJacobian;

            normOneJacVec(i) = getOneNorm(errorJacobian);
            normTwoJacVec(i) = getTwoNorm(errorJacobian);
            normInfJacVec(i) = getInfNorm(errorJacobian);
        }

        {
            auto exactHessian = exact->evalHessian(x);
            auto approxHessian = approx->evalHessian(x);
            auto errorHessian = exactHessian - approxHessian;

            normOneHesVec(i) = getOneNorm(errorHessian);
            normTwoHesVec(i) = getTwoNorm(errorHessian);
            normInfHesVec(i) = getInfNorm(errorHessian);
        }

        i++;
    }

    DenseMatrix errorNorms(3,3);
    errorNorms(0,0) = getOneNorm(normOneValVec);
    errorNorms(0,1) = getTwoNorm(normTwoValVec);
    errorNorms(0,2) = getInfNorm(normInfValVec);

    errorNorms(1,0) = getOneNorm(normOneJacVec);
    errorNorms(1,1) = getTwoNorm(normTwoJacVec);
    errorNorms(1,2) = getInfNorm(normInfJacVec);

    errorNorms(2,0) = getOneNorm(normOneHesVec);
    errorNorms(2,1) = getTwoNorm(normTwoHesVec);
    errorNorms(2,2) = getInfNorm(normInfHesVec);

    return errorNorms;
}

void checkNorms(DenseMatrix normValues, size_t numPoints, double one_eps, double two_eps, double inf_eps)
{
    checkNorm(normValues, TestType::All, numPoints, one_eps, two_eps, inf_eps);
}

void checkNorm(DenseMatrix normValues, TestType type, size_t numPoints, double one_eps, double two_eps, double inf_eps)
{
    if(type == TestType::All || type == TestType::FunctionValue) {
        INFO("Function value error:");
        _checkNorm(normValues, 0, numPoints, one_eps, two_eps, inf_eps);
    }
    if(type == TestType::All || type == TestType::Jacobian) {
        INFO("Jacobian value error:");
        _checkNorm(normValues, 1, numPoints, one_eps, two_eps, inf_eps);
    }
    if(type == TestType::All || type == TestType::Hessian) {
        INFO("Hessian value error:");
        _checkNorm(normValues, 2, numPoints, one_eps, two_eps, inf_eps);
    }
}

void _checkNorm(DenseMatrix normValues, int row, size_t numPoints, double one_eps, double two_eps, double inf_eps)
{
    bool withinThreshold =\
            normValues(row,0) / numPoints <= one_eps
         && normValues(row,1)             <= two_eps
         && normValues(row,2)             <= inf_eps;

    INFO(std::setw(16) << std::left << "1-norm (\"avg\"):" << std::setw(16) << std::right << normValues(row,0) / numPoints);
    INFO(std::setw(16) << std::left << "2-norm:"           << std::setw(16) << std::right << normValues(row,1));
    INFO(std::setw(16) << std::left << "inf-norm:"         << std::setw(16) << std::right << normValues(row,2));

    CHECK(withinThreshold);
}

// Must use std::function because a capturing alpha cannot be converted to a function pointer
void testApproximation(std::vector<TestFunction *> funcs,
                       std::function<Function *(const DataTable &table)> approx_gen_func,
                       TestType type, size_t numSamplePoints, size_t numEvalPoints,
                       double one_eps, double two_eps, double inf_eps)
{
    for(auto &exact : funcs) {

        auto dim = exact->getNumVariables();
        CHECK(dim > 0);
        if(dim > 0) {
            auto samplePoints = linspace(dim, -5, 5, std::pow(numSamplePoints, 1.0/dim));
            auto evalPoints = linspace(dim, -4.95, 4.95, std::pow(numEvalPoints, 1.0/dim));

            DataTable table = sample(exact, samplePoints);

            Function *approx = approx_gen_func(table);

            INFO("Function: " << exact->getFunctionStr());
            INFO("Approximant: " << approx->getDescription());

            DenseMatrix errorNorms = getErrorNorms(exact, approx, evalPoints);

            checkNorm(errorNorms, type, evalPoints.size(), one_eps, two_eps, inf_eps);

            delete approx;
        }
    }
}

DenseMatrix centralDifference(const Function &approx, const DenseVector &x)
{
    DenseMatrix dx(1, x.size());

    double h = 1e-6; // perturbation step size
    double hForward = 0.5*h;
    double hBackward = 0.5*h;

    for (unsigned int i = 0; i < approx.getNumVariables(); ++i)
    {
        DenseVector xForward(x);
//        if (xForward(i) + hForward > variables.at(i)->getUpperBound())
//        {
//            hForward = 0;
//        }
//        else
//        {
        xForward(i) = xForward(i) + hForward;
//        }

        DenseVector xBackward(x);
//        if (xBackward(i) - hBackward < variables.at(i)->getLowerBound())
//        {
//            hBackward = 0;
//        }
//        else
//        {
            xBackward(i) = xBackward(i) - hBackward;
//        }

        double yForward = approx.eval(xForward);
        double yBackward = approx.eval(xBackward);

        dx(i) = (yForward - yBackward)/(hBackward + hForward);
    }

    return dx;
}

/*
 * Checks that the hessian is symmetric across the diagonal
 */
bool isSymmetricHessian(const Function &approx, const DenseVector &x)
{
    DenseMatrix hessian = approx.evalHessian(x);

    for(int row = 0; row < (int) hessian.rows(); ++row)
    {
        for(int col = 0; col < (int) hessian.cols(); ++col)
        {
            if(getError(hessian(row, col), hessian(col, row)) > 1e-9)
            {
                return false;
            }
        }
    }

    return true;
}

} // namespace SPLINTER
