/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "utilities.h"

namespace SPLINTER
{

std::vector<double> denseVectorToVector(const DenseVector &denseVec)
{
    std::vector<double> vec(denseVec.size());

    for(size_t i = 0; i < (size_t) denseVec.size(); ++i)
    {
        vec.at(i) = denseVec(i);
    }

    return vec;
}

DenseVector vectorToDenseVector(const std::vector<double> &vec)
{
    DenseVector denseVec(vec.size());

    for(size_t i = 0; i < vec.size(); ++i)
    {
        denseVec(i) = vec.at(i);
    }

    return denseVec;
}

std::vector<std::vector<double>> denseMatrixToVectorVector(const DenseMatrix &mat)
{
    std::vector<std::vector<double>> vec(mat.rows());

    for(size_t i = 0; i < (size_t) mat.rows(); ++i)
    {
        for(size_t j = 0; j < (size_t) mat.cols(); ++j)
        {
            vec.at(i).push_back(mat(i, j));
        }
    }

    return vec;
}

DenseMatrix vectorVectorToDenseMatrix(const std::vector<std::vector<double>> &vec)
{
    size_t numRows = vec.size();
    size_t numCols = numRows > 0 ? vec.at(0).size() : 0;

    DenseMatrix mat(numRows, numCols);

    for(size_t i = 0; i < numRows; ++i)
    {
        for(size_t j = 0; j < numCols; ++j)
        {
            mat(i, j) = vec.at(i).at(j);
        }
    }

    return mat;
}

std::vector<double> linspace(double start, double stop, unsigned int num)
{
    std::vector<double> ret;
    double dx = 0;
    if (num > 1)
        dx = (stop - start)/(num-1);
    for (unsigned int i = 0; i < num; ++i)
        ret.push_back(start + i*dx);
    return ret;
}

} // namespace SPLINTER
