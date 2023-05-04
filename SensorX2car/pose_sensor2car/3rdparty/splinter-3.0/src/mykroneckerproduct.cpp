/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "mykroneckerproduct.h"
#include "unsupported/Eigen/KroneckerProduct"

namespace SPLINTER
{

/* 
 * Implementation of Kronecker product. 
 * Eigen has an implementation of the Kronecker product,
 * but it is very slow due to poor memory reservation.
 * See: https://forum.kde.org/viewtopic.php?f=74&t=106955&p=309990&hilit=kronecker#p309990
 * When Eigen update their implementation, and officially support it, we switch to that.
 */
SparseMatrix myKroneckerProduct(const SparseMatrix &A, const SparseMatrix &B)
{
    SparseMatrix AB(A.rows()*B.rows(), A.cols()*B.cols());

    // Reserve memory for AB

    //AB.reserve(A.nonZeros()*B.nonZeros()); // Does not reserve inner vectors (slow)
    //int innernnz = std::ceil(A.nonZeros()*B.nonZeros()/AB.outerSize());
    //AB.reserve(Eigen::VectorXi::Constant(AB.outerSize(), innernnz)); // Assumes equal distribution of non-zeros (slow)

    // Calculate exact number of non-zeros for each inner vector
    Eigen::VectorXi nnzA = Eigen::VectorXi::Zero(A.outerSize());
    Eigen::VectorXi nnzB = Eigen::VectorXi::Zero(B.outerSize());
    Eigen::VectorXi nnzAB = Eigen::VectorXi::Zero(AB.outerSize());
    //innerNonZeros.setZero();

    for (int jA = 0; jA < A.outerSize(); ++jA)
    {
        int nnz = 0;
        for (SparseMatrix::InnerIterator itA(A,jA); itA; ++itA) nnz++;
        nnzA(jA) = nnz;
    }

    for (int jB = 0; jB < B.outerSize(); ++jB)
    {
        int nnz = 0;
        for (SparseMatrix::InnerIterator itB(B,jB); itB; ++itB) nnz++;
        nnzB(jB) = nnz;
    }

    int innz = 0;
    for (int i = 0; i < nnzA.rows(); ++i)
    {
        for (int j = 0; j < nnzB.rows(); ++j)
        {
            nnzAB(innz) = nnzA(i)*nnzB(j);
            innz++;
        }
    }

    AB.reserve(nnzAB);

    // Non-zero tolerance
    double tolerance = std::numeric_limits<SparseMatrix::Scalar>::epsilon();

    // Compute Kronecker product
    for (int jA = 0; jA < A.outerSize(); ++jA)
    {
        for (SparseMatrix::InnerIterator itA(A,jA); itA; ++itA)
        {
            if (std::abs(itA.value()) > tolerance)
            {
                int jrow = itA.row()*B.rows();
                int jcol = itA.col()*B.cols();

                for (int jB = 0; jB < B.outerSize(); ++jB)
                {
                    for (SparseMatrix::InnerIterator itB(B,jB); itB; ++itB)
                    {
                        if (std::abs(itA.value()*itB.value()) > tolerance)
                        {
                            int row = jrow + itB.row();
                            int col = jcol + itB.col();
                            AB.insert(row,col) = itA.value()*itB.value();
                        }
                    }
                }
            }
        }
    }
    AB.makeCompressed();
    return AB;
}

SparseVector kroneckerProductVectors(const std::vector<SparseVector> &vectors)
{
    // Create two temp matrices
    SparseMatrix temp1(1,1);
    temp1.insert(0,0) = 1;
    SparseMatrix temp2 = temp1;

    // Multiply from left
    int counter = 0;
    for (const auto &vec : vectors)
    {
        // Avoid copy
        if (counter % 2 == 0)
            temp1 = kroneckerProduct(temp2, vec);
        else
            temp2 = kroneckerProduct(temp1, vec);

        ++counter;
    }

    // Return correct product
    if (counter % 2 == 0)
        return temp2;
    return temp1;
}

DenseVector kroneckerProductVectors(const std::vector<DenseVector> &vectors)
{
    // Create two temp matrices
    DenseVector temp1(1);
    temp1(0) = 1;
    DenseVector temp2 = temp1;

    // Multiply from left
    int counter = 0;
    for (const auto &vec : vectors)
    {
        // Avoid copy
        if (counter % 2 == 0)
            temp1 = kroneckerProduct(temp2, vec);
        else
            temp2 = kroneckerProduct(temp1, vec);

        ++counter;
    }

    // Return correct product
    if (counter % 2 == 0)
        return temp2;
    return temp1;
}

SparseMatrix kroneckerProductMatrices(const std::vector<SparseMatrix> &matrices)
{
    // Create two temp matrices
    SparseMatrix temp1(1,1);
    temp1.insert(0,0) = 1;
    SparseMatrix temp2 = temp1;

    // Multiply from left
    int counter = 0;
    for (const auto &mat : matrices)
    {
        // Avoid copy
        if (counter % 2 == 0)
            temp1 = kroneckerProduct(temp2, mat);
        else
            temp2 = kroneckerProduct(temp1, mat);

        ++counter;
    }

    // Return correct product
    if (counter % 2 == 0)
        return temp2;
    return temp1;
}

} // namespace SPLINTER
