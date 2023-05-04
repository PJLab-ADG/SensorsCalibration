/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_MYKRONECKERPRODUCT_H
#define SPLINTER_MYKRONECKERPRODUCT_H

#include "definitions.h"

namespace SPLINTER
{

SparseMatrix myKroneckerProduct(const SparseMatrix &A, const SparseMatrix &B);

// Apply Kronecker product on several vectors or matrices
SparseVector kroneckerProductVectors(const std::vector<SparseVector> &vectors);
DenseVector kroneckerProductVectors(const std::vector<DenseVector> &vectors);
SparseMatrix kroneckerProductMatrices(const std::vector<SparseMatrix> &matrices);

} // namespace SPLINTER

#endif // SPLINTER_MYKRONECKERPRODUCT_H
