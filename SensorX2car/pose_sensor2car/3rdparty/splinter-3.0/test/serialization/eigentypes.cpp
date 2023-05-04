/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Catch.h>
#include <serializer.h>
#include <operatoroverloads.h>
#include <definitions.h>

using namespace SPLINTER;


#define COMMON_TAGS "[serialization][eigen]"


TEST_CASE("DenseMatrix are serialized and deserialized correctly", COMMON_TAGS "[densematrix]")
{
    size_t numRows = 200, numCols = 333;

    DenseMatrix denseMatrix = 1000 * DenseMatrix::Random(numRows, numCols);
    DenseMatrix loadedDenseMatrix;
    const char *fileName = "test.densematrix";

    Serializer serializer;
    serializer.serialize(denseMatrix);
    serializer.saveToFile(fileName);

    Serializer deserializer(fileName);
    deserializer.deserialize(loadedDenseMatrix);

    REQUIRE(loadedDenseMatrix == denseMatrix);

    // Verify that the size of the serialized representation is of the expected size
    REQUIRE(Serializer::get_size(denseMatrix) == (numRows * numCols * sizeof(denseMatrix(0, 0)) + 2 * sizeof(denseMatrix.rows())));

    remove(fileName);
}

TEST_CASE("DenseVector are serialized and deserialized correctly", COMMON_TAGS "[densevector]")
{
    size_t numRows = 500;
    DenseVector denseVector = 1000 * DenseVector::Random(numRows);
    DenseVector loadedDenseVector;
    const char *fileName = "test.densevector";

    Serializer serializer;
    serializer.serialize(denseVector);
    serializer.saveToFile(fileName);

    Serializer deserializer(fileName);
    deserializer.deserialize(loadedDenseVector);

    REQUIRE(loadedDenseVector == denseVector);

    // Verify that the size of the serialized representation is of the expected size
    REQUIRE(Serializer::get_size(denseVector) == (numRows * sizeof(denseVector(0)) + sizeof(denseVector.rows())));

    remove(fileName);
}

TEST_CASE("SparseMatrix are serialized and deserialized correctly", COMMON_TAGS "[sparsematrix]")
{
    size_t numRows = 200, numCols = 333;

    SparseMatrix sparseMatrix = (1000 * DenseMatrix::Random(numRows, numCols)).sparseView();
    SparseMatrix loadedSparseMatrix;
    const char *fileName = "test.sparsematrix";

    Serializer serializer;
    serializer.serialize(sparseMatrix);
    serializer.saveToFile(fileName);

    Serializer deserializer(fileName);
    deserializer.deserialize(loadedSparseMatrix);

    REQUIRE(loadedSparseMatrix == sparseMatrix);

    // Check that the serialized representation of the SparseMatrix is of the expected size
    // Note: SparseMatrix is first converted to a DenseMatrix before serialization
    // SparseMatrix::rows() and DenseMatrix::rows() does not seem to return the same type, therefore we must use DenseMatrix::rows here.
    REQUIRE(Serializer::get_size(sparseMatrix) == (numRows * numCols * sizeof(sparseMatrix.coeff(0, 0)) + 2 * sizeof(DenseMatrix(sparseMatrix).rows())));

    remove(fileName);
}

TEST_CASE("SparseVector are serialized and deserialized correctly", COMMON_TAGS "[sparsevector]")
{
    size_t numRows = 500;

    SparseVector sparseVector = (1000 * DenseVector::Random(numRows)).sparseView();
    SparseVector loadedSparseVector;
    const char *fileName = "test.sparsevector";

    Serializer serializer;
    serializer.serialize(sparseVector);
    serializer.saveToFile(fileName);

    Serializer deserializer(fileName);
    deserializer.deserialize(loadedSparseVector);

    REQUIRE(loadedSparseVector == sparseVector);

    // Check that the serialized representation of the SparseVector is of the expected size
    // Note: SparseVector is first converted to a DenseVector before serialization
    // SparseVector::rows() and DenseVector::rows() does not seem to return the same type, therefore we must use DenseVector::rows here.
    REQUIRE(Serializer::get_size(sparseVector) == (numRows * sizeof(sparseVector.coeff(0)) + sizeof(DenseVector(sparseVector).rows())));

    remove(fileName);
}
