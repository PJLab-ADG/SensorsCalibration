/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_UTILITIES_H
#define SPLINTER_UTILITIES_H

#include "datatable.h"
#include "function.h"
#include "cinterface.h"
#include "bspline.h"

namespace SPLINTER
{

// Declare the global variables for use in all source files
// All extern variables are defined in cinterface/utilities.cpp
// Keep a list of objects so we avoid performing operations on objects that don't exist
extern std::set<splinter_obj_ptr> dataTables;
extern std::set<splinter_obj_ptr> bsplines;
extern std::set<splinter_obj_ptr> bspline_builders;

extern int splinter_last_func_call_error; // Tracks the success of the last function call
extern const char *splinter_error_string; // Error string (if the last function call resulted in an error)

void set_error_string(const char *new_error_string);

/* Check for existence of datatable_ptr, then cast splinter_obj_ptr to a DataTable * */
DataTable *get_datatable(splinter_obj_ptr datatable_ptr);

/* Check for existence of bspline_ptr, then cast splinter_obj_ptr to a BSpline * */
BSpline *get_bspline(splinter_obj_ptr bspline_ptr);

/* Check for existence of bspline_builder_ptr, then cast splinter_obj_ptr to a BSpline::Builder * */
BSpline::Builder *get_builder(splinter_obj_ptr bspline_builder_ptr);

/**
 * Convert from column major to row major with point_dim number of columns.
 *
 * @param col_major Column major data
 * @param point_dim Dimension of each point (= number of columns)
 * @return col_major data stored row major.
 */
double *get_row_major(double *col_major, size_t point_dim, size_t x_len);

/**
 * Convert from standard C array to DenseVector.
 *
 * @param x C array to convert from.
 * @param x_dim The size of x.
 * @return DenseVector with the same data as x.
 */
template <class NUMERICAL_TYPE>
DenseVector get_densevector(NUMERICAL_TYPE *x, size_t x_dim)
{
    DenseVector xvec(x_dim);
    for (size_t i = 0; i < x_dim; i++)
    {
        xvec(i) = (double) x[i];
    }

    return xvec;
}

/**
 * Convert from DenseVector to a vector of NUMERICAL_TYPE.
 * It must be possible to cast from double to NUMERICAL_TYPE.
 *
 * @param x DenseVector to convert from.
 * @return Vector with the same data as x.
 */
template <class NUMERICAL_TYPE>
std::vector<NUMERICAL_TYPE> get_vector(DenseVector x)
{
    auto vector = std::vector<NUMERICAL_TYPE>(x.size());
    for (size_t i = 0; i < x.size(); ++i)
    {
        vector.at(i) = (NUMERICAL_TYPE) x(i);
    }

    return vector;
}

/**
 * Convert from pointer to NUMERICAL_TYPE to std::vector<NUMERICAL_TYPE>
 *
 * @param array Pointer to NUMERICAL_TYPE
 * @param n Number of elements to copy
 * @return std::vector<NUMERICAL_TYPE> with the same elements as in array
 */
template <typename NUMERICAL_TYPE>
std::vector<NUMERICAL_TYPE> get_vector(NUMERICAL_TYPE *array, int n)
{
    auto as_vec = std::vector<NUMERICAL_TYPE>(n);

    for (int i = 0; i < n; ++i)
    {
        as_vec.at(i) = array[i];
    }

    return as_vec;
}

} // namespace SPLINTER

#endif // SPLINTER_UTILITIES_H
