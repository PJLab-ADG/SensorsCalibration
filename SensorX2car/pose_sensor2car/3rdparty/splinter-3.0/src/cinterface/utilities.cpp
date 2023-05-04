/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <bspline.h>
#include "cinterface/utilities.h"

namespace SPLINTER
{

std::set<splinter_obj_ptr> dataTables = std::set<splinter_obj_ptr>();
std::set<splinter_obj_ptr> bsplines = std::set<splinter_obj_ptr>();
std::set<splinter_obj_ptr> bspline_builders = std::set<splinter_obj_ptr>();

// 1 if the last function call caused an error, 0 else
int splinter_last_func_call_error = 0;

const char *splinter_error_string = "No error.";

void set_error_string(const char *new_error_string)
{
    splinter_error_string = new_error_string;
    splinter_last_func_call_error = 1;
}

/* Cast the splinter_obj_ptr to a DataTable * */
DataTable *get_datatable(splinter_obj_ptr datatable_ptr)
{
    if (dataTables.count(datatable_ptr) > 0)
    {
        return static_cast<DataTable *>(datatable_ptr);
    }

    set_error_string("Invalid reference to DataTable: Maybe it has been deleted?");

    return nullptr;
}

/* Cast the splinter_obj_ptr to a BSpline * */
BSpline *get_bspline(splinter_obj_ptr bspline_ptr)
{
    if (bsplines.count(bspline_ptr) > 0)
    {
        return static_cast<BSpline *>(bspline_ptr);
    }

    set_error_string("Invalid reference to BSpline: Maybe it has been deleted?");

    return nullptr;
}

/* Check for existence of bspline_builder_ptr, then cast splinter_obj_ptr to a BSpline::Builder * */
BSpline::Builder *get_builder(splinter_obj_ptr bspline_builder_ptr)
{
    if (bspline_builders.count(bspline_builder_ptr) > 0)
    {
        return static_cast<BSpline::Builder *>(bspline_builder_ptr);
    }

    set_error_string("Invalid reference to BSpline::Builder: Maybe it has been deleted?");

    return nullptr;
}

/**
 * Convert from column major to row major with point_dim number of columns.
 *
 * @param col_major Column major data
 * @param point_dim Dimension of each point (= number of columns)
 * @return col_major data stored row major.
 */
double *get_row_major(double *col_major, size_t point_dim, size_t x_len)
{
    if (point_dim == 0)
    {
        set_error_string("Dimension of x should be larger than 0!");
        return nullptr;
    }

    double *row_major = (double *) malloc(sizeof(double) * x_len);
    if(row_major == nullptr)
    {
        set_error_string("Out of memory!");
        return nullptr;
    }

    size_t num_points = x_len / point_dim;
    for (size_t i = 0; i < x_len; ++i)
    {
        size_t row_num = i / point_dim; // Intentional integer division
        size_t col_num = i % point_dim;
        row_major[i] = col_major[col_num * num_points + row_num];
    }

    return row_major;
}

} // namespace SPLINTER
