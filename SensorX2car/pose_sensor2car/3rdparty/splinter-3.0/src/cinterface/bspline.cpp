/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <fstream>
#include "bspline.h"
#include "cinterface/utilities.h"

using namespace SPLINTER;

extern "C"
{

splinter_obj_ptr splinter_bspline_load_init(const char *filename)
{
    splinter_obj_ptr bspline = nullptr;

    try
    {
        bspline = (splinter_obj_ptr) new BSpline(filename);
        bsplines.insert(bspline);
    }
    catch(const Exception &e)
    {
        set_error_string(e.what());
    }

    return bspline;
}

int *splinter_bspline_get_knot_vector_sizes(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    int *sizes = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            auto knot_vectors = bspline->getKnotVectors();

            sizes = (int *) malloc(knot_vectors.size() * sizeof (int));

            if (sizes != nullptr)
            {
                int i = 0;
                for (auto knot_vector : knot_vectors)
                {
                    sizes[i++] = (int) knot_vector.size();
                }
            }
            else
            {
                set_error_string("Unable to allocate memory!");
            }
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return sizes;
}

double *splinter_bspline_get_knot_vectors(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    double *knot_vectors_as_array = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            auto knot_vectors = bspline->getKnotVectors();

            // Overkill, but some C++11 is nice
            int total_n_elements = 0;
            std::for_each(knot_vectors.cbegin(), knot_vectors.cend(),
                          [&total_n_elements](const std::vector<double> &knot_vector)
                          {
                              total_n_elements += knot_vector.size();
                          });
            knot_vectors_as_array = (double *) malloc(total_n_elements * sizeof (double));

            if (knot_vectors_as_array != nullptr)
            {
                int i = 0;
                for (auto knot_vector : knot_vectors)
                {
                    // Even more unnecessary C++11 stuff
                    std::copy(knot_vector.begin(), knot_vector.end(), knot_vectors_as_array + i);
                    i += knot_vector.size();
                }
            }
            else
            {
                set_error_string("Unable to allocate memory!");
            }
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return knot_vectors_as_array;
}

int splinter_bspline_get_num_coefficients(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            return bspline->getNumCoefficients();
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return -1;
}

double *splinter_bspline_get_coefficients(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    double *coefficients_as_array = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            auto coefficients = bspline->getCoefficients();

            coefficients_as_array = (double *) malloc(coefficients.size() * sizeof (double));

            if (coefficients_as_array != nullptr)
            {
                for (int i = 0; i < coefficients.size(); ++i)
                {
                    coefficients_as_array[i] = coefficients(i);
                }
            }
            else
            {
                set_error_string("Unable to allocate memory!");
            }
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return coefficients_as_array;
}

double *splinter_bspline_get_control_points(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    double *control_points_as_array = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            auto control_points = bspline->getControlPoints();

            control_points_as_array = (double *) malloc(control_points.size() * sizeof (double));

            if (control_points_as_array != nullptr)
            {
                for (int i = 0; i < control_points.rows(); ++i)
                {
                    for (int j = 0; j < control_points.cols(); ++j)
                    {
                        control_points_as_array[i*control_points.cols() + j] = control_points(i, j);
                    }
                }
            }
            else
            {
                set_error_string("Unable to allocate memory!");
            }
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return control_points_as_array;
}

int *splinter_bspline_get_basis_degrees(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    int *basis_degrees_as_array = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            auto basis_degrees = bspline->getBasisDegrees();

            basis_degrees_as_array = (int *) malloc(basis_degrees.size() * sizeof (int));

            if (basis_degrees_as_array != nullptr)
            {
                for (int i = 0; i < basis_degrees.size(); ++i)
                {
                    basis_degrees_as_array[i] = basis_degrees[i];
                }
            }
            else
            {
                set_error_string("Unable to allocate memory!");
            }
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
    return basis_degrees_as_array;
}

double *splinter_bspline_eval_row_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            size_t num_variables = bspline->getNumVariables();
            size_t num_points = x_len / num_variables;

            retVal = (double *) malloc(sizeof(double) * num_points);
            for (size_t i = 0; i < num_points; ++i)
            {
                auto xvec = get_densevector<double>(x, num_variables);
                retVal[i] = bspline->eval(xvec);
                x += num_variables;
            }
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }

    return retVal;
}

double *splinter_bspline_eval_jacobian_row_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            size_t num_variables = bspline->getNumVariables();
            size_t num_points = x_len / num_variables;

            retVal = (double *) malloc(sizeof(double) * num_variables * num_points);
            for (size_t i = 0; i < num_points; ++i)
            {
                auto xvec = get_densevector<double>(x, num_variables);
                DenseMatrix jacobian = bspline->evalJacobian(xvec);

                /* Copy jacobian from stack to heap */
                memcpy(retVal + i*num_variables, jacobian.data(), sizeof(double) * num_variables);
                x += num_variables;
            }
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }

    return retVal;
}

double *splinter_bspline_eval_hessian_row_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            size_t num_variables = bspline->getNumVariables();
            size_t num_points = x_len / num_variables;

            retVal = (double *) malloc(sizeof(double) * num_variables * num_variables * num_points);
            for (size_t i = 0; i < num_points; ++i)
            {
                auto xvec = get_densevector<double>(x, num_variables);
                DenseMatrix hessian = bspline->evalHessian(xvec);

                /* Copy hessian from stack to heap */
                memcpy(retVal + i*num_variables*num_variables, hessian.data(), sizeof(double) * num_variables * num_variables);
                x += num_variables;
            }
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }

    return retVal;
}

double *splinter_bspline_eval_col_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        double *row_major = nullptr;
        try
        {
            row_major = get_row_major(x, bspline->getNumVariables(), x_len);
            if (row_major == nullptr)
            {
                return nullptr; // Pass on the error message set by get_row_major
            }

            retVal = splinter_bspline_eval_row_major(bspline, row_major, x_len);
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
        free(row_major);
    }

    return retVal;
}

double *splinter_bspline_eval_jacobian_col_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        double *row_major = nullptr;
        try
        {
            row_major = get_row_major(x, bspline->getNumVariables(), x_len);
            if (row_major == nullptr)
            {
                return nullptr; // Pass on the error message set by get_row_major
            }

            retVal = splinter_bspline_eval_jacobian_row_major(bspline, row_major, x_len);
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
        free(row_major);
    }

    return retVal;
}

double *splinter_bspline_eval_hessian_col_major(splinter_obj_ptr bspline_ptr, double *x, int x_len)
{
    double *retVal = nullptr;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        double *row_major = nullptr;
        try
        {
            row_major = get_row_major(x, bspline->getNumVariables(), x_len);
            if (row_major == nullptr)
            {
                return nullptr; // Pass on the error message set by get_row_major
            }

            retVal = splinter_bspline_eval_hessian_row_major(bspline, row_major, x_len);
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
        free(row_major);
    }
    return retVal;
}

int splinter_bspline_get_num_variables(splinter_obj_ptr bspline_ptr)
{
    int retVal = 0;

    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        retVal = bspline->getNumVariables();
    }

    return retVal;
}

void splinter_bspline_save(splinter_obj_ptr bspline_ptr, const char *filename)
{
    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            bspline->save(filename);
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

void splinter_bspline_delete(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);

    if (bspline != nullptr)
    {
        bsplines.erase(bspline_ptr);
        delete bspline;
    }
}

void splinter_bspline_insert_knots(splinter_obj_ptr bspline_ptr, double tau, unsigned int dim, unsigned int multiplicity)
{
    auto bspline = get_bspline(bspline_ptr);
    if (bspline != nullptr)
    {
        try
        {
            bspline->insertKnots(tau, dim, multiplicity);
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

void splinter_bspline_decompose_to_bezier_form(splinter_obj_ptr bspline_ptr)
{
    auto bspline = get_bspline(bspline_ptr);
    int *sizes = nullptr;
    if (bspline != nullptr)
    {
        try
        {
            bspline->decomposeToBezierForm();
        }
        catch (const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

} // extern "C"
