/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "cinterface/cinterface.h"
#include "cinterface/utilities.h"
#include "datatable.h"
//#include <fstream>

using namespace SPLINTER;

extern "C"
{

/* DataTable constructor */
splinter_obj_ptr splinter_datatable_init()
{
    splinter_obj_ptr dataTable = (splinter_obj_ptr) new DataTable();

    dataTables.insert(dataTable);

    return dataTable;
}

splinter_obj_ptr splinter_datatable_load_init(const char *filename)
{
    splinter_obj_ptr dataTable = nullptr;

    try
    {
        dataTable = (splinter_obj_ptr) new DataTable(filename);
        dataTables.insert(dataTable);
    }
    catch(const Exception &e)
    {
        set_error_string(e.what());
    }

    return dataTable;
}

void splinter_datatable_add_samples_row_major(splinter_obj_ptr datatable_ptr, double *x, int n_samples, int x_dim)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        try
        {
//            std::ofstream out("test.txt");
            DenseVector vec(x_dim);
            for (int i = 0; i < n_samples; ++i)
            {
                int sample_start = i*(x_dim+1);
                for (int offset = 0; offset < x_dim; ++offset)
                {
                    vec(offset) = x[sample_start + offset];
                }

//                out << "Adding sample: (";
//                for(int l = 0; l < vec.size(); l++)
//                {
//                    if(l != 0)
//                        out << ",";
//                    out << vec(l);
//                }
//                out << ") = ";
//                out  << x[sample_start + x_dim] << std::endl;

                dataTable->addSample(vec, x[sample_start + x_dim]);
            }
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

void splinter_datatable_add_samples_col_major(splinter_obj_ptr datatable_ptr, double *x, int n_samples, int x_dim)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        try
        {
//            std::ofstream out("test.txt");
            DenseVector vec(x_dim);
            for (int i = 0; i < n_samples; ++i)
            {
                for (int j = 0; j < x_dim; ++j)
                {
                    vec(j) = x[i + j * n_samples];
                }

//                out << "Adding sample: (";
//                for(int l = 0; l < vec.size(); l++)
//                {
//                    if(l != 0)
//                        out << ",";
//                    out << vec(l);
//                }
//                out << ") = ";
//                out  << x[i + x_dim * n_samples] << std::endl;

                dataTable->addSample(vec, x[i + x_dim * n_samples]);
            }
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

int splinter_datatable_get_num_variables(splinter_obj_ptr datatable_ptr)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        return dataTable->getNumVariables();
    }

    return 0;
}


int splinter_datatable_get_num_samples(splinter_obj_ptr datatable_ptr)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        return dataTable->getNumSamples();
    }

    return 0;
}

void splinter_datatable_save(splinter_obj_ptr datatable_ptr, const char *filename)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        try
        {
            dataTable->save(filename);
        }
        catch(const Exception &e)
        {
            set_error_string(e.what());
        }
    }
}

void splinter_datatable_delete(splinter_obj_ptr datatable_ptr)
{
    auto dataTable = get_datatable(datatable_ptr);
    if (dataTable != nullptr)
    {
        dataTables.erase(datatable_ptr);
        delete dataTable;
    }
}

} // extern "C"
