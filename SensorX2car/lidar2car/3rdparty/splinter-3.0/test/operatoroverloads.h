/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_OPERATOROVERLOADS_H
#define SPLINTER_OPERATOROVERLOADS_H

#include <definitions.h>
#include <vector>
#include <set>
#include <datapoint.h>
#include <datatable.h>
#include <bspline.h>


/*
 * Overload of operators where an Eigen type is one or more of the arguments.
 * Must be defined in the Eigen namespace for Clang to find the overloads.
 */
namespace Eigen
{

// Eigen doesn't provide operator== overloads, so we define them ourselves (in terms of their Dense counterparts)
// Note: SLOW
bool operator==(const SPLINTER::SparseMatrix &lhs, const SPLINTER::SparseMatrix &rhs);
bool operator==(const SPLINTER::SparseVector &lhs, const SPLINTER::SparseVector &rhs);
bool operator==(const std::vector<double> &vec, const SPLINTER::DenseVector &denseVec);
bool operator==(const std::vector<std::vector<double>> &vecVec, const SPLINTER::DenseMatrix &denseMat);
bool operator==(const SPLINTER::DenseVector &denseVec, const std::vector<double> &vec);
bool operator==(const SPLINTER::DenseMatrix &denseMat, const std::vector<std::vector<double>> &vecVec);

}

namespace SPLINTER
{

/*
 * Comparison operators (==)
 */
bool operator==(const DataTable &lhs, const DataTable &rhs);
bool operator==(const DataPoint &lhs, const DataPoint &rhs);
bool operator==(const BSpline &lhs, const BSpline &rhs);
bool operator==(const BSplineBasis &lhs, const BSplineBasis &rhs);
bool operator==(const BSplineBasis1D &lhs, const BSplineBasis1D &rhs);

template <class T>
bool operator==(const std::vector<T> &lhs, const std::vector<T> &rhs)
{
    auto lit = lhs.cbegin(), rit = rhs.cbegin();
    for (; lit != lhs.cend() && rit != rhs.cend(); ++lit, ++rit)
    {
        if(*lit != *rit) {
            return false;
        }
    }
    return true;
}

template <class T>
bool operator==(const std::set<T> &lhs, const std::set<T> &rhs)
{
    auto lit = lhs.cbegin(), rit = rhs.cbegin();
    for (; lit != lhs.cend() && rit != rhs.cend(); ++lit, ++rit)
    {
        if(*lit != *rit) {
            return false;
        }
    }
    return true;
}

template <class T>
bool operator==(const std::multiset<T> &lhs, const std::multiset<T> &rhs)
{
    auto lit = lhs.cbegin(), rit = rhs.cbegin();
    for (; lit != lhs.cend() && rit != rhs.cend(); ++lit, ++rit)
    {
        if(*lit != *rit) {
            return false;
        }
    }
    return true;
}

/*
 * Comparison operators (!=)
 */
bool operator!=(const BSplineBasis1D &lhs, const BSplineBasis1D &rhs);
bool operator!=(const DataPoint &lhs, const DataPoint &rhs);

/*
 * Output stream operator
 */
std::ostream &operator<<(std::ostream &out, const DataPoint &sample);
std::ostream &operator<<(std::ostream &out, const DataTable &table);
template <class T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &obj);
template <class T>
std::ostream &operator<<(std::ostream &out, const std::set<T> &obj);
template <class T>
std::ostream &operator<<(std::ostream &out, const std::multiset<T> &obj);

} // namespace SPLINTER

#endif // SPLINTER_OPERATOROVERLOADS_H
