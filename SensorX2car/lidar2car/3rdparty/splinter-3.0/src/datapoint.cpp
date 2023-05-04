/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "datapoint.h"

namespace SPLINTER
{

DataPoint::DataPoint()
{
}

DataPoint::DataPoint(double x, double y)
{
    setData(std::vector<double>(1, x), y);
}

DataPoint::DataPoint(std::vector<double> x, double y)
{
    setData(x, y);
}

DataPoint::DataPoint(DenseVector x, double y)
{
    std::vector<double> newX;

    for (int i = 0; i < x.size(); i++)
    {
        newX.push_back(x(i));
    }

    setData(newX, y);
}

void DataPoint::setData(const std::vector<double> &x, double y)
{
    this->x = x;
    this->y = y;
}

bool DataPoint::operator<(const DataPoint &rhs) const
{
    if (this->getDimX() != rhs.getDimX())
        throw Exception("DataPoint::operator<: Cannot compare data points of different dimensions");

    for (unsigned int i = 0; i < this->getDimX(); i++)
    {
        if (x.at(i) < rhs.getX().at(i))
            return true;
        else if (x.at(i) > rhs.getX().at(i))
            return false;
    }

    return false;
}

/*
* Computes Euclidean distance ||x-y||
*/
double dist(const std::vector<double> x, const std::vector<double> y)
{
    if (x.size() != y.size())
        throw Exception("DataPoint::dist: Cannot measure distance between two points of different dimension");
    double sum = 0.0;
    for (unsigned int i=0; i<x.size(); i++)
        sum += (x.at(i)-y.at(i))*(x.at(i)-y.at(i));
    return std::sqrt(sum);
}

/*
* Computes Euclidean distance ||x-y||
*/
double dist(const DataPoint x, const DataPoint y)
{
    return dist(x.getX(), y.getX());
}

bool dist_sort(const DataPoint x, const DataPoint y)
{
    std::vector<double> zeros(x.getDimX(), 0);
    DataPoint origin(zeros, 0.0);
    double x_dist = dist(x, origin);
    double y_dist = dist(y, origin);
    return (x_dist<y_dist);
}

} // namespace SPLINTER
