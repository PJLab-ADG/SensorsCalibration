/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_DATAPOINT_H
#define SPLINTER_DATAPOINT_H

#include "definitions.h"

namespace SPLINTER
{

/*
 * DataPoint is a class representing a data point (x, y),
 * where y is the value obtained by sampling at a point x.
 * Note that x is a vector and y is a scalar.
 */
class DataPoint
{
public:
    DataPoint(double x, double y);
    DataPoint(std::vector<double> x, double y);
    DataPoint(DenseVector x, double y);

    bool operator<(const DataPoint &rhs) const; // Returns false if the two are equal

    std::vector<double> getX() const { return x; }
    double getY() const { return y; }
    unsigned int getDimX() const { return x.size(); }

private:
    DataPoint();

    std::vector<double> x;
    double y;
    void setData(const std::vector<double> &x, double y);

    friend class Serializer;
};

// Measure distance between two points
double dist(const std::vector<double> x, const std::vector<double> y);
double dist(const DataPoint x, const DataPoint y);
bool dist_sort(const DataPoint x, const DataPoint y);

} // namespace SPLINTER

#endif // SPLINTER_DATAPOINT_H
