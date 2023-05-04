/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_DATATABLE_H
#define SPLINTER_DATATABLE_H

#include <set>
#include "datapoint.h"

#include <ostream>

namespace SPLINTER
{

/*
 * DataTable is a class for storing multidimensional data samples (x,y).
 * The samples are stored in a continuously sorted table.
 */
class SPLINTER_API DataTable
{
public:
    DataTable();
    DataTable(bool allowDuplicates);
    DataTable(bool allowDuplicates, bool allowIncompleteGrid);
    DataTable(const char *fileName);
    DataTable(const std::string &fileName); // Load DataTable from file

    /*
     * Functions for adding a sample (x,y)
     */
    void addSample(const DataPoint &sample);
    void addSample(double x, double y);
    void addSample(std::vector<double> x, double y);
    void addSample(DenseVector x, double y);

    /*
     * Getters
     */
    std::multiset<DataPoint>::const_iterator cbegin() const;
    std::multiset<DataPoint>::const_iterator cend() const;

    unsigned int getNumVariables() const {return numVariables;}
    unsigned int getNumSamples() const {return samples.size();}
    const std::multiset<DataPoint>& getSamples() const {return samples;}

    std::vector<std::set<double>> getGrid() const { return grid; }
    std::vector< std::vector<double> > getTableX() const;
    std::vector<double> getVectorY() const;
    
    bool isGridComplete() const;

    void save(const std::string &fileName) const;

private:
    bool allowDuplicates;
    bool allowIncompleteGrid;
    unsigned int numDuplicates;
    unsigned int numVariables;

    std::multiset<DataPoint> samples;
    std::vector< std::set<double> > grid;

    void initDataStructures(); // Initialise grid to be a std::vector of xDim std::sets
    unsigned int getNumSamplesRequired() const;

    void recordGridPoint(const DataPoint &sample);

    // Used by functions that require the grid to be complete before they start their operation
    // This function prints a message and exits the program if the grid is not complete.
    void gridCompleteGuard() const;

    void load(const std::string &fileName);

    friend class Serializer;
    friend bool operator==(const DataTable &lhs, const DataTable &rhs);
};

DataTable operator+(const DataTable &lhs, const DataTable &rhs);
DataTable operator-(const DataTable &lhs, const DataTable &rhs);

} // namespace SPLINTER

#endif // SPLINTER_DATATABLE_H
