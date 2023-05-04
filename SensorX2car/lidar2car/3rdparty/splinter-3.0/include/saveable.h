/*
 * This file is part of the SPLINTER library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef SPLINTER_SAVEABLE_H
#define SPLINTER_SAVEABLE_H

namespace SPLINTER
{

/**
 * Interface for save and load functionality
 */
class Saveable
{
public:
    Saveable() {}
    virtual ~Saveable() {}

    /**
     * Serialize and save object to fileName
     * Should throw exception if file could not be opened
     */
    virtual void save(const std::string &fileName) const = 0;

protected:
    /**
     * Deserialize and load object from fileName
     * Should throw exception if file could not be opened
     * or if the file format is wrong
     */
    virtual void load(const std::string &fileName) = 0;
};

} // namespace SPLINTER

#endif // SPLINTER_SAVEABLE_H
