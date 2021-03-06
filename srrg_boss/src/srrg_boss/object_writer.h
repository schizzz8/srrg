/*
    Abstract object writer interface
    Copyright (C) 2013  Daniele Baldassari <daniele@dikappa.org>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include<iostream>
#include<string>

namespace srrg_boss {

class ObjectData;

class ObjectWriter {
public:
  virtual void writeObject(std::ostream& os, const std::string& type, ObjectData& object)=0;
  virtual ~ObjectWriter() {}
};

}
