/*
    JSON object parser implementation
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

#include "object_parser.h"

namespace srrg_boss {

struct _json_object_parser_impl;

class JSONObjectParser: virtual public ObjectParser {
public:
  JSONObjectParser();
  virtual ObjectData* readObject(std::istream& is, std::string& type);
  virtual ObjectData* readObject(const std::string& s, std::string& type);
  virtual ~JSONObjectParser();
protected:
  _json_object_parser_impl* _parser_impl;
};

}
