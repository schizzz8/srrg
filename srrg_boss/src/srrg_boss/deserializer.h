/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

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

#include <map>
#include <set>
#include <vector>
#include <istream>
#include <string>

#include "id_context.h"
#include "serialization_context.h"

namespace srrg_boss {
  
class Serializable;
class ValueData;
class Message;
class MessageParser;
class ObjectParser;
class MessageData;

  class Deserializer: virtual public IdContext {
public:
  Deserializer(SerializationContext* serContext = 0);

  /*
   * Set file path for main data file.
   * Binary files use this path as base directory.
   */
  void setFilePath(const std::string& fpath);
  
  /*
   * Set data file format (default is "JSON").
   */
  bool setFormat(const std::string& format);

  /*!
   * \brief Read a single message and create the related Serializable instance.
   * \details Read and parse a single message, returning a pointer to the message, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of both the returned Message and its associated Serializable instance.
   * \return the read message
   */
  Message* readMessage();

  /*!
   * \brief Read a single object and create the related Serializable instance.
   * \details Read and parse a single object, returning a pointer to the instance, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of returned pointer.
   * \return the read object
   */
  Serializable* readObject();

  /*!
   * \brief Read a single object and create the related Serializable instance.
   * \details Read and parse a single object, returning a pointer to the instance, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of returned pointer.
   * \return the read object
   */
    Serializable* readObject(const std::string& line);

  bool readObjectData(ObjectData*& objectData, std::string& type);

  virtual ~Deserializer();
protected:

  MessageParser* _parser;
  ObjectParser* _objectParser;
  
  //Map with Identifiable objects that refers to unresolved pointers
  std::map<Serializable*,std::set<int> > _waitingInstances;
  //Reverse map
  std::map<int,std::set<Serializable*> > _danglingReferences;
};

}
