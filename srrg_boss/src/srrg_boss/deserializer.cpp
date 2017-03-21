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

#include <stdexcept>
#include <memory>
#include <fstream>

#include "deserializer.h"

#include "message_data.h"
#include "object_data.h"
#include "identifiable.h"
#include "message.h"
#include "json_message_parser.h"
#include "json_object_parser.h"
#include "id_placeholder.h"

using namespace std;
using namespace srrg_boss;

static ValueData* processData(ValueData* vdata, IdContext& context, vector<int>& danglingRefs, vector<int>& declaredIDs) {
  switch (vdata->type()) {
  case OBJECT: {
    ObjectData* data=static_cast<ObjectData*>(vdata);
    ValueData* pfield=data->getField("#pointer");
    if (pfield) {
      int id=pfield->getInt();

      //Negative IDs map to null pointer
      Identifiable* pointer=0;
      if (id>=0) {
        pointer=context.getById(id);
        if (!pointer) {
          pointer=context.createPlaceHolder(id);
        }
	danglingRefs.push_back(id);
      }
      return new PointerReference(pointer);
    }
    ValueData* idfield=data->getField("#id");
    if (idfield) {
      declaredIDs.push_back(idfield->getInt());
    }

    const vector<string>& fields=data->fields();
    for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();f_it++) {
      ValueData* fdata=data->getField(*f_it);
      ValueData* replaceData=processData(fdata, context, danglingRefs, declaredIDs);
      if (replaceData) {
        data->setField(*f_it,replaceData);
      }
    }
    break;
  }
  case ARRAY: {
    ArrayData* v_array=static_cast<ArrayData*>(vdata);
    for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++) {
      ValueData* replaceData=processData(*v_it, context, danglingRefs, declaredIDs);
      if (replaceData) {
        v_array->set(v_it-v_array->begin(),replaceData);
      }
    }
    break;
  }
  default:
    //Nothing to do
    break;
  }
  return 0;
}

#define UNKNOWN_OBJECT_CLASS_ASSERTION 0

static Serializable* createInstance(ObjectData* odata, const string& type, IdContext& context,
    map<Serializable*,std::set<int> >& waitingInstances, map<int,std::set<Serializable*> >& danglingReferences) {
  ValueData* idValue=odata->getField("#id");

  //Identifiable objects are overwritten if another object
  //with the same ID is read, so first check if the ID is already in the context
  Serializable* instance=0;
  if (idValue) {
    instance=context.getById(idValue->getInt());
    if (instance) {
      IdPlaceholder* placeHolder=dynamic_cast<IdPlaceholder*>(instance);
      if (placeHolder) {
        //Found instance is a placeholder, replace it
        instance=0;
      }
    }
  }

  if (instance) {
    //Just to ensure that the found instance has right type
    if (instance->className()!=type) {
      stringstream msg;
      msg << "Trying to overwrite " << instance->className() << " (ID " << idValue->getInt() << ") with " << type;
      throw logic_error(msg.str());
    }
  } else {
    try {
      instance=Serializable::createInstance(type);
    } catch (logic_error& e) {
      cerr << "EE Cannot create an instance of type '" << type << "'" << endl;
      exit(-1);
      //TODO Notify this occurrence
      return 0;
    }
  }

  vector<int> danglingPointers;
  vector<int> declaredIDs;

  processData(odata, context, danglingPointers,declaredIDs);
  instance->deserialize(*odata,context);

  for (vector<int>::iterator dp_it=danglingPointers.begin();dp_it!=danglingPointers.end();dp_it++) {
    waitingInstances[instance].insert(*dp_it);
    danglingReferences[*dp_it].insert(instance);
  }
  for (vector<int>::iterator id_it=declaredIDs.begin();id_it!=declaredIDs.end();id_it++) {
    //Further check, just in case the ID was a fake field
    if (context.getById(*id_it)) {
      map<int,set<Serializable*> >::iterator entry=danglingReferences.find(*id_it);
      if (entry!=danglingReferences.end()) {
        set<Serializable*>& instSet=(*entry).second;
        for (set<Serializable*>::iterator instance_it=instSet.begin();instance_it!=instSet.end();instance_it++) {
          waitingInstances[*instance_it].erase(*id_it);
          if (waitingInstances[*instance_it].empty()) {
            waitingInstances.erase(*instance_it);
            (*instance_it)->deserializeComplete();
          }
        }
        danglingReferences.erase(*id_it);
      }
    }
  }
  if (danglingPointers.empty()) {
    instance->deserializeComplete();
  }
  return instance;
}

Deserializer::Deserializer(SerializationContext* sc){
  _parser=new JSONMessageParser();
  _objectParser=new JSONObjectParser();
  _serializationContext = sc;
}

Serializable* Deserializer::readObject() {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return 0;
  }

  string type;
  auto_ptr<ObjectData> odata(_objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  return createInstance(odata.get(), type, *this, _waitingInstances, _danglingReferences);
}

Serializable* Deserializer::readObject(const std::string& line) {
  _serializationContext->setInputStream(new istringstream(line.c_str()));
  string type;
  auto_ptr<ObjectData> odata(_objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  return createInstance(odata.get(), type, *this, _waitingInstances, _danglingReferences);
}

bool Deserializer::readObjectData(ObjectData*& objdata, string& type) {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return false;
  }

  objdata = _objectParser->readObject(*_serializationContext->inputStream(), type);
  return true;
}

Message* Deserializer::readMessage() {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return 0;
  }

  //First parse the raw message data struct
  auto_ptr<MessageData> msgData(_parser->readMessage(*_serializationContext->inputStream()));
  if (!msgData.get()) {
    //Parsing error
    //TODO Notify this occurrence
    return 0;
  }

  //Check if this is an Identifiable object
  ObjectData* odata=msgData->getData();
  ValueData* idValue=odata->getField("#id");

  //Identifiable objects are overwritten if another message
  //with the same ID is read, so first check if the ID is already in the context
  Serializable* instance=0;
  if (idValue) {
    instance=getById(idValue->getInt());
    if (instance) {
      IdPlaceholder* placeHolder=dynamic_cast<IdPlaceholder*>(instance);
      if (placeHolder) {
        //Found instance is a placeholder, replace it
        instance=0;
      }
    }
  }

  if (instance) {
    //Just to ensure that the found instance has right type
    if (instance->className()!=msgData->getType()) {
      stringstream msg;
      msg << "Trying to overwrite " << instance->className() << " (ID " << idValue->getInt() << ") with " << msgData->getType();
      throw logic_error(msg.str()); 
    }
  } else {
    try {
      instance=Serializable::createInstance(msgData->getType());
    } catch (logic_error& e) {
      cerr << "EE Cannot create an instance of type '" << msgData->getType() << "'" << endl;
      exit(-1);
      //TODO Notify this occurrence
      return 0;
    }
  }

  vector<int> danglingPointers;
  vector<int> declaredIDs;

  processData(msgData->getData(), *this, danglingPointers,declaredIDs);
  instance->deserialize(*odata,*this);

  for (vector<int>::iterator dp_it=danglingPointers.begin();dp_it!=danglingPointers.end();dp_it++) {
    _waitingInstances[instance].insert(*dp_it);
    _danglingReferences[*dp_it].insert(instance);
  }
  for (vector<int>::iterator id_it=declaredIDs.begin();id_it!=declaredIDs.end();id_it++) {
    //Further check, just in case the ID was a fake field
    if (getById(*id_it)) {
      map<int,set<Serializable*> >::iterator entry=_danglingReferences.find(*id_it);
      if (entry!=_danglingReferences.end()) {
        set<Serializable*>& instSet=(*entry).second;
        for (set<Serializable*>::iterator instance_it=instSet.begin();instance_it!=instSet.end();instance_it++) {
          _waitingInstances[*instance_it].erase(*id_it);
          if (_waitingInstances[*instance_it].empty()) {
            _waitingInstances.erase(*instance_it);
            (*instance_it)->deserializeComplete();
          }
        }
        _danglingReferences.erase(*id_it);
      }
    }
  }
  if (danglingPointers.empty()) {
    instance->deserializeComplete();
  }
  return new Message(msgData->getTimestamp(),msgData->getSource(),instance);
}

void Deserializer::setFilePath(const string& fpath) {
  if (! _serializationContext)
    _serializationContext = new SerializationContext;
  _serializationContext->setInputFilePath(fpath);

}

Deserializer::~Deserializer() {
  delete _parser;
  delete _objectParser;
}

