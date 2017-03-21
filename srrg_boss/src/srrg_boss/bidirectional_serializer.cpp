#include "bidirectional_serializer.h"

namespace srrg_boss {
  BidirectionalSerializer::BidirectionalSerializer(SerializationContext* sc) : Serializer(sc), Deserializer(sc){
  }

  BidirectionalSerializer::BidirectionalSerializer(){
    SerializationContext* sc = new SerializationContext;
    Serializer::_serializationContext = sc;
    Deserializer::_serializationContext = sc;
  }

  void BidirectionalSerializer::setOutputFilePath(const std::string& fpath){
    Serializer::setFilePath(fpath);
  }

  void BidirectionalSerializer::setOutputBinaryPath(const std::string& fpath) {
    Serializer::setBinaryPath(fpath);
  }


  void BidirectionalSerializer::setInputFilePath(const std::string& fpath){
    Deserializer::setFilePath(fpath);
  }
  
  
  std::ostream* BidirectionalSerializer::getBinaryOutputStream(const std::string& fname){
    return Serializer::serializationContext()->getBinaryOutputStream(fname);
  }
  
  std::istream* BidirectionalSerializer::getBinaryInputStream(const std::string& fname){
    return Deserializer::serializationContext()->getBinaryInputStream(fname);
  }

  
}
