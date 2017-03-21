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

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>

#include "json_object_parser.h"

#include "object_data.h"

using namespace std;
using namespace srrg_boss;

typedef string::iterator Iterator;

namespace srrg_boss {

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phoenix = boost::phoenix;
namespace fusion = boost::fusion;

struct _json_object_parser_impl: qi::grammar<Iterator, fusion::vector<std::string, ObjectData*>(), ascii::space_type> {
  
  _json_object_parser_impl(): _json_object_parser_impl::base_type(message_struct) {
    using qi::double_;
    using qi::char_;
    using qi::_val;
    using qi::lit;
    using qi::_1;
    using qi::_2;
    using phoenix::new_;
    using phoenix::bind;
    using phoenix::at_c;
    using qi::on_error;
    using qi::fail;
    using phoenix::construct;
    using phoenix::val;

    //start=message_struct; //[_val=pair<ObjectData*,string>(at_c<0>(_1),at_c<1>(_1))];
    
    message_struct = string_content >> object_;
    
    value_ = bool_ | number_ | string_ | array_ | object_;
    
    bool_ = lit("true")[_val=new BoolData(true)] | lit("false")[_val=new BoolData(false)];
    
    number_ = double_[_val=new_<NumberData>(_1)];
    
    string_content = lit('"') >> *(
      +(~char_("\n\\\""))[_val +=_1] |
      lit("\\\\")[_val += '\\'] |
      lit("\\\"")[_val += '\"'] |
      lit("\\/")[_val += '/'] |
      lit("\\n")[_val += '\n'] |
      lit("\\b")[_val += '\b'] |
      lit("\\f")[_val += '\f'] |
      lit("\\r")[_val += '\r'] |
      lit("\\t")[_val += '\t']
      ) >> lit('"');
      
    string_ = string_content[_val=new_<StringData>(_1)];

    array_ = lit('[')[_val=new_<ArrayData>()] >>
      -(value_[bind(static_cast<void (ArrayData::*)(ValueData*)> (&ArrayData::add),_val,_1)] % lit(',')) >> lit(']');
    

    field_struct = string_content >> lit(':') >> value_;
    
    object_ = lit('{')[_val=new_<ObjectData>()] >>
      -(field_struct[bind(static_cast<void (ObjectData::*)(const std::string&, ValueData*)> (&ObjectData::setField),_val,at_c<0>(_1),at_c<1>(_1))] % lit(',')) >>
      lit('}');
  }
  
  
  qi::rule<Iterator, fusion::vector<std::string, ObjectData*>(), ascii::space_type> message_struct;
  qi::rule<Iterator, std::string(), ascii::space_type> message_type;
  qi::rule<Iterator, std::string(), ascii::space_type> message_source;
  qi::rule<Iterator, ValueData*(), ascii::space_type> value_;
  qi::rule<Iterator, NumberData*(), ascii::space_type> number_;
  qi::rule<Iterator, StringData*(), ascii::space_type> string_;
  qi::rule<Iterator, std::string()> string_content;
  qi::rule<Iterator, BoolData*(), ascii::space_type> bool_;
  qi::rule<Iterator, ArrayData*(), ascii::space_type> array_;
  qi::rule<Iterator, ObjectData*(), ascii::space_type> object_;
  qi::rule<Iterator, fusion::vector<std::string, ValueData*>(), ascii::space_type> field_struct;
};

JSONObjectParser::JSONObjectParser() {
  _parser_impl=new _json_object_parser_impl();
}

ObjectData* JSONObjectParser::readObject(istream& is, string& type) {
  string line;
  if (!getline(is,line)) {
    return 0;
  }
  fusion::vector<std::string, ObjectData*> v;
  qi::phrase_parse(line.begin(),line.end(),*_parser_impl, ascii::space, v);
  type=fusion::at_c<0>(v);
  return fusion::at_c<1>(v);
}

ObjectData* JSONObjectParser::readObject(const std::string& line, std::string& type){
  std::string _line = line;
  fusion::vector<std::string, ObjectData*> v;
  qi::phrase_parse(_line.begin(),_line.end(),*_parser_impl, ascii::space, v);
  type=fusion::at_c<0>(v);
  return fusion::at_c<1>(v);
}

JSONObjectParser::~JSONObjectParser() {
  delete _parser_impl;
}

}

