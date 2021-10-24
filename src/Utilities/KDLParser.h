#ifndef _PPL_KDL_PARSING_H_
#define _PPL_KDL_PARSING_H_

#include <kdl_parser/kdl_parser.hpp>

#include <string>
KDL::Tree
ParseKDLFromFile(const std::string _filename);

#endif
