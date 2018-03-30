#ifndef IO_UTILS_H_
#define IO_UTILS_H_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include "CountingStreamBuffer.h"
#include "PMPLExceptions.h"

class GroupCfg;


/*------------------------------- Vizmo Debug --------------------------------*/

/// @TODO
extern std::ofstream* vdo;


/// @TODO
void VDInit(std::string _filename);


/// @TODO
void VDClose();


/// @TODO
void VDComment(std::string _s);


/// @TODO
void VDClearAll();


/// @TODO
void VDClearLastTemp();


/// @TODO
void VDClearComments();


/// @TODO
template <typename CfgType>
void
VDAddNode(const CfgType& _cfg) {
  if(vdo)
    (*vdo) << "AddNode " << _cfg << std::endl;
}

template <>
inline
void
VDAddNode<GroupCfg>(const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDRemoveNode(const CfgType& _cfg) {
  if(vdo)
    (*vdo) << "RemoveNode " << _cfg << std::endl;
}

template <>
inline
void
VDRemoveNode<GroupCfg>(const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDAddEdge(const CfgType& _cfg1, const CfgType& _cfg2) {
  if(vdo)
    (*vdo) << "AddEdge " << _cfg1 << " " << _cfg2 << std::endl;
}

template <>
inline
void
VDAddEdge<GroupCfg>(const GroupCfg&, const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDRemoveEdge(const CfgType& _cfg1, const CfgType& _cfg2) {
  if(vdo)
    (*vdo) << "RemoveEdge " << _cfg1 << " " << _cfg2 << std::endl;
}

template <>
inline
void
VDRemoveEdge<GroupCfg>(const GroupCfg&, const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDAddTempCfg(const CfgType& _cfg, const bool _valid) {
  if(vdo)
    (*vdo) << "AddTempCfg " << _cfg << " " << _valid << std::endl;
}

template <>
inline
void
VDAddTempCfg<GroupCfg>(const GroupCfg&, const bool) { }


/// @TODO
template <typename CfgType>
void
VDAddTempRay(const CfgType& _cfg) {
  if(vdo)
    (*vdo) << "AddTempRay " << _cfg << std::endl;
}

template <>
inline
void
VDAddTempRay<GroupCfg>(const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDAddTempEdge(const CfgType& _cfg1, const CfgType& _cfg2) {
  if(vdo)
    (*vdo) << "AddTempEdge " << _cfg1 << " " << _cfg2 << std::endl;
}

template <>
inline
void
VDAddTempEdge<GroupCfg>(const GroupCfg&, const GroupCfg&) { }


/// @TODO
template <typename CfgType>
void
VDQuery(const CfgType& _cfg1, const CfgType& _cfg2) {
  if(vdo)
    (*vdo) << "Query " << _cfg1 << " " << _cfg2 << std::endl;
}

template <>
inline
void
VDQuery<GroupCfg>(const GroupCfg&, const GroupCfg&) { }

/*----------------------------- Other IO Utils -------------------------------*/

/// Write a list of Cfgs in a give path to file with given filename. Note if
/// file couldn't be opened, error message will be post and process will be
/// terminated.
template <typename CfgType>
void
WritePath(const std::string& _outputFile, const std::vector<CfgType>& _path) {
  std::ofstream ofs(_outputFile);
  if(!ofs)
    throw RunTimeException(WHERE, "Cannot open file \"" + _outputFile + "\"");

  // Print header.
  ofs << "VIZMO_PATH_FILE   Path Version 2012" << std::endl
      << "1" << std::endl
      << _path.size() << std::endl;

  // Print path.
  for(auto cit = _path.begin(); cit != _path.end(); ++cit)
    ofs << *cit << std::endl;
}


/// Determine whether a file exists.
/// @param _filename The name of the file to check.
/// @return True if the named file exists.
bool FileExists(const std::string& _filename);


/// Discard all commented lines util the next uncommented line is found
/// @param _is Stream
void GoToNext(std::istream& _is);


/// Determines if character starts a comment ('#')
/// @param _c Character in question
/// @return Comment or not, that is the question
bool IsCommentLine(const char _c);


/// Get directory path from filename
/// @param _filename Filename
/// @return Directory
std::string GetPathName(const std::string& _filename);


/// Read data from stream
/// @tparam T Data type
/// @param _is Stream
/// @param _cbs Counting stream buffer
/// @param _desc String describing field
/// @return Data
template <typename T>
T
ReadField(std::istream& _is, CountingStreamBuffer& _cbs,
    const std::string& _desc) {
  char c;
  std::string line;
  T element = T();
  while(_is) {
    c = _is.peek();
    if(c == '#')
      getline(_is, line);
    else if(!isspace(c)) {
      if (!(_is >> element))
        throw ParseException(_cbs.Where(), _desc);
      else
        break;
    }
    else
      _is.get(c);
  }
  if(_is.eof())
    throw ParseException(_cbs.Where(), _desc + " End of file reached.");

  return element;
};


/// Read string from stream
/// @param _is Stream
/// @param _cbs Counting stream buffer
/// @param _desc String describing field
/// @param _toUpper True means convert string to all upper case
/// @return Data string
std::string ReadFieldString(std::istream& _is, CountingStreamBuffer& _cbs,
    const std::string& _desc, const bool _toUpper = true);


/*
/// Read color from a comment line
/// @param _is Stream
/// @return Color
Color4 GetColorFromComment(std::istream& _is);
*/


/// Split string based on delimiter.
std::vector<std::string> GetTags(std::string _stags, std::string _delim);

/*----------------------------------------------------------------------------*/

#endif
