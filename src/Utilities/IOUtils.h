#ifndef IO_UTILS_H_
#define IO_UTILS_H_

#ifndef TIXML_USE_STL
#define TIXML_USE_STL
#endif

#include <cctype>
#include <string>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <ostream>
#include <fstream>
#include <iostream>
using namespace std;

#include "tinyxml.h"

#include "PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Wrapper class for XML handeling
///
/// This is a wrapper class for XML handeling.  It is READONLY and supports only
/// trivial XML parsing.  Wrapping up TinyXML, it only supports TiXmlElements;
/// this is because this is our only need for intput XML files - for now.
////////////////////////////////////////////////////////////////////////////////
class XMLNode {
  public:
    typedef vector<XMLNode>::iterator iterator;

    ////////////////////////////////////////////////////////////////////////////
    /// @param _filename XML Filename
    /// @param _desiredNode Desired XML Node to make root of tree
    ///
    /// Will throw ParseException when \p _desiredNode cannot be found of
    /// \p _filename is poorly formed input
    XMLNode(const string& _filename, const string& _desiredNode);

    ////////////////////////////////////////////////////////////////////////////
    /// @return Name of XMLNode
    const string& Name() const {return m_node->ValueStr();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Name of XML file
    const string& Filename() const {return m_filename;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Iterator to first child
    iterator begin();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Iterator to end of children
    iterator end();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read XML attribute
    /// @tparam T Type of attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _min Minimum valud of attribute
    /// @param _max Maximum valud of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned, otherwise input value is
    /// required to be in the range [\p _min, \p _max]. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    template<typename T>
      T Read(const string& _name,
          bool _req,
          const T& _default,
          const T& _min,
          const T& _max,
          const string& _desc);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read XML boolean attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    bool Read(const string& _name,
        bool _req,
        bool _default,
        const string& _desc);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read XML string attribute
    /// @return Value of attribute
    ///
    /// Calls string version of function to avoid confusion with bool -> const
    /// char* conversion in compile.
    string Read(const string& _name,
        bool _req,
        const char* _default,
        const string& _desc) {
      return Read(_name, _req, string(_default), _desc);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read XML string attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    string Read(const string& _name,
        bool _req,
        const string& _default,
        const string& _desc);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Report warnings for XML tree rooted at this node
    /// @param _warningsAsErrors True will throw exceptions for warnings
    ///
    /// To be called after parsing phase. This will report warnings throughout
    /// entire XML document. Should only be called on root XML node. Warnings to
    /// be reported:
    ///   - unknown/unparsed nodes
    ///   - unrequested attribues
    void WarnAll(bool _warningsAsErrors = false);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate string describing where the node is
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, row (line number), and column of XMLNode.
    string Where() const;

  private:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate XMLNodes for all children
    ///
    /// Builds the internal vector of children, used when iterating over
    /// the children. This vector is only built with ELEMENT type nodes.
    void BuildChildVector();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Private constructor for use within ::BuildChildVector
    /// @param _node New TiXMLNode
    /// @param _filename XML filename
    /// @param _doc TiXmlDocument from tree's root node
    XMLNode(TiXmlNode* _node, const string& _filename, TiXmlDocument* _doc);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Return error report for attribute being the wrong type
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    string AttrWrongType(const string& _name, const string& _desc) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Return error report for missing attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    string AttrMissing(const string& _name, const string& _desc) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Return error report for attribute being in an invalid range
    /// @tparam T Type of attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @param _min Minimum value of attribute
    /// @param _max Maximum value of attribute
    /// @return Error report
    template<typename T>
      string AttrInvalidBounds(const string& _name, const string& _desc,
          const T& _min, const T& _max) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Recursive function computing whether nodes have been accessed
    void ComputeAccessed();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Recursive function reporting all unknown/unparsed nodes and
    ///        unrequested attributes
    /// @param[out] _anyWarnings Initially should be false, and stores whether
    ///                          any warnings have been reported
    void WarnAllRec(bool& _anyWarnings);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Report unknown node warning to cerr
    void WarnUnknownNode();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Report unrequested attributes to cerr
    bool WarnUnrequestedAttributes();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate string describing where the node is
    /// @param _f Filename
    /// @param _l Line number
    /// @param _c Column number
    /// @param _name Report name of node
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, name, row (line number), and column of XMLNode.
    string Where(const string& _f, int _l, int _c, bool _name = true) const;

    TiXmlNode* m_node;          ///< TiXmlNode
    bool m_childBuilt;          ///< Have children been parsed into nodes?
    bool m_accessed;            ///< Has this node been accessed or not?
    vector<XMLNode> m_children; ///< Children of node
    unordered_set<string>
      m_reqAttributes;          ///< Attributes which have been requested
    string m_filename;          ///< XML Filename
    TiXmlDocument* m_doc;       ///< Overall TiXmlDocument
};

template<typename T>
T
XMLNode::
Read(const string& _name,
    bool _req,
    const T& _default,
    const T& _min,
    const T& _max,
    const string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  T toReturn;
  int qr = m_node->ToElement()->QueryValueAttribute(_name, &toReturn);
  switch(qr) {
    case TIXML_WRONG_TYPE:
      throw ParseException(Where(), AttrWrongType(_name, _desc));
      break;
    case TIXML_NO_ATTRIBUTE:
      {
        if(_req)
          throw ParseException(Where(), AttrMissing(_name, _desc));
        else
          toReturn = _default;
        break;
      }
    case TIXML_SUCCESS:
      {
        if(toReturn < _min || toReturn > _max)
          throw ParseException(Where(),
              AttrInvalidBounds(_name, _desc, _min, _max));
        break;
      }
    default:
      throw RunTimeException(WHERE, "Logic shouldn't be able to reach this.");
  }

  return toReturn;
}

template<typename T>
string
XMLNode::
AttrInvalidBounds(const string& _name, const string& _desc,
    const T& _min, const T& _max) const {
  ostringstream oss;
  oss << "Invalid attribute range on '" << _name << "'." << endl;
  oss << "\tAttribute description: " << _desc << "." << endl;
  oss << "\tValid range: ["<< _min << ", " << _max << "].";
  return oss.str();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  Vizmo Debug output
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

extern ofstream* vdo;

void VDInit(string _filename);

void VDClose();

template<class CfgType>
void VDAddNode(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddNode " << _cfg << endl;
  }
};

template<class CfgType>
void VDRemoveNode(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "RemoveNode " << _cfg << endl;
  }
};

template<class CfgType>
void VDAddEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

template<class CfgType>
void VDRemoveEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "RemoveEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

template<class CfgType>
void VDAddTempCfg(const CfgType& _cfg, bool _valid){
  if(vdo!=NULL){
    (*vdo) << "AddTempCfg " << _cfg << " " << _valid << endl;
  }
};

template<class CfgType>
void VDAddTempRay(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddTempRay " << _cfg << endl;
  }
};

template<class CfgType>
void VDAddTempEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddTempEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

void VDComment(string _s);

void VDClearAll();

void VDClearLastTemp();

void VDClearComments();

template<class CfgType>
void VDQuery(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "Query " << _cfg1 << " " << _cfg2 << endl;
  }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  Path output
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/**Write a list of Cfgs in a give path
 *to file with given filename.
 *note if file couldn't be opened, error message will be post
 *and process will be terminated.
 */
template<class CfgType>
void WritePath(string _outputFile, const vector<CfgType>& _path) {
  ofstream ofs(_outputFile.c_str());
  if(!ofs){
    cerr << "Error in WritePath::Cannot open file \"" << _outputFile << "\". Exitting." << endl;
    exit(1);
  }
  ofs << "VIZMO_PATH_FILE   Path Version " << 2012 << endl;
  ofs << "1" <<endl;
  ofs << _path.size() << endl;
  typedef typename vector<CfgType>::const_iterator CIT;
  for(CIT cit = _path.begin(); cit!=_path.end(); ++cit)
    ofs << *cit << endl;
  ofs.close();
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  Reading Fields, used in Multibody.h
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//determine if a file exists or not
bool FileExists(const string& _filename, bool _err = true);

//discard all commented lines util the next uncommented line is found
void GoToNext(istream& _is);

//is given char a start of command line? comments denoted with # character
inline bool IsCommentLine(char _c) {return _c == '#';}

//GetPathName from given filename
string GetPathName(const string& _filename);

//read type by eating all white space. If type cannot be read report the error
//provided
template <class T>
T
ReadField(istream& _is, const string& _where, const string& _error) {
  char c;
  string line;
  T element = T();
  while(_is) {
    c = _is.peek();
    if(c == '#') {
      getline(_is, line);
    }
    else if(!isspace(c)) {
      if (!(_is >> element))
        throw ParseException(_where, _error);
      else
        break;
    }
    else
      _is.get(c);
  }
  if(_is.eof())
    throw ParseException(_where, "End of file reached");

  return element;
};

//read the string using above ReadField and tranform it to upper case
string ReadFieldString(istream& _is, const string& _where, const string& _error, bool _toUpper = true);

//optionally read a color from a comment line
//Color4 GetColorFromComment(istream& _is);

//----------------------------------------------------------------------------
// GetTags: split string based on delimiter.
//----------------------------------------------------------------------------
vector<string> GetTags(string _stags, string _delim);

#endif
