#ifndef IO_UTILS_H_
#define IO_UTILS_H_

#include <cctype>
#include <string>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <ostream>
#include <fstream>
#include <iostream>
#include <streambuf>
using namespace std;

#ifndef TIXML_USE_STL
#define TIXML_USE_STL
#endif
#include "tinyxml.h"

#include "PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// Wrapper class for XML handling
///
/// @ingroup IOUtils
/// @details This is a wrapper class for XML handling with tiny xml. It is read
///          only and supports trivial XML parsing.
////////////////////////////////////////////////////////////////////////////////
class XMLNode {

  public:

    typedef vector<XMLNode>::iterator iterator;

    ///@name Construction
    ///@{

    /// Construct an XML node object from an XML file.
    /// @param _filename XML Filename
    /// @param _desiredNode Desired XML Node to make root of tree
    ///
    /// Will throw ParseException when \p _desiredNode cannot be found of
    /// \p _filename is poorly formed input
    XMLNode(const string& _filename, const string& _desiredNode);

    ///@}
    ///@name Metadata Accessors
    ///@{

    /// Get the XMLNode name.
    const string& Name() const {return m_node->ValueStr();}

    /// Get the XML filename.
    const string& Filename() const {return m_filename;}

    ///@}
    ///@name Iteration
    ///@{

    /// Get an iterator to this node's first child.
    iterator begin();

    /// Get an iterator to this node's last child.
    iterator end();

    ///@}
    ///@name Attribute Parsing
    ///@{

    /// Read XML attribute.
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

    /// Read XML boolean attribute
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

    /// Read XML string attribute
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

    /// Read XML string attribute
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

    /// Report warnings for XML tree rooted at this node
    /// @param _warningsAsErrors True will throw exceptions for warnings
    ///
    /// To be called after parsing phase. This will report warnings throughout
    /// entire XML document. Should only be called on root XML node. Warnings to
    /// be reported:
    ///   - unknown/unparsed nodes
    ///   - unrequested attribues
    void WarnAll(bool _warningsAsErrors = false);

    /// Generate string describing where the node is
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, row (line number), and column of XMLNode.
    string Where() const;

  private:

    /// Generate XMLNodes for all children
    ///
    /// Builds the internal vector of children, used when iterating over
    /// the children. This vector is only built with ELEMENT type nodes.
    void BuildChildVector();

    /// Private constructor for use within BuildChildVector
    /// @param _node New TiXMLNode
    /// @param _filename XML filename
    /// @param _doc TiXmlDocument from tree's root node
    XMLNode(TiXmlNode* _node, const string& _filename, TiXmlDocument* _doc);

    /// Return error report for attribute being the wrong type
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    string AttrWrongType(const string& _name, const string& _desc) const;

    /// Return error report for missing attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    string AttrMissing(const string& _name, const string& _desc) const;

    /// Return error report for attribute being in an invalid range
    /// @tparam T Type of attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @param _min Minimum value of attribute
    /// @param _max Maximum value of attribute
    /// @param _val The specified value.
    /// @return Error report
    template<typename T>
    string AttrInvalidBounds(const string& _name, const string& _desc,
        const T& _min, const T& _max, const T& _val) const;

    /// Recursive function computing whether nodes have been accessed
    void ComputeAccessed();

    /// Recursive function reporting all unknown/unparsed nodes and unrequested
    /// attributes.
    /// @param[out] _anyWarnings Initially should be false, and stores whether
    ///                          any warnings have been reported
    void WarnAllRec(bool& _anyWarnings);

    /// Report unknown node warning to cerr
    void WarnUnknownNode();

    /// Report unrequested attributes to cerr
    bool WarnUnrequestedAttributes();

    /// Generate string describing where the node is
    /// @param _f Filename
    /// @param _l Line number
    /// @param _c Column number
    /// @param _name Report name of node
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, name, row (line number), and column of XMLNode.
    string Where(const string& _f, int _l, int _c, bool _name = true) const;

    ///@name Internal State
    ///@{

    TiXmlNode* m_node{nullptr}; ///< TiXmlNode
    bool m_childBuilt{false};   ///< Have children been parsed into nodes?
    bool m_accessed{false};     ///< Has this node been accessed or not?
    vector<XMLNode> m_children; ///< Children of node
    unordered_set<string>
      m_reqAttributes;          ///< Attributes which have been requested
    string m_filename;          ///< XML Filename
    TiXmlDocument* m_doc;       ///< Overall TiXmlDocument

    ///@}
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
              AttrInvalidBounds(_name, _desc, _min, _max, toReturn));
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
    const T& _min, const T& _max, const T& _val) const {
  ostringstream oss;
  oss << "Invalid value for attribute '" << _name << "'." << endl
      << "\tAttribute description: " << _desc << "." << endl
      << "\tValid range: ["<< _min << ", " << _max << "]" << endl
      << "\tValue specified: " << _val;
  return oss.str();
}

/*--------------------------- Vizmo Debug output -----------------------------*/

extern ofstream* vdo;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDInit(string _filename);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDClose();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDAddNode(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddNode " << _cfg << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDRemoveNode(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "RemoveNode " << _cfg << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDAddEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDRemoveEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "RemoveEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDAddTempCfg(const CfgType& _cfg, bool _valid){
  if(vdo!=NULL){
    (*vdo) << "AddTempCfg " << _cfg << " " << _valid << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDAddTempRay(const CfgType& _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddTempRay " << _cfg << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDAddTempEdge(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddTempEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDComment(string _s);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDClearAll();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDClearLastTemp();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
void VDClearComments();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
template<class CfgType>
void VDQuery(const CfgType& _cfg1, const CfgType& _cfg2){
  if(vdo!=NULL){
    (*vdo) << "Query " << _cfg1 << " " << _cfg2 << endl;
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief TODO
///
/// Write a list of Cfgs in a give path to file with given filename. Note if
/// file couldn't be opened, error message will be post and process will be
/// terminated.
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


////////////////////////////////////////////////////////////////////////////////
/// @TODO
/// @ingroup IOUtils
////////////////////////////////////////////////////////////////////////////////
class CountingStreamBuffer : public streambuf {

  public:

    /// @param _filename Filename
    CountingStreamBuffer(string _filename);

    /// @return Current line number
    size_t LineNumber() const { return m_line; }

    /// @return Line number of previously read character
    size_t PrevLineNumber() const { return m_prevLine; }

    /// @return Current column
    size_t Column() const { return m_column; }

    /// @return Current file position
    streamsize FilePos() const { return m_filePos; }

    /// @return String describing current file position
    string Where() const;

  protected:

    //Disallow copy and assignment
    CountingStreamBuffer(const CountingStreamBuffer&);
    CountingStreamBuffer& operator=(const CountingStreamBuffer&);

    /// Extract next character from stream without advancing read position.
    /// @return Next character or EOF
    streambuf::int_type underflow() { return m_streamBuffer->sgetc(); }

    /// Extract next character from stream
    /// @return Next character of EOF
    streambuf::int_type uflow();

    /// Put back last character
    /// @param _c Character
    /// @return Value of character put back or EOF
    streambuf::int_type pbackfail(streambuf::int_type _c);

    /// Change position by offset according to dir and mode
    /// @param _pos Position
    /// @param _dir Direction
    /// @param _mode Mode
    /// @return Position
    virtual ios::pos_type seekoff(ios::off_type _pos, ios_base::seekdir _dir,
        ios_base::openmode _mode);

    /// Change to specified position according to mode
    /// @param _pos Position
    /// @param _mode Mode
    /// @return Position
    virtual ios::pos_type seekpos(ios::pos_type _pos, ios_base::openmode _mode);

  private:

    ///@name Internal State
    ///@{

    string m_filename;         ///< Filename
    ifstream m_fileStream;     ///< Hosted file stream
    streambuf* m_streamBuffer; ///< Hosted streambuffer
    size_t m_line;             ///< Current line number
    size_t m_prevLine;         ///< Line number of last read character
    size_t m_column;           ///< Current column
    size_t m_prevColumn;       ///< Previous column
    streamsize m_filePos;      ///< File position

    ///@}
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Determine if a file exists or not
/// @param _filename Filename
/// @return Exists
bool FileExists(const string& _filename);

////////////////////////////////////////////////////////////////////////////////
/// @brief Discard all commented lines util the next uncommented line is found
/// @param _is Stream
void GoToNext(istream& _is);

////////////////////////////////////////////////////////////////////////////////
/// @brief Determines if character starts a comment ('#')
/// @param _c Character in question
/// @return Comment or not, that is the question
inline bool IsCommentLine(char _c) {return _c == '#';}

////////////////////////////////////////////////////////////////////////////////
/// @brief Get directory path from filename
/// @param _filename Filename
/// @return Directory
string GetPathName(const string& _filename);

////////////////////////////////////////////////////////////////////////////////
/// @brief Read data from stream
/// @tparam T Data type
/// @param _is Stream
/// @param _cbs Counting stream buffer
/// @param _desc String describing field
/// @return Data
template<typename T>
T
ReadField(istream& _is, CountingStreamBuffer& _cbs, const string& _desc) {
  char c;
  string line;
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Read string from stream
/// @param _is Stream
/// @param _cbs Counting stream buffer
/// @param _desc String describing field
/// @param _toUpper True means convert string to all upper case
/// @return Data string
string ReadFieldString(istream& _is, CountingStreamBuffer& _cbs,
    const string& _desc, bool _toUpper = true);

/*
////////////////////////////////////////////////////////////////////////////////
/// @brief Read color from a comment line
/// @param _is Stream
/// @return Color
Color4 GetColorFromComment(istream& _is);
*/

////////////////////////////////////////////////////////////////////////////////
/// @ingroup IOUtils
/// @brief Split string based on delimiter.
vector<string> GetTags(string _stags, string _delim);

#endif
