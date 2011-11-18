#ifndef IOUTILS_H_
#define IOUTILS_H_

#ifndef TIXML_USE_STL
#define TIXML_USE_STL
#endif 

#include <cctype>
#include <string>
#include <vector>
#include <algorithm>
#include <ostream>
#include <fstream>
#include <iostream>

#include "tinyxml.h"

using namespace std;

class Cfg;
class Environment;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  XML Wrapper
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/* class XMLNode
   This is a wrapper class for XML handeling.  It is READONLY
   and supports only trivial XML parsing.  Wrapping up TinyXML, 
   it only supports TiXmlElements; this is because this is our
   only need for intput XML files.
  
   If for some reason you really really need the underlying TiXMLNode,
   then make your class a friend of this one.  Adding the neccessary
   functionality here is a better solution!
  
   \todo Testing for inproper int,float,double is still not good
   things like int=3.5 or double=0.6q are accepted but truncated.
  
   Questions:  Should this ever terminate on fail?  or return -1?
*/
class XMLNodeReader {
  /////////////////////////////////////////////////////////////////////////////
  //  Public Methods
  /////////////////////////////////////////////////////////////////////////////
public:
  typedef vector<XMLNodeReader>::iterator childiterator;
  
  // Constructor for XMLNodeReader
  // param in_pnode the TiXmlNode this class wraps 
  explicit XMLNodeReader(TiXmlNode* _node, const string& _filename );

  // This constructor takes as input a XML document and searches for a toplevel
  // node by the name of in_desiredNode
  //
  // param in_fileName The XML filename to parse
  // param in_desiredNode The top-level XML Node to return
  explicit XMLNodeReader(const string& _filename, TiXmlDocument& _doc, 
                            const string& _desiredNode);


  //Accessing Methods//
  string getName() const;

  // Returns true if XMLNode has a specific child
  //
  // param in_childName Name of child queried
  bool hasChild(const string& _childName) const;

  //WARNING: will terminate with msg if child doesn't exist, 
  //so its better to check outside!
  XMLNodeReader getFirstChild(const string& _childName) ;


  childiterator children_begin();

  childiterator children_end();


  template<typename T>
  T numberXMLParameter (const string& _name,
                        bool _req,
                        const T& _default,
                        const T& _min, 
                        const T& _max, 
                        const string& _desc) {
    VerifyElement();
    m_reqAttributes.push_back(_name);
    T toReturn;  
    int qrReturn = m_node->ToElement()->QueryValueAttribute(_name,&toReturn);
    if(qrReturn == TIXML_WRONG_TYPE) {
      PrintAttrWrongType(_name,_desc);
      exit(-1);
    } else if(qrReturn == TIXML_NO_ATTRIBUTE && _req == true) {
      PrintAttrMissing(_name, _desc);
      exit(-1);
    } else if(qrReturn == TIXML_NO_ATTRIBUTE && _req == false) {
      toReturn = _default; //defaulting value b/c not found in XML file
    } else if(qrReturn == TIXML_SUCCESS) {
      if(toReturn < _min || toReturn > _max) {
        PrintAttrInvalidBounds(_name, _desc, _min, _max);
        exit(-1);
      }
    } else {
       cerr << "num_XML_param::numberXMLParameter Logic Error!" << endl;
    }

    return toReturn;
  }

  string stringXMLParameter (const string& _name,
                                  bool _req,
                                  const string& _default,
                                  const string& _desc);

  bool hasXMLParameter(const string& _name) ;

  bool boolXMLParameter (const string& _name, bool _req,
                         bool _default, const string& _desc);
  // Will output a warning to the user that XML Node 
  // contains extra attributes.  This could be a sign
  // of a user typeof on an optional attribute.
  void warnUnrequestedAttributes();

  void warnUnknownNode() ;
  //Verify this node is what is expected, will terminate on fail
  void verifyName(const string& _name);

  ostream& operator<< (ostream& _out) {
    _out << *(this->m_node);
    return _out;
  }

  bool operator==(const XMLNodeReader& _node) const {
    return (m_node == _node.m_node) &&
           (m_childBuilt == _node.m_childBuilt) &&
           (m_children == _node.m_children) &&
           (m_reqAttributes == _node.m_reqAttributes);
  }

  /////////////////////////////////////////////////////////////////////////////
  //  Private Methods
  /////////////////////////////////////////////////////////////////////////////
  private:
  // Returns true if the XMLNode is an element.  Used for error checking,
  // Elements are may carry attributes.
  bool IsElement() const {
    return (m_node->Type() == TiXmlNode::ELEMENT) ? true : false;
  }

  void VerifyElement() const ;

  void PrintAttrWrongType(const string& _name, const string& _desc) const ;
  void PrintAttrMissing(const string& _name, const string& _desc) const;
  template<typename T>
  void PrintAttrInvalidBounds(const string& _name, const string& _desc, 
                              const T& _min, const T& _max) const {
    cerr << "*************************************************************" << endl;
    cerr << "XML Error:: Invalid Attribute Range" << endl;
    cerr << "XML file:: " << m_xmlFilename << endl;
    cerr << "Parent Node: " << getName() << ", Attribute: "; 
    cerr << _name << endl;
    cerr << "Attribute Range: ["<< _min << ", " << _max << "]" << endl;
    cerr << "Attribute Description: " << _desc << endl;
    cerr << *m_node << endl;
    cerr << "*************************************************************" << endl;
  }

  void PrintMissingRequestedChild(const string& _childName);
  // Builds the internal vector of children, used when iterating over
  // the children.  This vector is only build with ELEMENT type nodes
  void BuildChildVector();

  friend ostream& operator<< (ostream& _out, const XMLNodeReader& _node);

  bool HasRequestedAttr(string _name);
  /////////////////////////////////////////////////////////////////////////////
  //  Protected Data
  /////////////////////////////////////////////////////////////////////////////
  protected:
  

  /////////////////////////////////////////////////////////////////////////////
  //  Private Data
  /////////////////////////////////////////////////////////////////////////////
  private:
  TiXmlNode* m_node;
  bool m_childBuilt;
  vector<XMLNodeReader> m_children;
  vector<string> m_reqAttributes;
  string m_xmlFilename;
};

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

template<class CFG>
void VDAddNode(CFG _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddNode " << _cfg << endl;
  }
};

template<class CFG>
void VDRemoveNode(CFG _cfg){
  if(vdo!=NULL){
    (*vdo) << "RemoveNode " << _cfg << endl;
  }
};

template<class CFG>
void VDAddEdge(CFG _cfg1, CFG _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

template<class CFG>
void VDRemoveEdge(CFG _cfg1, CFG _cfg2){
  if(vdo!=NULL){
    (*vdo) << "RemoveEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

template<class CFG>
void VDAddTempCfg(CFG _cfg, bool _valid){
  if(vdo!=NULL){
    (*vdo) << "AddTempCfg " << _cfg << " " << _valid << endl;
  }
};

template<class CFG>
void VDAddTempRay(CFG _cfg){
  if(vdo!=NULL){
    (*vdo) << "AddTempRay " << _cfg << endl;
  }
};

template<class CFG>
void VDAddTempEdge(CFG _cfg1, CFG _cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddTempEdge " << _cfg1 << " " << _cfg2 << endl;
  }
};

void VDComment(string _s);

void VDClearAll();

void VDClearLastTemp();

void VDClearComments();

template<class CFG>
void VDQuery(CFG _cfg1, CFG _cfg2){
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

/**Write a list of Cfgs in a give path and path size 
 *to file with given filename.
 *
 *This method writes number of links of robot,
 *path size to output_file. Then Cfgs for
 *each link in robot will be output to file 
 *named "output_file".cfg
 *
 *
 *@param filename Filename for Cfg data.
 *@param path All Gfgs will be written to file.
 *@param env is used to get "robot link" information.
 *@note if file couldn't be opened, error message will be post 
 *and process will be terminated.
 *
 *@see Cfg::printLinkConfigurations
 */
void WritePathLinkConfigurations(const string _outputFile, 
    vector<Cfg*>& _path, 
    Environment* _env);

template <class CFG>
void WritePathLinkConfigurations(const string _outputFile, 
    vector<CFG>& _path, 
    Environment* _env){
  vector<Cfg*> ppath;
  for(size_t i=0; i<_path.size(); i++)
    ppath.push_back(&_path[i]);
  WritePathLinkConfigurations(_outputFile, ppath, _env);
}



/**Write a list of Cfgs in a give path and path size 
 *to file with given filename.
 *@param filename Filename for Cfg data.
 *@param path All Gfgs will be written to file.
 *@param env is not used.
 *@note if file couldn't be opened, error message will be post 
 *and process will be terminated.
 */   
void WritePathConfigurations(const string _outputFile, 
    vector<Cfg*>& _path, 
    Environment* _env);  

template <class CFG>
void WritePathConfigurations(const string _outputFile, 
    vector<CFG>& _path, 
    Environment* _env){
  vector<Cfg*> ppath;
  for(size_t i=0; i<_path.size(); i++) 
    ppath.push_back(&_path[i]);
  WritePathConfigurations(_outputFile, ppath, _env);
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

#define PMPL_EXIT 1      ///< Actions for VerifyFileExists
/**Check if or not this given filename exists.
 *@param _fname File name that is going to be checked.
 *@action What should be done if file not found.
 *Its value should be PMPL_EXIT or RETURN.
 *
 *@return true if file exists. If file dosen't exist 
 *and action is PMPL_EXIT, process will be terminated.
 *Otherwise false will be returned.
 */
bool VerifyFileExists(const string _name, int _action);

/**Read data for element from input stream.
 *This method throws away comments starting from "#", and
 *find out read data for element.
 *
 *@param element An element which will read data from file.
 *@note type T should have istream>> overloading.
 *@note >> overloading should return 0 (NULL or false)
 *if data couldn't be read correctly.
 */

//The character to distinguish commnets.
#define COMMENT_DELIMITER '#'
//The maximum number of characters in each line.
#define LINEMAX 256

template <class T> bool ReadField(istream& _is, T* _element, vector<char*>& _comment) {
  char c;
  char ThrowAwayLine[LINEMAX];

  while ( _is.get(c) ) {
    if (c == '#') {
      _is.getline(ThrowAwayLine,LINEMAX,'\n');
      _comment.push_back(strdup(ThrowAwayLine));
    }
    else if (! isspace(c) ) {
      _is.putback(c);
      if (_is >> *_element) {
        return true;
      } else {
        break;
      }
    }
  }
  // could not read correctly ...
  cout << "Error in reading!!! at util::readfield. " << endl;
  return false;
}

template <class T> bool ReadField(istream& _is, T* _element){
  vector <char*> comment;
  bool ret=ReadField(_is,_element,comment);
  comment.clear();
  return ret;
}

#endif 
