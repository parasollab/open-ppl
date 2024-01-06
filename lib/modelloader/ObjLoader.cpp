#include <cstdio>
#include <fstream>

#include "ObjLoader.h"

using namespace std;

/*-------------------------- Local Helpers -----------------------------------*/

inline bool
v_n(string& s, int& v, int& n) {
  if(sscanf(s.c_str(), "%d//%d", &v, &n) == 2) {
    v--; n--;
    return true;
  }
  return false;
}

inline bool
vtn(string& s, int& v, int& t, int& n) {
  if(sscanf(s.c_str(), "%d/%d/%d", &v, &t, &n) == 3) {
    v--; t--; n--;
    return true;
  }
  return false;
}

inline bool
vt_(string& s, int& v, int& t) {
  if(sscanf(s.c_str(), "%d/%d", &v, &t) == 2) {
    v--; t--;
    return true;
  }
  return false;
}

inline bool
v__(string& s, int& v) {
  if(sscanf(s.c_str(), "%d", &v) == 1) {
    v--;
    return true;
  }
  return false;
}

/*----------------------------- CObjLoader -----------------------------------*/

bool
CObjLoader::
ParseFile(bool _silent) {
  if(m_filename == "")
    return false;

  ifstream fin(m_filename);
  if(!fin.good()) {
    if(!_silent)
      cerr << "CObjLoader::File " << m_filename << " not found" << endl;
    return false;
  }

  return ReadOBJ(fin);
}


bool
CObjLoader::
ReadOBJ(istream& _in) {
  if(!FirstPass(_in))
    return false;
  _in.clear();
  _in.seekg(0, ios::beg);
  if(!SecondPass(_in))
    return false;
  if(m_normals.empty())
    ComputeFaceNormal();
  return true;
}


bool
CObjLoader::
FirstPass(istream& _in) {
  unsigned int numvertices = 0;  // number of vertices in model
  unsigned int numnormals = 0;   // number of normals in model
  unsigned int numtexcoords = 0; // number of texcoords in model
  unsigned int numtriP = 0;      // number of triP in model
  unsigned int numtriN = 0;      // number of triP in model
  unsigned int numtriT = 0;      // number of triP in model

  while(true) {
    char c1;
    _in >> c1;
    if(_in.eof())
      break;

    string buf;
    switch(c1) {
      case '#': /* comment */
        getline(_in, buf); // eat up rest of line
        break;
      case 'v': /* v, vn, vt */
        {
          char c2;
          _in.get(c2);
          switch(c2) {
            case ' ': /* vertex */
              getline(_in, buf);
              numvertices++;
              break;
            case 'n': /* normal */
              getline(_in, buf);
              numnormals++;
              break;
            case 't': /* texcoord */
              getline(_in, buf);
              numtexcoords++;
              break;
            default:
              cerr << "Error: FirstPass: Unknown token '" << buf << "'" << endl;
              return false;
          }
          break;
        }
      case 'm': /* ? */
        _in >> buf >> buf;
        getline(_in, buf);
        //model->mtllibname = strdup(buf);
        //glmReadMTL(model, buf);
        break;
      case 'u': /* material? */
        getline(_in, buf);
        break;
      case 'g': /* group */
        getline(_in, buf);
        break;
      case 'f': /* face */
        {
          string junk;
          int v, t, n;
          _in >> junk >> junk >> junk;
          if(v_n(junk, v, n))
            numtriN++;
          else if(vtn(junk, v, t, n)) {
            numtriN++;
            numtriT++;
          }
          else if(vt_(junk, v, t))
            numtriT++;
          numtriP++;
        }
        getline(_in, buf);
        break;
      default:
        getline(_in, buf);
        break;
    }
  }

  m_points.reserve(numvertices);
  m_textures.reserve(numtexcoords);
  m_normals.reserve(numnormals);
  m_triP.reserve(numtriP);
  m_triN.reserve(numtriN);
  m_triT.reserve(numtriT);
  m_cgalPoints.reserve(numvertices);
  return true;
}


bool
CObjLoader::
SecondPass(istream& _in) {
  while(true) {
    char c1;
    _in >> c1;
    if(_in.eof())
      break;

    string buf;
    switch(c1) {
      case '#': /* comment */
        getline(_in, buf);
        break;
      case 'v': /* v, vn, vt */
        {
          char c2;
          _in.get(c2);
          switch(c2) {
            case ' ': /* vertex */
              {
                CGALPoint cp;
                _in >> cp;
                m_cgalPoints.push_back(cp);
                using CGAL::to_double;
                m_points.emplace_back(to_double(cp[0]), to_double(cp[1]),
                    to_double(cp[2]));
                break;
              }
            case 'n': /* normal */
              {
                Vector3d n;
                _in >> n;
                m_normals.push_back(n);
                break;
              }
            case 't': /* texcoord */
              {
                Vector2d t;
                _in >> t;
                m_textures.push_back(t);
                break;
              }
          }
        }
        break;
      case 'u': /* material? */
        _in >> buf >> buf;
        getline(_in, buf);
        break;
      case 'g': /* group */
        getline(_in, buf);
        break;
      case 'f': /* triangles */
        {
          _in >> buf;
          Tri tv, tn, tt;
          // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
          if(v_n(buf, tv[0], tn[0])) { // v//n
            _in >> buf;
            v_n(buf, tv[1], tn[1]);
            _in >> buf;
            v_n(buf, tv[2], tn[2]);
            m_triP.push_back(tv);
            m_triN.push_back(tn);
          }
          else if(vtn(buf, tv[0], tt[0], tn[0])) { // v/t/n
            _in >> buf;
            vtn(buf, tv[1], tt[1], tn[1]);
            _in >> buf;
            vtn(buf, tv[2], tt[2], tn[2]);
            m_triP.push_back(tv);
            m_triT.push_back(tt);
            m_triN.push_back(tn);
          }
          else if(vt_(buf,tv[0],tt[0])) { // v/t
            _in >> buf;
            vt_(buf, tv[1], tt[1]);
            _in >> buf;
            vt_(buf, tv[2], tt[2]);
            m_triP.push_back(tv);
            m_triT.push_back(tt);
          }
          else { // v
            v__(buf,tv[0]);
            _in >> buf;
            v__(buf, tv[1]);
            _in >> buf;
            v__(buf, tv[2]);
            m_triP.push_back(tv);
          }
        }
        break;
      default:
        getline(_in, buf);
        break;
    }
  }

  return true;
}

/*------------------- Unfinished code for groups and materials ---------------*/

//CObjGroup&
//CObjLoader::
//FindGroup(const string& _name) {
//  typedef vector<CObjGroup>::iterator GIT;
//  for(GIT i=groups.begin();i!=groups.end();i++)
//    if(i->name == _name) return *i;
//  return *groups.end();
//}
//
//
//CObjGroup&
//CObjLoader::
//AddGroup(const string& _name) {
//  CObjGroup& group = FindGroup(_name);
//  if(&group == groups.end()) {
//    CObjGroup g;
//    g.name = _name;
//    g.material = 0;
//    groups.push_back(g);
//    return groups.back();
//  }
//  else
//    return group;
//}

/*----------------------------------------------------------------------------*/
