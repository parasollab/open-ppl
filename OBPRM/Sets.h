// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Sets.h
//
//  General Description
//      This set of template classes provides an implementation for
//      maintaining a universe of *unique* elements, and forming
//      sets of elements from the universe. Capabilities for both:
//        o ordered sets (duplicate elements allowed)
//        o unordered sets (duplicate elements not allowed)
//
//      Both elements and sets are assigned unique ids (short ints).
//  
//      NOTE: elements can be added, but *not* deleted from the universe,
//            sets can be created/deleted/modified. 
//
//      The user must provide the parameter type ELEMENT, which is
//      the data that will be stored in each element of the universe.
//      It is assumed the ELEMENT class has the following operators
//      defined for it: <<  ==
//
//      The classes in this hierarchy (so far) include:
//        o BasicSets 
//
//
//  Created
//      8/5/98  Nancy Amato
/////////////////////////////////////////////////////////////////////

#ifndef Sets_h
#define Sets_h

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>

#include <function.h>

#include <algo.h>
#include <list.h>
#include <vector.h>
#include <bvector.h>
#include <multimap.h>

#include "BasicDefns.h"

/////////////////////////////////////////////////////////////////////
//  class BasicSets<ELEMENT>
//
//  General Description
//      Base class for universe and sets of ELEMENTS 
/////////////////////////////////////////////////////////////////////

template <class ELEMENT>
class BasicSets {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    BasicSets();
    BasicSets(const ELEMENT);          // initial universe elt, no sets made
    BasicSets(const vector<ELEMENT>);  // initial universe elts, no sets made
    ~BasicSets();

  //===================================================================
  //  Other Methods
  //===================================================================

        // Querying Elements of Universe
   bool IsMember(const SID _sid) const;

        // Adding Elements to Universe (they are never deleted)
   virtual EID AddElementToUniverse(const ELEMENT _e); 
   virtual EID AddElementToUniverse(const vector<ELEMENT> _evec); 
   virtual int ChangeElementInfo(EID _eid, ELEMENT _e); 

        // Making, Modifying & Deleting Sets of Elements
                   // UNORDERED SETS
   virtual SID MakeSet(const EID _eid);
   virtual SID MakeSet(const ELEMENT& _e);
   virtual SID MakeSet(const vector<EID>& _eidvector );
   virtual SID MakeSet(const vector<ELEMENT>& _evector );
   virtual int AddElementToSet(const SID _sid, const EID _eid );
   virtual int AddElementToSet(const SID _sid, const ELEMENT& _e );
   virtual int DeleteElementFromSet(const SID _sid, const EID _eid );
   virtual int DeleteElementFromSet(const SID _sid, const ELEMENT& _e );
   virtual SID MergeSets(const SID _s1id, const SID _s2id );
   virtual int DeleteSet(const SID _sid );
    
                   // ORDERED SETS
   virtual SID MakeOSet(const EID _eid);
   virtual SID MakeOSet(const ELEMENT& _e);
   virtual SID MakeOSet(const vector<EID>& _eidvector );
   virtual SID MakeOSet(const vector<ELEMENT>& _evector );
   virtual int AddElementToOSet(const SID _sid, const EID _eid );
   virtual int AddElementToOSet(const SID _sid, const ELEMENT& _e );
   virtual int DeleteElementFromOSet(const SID _sid, const EID _eid );
   virtual int DeleteElementFromOSet(const SID _sid, const ELEMENT& _e );
   virtual SID SpliceOSets(const SID _s1id, const SID _s2id );
   virtual int DeleteOSet(const SID _sid );


        // Getting Data & Statistics
   virtual vector< pair<EID,ELEMENT> > GetElements() const;    // eid is index
   virtual ELEMENT GetElement(const EID _eid) const; 
   virtual EID GetElementID(const ELEMENT& _e) const; 
   virtual vector<pair<SID,vector<pair<EID,ELEMENT> > > > GetSets() const; // (sid,set) pairs
   virtual vector<pair<SID,vector<pair<EID,ELEMENT> > > > GetOSets() const; // (sid,set) pairs
   virtual vector< pair<EID,ELEMENT> > GetSet(const SID _sid) const; 
   virtual vector< pair<EID,ELEMENT> > GetOSet(const SID _sid) const; 

        // Display, Input & Output
   virtual void DisplayElements() const;    
   virtual void DisplayElement(const EID _eid) const;    
   virtual void DisplaySets() const; 
   virtual void DisplayOSets() const; 
   virtual void DisplaySet(const SID _sid) const; 
   virtual void DisplayOSet(const SID _sid) const; 

   
protected:
  //===================================================================
  //  Utility Stuff for Basic Sets
  //===================================================================
   typedef  vector< pair<EID,ELEMENT> >::iterator EI;
   typedef  vector< pair<EID,ELEMENT> >::const_iterator CEI;
   typedef  vector< pair<SID,bit_vector> >::iterator SI;
   typedef  vector< pair<SID,bit_vector> >::const_iterator CSI;
   typedef  vector< pair<SID,vector<EID> > >::iterator OSI;
   typedef  vector< pair<SID,vector<EID> > >::const_iterator COSI;

        // Predicates, Comparisons, & Operations
   pair<EID,ELEMENT>* my_find_ELEM_eq(const ELEMENT& _e) const;
   pair<EID,ELEMENT>* my_find_EID_eq(const EID _eid) const;
   pair<SID,bit_vector>* my_find_SID_eq(const SID _sid) const;
   pair<SID,vector<EID> >* my_find_OSID_eq(const SID _sid) const;

  //===================================================================
  // Data
  //===================================================================
    EID ElementIDs;
    SID SetIDs;
    vector< pair<EID,ELEMENT> > elements;
    vector< pair<SID,bit_vector> > sets; // (id,set) pairs, unordered
    vector< pair< SID,vector<EID> > > osets; // (id,set) pairs, ordered
};



//=====================================================================
// METHODS FOR template BasicSets Class
//=====================================================================
  //==================================
  // BasicSets class Methods: Constructors and Destructor
  //==================================

template <class ELEMENT>
BasicSets<ELEMENT>::
BasicSets(){
   ElementIDs = 0;
   SetIDs = 0;
};

template <class ELEMENT>
BasicSets<ELEMENT>::
BasicSets(const ELEMENT _e){
   ElementIDs = 0;
   SetIDs = 0;
   AddElementToUniverse(_e);
};

template <class ELEMENT>
BasicSets<ELEMENT>::
BasicSets(const vector<ELEMENT> _evec){
   ElementIDs = 0;
   SetIDs = 0;
   AddElementToUniverse(_evec);
};

template <class ELEMENT>
BasicSets<ELEMENT>::
~BasicSets(){
};

  //===================================================================
  // BasicSets class Methods: Querying Elements of Universe
  //===================================================================

template <class ELEMENT>
bool
BasicSets<ELEMENT>::
IsMember(const SID _sid) const {

   COSI s = my_find_OSID_eq(_sid);

   if ( s != osets.end() ) {
	return true;
   } else {
	return false;
   }
}



  //===================================================================
  // BasicSets class Methods: Adding Elements To Universe
  //===================================================================

template <class ELEMENT>
EID 
BasicSets<ELEMENT>::
AddElementToUniverse(const ELEMENT _e){

   EI ei = my_find_ELEM_eq(_e);

   if ( ei != elements.end() ) {
      return ei->first;
   } else {
      elements.push_back( pair<EID,ELEMENT>(ElementIDs,_e) );
      return ElementIDs++;
   }
};

template <class ELEMENT>
EID 
BasicSets<ELEMENT>::
AddElementToUniverse(const vector<ELEMENT> _evec){

   EID eid;

   for (int i=0; i < _evec.size(); i++) {
     eid = AddElementToUniverse( _evec[i] ); 
   }
   return eid; // return eid of last element in the vector
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
ChangeElementInfo(EID _eid, ELEMENT _e){

   EI ei = my_find_EID_eq(_eid);

   if ( ei != elements.end() ) {
      ei->second = _e;
      return OK;
   } else {
      cout << endl << "In ChangeElementInfo: element not here to change.";
      return ERROR;
   }
};


  //===================================================================
  //            UN-ordered Sets 
  // BasicSets class Methods: Making, Modifying & Deleting Sets of Elements
  //===================================================================

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeSet( const EID _eid ) {

   //CEI ei = find_if(elements.begin(), elements.end(), EID_eq(_eid) );
   CEI ei = my_find_EID_eq(_eid);
   if ( ei != elements.end() ) {
      bit_vector newset(ElementIDs,false);
      newset[_eid] = true;
   
      sets.push_back( pair<SID,bit_vector>(SetIDs,newset) );
      return SetIDs++;
   } else {
      cout << endl << "In MakeSet: element id=" << _eid << " not present";
      return INVALID_SID;
   }
}

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeSet( const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return MakeSet(eid);
}

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeSet( const vector<EID>& _eidvector ) {
   typedef  vector<EID>::const_iterator CEIDI; 

   bool added = false;
   bit_vector newset(ElementIDs,false);

   for ( CEIDI eid = _eidvector.begin(); eid != _eidvector.end(); eid++ ) {
      CEI ei = my_find_EID_eq(*eid);
      if ( ei != elements.end() ) {
         newset[*eid] = true;
         added = true;
      } else {
         cout << endl << "In MakeSet: element id=" << *eid << " not present";
      }
   }
   
   if ( added ) {  // new set not empty
      sets.push_back( pair<SID,bit_vector>(SetIDs,newset) );
      return SetIDs++;
   } else {
      return INVALID_SID;
   }  
}

template <class ELEMENT>
SID
BasicSets<ELEMENT>::
MakeSet( const vector<ELEMENT>& _evector ) {
   typedef  vector<ELEMENT>::const_iterator CELI;

   bool added = false;
   bit_vector newset(ElementIDs,false);

   for ( CELI e = _evector.begin(); e != _evector.end(); e++ ) {
      EI ei = my_find_ELEM_eq(*e);
      if ( ei != elements.end() ) {
         newset[ei->first] = true;
         added = true;
      } else {
         cout << endl << "In MakeSet: element =" << *e << " not present";
      }
   }

   if ( added ) {  // new set not empty
      sets.push_back( pair<SID,bit_vector>(SetIDs,newset) );
      return SetIDs++;
   } else {
      return INVALID_SID;
   }
}



template <class ELEMENT>
int
BasicSets<ELEMENT>::
AddElementToSet( const SID _sid, const EID _eid ) {

   CEI e = my_find_EID_eq(_eid);
   SI s = my_find_SID_eq(_sid);

   if ( e != elements.end() && s != sets.end() ) {
      while (s->second.size() != ElementIDs) {
         s->second.push_back(false); 
      }
      s->second[_eid] = true;
      return OK;
   } else {
      cout << endl << "In AddElementToSet:";
      cout << " set=" << _sid << "or element=" << _eid << " doesn't exist";
      return ERROR;
   } 
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
AddElementToSet( const SID _sid, const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return AddElementToSet(_sid,eid);
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteElementFromSet( const SID _sid, const EID _eid ) {

   CEI e = my_find_EID_eq(_eid);
   SI s = my_find_SID_eq(_sid);

   if ( e != elements.end() && s != sets.end() ) {
      while ( s->second.size() != ElementIDs ) {
        s->second.push_back(false);
      }
      s->second[_eid] = false;
      return OK;
   } else {
      cout << endl << "In DeleteElementFromSet:";
      cout << " set=" << _sid << "or element=" << _eid << " doesn't exist";
      return ERROR;
   } 
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteElementFromSet( const SID _sid, const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return DeleteElementFromSet(_sid,eid);
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteSet(const SID _sid ){

   SI s = my_find_SID_eq(_sid);

   if ( s != sets.end() ) {
      sets.erase(s);
      return OK;
   } else {
      cout << endl << "In DeleteSet: set=" << _sid << " doesn't exist";
      return ERROR;
   }
}

// actually make a new set which is union, don't delete originals
template <class ELEMENT>
SID
BasicSets<ELEMENT>::
MergeSets(const SID _s1id, const SID _s2id ){

   SI s1 = my_find_SID_eq(_s1id);
   SI s2 = my_find_SID_eq(_s2id);

   if ( s1 != sets.end() && s2 != sets.end() ) {
      while (s1->second.size() != ElementIDs) 
         s1->second.push_back(false); 
      while (s2->second.size() != ElementIDs) 
         s2->second.push_back(false);
      bit_vector newset(ElementIDs,false); 
      for (int i=0; i < ElementIDs; i++ ) {
         newset[i] = s1->second[i] || s2->second[i];
      }
      sets.push_back( pair<SID,bit_vector >(SetIDs,newset) );
      return SetIDs++; 
   } else {
      cout << endl << "In MergeSets: set1 or set1 doesn't exist";
      return ERROR;
   }
}

  //===================================================================
  //            ordered Sets 
  // BasicSets class Methods: Making, Modifying & Deleting Sets of Elements
  //===================================================================

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeOSet( const EID _eid ) {

   CEI ei = my_find_EID_eq(_eid);
   if ( ei != elements.end() ) {
      vector<EID> newset;
      newset.push_back(_eid); 
   
      osets.push_back( pair<SID,vector<EID> >(SetIDs,newset) );
      return SetIDs++;
   } else {
      cout << endl << "In MakeOSet: element id=" << _eid << " not present";
      return INVALID_SID;
   }
}

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeOSet( const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return MakeOSet(eid);
}

template <class ELEMENT>
SID 
BasicSets<ELEMENT>::
MakeOSet( const vector<EID>& _eidvector ) {
   typedef  vector<EID>::const_iterator CEIDI; 

   bool added = false;
   vector<EID> newset;

   for ( CEIDI eid = _eidvector.begin(); eid != _eidvector.end(); eid++ ) {
      CEI ei = my_find_EID_eq(*eid);
      if ( ei != elements.end() ) {
         newset.push_back(*eid);
         added = true;
      } else {
         cout << endl << "In MakeOSet: element id=" << *eid << " not present";
      }
   }
   
   if ( added ) {  // new set not empty
      osets.push_back( pair<SID,vector<EID> >(SetIDs,newset) );
      return SetIDs++;
   } else {
      return INVALID_SID;
   }  
}

template <class ELEMENT>
SID
BasicSets<ELEMENT>::
MakeOSet( const vector<ELEMENT>& _evector ) {
   typedef  vector<ELEMENT>::const_iterator CELI;

   bool added = false;
   vector<EID> newset;

   for ( CELI e = _evector.begin(); e != _evector.end(); e++ ) {
      EI ei = my_find_ELEM_eq(*e);

      if ( ei != elements.end() ) {
         newset.push_back(ei->first);
         added = true;
      } else {
         cout << endl << "In MakeOSet: element =" << *e << " not present";
      }
   }

   if ( added ) {  // new set not empty
      osets.push_back( pair<SID,vector<EID> >(SetIDs,newset) );
      return SetIDs++;
   } else {
      return INVALID_SID;
   }
}


// append to end
template <class ELEMENT>
int
BasicSets<ELEMENT>::
AddElementToOSet( const SID _sid, const EID _eid ) {

   CEI e = my_find_EID_eq(_eid);
   OSI s = my_find_OSID_eq(_sid);

   if ( e != elements.end() && s != osets.end() ) {
      s->second.push_back(_eid);
      return OK;
   } else {
      cout << endl << "In AddElementToOSet:";
      cout << " set=" << _sid << "or element=" << _eid << " doesn't exist";
      return ERROR;
   } 
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
AddElementToOSet( const SID _sid, const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return AddElementToOSet(_sid,eid);
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteElementFromOSet( const SID _sid, const EID _eid ) {
   typedef  vector<EID>::iterator EIDI;

   CEI e = my_find_EID_eq(_eid);
   OSI s = my_find_OSID_eq(_sid);

   if ( e != elements.end() && s != osets.end() ) {
       EIDI ei = find(s->second.begin(), s->second.end(), _eid );
      s->second.erase(ei);
      return OK;
   } else {
      cout << endl << "In DeleteElementFromOSet:";
      cout << " set=" << _sid << "or element=" << _eid << " doesn't exist";
      return ERROR;
   } 
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteElementFromOSet( const SID _sid, const ELEMENT& _e ) {
   EID eid = GetElementID(_e);
   return DeleteElementFromOSet(_sid,eid);
}

template <class ELEMENT>
int
BasicSets<ELEMENT>::
DeleteOSet(const SID _sid ){

   OSI s = my_find_OSID_eq(_sid);

   if ( s != osets.end() ) {
      osets.erase(s);
      return OK;
   } else {
      cout << endl << "In DeleteOSet: set=" << _sid << " doesn't exist";
      return ERROR;
   }
}

// actually make a new set which is union, don't delete originals
template <class ELEMENT>
SID
BasicSets<ELEMENT>::
SpliceOSets(const SID _s1id, const SID _s2id ){

   OSI s1 = my_find_OSID_eq(_s1id);
   OSI s2 = my_find_OSID_eq(_s2id);

   if ( s1 != osets.end() && s2 != osets.end() ) {
      vector<EID> newset = s1->second; 
      for (int i=0; i < s2->second.size(); i++ ) {
         newset.push_back(s2->second[i]);
      }
      osets.push_back( pair<SID,vector<EID> >(SetIDs,newset) );
      return SetIDs++; 
   } else {
      cout << endl << "In SpliceOSets: set1 or set2 doesn't exist";
      return ERROR;
   }
}

  //===================================================================
  // BasicSets class Methods: Getting Data & Statistics
  //===================================================================

template <class ELEMENT>
vector< pair<EID,ELEMENT> > 
BasicSets<ELEMENT>::
GetElements() const {
   vector< pair<EID,ELEMENT> > el;

   for (CEI ei = elements.begin(); ei != elements.end(); ei++ ) {
      el.push_back( *ei);
   }
   return el;
}

template <class ELEMENT>
ELEMENT
BasicSets<ELEMENT>::
GetElement(const EID _eid) const {
   CEI e = my_find_EID_eq(_eid);
   return e->second;
}

template <class ELEMENT>
EID
BasicSets<ELEMENT>::
GetElementID(const ELEMENT& _e) const {
   CEI e = my_find_ELEM_eq(_e);

   if ( e != elements.end() ) {
      return e->first;
   } else { 
      return INVALID_EID;
   }
}

template <class ELEMENT>
vector< pair<SID,vector< pair<EID,ELEMENT> > > > 
BasicSets<ELEMENT>::
GetSets() const {
   vector< pair<SID,vector< pair<EID,ELEMENT> > > > mysets;
   vector< pair<EID,ELEMENT> > thisset;

   for (CSI s = sets.begin(); s != sets.end(); s++ ) {
      thisset = GetSet(s->first);
      mysets.push_back(pair<SID,vector<pair<EID,ELEMENT> > >(s->first,thisset));
   }
    
   return mysets;
}

template <class ELEMENT>
vector< pair<SID,vector< pair<EID,ELEMENT> > > >
BasicSets<ELEMENT>::
GetOSets() const {
   vector< pair<SID,vector< pair<EID,ELEMENT> > > > mysets;
   vector< pair<EID,ELEMENT> > thisset;

   for (COSI s = osets.begin(); s != osets.end(); s++ ) {
      thisset = GetOSet(s->first);
      mysets.push_back(pair<SID,vector<pair<EID,ELEMENT> > >(s->first,thisset));
   }

   return mysets;
}


template <class ELEMENT>
vector< pair<EID,ELEMENT> >
BasicSets<ELEMENT>::
GetSet(const SID _sid) const {
   vector< pair<EID,ELEMENT> > myset;

   CSI s = my_find_SID_eq(_sid);

   if ( s != sets.end() ) {
      for (int i = 0; i < s->second.size(); i++) {
         if ( s->second[i] ) {
            //CEI e = find_if(elements.begin(),elements.end(),EID_eq(i) );   
            CEI e = my_find_EID_eq(i);
            myset.push_back( *e );
         }
      }
   } else {
      cout << endl << "In GetSet: set id=" << _sid << " not present";
   } 
   return myset;
}

template <class ELEMENT>
vector< pair<EID,ELEMENT> >
BasicSets<ELEMENT>::
GetOSet(const SID _sid) const {
   vector< pair<EID,ELEMENT> > myset;

   COSI s = my_find_OSID_eq(_sid);

   if ( s != osets.end() ) {
      for (int i = 0; i < s->second.size(); i++) {
         CEI e = my_find_EID_eq(s->second[i]);
         myset.push_back( *e );
      }
   } else {
      cout << endl << "In GetOSet: set id=" << _sid << " not present";
   }
   return myset;
}


  //===================================================================
  // BasicSets class Methods: Display, Input & Output
  //===================================================================

template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplayElements() const {

   cout << endl << "The elements in the universe are:";
   for (int i=0; i < elements.size(); i++) {
      cout << endl << " ";
      DisplayElement( elements[i].first ); 
   }   
}

template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplayElement(const EID _eid) const {
   cout << "Element id=" << _eid << ", " << GetElement(_eid);
}

template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplaySets() const {
   cout << endl << "The current sets are:";
   for (int i=0; i < sets.size(); i++) {
      cout << endl << " ";
      DisplaySet( sets[i].first ); 
   }   
}

template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplayOSets() const {
   cout << endl << "The current sets are:";
   for (int i=0; i < osets.size(); i++) {
      cout << endl << " ";
      DisplayOSet( osets[i].first );
   }
}


template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplaySet(const SID _sid) const {

   vector< pair<EID,ELEMENT> > myset = GetSet(_sid);

   cout << endl << "Set id=" << _sid << ":";
   for (int i=0; i < myset.size(); i++) {
       cout << endl << "  ";
       DisplayElement( myset[i].first );
   }

   return;
}

template <class ELEMENT>
void
BasicSets<ELEMENT>::
DisplayOSet(const SID _sid) const {

   vector< pair<EID,ELEMENT> > myset = GetOSet(_sid);

   cout << endl << "Set id=" << _sid << ":";
   for (int i=0; i < myset.size(); i++) {
       cout << endl << "  ";
       DisplayElement( myset[i].first );
   }

   return;
}


  //===================================================================
  // BasicSets class Methods Predicates
  //===================================================================

template<class ELEMENT>
pair<EID,ELEMENT>*
BasicSets<ELEMENT>::
my_find_ELEM_eq(const ELEMENT& _e) const {
   CEI ei = elements.begin();
   bool found = false;
   while (ei != elements.end() && !found) {
      if ( ei->second == _e) {
         found = true;
      } else {
         ei++;
      } 
   }
   return const_cast<pair<EID,ELEMENT>*>(ei);
}

template<class ELEMENT>
pair<EID,ELEMENT>*
BasicSets<ELEMENT>::
my_find_EID_eq(const EID _eid) const {
   CEI ei = elements.begin();
   bool found = false;
   while (ei != elements.end() && !found) {
      if ( ei->first == _eid) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<pair<EID,ELEMENT>*>(ei);
}

template<class ELEMENT>
pair<SID,bit_vector>*
BasicSets<ELEMENT>::
my_find_SID_eq(const SID _sid) const {
   CSI si = sets.begin();
   bool found = false;
   while (si != sets.end() && !found) {
      if ( si->first == _sid) {
         found = true;
      } else {
         si++;
      }
   }
   return const_cast<pair<SID,bit_vector>*>(si);
}

template<class ELEMENT>
pair<SID,vector<EID> >*
BasicSets<ELEMENT>::
my_find_OSID_eq(const SID _sid) const {
   COSI si = osets.begin();
   bool found = false;
   while (si != osets.end() && !found) {
      if ( si->first == _sid) {
         found = true;
      } else {
         si++;
      }
   }
   return const_cast<pair<SID,vector<EID> >*>(si);
}

#endif
