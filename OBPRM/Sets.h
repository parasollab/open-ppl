// $Id$

/////////////////////////////////////////////////////////////////////
/**@file Sets.h
   General Description
      This set of template classes provides an implementation for
      maintaining a universe of *unique* elements, and forming
      sets of elements from the universe. Capabilities for both:
        o ordered sets (duplicate elements allowed)
        o unordered sets (duplicate elements not allowed)

      Both elements and sets are assigned unique ids (short ints).
  
      NOTE: elements can be added, but *not* deleted from the universe,
            sets can be created/deleted/modified. 

      The user must provide the parameter type ELEMENT, which is
      the data that will be stored in each element of the universe.
      It is assumed the ELEMENT class has the following operators
      defined for it: <<  ==

      The classes in this hierarchy (so far) include:
        o BasicSets 

   @author Nancy Amato
   @date 8/5/98
*/
/////////////////////////////////////////////////////////////////////

#ifndef Sets_h
#define Sets_h

////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers
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
#include <bvector.h> //for bit_vector
#include <multimap.h>

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "BasicDefns.h"
#include "OBPRM.h"


template <class ELEMENT>
class BasicSets {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
	//@{

	/**Constructor. 
	  *Create a empty set.
	  */
    BasicSets();

	/**Constructor and initial universe with an element.
	  *Create a empty set.
	  *@param ELEMENT this given element will be add to universe.
	  *@see AddElementToUniverse
	  */
    BasicSets(const ELEMENT);

    /**Constructor and initial universe with elements.
	  *Create a empty set.
	  *@param vector<ELEMENT> these given elements will be add to universe.
	  *@see AddElementToUniverse
	  */
    BasicSets(const vector<ELEMENT>);

	/**Destructor. Do nothing.
	  */
    ~BasicSets();

	//@}

    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	UnORDERED SETS
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name UnOrdered Set Methods.
	  *Making, Modifying & Deleting Sets of Elements.
      */
	//@{

	   /**Make a new unordered set using element labled _eid.
	     *There is only one element in this new set which is 
		 *specified by _eid.
	     *@param _eid The id for element in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned if no element with _eid found in universe.
	     */
	   virtual SID MakeSet(const EID _eid);

	   /**Make a new unordered set using element of user data, _e.
	     *There is only one element in this new set which is 
		 *specified by _e, the user data contained in this element.
	     *@param _e The user data for element in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned if no element of _e found in universe.
	     */
	   virtual SID MakeSet(const ELEMENT& _e);

	   /**Make a new unordered set using elements labled ids in given list.
	     *Elemets with EID contanied in _eidvector will be added to this new 
		 *set.
		 *
	     *@param _eidvector A list of ids for elements in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned (Only if _eidvector is empty)
		 *@note If any EID in _eidvector is not found in universe, error message will be post
		 *on standard ouput, but no INVALID_SID will be returned. New unordered set will still
		 *be created if this set is not empty.
	     */
	   virtual SID MakeSet(const vector<EID>& _eidvector );

	   /**Make a new unordered set using elements of user data in given list.
	     *Elemets has user data contanied in _eidvector will be added to this new 
		 *set.
		 *
	     *@param _evector A list of user data for elements in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned (Only if _eidvector is empty)
		 *@note If any element is not found in universe, error message will be post
		 *on standard ouput, but no INVALID_SID will be returned. New unordered set will still
		 *be created if this set is not empty.
	     */
	   virtual SID MakeSet(const vector<ELEMENT>& _evector );


	   /**Add element labled with _eid to unordered set labled with _sid.
	     *Both _eid and _sid should be found in universe.
		 *
		 *@param _eid ID for element in universe.
		 *@param _sid ID for unordered set in universe.
		 *
		 *@return ERROR if there is not element been added. 
		 *OK if every thing is fine.
	     */
	   virtual int AddElementToSet(const SID _sid, const EID _eid );

	   /**Add element labled of user data, _e, to unordered set labled with _sid.
	     *Both _e and _sid should be found in universe.
		 *
		 *@param _e User data of element in universe.
		 *@param _sid ID for unordered set in universe.
		 *
		 *@return ERROR if there is not element been added. 
		 *OK if every thing is fine.
	     */
	   virtual int AddElementToSet(const SID _sid, const ELEMENT& _e );

	   /**Remove element labled with _eid to unordered set labled with _sid.
	     *Both _eid and _sid should be found in universe.
		 *
		 *@param _eid ID for element in universe.
		 *@param _sid ID for unordered set in universe.
		 *
		 *@return ERROR if there is not element been removed.
		 *OK if every thing is fine.
	     */
	   virtual int DeleteElementFromSet(const SID _sid, const EID _eid );

	   /**Remove element labled of user data, _e, to unordered 
	     *set labled with _sid.
	     *Both _e and _sid should be found in universe.
		 *
		 *@param _e User data of element in universe.
		 *@param _sid ID for unordered set in universe.
		 *
		 *@return ERROR if there is not element been removed.
		 *OK if every thing is fine.
	     */
	   virtual int DeleteElementFromSet(const SID _sid, const ELEMENT& _e );

	   /**Make a unordered new set which is union, don't delete originals.
         *@return ERROR if _s1id and/or _s2id are not found in universe.
		 *Otherwise a new ID for this set will be returned.
	     */
	   virtual SID MergeSets(const SID _s1id, const SID _s2id );

	   /**Delete a unordered set.
	     *@return ERROR if there is no set labled with _sid. OK if
		 *this specified set was removed from universe.
	     */
	   virtual int DeleteSet(const SID _sid );
   
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	ORDERED SETS
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Ordered Set Methods.
	  *Making, Modifying & Deleting Sets of Elements.
      */
	//@{

	   /**Make a new ordered set using element labled _eid.
	     *There is only one element in this new set which is 
		 *specified by _eid.
	     *@param _eid The id for element in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned if no element with _eid found in universe.
	     */
	   virtual SID MakeOSet(const EID _eid);

	   /**Make a new ordered set using element of user data, _e.
	     *There is only one element in this new set which is 
		 *specified by _e, the user data contained in this element.
	     *@param _e The user data for element in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned if no element of _e found in universe.
	     */
	   virtual SID MakeOSet(const ELEMENT& _e);

	   /**Make a new ordered set using elements labled ids in given list.
	     *Elemets with EID contanied in _eidvector will be added to this new 
		 *set.
		 *
	     *@param _eidvector A list of ids for elements in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned (Only if _eidvector is empty)
		 *@note If any EID in _eidvector is not found in universe, error message will be post
		 *on standard ouput, but no INVALID_SID will be returned. New ordered set will still
		 *be created if this set is not empty.
	     */
	   virtual SID MakeOSet(const vector<EID>& _eidvector );

	   /**Make a new ordered set using elements of user data in given list.
	     *Elemets has user data contanied in _eidvector will be added to this new 
		 *set.
		 *
	     *@param _evector A list of user data for elements in universe.
		 *
		 *@return If this new set was created then SID for this set will be returned.
		 *Otherwise, INVALID_SID will be returned (Only if _eidvector is empty)
		 *@note If any element is not found in universe, error message will be post
		 *on standard ouput, but no INVALID_SID will be returned. New ordered set will still
		 *be created if this set is not empty.
	     */
	   virtual SID MakeOSet(const vector<ELEMENT>& _evector );

	   /**Add element labled with _eid to ordered set labled with _sid.
	     *Both _eid and _sid should be found in universe.
		 *
		 *@param _eid ID for element in universe.
		 *@param _sid ID for ordered set in universe.
		 *
		 *@return ERROR if there is not element been added. 
		 *OK if every thing is fine.
		 *@note it is possible that more than one identical elements
		 *will be added to this set...be aware.
	     */
	   virtual int AddElementToOSet(const SID _sid, const EID _eid );

	   /**Add element labled of user data, _e, to ordered set labled with _sid.
	     *Both _e and _sid should be found in universe.
		 *
		 *@param _e User data of element in universe.
		 *@param _sid ID for ordered set in universe.
		 *
		 *@return ERROR if there is not element been added. 
		 *OK if every thing is fine.
		 *@note it is possible that more than one identical elements
		 *will be added to this set...be aware of this.
	     */
	   virtual int AddElementToOSet(const SID _sid, const ELEMENT& _e );

	   /**Remove element labled with _eid to ordered set labled with _sid.
	     *Both _eid and _sid should be found in universe.
		 *
		 *@param _eid ID for element in universe.
		 *@param _sid ID for ordered set in universe.
		 *
		 *@return ERROR if there is not element been removed.
		 *OK if every thing is fine.
	     */
	   virtual int DeleteElementFromOSet(const SID _sid, const EID _eid );

	   /**Remove element labled of user data, _e, to ordered 
	     *set labled with _sid.
	     *Both _e and _sid should be found in universe.
		 *
		 *@param _e User data of element in universe.
		 *@param _sid ID for ordered set in universe.
		 *
		 *@return ERROR if there is not element been removed.
		 *OK if every thing is fine.
	     */
	   virtual int DeleteElementFromOSet(const SID _sid, const ELEMENT& _e );

	   /**Make a ordered new set which is combination of _s1id and _s2id
	     *, don't delete originals.
		 *
         *@return ERROR if _s1id and/or _s2id are not found in universe.
		 *Otherwise a new ID for this set will be returned.
		 *@note this new set might have duplicate elements.
	     */
	   virtual SID SpliceOSets(const SID _s1id, const SID _s2id );

	   /**Delete a ordered set.
	     *@return ERROR if there is no set labled with _sid. OK if
		 *this specified set was removed from universe.
	     */
	   virtual int DeleteOSet(const SID _sid );

     //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods. 
      *Use these method to acess or change internal state
      */
	//@{

	   /**Get all elements in universe.
	     *@return a list of elements with id and user data.
	     */
	   virtual vector< pair<EID,ELEMENT> > GetElements() const;    // eid is index

	   /**Get user data by element's index.
	     *@see my_find_EID_eq
         *@warning if _eid is not in universe, this method might cause segmentation fault.
	     */
	   virtual ELEMENT GetElement(const EID _eid) const; 

	   /**Get element ID according to given user data, _e.
	     *@return ID if this element is found in universe.
		 *Otherwise, INVALID_EID will be returned.
		 *@see my_find_ELEM_eq 
	     */
	   virtual EID GetElementID(const ELEMENT& _e) const;

	   /**Get all unordered sets in universe.
	     *@return a list of unordered sets.
		 *@see GetSet
	     */
	   virtual vector<pair<SID,vector<pair<EID,ELEMENT> > > > GetSets() const; // (sid,set) pairs

	   /**Get all ordered sets in universe.
	     *@return a list of ordered sets.
		 *@see GetOSet
	     */
	   virtual vector<pair<SID,vector<pair<EID,ELEMENT> > > > GetOSets() const; // (sid,set) pairs

	   /**Get a copy of unodered set with given ID, _sid.
	     *This set is represented as a list of elements with EID and user data.
		 *@retuen if there is no such unodered set in universe, the empty list will
		 *be returned and error message will be post on standard output.
	     */
	   virtual vector< pair<EID,ELEMENT> > GetSet(const SID _sid) const; 

	   /**Get a copy of odered set with given ID, _sid.
	     *This set is represented as a list of elements with EID and user data.
		 *@retuen if there is no such odered set in universe, the empty list will
		 *be returned and error message will be post on standard output.
	     */
	   virtual vector< pair<EID,ELEMENT> > GetOSet(const SID _sid) const; 

     //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	I/O (Display, Input & Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. 
      *Use these method to read in/write/print out internal state
      */
    //@{

	   /**Output information of all elements in universe.
	     *@see DisplayElement
	     */
	   virtual void DisplayElements() const;

	   /**Output information of element labled _eid.
	     *This methods prints out EID and user data info to
		 *standard output.
	     *@see GetElement
		 *@note user data should provide << overloading for ostream.
	     */
	   virtual void DisplayElement(const EID _eid) const;

	   /**Output information of all unordered sets
	     *in universe.
	     *@see DisplaySet
	     */
	   virtual void DisplaySets() const;

	   /**Output information of all ordered sets
	     *in universe.
	     *@see DisplayOSet
	     */
	   virtual void DisplayOSets() const;

	   /**Output information of unordered set with _sid.
	     *This method prints out information of all elements in this 
		 *specified set to standard output and SID.
	     *@see DisplayElement and GetSet.
	     */
	   virtual void DisplaySet(const SID _sid) const;

	   /**Output information of ordered set with _sid.
	     *This method prints out information of all elements in this 
		 *specified set to standard output and SID..
	     *@see DisplayElement and GetOSet.
	     */
	   virtual void DisplayOSet(const SID _sid) const;

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Querying Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
	/**Querying Methods*/
	//@{

	   /**Check if there is any **ORDERED** set labled 
	     *with this SID, _sid.
	     *@see my_find_OSID_eq
	     */
	   bool IsMember(const SID _sid) const;

	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Adding Elements to Universe (they are never deleted)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
	/**Adding Elements to Universe (they are never deleted)*/
	//@{

	   /**Add an element to universe.
	     *If this element is already in universe, then nothing will
		 *be added.
		 *@return If this element is added, then a new EID for this element 
		 *will be returned. If this element is already in universe, then
		 *old id of this element will be returned.
	     */
	   virtual EID AddElementToUniverse(const ELEMENT _e); 

	   /**Add a list of elements to universe.
	     *This method calls AddElementToUniverse(const ELEMENT _e) for
		 *each element in this given list, _evec.
		 *@return return eid of last element in the vector.
		 *@see AddElementToUniverse(const ELEMENT _e)
	     */
	   virtual EID AddElementToUniverse(const vector<ELEMENT> _evec); 

	   /**Update the element's user data by _e in universe.
	     *@param _eid The EID for this specified element.
		 *@param _e New data for this specified element.
		 *@return ERROR if no _eid in universe. OK if updated.
	     */
	   virtual int ChangeElementInfo(EID _eid, ELEMENT _e); 

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:
  //===================================================================
  //  Utility Stuff for Basic Sets
  //===================================================================
   
   //Modified for VC
   typedef vector< pair<EID,ELEMENT> > PAIR_EID_ELEM_VECTOR;///<Element Vector
   typedef PAIR_EID_ELEM_VECTOR::iterator EI;				///<EI Element Iterator
   typedef PAIR_EID_ELEM_VECTOR::const_iterator CEI;		///<CEI Constant Element Iterator

   typedef vector< pair<SID,bit_vector> > PAIR_SID_BITV_VECTOR;	///<Set Vector
   typedef PAIR_SID_BITV_VECTOR::iterator SI;					///<SI Set Iterator
   typedef PAIR_SID_BITV_VECTOR::const_iterator CSI;			///<CSI Constant Set Iterator

   typedef vector< pair<SID,vector<EID> > > PAIR_SID_EIDV_VECTOR;///<Ordered Set Vector
   typedef PAIR_SID_EIDV_VECTOR::iterator OSI;					 ///<OSI Ordered Set Iterator
   typedef PAIR_SID_EIDV_VECTOR::const_iterator COSI;			 ///<COSI Constant Ordered Set Iterator

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Search
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Search Methods*/
   //@{

   /**Find element of user data, _e, in universe.
     *@return Pointer to this element if one exists. Otherwise, element.end()
	 *will be returned.
     */
   pair<EID,ELEMENT>* my_find_ELEM_eq(const ELEMENT& _e) const;

   /**Find element using _eid, in universe.
     *@return Pointer to this element if one exists. Otherwise, element.end()
	 *will be returned.
     */
   pair<EID,ELEMENT>* my_find_EID_eq(const EID _eid) const;

   /**Find unordered using _sid, in universe.
     *@return Pointer to this unordered set if one exists. 
	 *Otherwise, sets.end() will be returned.
     */
   pair<SID,bit_vector>* my_find_SID_eq(const SID _sid) const;

   /**Find ordered using _sid, in universe.
     *@return Pointer to this ordered set if one exists. 
	 *Otherwise, sets.end() will be returned.
     */
   pair<SID,vector<EID> >* my_find_OSID_eq(const SID _sid) const;
   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**Element ID for last added element. 
	  *This value will be increased when a new element
	  *is added to Universe.
	  */
    EID ElementIDs;

	/**Set ID for last added set. 
	  *This value will be increased when a new set is 
	  *added to Universe.
	  */
    SID SetIDs;

	/**@name Universe*/
	//@{
		vector< pair<EID,ELEMENT> > elements; ///<A list of elements in Universe
		vector< pair<SID,bit_vector> > sets; // (id,set) pairs, unordered set in Universe
		vector< pair< SID,vector<EID> > > osets; // (id,set) pairs, ordered set in Universe
	//@}
};


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	METHODS FOR template BasicSets Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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
	
	///Modified for VC
	typedef vector<EID> EID_VECTOR;
	typedef EID_VECTOR::const_iterator CEIDI; 
	
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
	
	///Modified for VC
	typedef vector<ELEMENT> ELEM_VECTOR;
	typedef ELEM_VECTOR::const_iterator CELI;
	
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
		//extend the vector size
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
	
	///Modified for VC
	typedef vector<EID> EID_VECTOR;
	typedef EID_VECTOR::const_iterator CEIDI; 
	
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
	
	///Modified for VC
	typedef vector<ELEMENT> ELEM_VECTOR;
	typedef ELEM_VECTOR::const_iterator CELI;
	
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
	
	///Modified for VC
	typedef vector<EID> EID_VECTOR;
	typedef EID_VECTOR::iterator EIDI;
	
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
