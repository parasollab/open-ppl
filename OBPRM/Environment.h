// $Id$
/////////////////////////////////////////////////////////////////////
//  Environment.h
//
//  Created   2/25/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
//  Modified  4/16/98 Wookho Son
//  Added     5/18/98 Wookho Son
//  Modified  7/14/98 Wookho Son
//  Added/Modified  7/31/98 Wookho Son
//
//  Header files should be included whenever if need to use 
//  the membership functions here. Otherwise, just the inclusion of 
//  the class definition is ok.
//
/////////////////////////////////////////////////////////////////////

#ifndef Environment_h
#define Environment_h




#include <fstream.h>
#include <list.h>
#include "MultiBody.h"      
#include "Matrix.h"
#include "Contact.h"

class Input;
//class MultiBody;

class Environment {
public:
    //-----------------------------------------------------------
    //  Enumerations
    //-----------------------------------------------------------
    enum DirectionType {
         GMS_NORMAL = 0,
         GMS_TANGENTIAL = 1,
         GMS_ORTHOGONAL = 2
    };

    //---------------------------------------------------------------
    //  Constructors and Destructor
    //---------------------------------------------------------------
    Environment();
    ~Environment();
    //---------------------------------------------------------------
    //  Methods
    //---------------------------------------------------------------
    void Draw();
    int GetMultiBodyCount();
//
// Wookho Son 4/10/98
//
    void AddMultiBody(MultiBody *_multibody);
    MultiBody * GetMultiBody(int _index);
    void Couple(MultiBody *_multibody[], int _number);
    void Decouple(MultiBody *_multibody[], int _number);
    void Get(Input * _input);
    void Write(ostream & _os);

protected:
private:
    //---------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------
    int multibodyCount;
    MultiBody **multibody;

};

//===================================================================
//  Inline functions
//===================================================================

//-------------------------------------------------------------------
//  GetMultiBodyCount
//
//  Output: the number of MultiBody's in the environment
//-------------------------------------------------------------------
inline int Environment::GetMultiBodyCount() {
    return multibodyCount;
}

//-------------------------------------------------------------------
//  GetMultiBody
//  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline MultiBody * Environment::GetMultiBody(int _index) {
    if (_index < multibodyCount)
        return multibody[_index];
    else
        return 0;
}

#endif
