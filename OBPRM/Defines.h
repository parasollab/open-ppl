/////////////////////////////////////////////////////////////////////
//
//  Defines.h
//
//  General Description
//  This an example of how the includes should look like in order to have compatibility
//  with different STLs
//  Created
//      Gabriel Tanase
//
/////////////////////////////////////////////////////////////////////

//include standard headers
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#ifndef  _STL_PORT

#if defined(__KCC)
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include <algorithm>    
#include <list>     
#include <vector>       
#include <deque>        
#include <stack>        
#include <map> 

#else
#if defined(sun) || defined(__sgi) || defined(__linux__) || defined(_WIN32)
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algo.h>   
#include <list.h>   
#include <vector.h>
#include <deque.h>  
#include <stack.h> 
#include <map.h> 
#endif
#endif

#if defined(hppa) && defined(__cplusplus)   //c++ in parasol
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
//#include <algo.h>    
#include <list>     
#include <vector.h>
#include <deque.h>  
#include <stack.h>  
#include <map> 
#endif

//aCC in parasol
#ifdef __HP_aCC
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>    
#include <list>     
#include <vector>       
#include <deque>        
#include <stack>        
#include <map> 
#endif
using namespace std;

//STL_PORT 
#else
#include <iostream>
#include <ostream>
#include <fstream>
#include <iomanip>
#include <algorithm>    
#include <list>     
#include <vector>       
#include <deque>        
#include <stack>   
using namespace _STLP_STD;
#endif

#ifndef OK
#define OK  0
#endif

#ifndef ERROR
#define ERROR -1
#endif
