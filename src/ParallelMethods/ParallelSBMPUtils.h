#ifndef PARALLELSBMPUTILS_H
#define PARALLELSBMPUTILS_H



namespace psbmp{

template<typename T>
void PrintValue(const char* _name, const T _par ) {
	std::cout << "location[" << stapl::get_location_id() <<"] " << _name << _par << std::endl;
}

template<typename T>
void PrintOnce(const char* _name, const T _par) {
	if(stapl::get_location_id() == 0) std::cout << _name << _par << std::endl;
}

////Needs to be in .cpp ( multiple definition) in fact move all to general utils
/*void PMPLPrintString(const char* _name){
	std::cout << "location[" << stapl::get_location_id() <<"] " << _name << std::endl;
}*/

}


#endif 
