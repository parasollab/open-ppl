#include "ExampleClass.h"
#include "OtherFile.h"

ExampleClass::ExampleClass(int _v) : m_value(_v) {
  if(m_debug){ //assume m_debug from BaseClass
    for(int i = 0; i<m_value; i++){
      cout<<"Counting::"<<i<<endl;
    }
  }
}

void ExampleClass::MyFunction(int _v){
  //do some fancy stuff
  m_value+=_v;
}
