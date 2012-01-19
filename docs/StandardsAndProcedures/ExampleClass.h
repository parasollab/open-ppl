#ifndef EXAMPLECLASS_H_
#define EXAMPLECLASS_H_

#include "SomeFile.h"

using namespace std;

class ExampleClass : public BaseClass {
  public:
    ExampleClass(int _v);

    void MyFunction(int _v);
    int GetValue() {return m_value;}

  private:
    int m_value;
    ExampleClass* m_self;
    ExampleClass& m_myFriend;
}

#endif
