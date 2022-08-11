---
author:
- Jory Denny and Read Sandström and James Motes and Shawna Thomas and Nancy M. Amato
title: "**Standards and Procedures of PPL**"
---

This document outlines the standards for all procedures within PPL.
This ranges from coding standards, to testing standards, to
documentation standards, etc. Everyone will be expected to adhere to
these standards.

# Coding Standards

## `.h/.cpp`

All classes and files will be split between `.h/.cpp` format when
possible, i.e., non-template classes/functions. All `.h` files should
have #define guards with the name of the file in all caps, with `_`s
separating words and `_H_` at the end. For example: for `ExampleClass.h`
-- `PPL_EXAMPLE_CLASS_H_`.

    #ifndef PPL_EXAMPLE_CLASS_H_
    #define PPL_EXAMPLE_CLASS_H_

    class Class;

    #endif

Newer files will include '`PPL_`' at the front of the include guard to
further reduce the possibility of name collision with an external
library. Please use this prefix in all new include guards.

## Naming

### #define

All #define preprocessor commands should be named in all capital
letters, with \_s separating words.

    #define MY_DEFINE

When possible, #defines should be avoided and use constant functions
instead. This is the C++ style.

### Types

All types, being all classes, structs, typedefs, enums, etc, should
begin with a capital letter and be in camel casing format (i.e., each
successive word will be capitalized).

``` {mathescape=""}
class MyClass;
struct MyStruct;
typedef int MyInt;
enum MyEnum {Value1, Value2, Value3};
```

#### Pointer and Reference Type Specification

When declaring a variable to be a pointer or reference the \* or &
should be adjacent to the type:

    Type* pointerName;
    Type& referenceName;

### Enums

Enum values should be named like types, i.e, camel casing beginning with
a capital letter.

``` {mathescape=""}
enum MyEnum {Value1, Value2, Value3};
```

### Functions

Functions shall be names with camel casing beginning with a capital
letter.

    void MyFunction();

### Variables

All variables should be named in camel casing beginning with a lowercase
letter.

    int myInt;

#### Member/class variables

All member variables should begin with an m\_. No type specification is
necessary (i.e., no m_pVariable for a pointer).

``` {mathescape=""}
class MyClass {
  public:
    int m_myInt;
}
```

#### Function parameters

All function parameters should begin with an \_.

    int MyFunction(int _a, int _b);

### Naming Exceptions

The following cases are exempt from naming conventions for language
specific reasons.

-   Functions defining iterators, namely, `begin()`, `end()`, and the
    iterator typedef. This is for compatability with `auto` keyword for
    range based for loops.

-   In classes which extend part of the STL, the methods which override
    an STL class's member function must be named against our standard,
    obviously.

## XML

Similarly to the class and variable names, within the XML, tags should
be camel cased beginning with a capital letter and variables should be
camel cased beginning with a lowercase letter.

    <MyXMLTag label="label" myXMLVariable="false"/>
    <MyXMLTagWithChildren label="label2">
      <Child method="label"/>
      <Child method="label2"/>
    </MyXMLTagWithChildren>

When loading labels for PPL methods, e.g., distance metrics, the
variable name should be the PPL acronym for that component with Label.
So for a distance metric label variable it should be named dmLabel, a
validity checker label should be vcLabel, etc. Acronyms are as follows:

-   Distance Metric - dm

-   Validity Checker - vc

-   NeighborhoodFinder - nf

-   Sampler - s

-   Local Planner - lp

-   Connector - c

-   Map Evaluator - me

-   Metric - m

-   Extender - e

-   Path Modifier - pm

-   Motion Planning Strategy - mps

## Spaces/Tabbing

All tab characters should be represented by 2 spaces (easy to make
default in all editors, vim is recommended). There should be no trailing
white space at the end of lines (this causes unnecessary conflicts with
in SVN).

## Line length

All lines of code should be a reasonable length for code clarity. 80
characters is an expected line limit. Though this is not a strictly
enforced standard, please try to adhere as best as possible to
accomodate those working on smaller screens.

## Curly Braces

Curly braces used in control flow, function definitions, or class
definitions should be placed on the same line as the statement. It is
recommended that a space occurs before the curly brace, e.g., if() { NOT
if(){.

    class MyClass {
      void MyFunction() {
        while(1) {
        }
      }
    }

Additionally, if the body of control flow structures is one line, curly
braces are not required, and the body must appear on a new line.

    if(true)
      DoSomething();

## Function Call/Declaration/Definition

Function calls or declarations which are \"long\" should be split
between multiple lines when necessary to aid in reading the code.

Definitions of member functions should be outside the class definition,
even for one-line functions (this helps fully separate a class's
implementation from its interface). The exception here is for small
classes which serve as containers or functors: in these cases, member
definitions may be placed directly within the class.

Definitions shall be formatted with the template, return type, class
name, and function name all on separate lines. The function parameters
may span multiple lines if necessary.

    template<class MPTraits>
    ReturnType
    ClassName<MPTraits>::
    FunctionName(parameter list which may span multiple lines) {
    }

## Overriding Functions

All overriding functions must be declared `virtual` and `override`. This
clarifies that the function does override some base implementation and
allows the compiler to throw an error if we inadvertently fail to match
the base function's prototype.

## MPBaseObject inherited variables/functions

### m_debug

There will be a variable, m_debug in the PMPL base class which will be
used to determine when output occurs, e.g., in debugging statements.
This means that there will be no #Define statements for debug output.
All debug output will be of the form:

    if(m_debug)
      cout << ``My debug statement'';

### GetNameAndLabel()

This function returns a unique identifier to this class based on XML
input. Returns m_name + "::" + m_label. This should be used in output
and record keeping.

    string callee = this->GetNameAndLabel();

### Print

In all classes one function Print should be implemented which displays
the values of the class variables at the time of calling. This is called
for every loaded variable after parsing the XML in the main function of
PPL.

    void Print(ostream& _os) const {
      _os << GetNameAndLabel() << endl;
      //output rest of class values
    }

### XML Parsing

All derived classes from MPBaseObject may support constructors that take
an `XMLNode` parameter. In all such cases, the `XMLNode` parameter must
be passed up the class hierarchy so that parent classes get an
opportunity to parse their parameters.

## Throwing Errors

All errors, unless otherwise approved should be thrown using a
PMPLException, or other base type. This is to encourage recovering from
errors in sister programs, like Vizmo and GroupBehaviors. Exception
classes are located within `src/Utilities/PMPLExceptions.h`. Use the
WHERE define to give an exact location of the error. The PMPLException
base is nicely formatted.

    throw PMPLException(``MyExceptionType'', WHERE, ``My error message'');

Our exceptions also support ostream-like usage to help with printing
non-string data that support `operator<<`, such as `Boundary` objects.
This example shows a `RunTimeException` for a boundary object:

    Boundary b;
    throw RunTimeException(WHERE) << "Something is wrong with this boundary:\n"
        << b << std::endl;

## STL and External Libraries

The STL and External Libraries, e.g, boost and CGAL should be used as
much as possible, except when there is a good reason not to, e.g.,
efficiency.

## Statistics Tracking

All statistics should be tracked and recorded in the StatClass. There
are standard map data structures to associate names with statistics,
e.g., sampling attempts. The name should include the class name and
label. There is a separate map for each piece of the code and the output
of such statistics is standardized to aid in automated data collection.

# Documentation

All documentation should be done with Doxygen. This style was chosen to
make documentation very obvious in the code, and to support automatic
generation of web-viewable docs for PPL. Note that the blank comment
lines shown in these templates are required by Doxygen in order to
introduce paragraphs.

The Doxygen documentation will be compiled nightly and copied onto
Nancy's group's intranet page on here:

<https://TODO>.

## General Documentation Proceedures

1.  The function operator of a function object (e.g., std::plus) doesn't
    need to be documented. The documentation of the class will provide
    all necessary information.

2.  Constructors, destructors, and assignment operators don't have to be
    documented, unless they do more than simple initialization and
    cleanup of the data members of the class.

3.  Template parameters do not need to be documented so long as their
    name unambiguously describes their semantic meaning. I.e., template
    parameter names like 'FloatingPointType' and 'CfgType' are
    self-documenting in terms of what type is expected.

## Class Documentation

Place this comment block before the class definition. Document
user-visible defects and restrictions of the class in the optional
section provided. Note that the top/bottom commenting lines should
extend to the 80th character across.

``` {mathescape=""}
///////////////////////////////////////////////////////////////////////////////
/// What information does this class abstract?
///
/// [details about the purpose of the class, for example:]
/// [If the class is implemented from a specific paper, cite authors, conference, and year]
/// [Non-obvious reasons for its place in the inheritance hierarchy, etc.]
///
/// [optional details about user-visible defects and restrictions]
/// $@$bug What is the problem
/// $@$remarks What test shows the problem?
/// $@$todo What must be done before the problem can be fixed?
///
/// $@$bug another bug
/// ...
///
/// $@$ingroup What group of classes does this belong to?
///////////////////////////////////////////////////////////////////////////////
```

## Enumeration Types

Enums and enum classes should also be documented with a doxygen block
like classes. This should explain the meaning of the enumeration. The
allowed values should have names which clearly describe their meaning,
and may additionally be documented with in-line doxygen comments. For
example, this enumeration type describes early termination conditions
for a single-source shortest path search:

``` {mathescape=""}
////////////////////////////////////////////////////////////////////////////////
/// The possible early-termination conditions for an SSSP run.
////////////////////////////////////////////////////////////////////////////////
enum class SSSPTermination {
  Continue,   ///< Proceed as usual.
  EndBranch,  ///< End the branch at this vertex and do not relax its neighbors.
  EndSearch   ///< End the entire search process.
};
```

## Function Documentation

Place this comment block before the method definition.

It is not necessary to include self-evident information in the comment
blocks. In particular, there is no need to put "default constructor" or
"destructor" in the brief description, since that is evident from the
name of the method. Doxygen comments should not include information on
the implementation of the method.

``` {mathescape=""}
/// What does this method do?
/// $@$param name meaning of this parameter?
/// $@$param ...
/// $@$return What is the meaning of the return value?
///
/// Optional: if the usage is not obvious, a 'usage' section can be helpful.
/// $@$usage
/// $@$code
/// int i = 0;
/// MyFunc(i);
/// $@$endcode
///
/// Optional: if this is a non-member function and it belongs to some other semantic group.
/// $@$ingroup Semantic Group Name
```

For void functions with no parameters the following shorthand can be
used (as long as it fits nicely on one line):

``` {mathescape=""}
void Foo(); ///< My brief description
```

For functions with obvious return values or parameter interpretations
(such as getters/setters), the `@param` and `@return` documentation can
be omitted. For these types of functions, this usually just repeats what
is stated in the brief description.

### Member Function Groups

This is optional, but if you would like to group members in your class
together based upon common functionality, this is acceptable and
encouraged. An example, would be grouping constructors, different types
of computations, or different sections of accessors. If this is desired,
the following standard is enforced:

``` {mathescape=""}
///$@$name My Group Name
///$@${

void Foo1();
void Foo2();
$\ldots$
void FooN();

///$@$}
```

Also, grouping functions for common documentation is accepted, and is
noted like this:

``` {mathescape=""}
///$@${
/// Common documentation

void Foo1();
void Foo2();

///$@$}
```

### Overloaded Functions

For overloaded functions. Put the long/descriptive comment at the most
general function, then for the others put the following, with the same
brief as the other overloaded functions and a short description of the
differences.

``` {mathescape=""}
/// The long brief which appears over each overload.
/// (params go here)
/// $@$overload
/// The specific differences for this version of the function.
```

## Overridden Functions

Functions which override a base class implementation do not need to be
documented; Doxygen will use the base class documentation for these.

### Inherited/derived Functions

For inherited/derived functions, if the comment block is in the parent
class, no further documentation is needed (Doxygen uses parent classes
comment by default). Simply describe the differences/purpose of that
function in the class comment block. For example, for
$\mathtt{TranslateAtS}$ local planner: no commenting of the
$\mathtt{IsConnected}$ function is necessary simply state the
description of the algorithm in the class block.

## Variable Documentation

All class (member) and public (global) variables should be documented
with their purpose.

For class variables:

    bool m_myVar; ///< Brief description after the member variable.

    /// This variable has a very long description which doesn't fit after the declaration.
    std::vector<double> m_myVarWithLongDescription;

For global variables:

``` {mathescape=""}
/// Brief description of variable
/// $@$ingroup Which group does this variable belong (optional)?
```

The best documentation of temporary variables is in how they are named.
If additional documentation for temporaries is required, this should not
use the doxygen format (use a regular comment instead).

## Algorithm Documentation

Any other documentation is up to the discretion of the programmer, but
C++ style comments should be used. This should be for commenting
algorithms and non-obvious code sections.

In general, comments within function implementations should be
non-doxygen type and should refer to *what* the code is doing rather
than *how* it is being done. The code itself is the best descriptor of
the *how*; comments can help other programmers quickly understand the
meaning of the code.

# Testing

PPL supports two types of testing: component and strategy. Every method class in
the MPLibrary or TMPLibrary should have a corresponding component test class
inside the Testing directory. Each component test should provide coverage for
the standard interface functions of that component. For example, a sampler methods
test class should cover all variations of single and group robot sample functions.
A tests executable can be compiled with ```make tests``` and run with
```./tests -f <Test.xml>```.

Strategy tests check the behavior of MPStrategies (we later want to support TMPStrategies).
Individual strategy tests have a cached 'Gold Standard' set of ouptut files.
Each strategy is run and the outputs are compared against the gold standard.
Instructions for how to run strategy tests can be found in the StrategyTests
directory inside the Testing directory.
