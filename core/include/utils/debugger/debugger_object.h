/*********************************************************
*
*     File:     debugger_object.h
*     Purpose:  Handles a generic object to put in the debugger.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/16/2022  <CRN>   File created, stubbed.
*********************************************************/

#include "vex.h"
#include <atomic>
using namespace vex;

template <typename T>
class debugger_object{
  public:
    debugger_object(T val);
  private:
    T val;
 };

template<int N>
struct select_type;

template<>
struct select_type<1> {
  typedef int type;
};

template<>
struct select_type<2> {
  typedef float type;
};

template<>
struct select_type<3> {
  typedef double type;
};

template<>
struct select_type<4> {
  typedef long type;
};

template<>
struct select_type<5> {
  typedef char type;
};

template<>
struct select_type<6> {
  typedef float type;
};