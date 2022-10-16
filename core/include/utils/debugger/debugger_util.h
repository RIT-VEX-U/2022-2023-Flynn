/*********************************************************
*
*     File:     debugger_util.h
*     Purpose:  Handles information used by an async running of 
*               a debugger output.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/07/2022  <CRN>   File created, stubbed.
* 10/09/2022  <CRN>   Added functionality to async post on value change.
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger.h"
#include <atomic>

using namespace vex;

template <typename T>
class debugger_util {
  public:
    /*
    CONSTRUCTOR:
    Delay       --  time between posts / val difference checks.
    Statement   --  base statement to post.
    valType     --  type of value being posted; 'i' for int, 'd' for double, 'n' for no value,
                    c for character, b for boolean, u for unsigned int.
    debugger    --  the debugger that called it; summoned in order to post the statement.
    line        --  line on the controller to post the output to; -1 if to terminal.
    */
    debugger_util(int delay, const char* statement, std::atomic<T> &val, 
                  Debugger debugger, int line=-1);

    /*
    Prints a basic statement, then waits [delay] miliseconds before returning.
    Checks for a value type; prints value as well if valType != 'n'.
    */
    void print();

    /*
    Prints the basic statement, if the current value is outside of a given acceptable
    range compared to the previous value; then waits [delay] miliseconds before returning.
    Can only take input of types double, int.
    */
    void printIfDiff();

    /*
    Sets the acceptable range of a value difference to valDiffIn;
    When calling printIfDiff(), the previous value + and - the valDiff is compared to the
    current value, and if it is on the boundary of or outside that range, it will be posted.
    */
    void setValDiff(double valDiffIN);
  
  private:
    int delay;                // time in MS between posts or checks
    std::atomic<T> &val;      
    const char* statement;    // statement to post, comes before the value, if one exists.
    double valDiff=0.0;       // the minimum difference between a current val and the previous val for it to be deemed post worthy.
    double valPrevious=-1.0;  // previous value of the val; defaults to -1.
    int line;                 // line to print on, -1 if told to print to terminal
    Debugger debugger;        // the debugger that summoned the task
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