/*********************************************************
*
*     File:     debugger_util.cpp
*     Purpose:  Handles a debugger.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/07/2022  <CRN>   File created, stubbed.
* 10/09/2022  <CRN>   Added functionality to async post on value change.
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger_util.h"
#include <atomic>

using namespace vex;

/*
CONSTRUCTOR:
Delay       --  time between posts / val difference checks.
Statement   --  base statement to post.
valType     --  type of value being posted; 'i' for int, 'd' for double, 'n' for no value,
                c for character, b for boolean, u for unsigned int.
debugger    --  the debugger that called it; summoned in order to post the statement.
line        --  line on the controller to post the output to; -1 if to terminal.
*/
debugger_util::debugger_util(int delay, const char* statement, std::atomic<T> &val,
                             Debugger debugger, int line)
  :delay(delay), val(val), statement(statement),
   line(line), debugger(debugger)
  { };

/*
Prints a basic statement, then waits [delay] miliseconds before returning.
Checks for a value type; prints value as well if valType != 'n'.
*/
void debugger_util::print(){
  if(valType == 'n') {
    debugger.print(statement, line);
  } else {
    debugger.printVal(statement, valPointer, valType, line);
  } vexDelay(delay);
}

/*
Prints the basic statement, if the current value is outside of a given acceptable
range compared to the previous value; then waits [delay] miliseconds before returning.
Can only take input of types double, int.
*/
void debugger_util::printIfDiff(){
  double val = 0;
  switch(valType){
    case 'i':
      val += *((int*) valPointer);
    case 'd':
      val += *((double*) valPointer);
      break;
    default:
      debugger.print("INVALID VALUE TYPE", line);
      vexDelay(delay);
      return;
  }
  if(val >= valPrevious + valDiff || val <= valPrevious - valDiff) {
    this->print();
    valPrevious = val;
  } else {
    vexDelay(delay);
  }
}

/*
Sets the acceptable range of a value difference to valDiffIn;
When calling printIfDiff(), the previous value + and - the valDiff is compared to the
current value, and if it is on the boundary of or outside that range, it will be posted.
*/
void debugger_util::setValDiff(double valDiffIN){
  valDiff = valDiffIN;
}