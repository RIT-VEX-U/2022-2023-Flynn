/*********************************************************
*
*     File:     Debugger.cpp
*     Purpose:  Generalized debugger to handle print statements
*               in core.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/02/2022  <CRN>   File created, stubbed.
* 10/06/2022  <CRN>   Completely restructured program, made it work kind of.
* 10/09/2022  <CRN>   Added async on value change functionality.
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger.h"
#include "../core/include/utils/debugger_util.h"
#include <string>
#include <iostream>
#include <sstream>
#include <atomic>

using namespace vex;
using namespace std;

// constructor
Debugger::Debugger(controller main_controller)
  :main_controller(main_controller) 
  { }

/*
Prints a given statement to the terminal or the controller.
Statement -- statement to print
line -- line printed to if using the controller; -1 puts output to the terminal, 0-3 put to the controller.
*/
void Debugger::print(const char* statement, int line){
  if(line != -1) {
    main_controller.Screen.clearLine(line);
    main_controller.Screen.print(statement, line);
  } else {
    printf("\n%s", statement);
  }
}


/*
Prints a given value to the terminal or the controller.
valPointer -- pointer to the value to be printed.
valType -- type of object or primitive in the value. Key is as follows:
      1. i  -- int
      2. d  -- double
      3. c  -- char
      4. b  -- boolean
      5. u  -- unsigned int
*/
template <typename T>
void Debugger::printVal(const char* statement, std::atomic<T> &val, int line){
  std::ostringstream os;
  const char* newStatement = (statement + os.str()).c_str();
  this->print(newStatement, line);
}

/*
Stops the current debug task; returns true if successful, false otherwise.
*/
bool Debugger::stopTask() { 
  if(taskRunning) {
    debugTask.stop(); 
    taskRunning = false;
    return true;
  }
  return false;
}

/*  Function called to handle periodic posts; calls the debugger_util's
    basic print function.  */
int debugTaskFunctionPeriodic(void* debugTaskUtilVP) {
  debugger_util debugTaskUtil = *((debugger_util*) debugTaskUtilVP);
  while(true){
    debugTaskUtil.print();
  }
  return 0;
}

/*  Function called to handle posts at value change;
    calls the debugger_task's printIfDiff function.  */
int debugTaskFunctionValChange(void* debugTaskUtilVP) {
  debugger_util debugTaskUtil = *((debugger_util*) debugTaskUtilVP);
  while(true){
    debugTaskUtil.printIfDiff();
  } return 0;
}

/*
Prints an async value given a certain delay.
delay     --  time, in milliseconds, between posts
valType   --  same as print, but 'n' if no value should be printed;
              not sure why it'd ever come up but it's here if it's needed.
returns true if the task has been started, false if it hasn't
*/
template <typename T>
bool Debugger::printAsyncPeriodic(const char* statement, int delay, 
                                  std::atomic<T> &val,
                                  int line){
  if(taskRunning) return false;
  debugger_util debugTaskUtil(delay, statement, val, *this, line);
  debugTask = task(debugTaskFunctionPeriodic, (void*) &debugTaskUtil);
  taskRunning = true;
  return true;
}

/*
Prints a value whenever it changes to a different value outside of a certain range.
Can only take input of types double and int; prints invalidity otherwise.
diffMin -- minimum difference in values for value to be printed; acceptable range.
*/
bool Debugger::printAsyncValueChange(const char* statement, void* valPointer, char valType,
                                     int line, double diffMin, int delay){
  if(taskRunning) return false;
  debugger_util debugTaskUtil(delay, statement, valType, valPointer, *this, line);
  debugTaskUtil.setValDiff(diffMin);
  debugTask = task(debugTaskFunctionValChange, (void*) &debugTaskUtil);
  taskRunning = true;
  return true;
}