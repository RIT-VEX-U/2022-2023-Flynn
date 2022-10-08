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
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger.h"
#include "../core/include/utils/debugger_task.h"
#include <string>
#include <iostream>
#include <sstream>

using namespace vex;
using namespace std;

Debugger::Debugger(controller main_controller)
  :main_controller(main_controller) 
  { }

void Debugger::printVal(const char* statement, void* valPointer, char valType, int line){
  std::ostringstream os;
  switch(valType) {
    case 'i':
      os << *((int*) valPointer);
      break;
    case 'd':
      os << *((double*) valPointer);
      break;
    case 'c':
      os << *((char*) valPointer);
      break;
    case 'b':
      if((bool*) valPointer){ os << "True";
      } else { os << "False"; }
      break;
    case 'u':
      os << *((unsigned int*) valPointer);
      break;
    default:
      os << "INVALID DATA TYPE";
      break;
  }
  const char* newStatement = (statement + os.str()).c_str();
  this->print(newStatement, line);
}

void Debugger::print(const char* statement, int line){
  if(line != -1) {
    main_controller.Screen.clearLine(line);
    main_controller.Screen.print(statement, line);
  } else {
    printf("\n%s", statement);
  }
}

bool Debugger::stopTask() { 
  if(taskRunning) {
    debugTask.stop(); 
    taskRunning = false;
    return true;
  }
  return false;
}

int debugTaskFunction(void* debugTaskUtilVP) {
  debugger_task debugTaskUtil = *((debugger_task*) debugTaskUtilVP);
  while(true){
    debugTaskUtil.print();
    vexDelay(debugTaskUtil.getDelay());
  }
  return 0;
}

bool Debugger::printAsyncPeriodic(const char* statement, int delay, 
                                  void* valPointer, char valType, 
                                  int line){
  if(taskRunning) return false;
  debugger_task debugTaskUtil(delay, statement, valType, valPointer, *this, line);
  debugTask = task(debugTaskFunction, (void*) &debugTaskUtil);
  taskRunning = true;
  return true;
}