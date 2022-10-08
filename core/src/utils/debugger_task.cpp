/*********************************************************
*
*     File:     debugger_task.cpp
*     Purpose:  Handles a debugger.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/07/2022  <CRN>   File created, stubbed.
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger_task.h"

using namespace vex;

debugger_task::debugger_task(int delay, const char* statement, char valType, void* valPointer, 
                             Debugger debugger, int line)
  :delay(delay), valType(valType), valPointer(valPointer), statement(statement),
   line(line), debugger(debugger)
  { };

void debugger_task::print(){
  if(valType == 'n') {
    debugger.print(statement, line);
  } else if(isRunningAverage) {
    runningAverageTimer++;
    runningAverage += *(double*) valPointer; 
    double currentRunner = runningAverage / runningAverageTimer;
    debugger.printVal(statement, (void*) &currentRunner, valType, line);
  } else {
    debugger.printVal(statement, valPointer, valType, line);
  }
}

void debugger_task::setToRunningAverage(){
  if(!isRunningAverage && valType != 'n') {
    runningAverage = 0;
    runningAverageTimer = 0;
    runningAverage = true;
  }
}


void debugger_task::resetRunningAverage(){
  runningAverage = 0;
  runningAverageTimer = 0;
}

int debugger_task::getDelay() { return delay; }

