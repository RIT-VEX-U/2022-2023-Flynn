/*********************************************************
*
*     File:     debugger_task.h
*     Purpose:  Handles a debugger.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/07/2022  <CRN>   File created, stubbed.
*********************************************************/

#include "vex.h"
#include "../core/include/utils/debugger.h"

using namespace vex;

class debugger_task{
  public:
    debugger_task(int delay, const char* statement, char valType, void* valPointer, 
                  Debugger debugger, int line=-1);

    void print();
    int getDelay();
    void setToRunningAverage();
    void resetRunningAverage();

  private:

    int delay;
    char valType;
    void* valPointer;
    const char* statement;
    int line;
    Debugger debugger;
    double runningAverage = 0;
    double runningAverageTimer = 0;
    bool isRunningAverage = false;
};