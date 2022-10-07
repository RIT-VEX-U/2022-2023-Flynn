/*********************************************************
*
*     File:     Debugger.h
*     Purpose:  Generalized debugger to handle print statements
*               in core.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 10/02/2022  <CRN>   File created, stubbed.
*********************************************************/

#include "vex.h"

using namespace vex;

class Debugger{
  public:

    // constructor
    Debugger(controller main_controller);

    /*
    Statement -- statement to print
    newOrClearline -- make a new line if printing to the terminal; clear the line before printing if on controller
    toController -- TRUE if printing to the controller, FALSE if printing to the terminal
    line -- line printed to if using the controller
    */
    void print(const char* statement, bool toController=false, bool newOrClearline=true, int line=3);

    /*
    valPointer -- pointer to the value to be printed.
    valType -- type of object or primitive in the value. Key is as follows:
                1. i  -- int
                2. d  -- double
                3. c  -- char
                4. b  -- boolean
                5. u  -- unsigned int
    */
    void printVal(const char* statement, void* valPointer, char valType='i', bool toController=false, bool newOrClearline=true, int line=3);

    // async management

    // async functions
    bool printAsyncPeriodic(const char* statement, void* data, char dataType='N', int delay=20, bool newline=true);

  private:
    
    controller main_controller;
    bool taskRunning();
};