#pragma once

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
* 10/06/2022  <CRN>   Simplified, documented, reformatted.
* 10/09/2022  <CRN>   Added async on value change functionality.
*********************************************************/

#include "vex.h"
#include <atomic>

using namespace vex;

class Debugger{
  public:

    // constructor
    Debugger(controller main_controller);

    /*
    Prints a given statement to the terminal or the controller.
    Statement -- statement to print
    line -- line printed to if using the controller; -1 puts output to the terminal, 0-3 put to the controller.
    */
    void print(const char* statement, int line=-1);

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
    void printVal(const char* statement, std::atomic<T> &val_ref, int line=-1);

    // ====== ASYNC MANAGEMENT ======

    /*
    Stops the current debug task; returns true if successful, false otherwise.
    */
    bool stopTask();

    // ====== ASYNC FUNCTIONS ======

    /*
    Prints an async value given a certain delay.
    delay     --  time, in milliseconds, between posts
    valType   --  same as print, but 'n' if no value should be printed;
                  not sure why it'd ever come up but it's here if it's needed.
    returns true if the task has been started, false if it hasn't
    */
    template <typename T>
    bool printAsyncPeriodic(const char* statement, int delay=20, 
                            std::atomic<T> &val=nullptr, int line=-1);
    
    /*
    Prints a value whenever it changes to a different value outside of a certain range.
    Can only take input of types double and int; prints invalidity otherwise.
    diffMin -- minimum difference in values for value to be printed; acceptable range.
    */
    template <typename T>
    bool printAsyncValueChange(const char* statement, std::atomic<T> &val=nullptr,
                              int line=-1, double diffMin=0.0, int delay = 0);

  private:
    controller main_controller;
    bool taskRunning = false;
    task debugTask;
};