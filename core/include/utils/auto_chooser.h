#pragma once
#include "vex.h"
#include <string>
#include <vector>

class AutoChooser
{
  public:
  AutoChooser(vex::brain &brain);

  void add(std::string name);
  std::string get_choice();

  protected:

  typedef struct
  {
    int x, y, width, height;
    std::string name;
  }entry_config_t;

  void render(entry_config_t *selected);
  std::string choice;
  std::vector<entry_config_t> list;
  vex::brain &brain;


};