/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
#pragma once
#include <array>
#include <iostream>
#include <string>

struct row {
  std::string name;
  double lat;
  double lon;
  double rate;
};

extern std::array<row, 303> network;
/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
