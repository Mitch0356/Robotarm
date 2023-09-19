#include <iostream>
#include <wx/wx.h>
#include "main_frame.hpp"



IMPLEMENT_APP(main_frame)

//g++ *.cpp -L /usr/lib/boost -o main -std=c++17 -lboost_unit_test_framework -lboost_system -lboost_filesystem -lboost_program_options -Wall -Wextra `wx-config --cxxflags --libs`