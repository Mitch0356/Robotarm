#include "gui.hpp"

gui::gui(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(250, 150))
{
    Centre();
}

gui::~gui() {}

gui::initialize()

