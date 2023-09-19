#include "main_frame.hpp"

bool main_frame::OnInit()
{
    gui *current = new gui(wxT("Simple"));
    current->Show(true);
    return true;
}