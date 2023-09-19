#include <wx/wx.h>

class gui: public wxFrame
{
public:
    gui(const wxString& title);
    ~gui();
    void initialize();
    
};