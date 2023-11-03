#pragma once

#include "owl/owl.h"
#include "owlViewer/OWLViewer.h"
struct NanoViewer : public owl::viewer::OWLViewer
{
    virtual void gui();

    void showAndRunWithGui();
    void showAndRunWithGui(std::function<bool()> keepgoing);
private:
    void initializeGui();
    void deinitializeGui();
};