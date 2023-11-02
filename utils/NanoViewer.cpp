#include "NanoViewer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
using namespace owl;
using namespace owl::viewer;

namespace
{
    void glfwindow_reshape_cb(GLFWwindow* window, int width, int height )
    {
        OWLViewer *gw = static_cast<OWLViewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->resize(vec2i(width,height));
    }

    void glfwindow_char_cb(GLFWwindow *window,
                                  unsigned int key)
    {
        OWLViewer *gw = static_cast<OWLViewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->key(key,gw->getMousePos());
    }

    void glfwindow_key_cb(GLFWwindow *window,
                                 int key,
                                 int scancode,
                                 int action,
                                 int mods)
    {
        OWLViewer *gw = static_cast<OWLViewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        if (action == GLFW_PRESS) 
        {
            gw->special(key,mods,gw->getMousePos());
        }
    }

    void glfwindow_mouseMotion_cb(GLFWwindow *window, double x, double y)
    {
        OWLViewer *gw = static_cast<OWLViewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->mouseMotion(vec2i((int)x, (int)y));
        auto& io = ImGui::GetIO();
    }

    void glfwindow_mouseButton_cb(GLFWwindow *window,
                                         int button,
                                         int action,
                                         int mods)
    {
        OWLViewer *gw = static_cast<OWLViewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->mouseButton(button,action,mods);
    }
}

void NanoViewer::gui()
{
    static bool show_demo_window = true;
    ImGui::ShowDemoWindow(&show_demo_window);
    
}

void NanoViewer::initializeGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(handle, true);
    ImGui_ImplOpenGL3_Init();
}

void NanoViewer::deinitializeGui()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void NanoViewer::showAndRunWithGui()
{
    showAndRunWithGui([]() { return true; }); 
}

void NanoViewer::showAndRunWithGui(std::function<bool()> keepgoing)
{
    int width, height;
    glfwGetFramebufferSize(handle, &width, &height);
    resize(vec2i(width,height));

    glfwSetFramebufferSizeCallback(handle, glfwindow_reshape_cb);
    glfwSetMouseButtonCallback(handle, glfwindow_mouseButton_cb);
    glfwSetKeyCallback(handle, glfwindow_key_cb);
    glfwSetCharCallback(handle, glfwindow_char_cb);
    glfwSetCursorPosCallback(handle, glfwindow_mouseMotion_cb);

    initializeGui();

    while (!glfwWindowShouldClose(handle) && keepgoing()) 
    {
        static double lastCameraUpdate = -1.f;
        if (camera.lastModified != lastCameraUpdate) 
        {
            cameraChanged();
            lastCameraUpdate = camera.lastModified;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        gui();
        ImGui::EndFrame();
        
        render();
        draw();
        
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(handle);
        glfwPollEvents();
    }

    deinitializeGui();
    glfwDestroyWindow(handle);
    glfwTerminate();
}
