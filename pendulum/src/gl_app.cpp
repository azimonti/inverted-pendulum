/************************/
/*    gl_app.cpp        */
/*    Version 1.0       */
/*     2023/04/13       */
/*  © Marco Azimonti    */
/************************/

#include <cstdio>
#include "log/log.h"
#include "gl_app.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
    LOGGER(logging::ERROR)
        << std::string("Glfw Error :").append(std::to_string(error)).append(": ").append(description);
}

static void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    GLApp* w = (GLApp*)glfwGetWindowUserPointer(window);
    if (key == GLFW_KEY_PRINT_SCREEN && action == GLFW_PRESS)
    {
        if (scancode & GLFW_MOD_CONTROL) {} // CTRL+PrtScr -> request FileName
        if (!scancode) {}                   // CTRL+PrtScr -> TimeBased FileName
    }
    else if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F12 && action == GLFW_PRESS)
    {
        w->GetEngineWnd()->onSpecialKeyDown(key, 0, 0);
    }
    else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) // SPACE
    {
    }
    else if (action == GLFW_PRESS) { w->GetEngineWnd()->onKeyDown(key == GLFW_KEY_ENTER ? 13 : key, 0, 0); }
    else if ((GLFW_MOD_CONTROL == mods) && ('2' == key))
    {
        LOGGER(logging::INFO) << "CTRL 2 - toggle 2d";
        w->GetEngineWnd()->SetFlag(wflags::F2D);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('3' == key))
    {
        LOGGER(logging::INFO) << "CTRL 3 - toggle 3d";
        w->GetEngineWnd()->UnsetFlag(wflags::F2D);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('C' == key))
    {
        LOGGER(logging::INFO) << "CTRL c - toggle control";
        w->GetEngineWnd()->TogglePendulumFlag(ge::pflags::CONTROL);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('G' == key))
    {
        LOGGER(logging::INFO) << "CTRL g - toggle grid";
        w->GetEngineWnd()->ToggleFlag(wflags::GRID);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('N' == key))
    {
        LOGGER(logging::INFO) << "CTRL N - toggle control nn";
        w->GetEngineWnd()->TogglePendulumFlag(ge::pflags::CONTROL_NN);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('R' == key))
    {
        LOGGER(logging::INFO) << "CTRL R - reset";
        w->GetEngineWnd()->Reset();
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('Q' == key))
    {
        LOGGER(logging::INFO) << "CTRL q - exit";
        w->SetWindowShouldClose(true);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('T' == key))
    {
        LOGGER(logging::INFO) << "CTRL T - toggle nn training";
        w->GetEngineWnd()->TogglePendulumFlag(ge::pflags::NN_TRAIN);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('V' == key))
    {
        LOGGER(logging::INFO) << "CTRL V - toggle verbose";
        w->GetEngineWnd()->ToggleFlag(wflags::VERBOSE);
    }
    else if ((GLFW_MOD_CONTROL == mods) && ('W' == key))
    {
        LOGGER(logging::INFO) << "CTRL W - toggle very verbose";
        w->GetEngineWnd()->TogglePendulumFlag(ge::pflags::VVERBOSE);
    }
    else {}
}

static void glfwMouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    GLApp* w = (GLApp*)glfwGetWindowUserPointer(window);

    double x, y;
    (void)mods;
    glfwGetCursorPos(window, &x, &y);

    if (action == GLFW_PRESS) { w->GetEngineWnd()->onMouseButton(button, GLFW_PRESS, (int)x, (int)y); }
    else if (action == GLFW_RELEASE) { w->GetEngineWnd()->onMouseButton(button, GLFW_RELEASE, (int)x, (int)y); }
}

void glfwWindowSizeCallback(GLFWwindow* window, int width, int height)
{
    GLApp* w = (GLApp*)glfwGetWindowUserPointer(window);
    w->GetEngineWnd()->onReshape(width, height);
}

GLApp::GLApp() : windowShouldClose(false)
{
    engineWnd = std::make_unique<GLWindow>();
}

GLApp::~GLApp()
{
    onExit();
}

void GLApp::onInit()
{
    // Imitialize both FrameWorks
    xPosition = yPosition = -1;
    width                 = 1280;
    height                = 720;
    windowTitle           = "2D Plane";
    frameInit();
    engineWnd->onInit();
}

void GLApp::frameInit()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) exit(EXIT_FAILURE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif
    glfwWindowHint(GLFW_SAMPLES, 8);

    mainGLFWwnd = glfwCreateWindow(GetWidth(), GetHeight(), GetWindowTitle(), NULL, NULL);
    if (!mainGLFWwnd)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(mainGLFWwnd);
    glfwSetWindowUserPointer(mainGLFWwnd, (void*)this);
    glfwSetKeyCallback(mainGLFWwnd, glfwKeyCallback);
    glfwSetMouseButtonCallback(mainGLFWwnd, glfwMouseButtonCallback);
    glfwSetWindowSizeCallback(mainGLFWwnd, glfwWindowSizeCallback);

    glfwSwapInterval(1);

    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Glew Error.\n");
        LOGGER(logging::ERROR) << std::string("Glew Error.");
    }
}

void GLApp::newFrame()
{
    int display_w{};
    int display_h{};
    glfwGetFramebufferSize(mainGLFWwnd, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    engineWnd->onRender(mainGLFWwnd);
    glfwMakeContextCurrent(mainGLFWwnd);
    glfwSwapBuffers(mainGLFWwnd);
}

int GLApp::onExit()
{
    engineWnd->onExit();
    frameExit();
    return 0;
}

int GLApp::frameExit()
{
    glfwDestroyWindow(mainGLFWwnd);
    glfwTerminate();
    return 0;
}

void GLApp::mainLoop()
{
    while (!glfwWindowShouldClose(mainGLFWwnd) && !windowShouldClose)
    {
        glfwPollEvents();
        newFrame();
    }
}
