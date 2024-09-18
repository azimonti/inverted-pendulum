#ifndef _GLAPP_H_082EED0BB7D24128B91F665EF2B1A363_
#define _GLAPP_H_082EED0BB7D24128B91F665EF2B1A363_

/************************/
/*     gl_app.h         */
/*    Version 2.0       */
/*     2023/04/13       */
/************************/

#include <memory>
#include <string>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "gl_window.h"

class GLApp
{
  public:
    GLApp();
    ~GLApp();

    void onInit();
    int onExit();
    void mainLoop();
    void newFrame();

    GLWindow* GetEngineWnd() const { return engineWnd.get(); }

    int GetWidth() const { return (width); }

    int GetHeight() const { return (height); }

    void SetWidth(int v) { width = v; }

    void SetHeight(int v) { height = v; }

    void SetWindowShouldClose(bool v) { windowShouldClose = v; }

    const char* GetWindowTitle() const { return (windowTitle.c_str()); }

  private:
    void frameInit();
    int frameExit();

    int xPosition, yPosition;
    int width, height;
    bool windowShouldClose;
    std::string windowTitle;

    GLFWwindow* mainGLFWwnd;
    std::unique_ptr<GLWindow> engineWnd;
};

#endif
