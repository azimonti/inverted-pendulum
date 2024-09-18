/************************/
/*    gl_window.cpp     */
/*    Version 1.0       */
/*     2023/04/13       */
/************************/

#include <algorithm>
#include <array>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdarg.h>
#include "log/log.h"
#include "solvers/runge_kutta.h"
#include "ge_mesh2.h"
#include "ge_mesh3.h"
#include "geomprocessing.h"
#include "geomshapes.h"
#include "gl_window.h"
#include "mesh.h"
#include "scene.h"

GLWindow::GLWindow() : flags(wflags::GRID | wflags::F2D | wflags::ANIMATE), xOrigin(0.f)
{
    const float CAM_ELEVATION = 0.0f; // meters
    const float CAM_DISTANCE  = 9.0f; //
    mCamera.GetTransform().SetMatrix(glm::mat4(1.f));
    mCamera.GetTransform().Translate(-glm::vec3(0.0f, CAM_ELEVATION, CAM_DISTANCE));
}

GLWindow::~GLWindow() {}

void GLWindow::MakeObject()
{
    mObjRodMesh   = std::make_unique<ge::Mesh>(!(flags & wflags::WIREDOBJ));
    mObjPivotMesh = std::make_unique<ge::Mesh>(!(flags & wflags::WIREDOBJ));
    mObjRoundMesh = std::make_unique<ge::Mesh>(!(flags & wflags::WIREDOBJ));
    mObjCartMesh  = std::make_unique<ge::Mesh>(!(flags & wflags::WIREDOBJ));

    if (flags & wflags::F2D) Make2dObject();
    else Make3dObject();

    mObjPivotMesh->OnGeometryUpdate();
    mObjRodMesh->OnGeometryUpdate();
    mObjRoundMesh->OnGeometryUpdate();
    mObjCartMesh->OnGeometryUpdate();
}

void GLWindow::Make2dObject()
{
    const int SHAPE_POINTS        = 100;
    const glm::vec4 SHAPE_PIV_COL = {0.97f, 0.6f, 0.0f, 1.0f};
    const glm::vec4 SHAPE_ROD_COL = {0.9f, 0.6f, 0.1f, 1.0f};
    const glm::vec4 SHAPE_ROU_COL = {0.9f, 0.1f, 0.1f, 1.0f};
    const glm::vec4 SHAPE_CAR_COL = {0.3f, 0.4f, 0.8f, 1.0f};

    const glm::vec3 centerPivot   = {xOrigin, 0.0f, 0.0f};
    const glm::vec3 centerRound   = {xOrigin + moInvPendulum->GetLength() * sin(moInvPendulum->GetTheta()),
                                     -moInvPendulum->GetLength() * cos(moInvPendulum->GetTheta()), 0.0f};
    const glm::vec3 centerRod     = {moInvPendulum->GetLength() / 2.0f, 0.0f, 0.0f};
    const glm::vec3 centerCart    = {xOrigin, 0.0f, 0.0f};

    mObjPivotMesh                 = gegt::MakeCircleMesh(0.1f, SHAPE_POINTS, centerPivot, SHAPE_PIV_COL, gegt::XY);
    mObjRoundMesh                 = gegt::MakeCircleMesh(0.25f, SHAPE_POINTS, centerRound, SHAPE_ROU_COL, gegt::XY);
    mObjRodMesh  = gegt::MakeRectMesh(moInvPendulum->GetLength(), 0.1f, centerRod, SHAPE_ROD_COL, gegt::XY);
    mObjCartMesh = gegt::MakeRectMesh(3.0f, 0.5f, centerCart, SHAPE_CAR_COL, gegt::XY);
    mObjRodMesh->TranslateObjMatrix(glm::vec3(xOrigin, 0.0f, 0.0f));
    mObjRodMesh->RotateObjMatrix(moInvPendulum->GetTheta() - glm::pi<float>() / 2.f, glm::vec3(0.f, 0.f, 1.0f));
}

void GLWindow::Make3dObject()
{

    const int SHAPE_POINTS        = 100;
    const glm::vec4 SHAPE_PIV_COL = {0.97f, 0.6f, 0.0f, 1.0f};
    const glm::vec4 SHAPE_ROD_COL = {0.9f, 0.6f, 0.1f, 1.0f};
    const glm::vec4 SHAPE_ROU_COL = {0.9f, 0.1f, 0.1f, 1.0f};
    const glm::vec4 SHAPE_CAR_COL = {0.3f, 0.4f, 0.8f, 1.0f};

    const glm::vec3 centerPivot   = {xOrigin, 0.0f, 0.0f};
    const glm::vec3 centerRound   = {xOrigin + moInvPendulum->GetLength() * sin(moInvPendulum->GetTheta()),
                                     -moInvPendulum->GetLength() * cos(moInvPendulum->GetTheta()), 0.0f};
    const glm::vec3 centerRod     = {moInvPendulum->GetLength() / 2.0f, 0.0f, 0.0f};
    const glm::vec3 centerCart    = {xOrigin, 0.0f, -0.2f};

    mObjPivotMesh = gegt::MakeCylinderMesh(0.1f, 0.1f, SHAPE_POINTS, centerPivot, SHAPE_PIV_COL, gegt::XY);
    gegp::GenMeshUV(*mObjRoundMesh, (size_t)(SHAPE_POINTS / 4.0f), (size_t)(SHAPE_POINTS / 2.0f), SHAPE_ROU_COL,
                    gegt::make_sphere(0.25, centerRound), true);
    mObjRodMesh  = gegt::MakeCylinderMesh(0.05f, 2.0f, SHAPE_POINTS, centerRod, SHAPE_ROD_COL, gegt::YZ);
    mObjCartMesh = gegt::MakeCylinderMesh(0.25f, 3.0f, SHAPE_POINTS, centerCart, SHAPE_CAR_COL, gegt::YZ);
    mObjRodMesh->TranslateObjMatrix(glm::vec3(xOrigin, 0.0f, 0.0f));
    mObjRodMesh->RotateObjMatrix(moInvPendulum->GetTheta() - glm::pi<float>() / 2.f, glm::vec3(0.f, 0.f, 1.0f));
}

void GLWindow::onInit()
{
    if (flags & wflags::GRID) moGridMesh = gegt::MakeGridMeshWired({10.f, 10.f}, {10, 10}, gegt::XY);
    moInvPendulum = std::make_unique<ge::InvertedPendulum<float>>();
    moInvPendulum->SetFlag(ge::pflags::ASYNC);
    moInvPendulum->SetFlag(ge::pflags::MULTITHREAD);
    moInvPendulum->onInit();
    MakeObject();
    moScene                       = std::make_unique<ge::Scene>();
    moScene->GetDirLight().mDirWS = glm::normalize(glm::vec3(0.1f, 0.6f, 0.3f));
}

void GLWindow::onExit() {}

void GLWindow::onRender(GLFWwindow* window)
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    // skip rendering if the window has no size (or aspect-ratio will blow)
    if (!width || !height) return;

    // camera projection setup
    mCamera.mFOV        = 0;
    mCamera.mOrthoScale = 4.f;
    mCamera.mAspect_woh = (float)width / (float)height;

    ge::RendSetup rsetup;
    rsetup.mpCamera    = &mCamera;
    rsetup.mRTargetSiz = ge::Vec2((float)width, (float)height);
    moScene->BeginRender(rsetup);
    if (flags & wflags::GRID) moScene->AddMesh(*moGridMesh);

    if (flags & wflags::ANIMATE) moInvPendulum->onUpdate();
    xOrigin = moInvPendulum->GetX();
    MakeObject();

    moScene->AddMesh(*mObjCartMesh);
    moScene->AddMesh(*mObjRodMesh);
    moScene->AddMesh(*mObjRoundMesh);
    moScene->AddMesh(*mObjPivotMesh);
}

void GLWindow::Reset()
{
    moInvPendulum->Reset();
    MakeObject();
}

void GLWindow::SetFlag(int flag)
{
    flags |= flag;
    if (flag & wflags::F2D) MakeObject();
}

void GLWindow::UnsetFlag(int flag)
{
    flags &= ~flag;
    if (flag & wflags::F2D) MakeObject();
}

void GLWindow::ToggleFlag(int flag)
{
    flags ^= flag;
    if (flag & wflags::GRID)
    {
        if (flags & wflags::GRID) moGridMesh = gegt::MakeGridMeshWired({10.f, 10.f}, {10, 10}, gegt::XY);
        else moGridMesh.reset();
    }
    if (flag & wflags::VERBOSE) moInvPendulum->ToggleFlag(ge::pflags::VERBOSE);
}

void GLWindow::onReshape(GLint w, GLint h)
{
    (void)w;
    (void)h;
}

void GLWindow::onKeyUp(int key, int x, int y)
{
    (void)key;
    (void)x;
    (void)y;
}

void GLWindow::onSpecialKeyDown(int key, int x, int y)
{
    (void)key;
    (void)x;
    (void)y;
}

void GLWindow::onKeyDown(int key, int x, int y)
{
    (void)key;
    (void)x;
    (void)y;
}

void GLWindow::onSpecialKeyUp(int key, int x, int y)
{
    (void)key;
    (void)x;
    (void)y;
}

void GLWindow::onMouseButton(int button, int upOrDown, int x, int y)
{
    (void)button;
    (void)upOrDown;
    (void)x;
    (void)y;
}

void GLWindow::onMouseWheel(int wheel, int direction, int x, int y)
{
    (void)wheel;
    (void)direction;
    (void)x;
    (void)y;
}

void GLWindow::onMotion(int x, int y)
{
    (void)x;
    (void)y;
}

void GLWindow::onPassiveMotion(int x, int y)
{
    (void)x;
    (void)y;
}
