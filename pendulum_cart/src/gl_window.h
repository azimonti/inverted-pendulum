#ifndef _GL_WINDOW_H_8A3D71EFF00845FFA4471E34874A6E27_
#define _GL_WINDOW_H_8A3D71EFF00845FFA4471E34874A6E27_

/************************/
/*     gl_window.h      */
/*    Version 1.0       */
/*     2023/04/13       */
/************************/

#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include "camera.h"
#include "pendulum_cart.h"

namespace ge
{
    class Mesh;
    class Scene;
} // namespace ge
  //

namespace wflags
{
    enum WindowFlags : int {
        VERBOSE  = 1 << 0,
        WIREDOBJ = 1 << 1,
        GRID     = 1 << 2,
        F2D      = 1 << 3,
        ANIMATE  = 1 << 4,
    };
}

class GLWindow
{
  public:
    GLWindow();
    virtual ~GLWindow();

    void onInit();
    void onExit();

    void onRender(GLFWwindow* window);
    void onReshape(GLint w, GLint h);

    const ge::Mesh* GetObjRodMesh() const { return mObjRodMesh.get(); }

    ge::Mesh* GetObjRodMesh() { return mObjRodMesh.get(); }

    const ge::Mesh* GetObjPivotMesh() const { return mObjPivotMesh.get(); }

    ge::Mesh* GetObjPivotMesh() { return mObjPivotMesh.get(); }

    const ge::Mesh* GetObjRoundMesh() const { return mObjRoundMesh.get(); }

    ge::Mesh* GetObjRoundMesh() { return mObjRoundMesh.get(); }

    const ge::Camera& GetCamera() const { return mCamera; }

    ge::Camera& GetCamera() { return mCamera; }

    const ge::Scene& GetScene() const { return *moScene; }

    ge::Scene& GetScene() { return *moScene; }

    const ge::InvertedPendulum<float>& GetInvPendulum() const { return *moInvPendulum; }

    ge::InvertedPendulum<float>& GetInvPendulum() { return *moInvPendulum; }

    void SetFlag(int flag);
    void UnsetFlag(int flag);
    void ToggleFlag(int flag);

    void Reset();

    void ToggleInvPendulumFlag(int flag) { moInvPendulum->ToggleFlag(flag); };

    void onMouseButton(int button, int upOrDown, int x, int y);
    void onMouseWheel(int wheel, int direction, int x, int y);
    void onMotion(int x, int y);
    void onPassiveMotion(int x, int y);

    void onKeyDown(int key, int x, int y);
    void onKeyUp(int key, int x, int y);
    void onSpecialKeyUp(int key, int x, int y);
    void onSpecialKeyDown(int key, int x, int y);

  private:
    void MakeObject();
    void Make2dObject();
    void Make3dObject();
    ge::Camera mCamera;
    std::unique_ptr<ge::Mesh> moGridMesh;
    std::unique_ptr<ge::Mesh> mObjRodMesh;
    std::unique_ptr<ge::Mesh> mObjPivotMesh;
    std::unique_ptr<ge::Mesh> mObjRoundMesh;
    std::unique_ptr<ge::Mesh> mObjCartMesh;
    std::unique_ptr<ge::Scene> moScene;
    std::unique_ptr<ge::InvertedPendulum<float>> moInvPendulum;
    int flags;
    float xOrigin{};
};

#endif
