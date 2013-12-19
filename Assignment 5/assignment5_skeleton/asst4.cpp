////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////
//	These skeleton codes are later altered by Ming Jin,
//	for "CS6533: Interactive Computer Graphics", 
//	taught by Prof. Andy Nealen at NYU-Poly
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <list>
#include <fstream>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "rigtform.h"
#include "geometrymaker.h"
#include "arcball.h"
#include "ppm.h"
#include "glsupport.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "sgutils.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
static int g_view = 0; // Sky camera frame: 0, cube 1 frame: 1, cube 2 frame: 2
static int g_skyFrame = 0;
static int g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
static int g_msBetweenKeyFrames = 2000;
static int g_animateFramesPerSecond = 60;
static int g_ms = 0;
static bool g_renderArcball = true;
static bool g_picking = false;
static bool g_animating = false;

static const int PICKING_SHADER = 2; // index of the picker shader is g_shaderFiles
static const int g_numShaders = 3; // 3 shaders instead of 2.
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
  {}

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex& v) {
    *this = v;
  }

  VertexPN& operator = (const GenericVertex& v) {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry {
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState& curSS) {
    // Enable the attributes used by our shader
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

    // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);
  }
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space

static RigTForm g_arcball;
static Cvec3f g_arcballColors = Cvec3f(0, 1, 0); // Green
static double g_arcballScale;
static Cvec3 g_v1 = Cvec3(0, 0, 0);

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_viewNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking.
static vector<shared_ptr<SgRbtNode> > g_nodes;
static list<vector<RigTForm> > g_keyFrames; // List of vectors of SgRbtNodes.
static list<vector<RigTForm> >::iterator g_keyFrameIter = g_keyFrames.begin();

///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initArcball() {
  float radius = 1;
  int slices = 20;
  int stacks = 20;
  int ibLen, vbLen;
  getSphereVbIbLen(slices, stacks, vbLen, ibLen);

  // Temporary storage for sphere geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeSphere(radius, slices, stacks, vtx.begin(), idx.begin());
  g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static void drawStuff(const ShaderState& curSS, bool picking) {
  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the specified frame as the eyeRbt
  if (g_view == 0) {
    g_viewNode = g_skyNode;
  }
  else if (g_view == 1) {
    g_viewNode = g_robot1Node;
  }
  else {
    g_viewNode = g_robot2Node;
  }
  const RigTForm eyeRbt = getPathAccumRbt(g_world,g_viewNode);
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  // draw ground and cubes
  // ===========
  //
  if (!picking) {
    Drawer drawer(invEyeRbt, curSS);
    g_world->accept(drawer);
    // Draw Arcball
    if (g_renderArcball && !(g_currentPickedRbtNode == g_skyNode && g_skyFrame  == 1) && !(g_currentPickedRbtNode == g_skyNode && g_viewNode != g_skyNode)  && !(g_currentPickedRbtNode != g_skyNode && g_viewNode == g_currentPickedRbtNode)) {
      if (g_currentPickedRbtNode == g_skyNode) {
        g_arcball = RigTForm();
      }
      else {
        g_arcball = getPathAccumRbt(g_world,g_currentPickedRbtNode);
      }
      if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))) {
        g_arcballScale = getScreenToEyeScale((invEyeRbt * g_arcball).getTranslation()[2], g_frustFovY, g_windowHeight);
      }
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // draw wireframe
      Matrix4 scaleMatrix = Matrix4::makeScale(Cvec3(g_arcballScale*g_arcballScreenRadius,g_arcballScale*g_arcballScreenRadius,g_arcballScale*g_arcballScreenRadius));
      Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_arcball) * scaleMatrix;
      Matrix4 NMVM = normalMatrix(MVM);
      sendModelViewNormalMatrix(curSS, MVM, NMVM);
      safe_glUniform3f(curSS.h_uColor, g_arcballColors[0], g_arcballColors[1], g_arcballColors[2]);
      g_sphere->draw(curSS);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // draw filled again
    }
  }
  else {
    Picker picker(invEyeRbt, curSS);
    g_world->accept(picker);
    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode) {
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
    }
    if (g_currentPickedRbtNode == NULL) {
      g_currentPickedRbtNode = g_viewNode;
    }
  }
}

static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff(*g_shaderStates[g_activeShader], false);

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
  const double scale = (g_currentPickedRbtNode == g_skyNode && g_viewNode == g_skyNode && g_skyFrame == 1) ? .01 : g_arcballScale;
  RigTForm m;
  RigTForm eyeRbt = getPathAccumRbt(g_world,g_viewNode);
  RigTForm invEyeRbt = inv(eyeRbt);
  // Allow translation and rotation when the object is not the sky camera, or if it is, allow it ony if the eye frame is also the sky camera.
  if (!(g_currentPickedRbtNode == g_skyNode && g_viewNode != g_skyNode)) {
    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
      if (g_currentPickedRbtNode != g_viewNode || (g_currentPickedRbtNode == g_viewNode && g_viewNode == g_skyNode && g_skyFrame == 0)) {
        Cvec2 arcballCoord;
        if (g_currentPickedRbtNode == g_skyNode) {
          arcballCoord = getScreenSpaceCoord((invEyeRbt * RigTForm()).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
        }
        else {
          arcballCoord = getScreenSpaceCoord((invEyeRbt * getPathAccumRbt(g_world, g_currentPickedRbtNode)).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
        }
        double xCoord = x - arcballCoord[0];
        double yCoord = (g_windowHeight - y - 1) - arcballCoord[1];
        // x**2 + y**2 + z**2 = r**2
        double zSquared = g_arcballScreenRadius * g_arcballScreenRadius - xCoord * xCoord - yCoord * yCoord;
        Cvec3 sphereCoord;
        // Mouse is off of the arcball.  Trackball movement.
        if (zSquared < 0) {
          //sphereCoord == v2
          sphereCoord = Cvec3(xCoord, yCoord, 0);
        }
        else {
          //sphereCoord == v3
          sphereCoord = Cvec3(xCoord, yCoord, sqrt(zSquared));
        }
        sphereCoord.normalize();
        if (g_v1[0] != 0 || g_v1[1] != 0 || g_v1[2] != 0) {
          // [0 sphereCoord][0 -v1]
          m = RigTForm(Quat(0, sphereCoord[0], sphereCoord[1], sphereCoord[2]) * Quat(0, -g_v1[0], -g_v1[1], -g_v1[2]));
        }
        g_v1 = sphereCoord;
      }
      // Arcball is not used.
      else {
        m = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
      }
    }
    else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
      m = RigTForm(Cvec3(dx,dy,0) * scale);
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
      m = RigTForm(Cvec3(0,0,-dy) * scale);
    }
  }
  RigTForm auxiliaryFrame;
  // Manipulations to m can be done down here rather than being repeted for each type of click.
  if (g_mouseClickDown) {
    if (g_currentPickedRbtNode == g_skyNode && g_skyFrame == 0 && g_viewNode == g_skyNode) { // World-Sky Frame
      m = inv(m);
      auxiliaryFrame.setRotation(eyeRbt.getRotation());
      g_currentPickedRbtNode->setRbt(auxiliaryFrame * m * inv(auxiliaryFrame) * g_currentPickedRbtNode->getRbt());
    }
    else if (g_currentPickedRbtNode == g_viewNode) { // Ego motion
      m.setRotation(inv(m).getRotation());
      auxiliaryFrame.setTranslation(eyeRbt.getTranslation());
      auxiliaryFrame.setRotation(eyeRbt.getRotation());
      g_currentPickedRbtNode->setRbt(auxiliaryFrame * m * inv(auxiliaryFrame) * g_currentPickedRbtNode->getRbt());
    }
    else {
      RigTForm transfact = getPathAccumRbt(g_world, g_currentPickedRbtNode);
      RigTForm linfact = getPathAccumRbt(g_world, g_viewNode);
      auxiliaryFrame = RigTForm(transfact.getTranslation(), linfact.getRotation());
      RigTForm parentRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode, 1);
      auxiliaryFrame = inv(parentRbt) * auxiliaryFrame;
      g_currentPickedRbtNode->setRbt(auxiliaryFrame * m * inv(auxiliaryFrame) * g_currentPickedRbtNode->getRbt());
    }
    glutPostRedisplay(); // we always redraw if we changed the scene
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  // using PICKING_SHADER as the shader
  glUseProgram(g_shaderStates[PICKING_SHADER]->program);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(*g_shaderStates[PICKING_SHADER], true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  //glutSwapBuffers();
  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
  checkGlErrors();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

  const double HEAD_RADIUS = 0.5,
               ARM_LEN = 0.7,
               ARM_THICK = 0.25,
               TORSO_LEN = 1.5,
               TORSO_THICK = 0.25,
               TORSO_WIDTH = 1;
  const int NUM_JOINTS = 12,
            NUM_SHAPES = 12; // Torso, Head, Upper Arm x2, Lower Arm x2, Upper Leg x2, Lower Leg x2.

  struct JointDesc {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
    {-1}, // torso
    {0,  TORSO_WIDTH/2, TORSO_LEN/2 - ARM_THICK/2, 0}, // upper right arm
    {1,  ARM_LEN, 0, 0}, // lower right arm
    {0,  -TORSO_WIDTH/2, TORSO_LEN/2 - ARM_THICK/2, 0}, // upper left arm
    {3,  -ARM_LEN, 0, 0}, // lower left arm
    {0,  TORSO_WIDTH/2 - ARM_THICK/2, -TORSO_LEN/2, 0}, // upper right leg
    {5,  0, -ARM_LEN, 0}, // lower right leg
    {6,  0, -ARM_LEN, 0}, // right foot
    {0,  -TORSO_WIDTH/2 + ARM_THICK/2, - TORSO_LEN/2, 0}, // upper left leg
    {8,  0, -ARM_LEN, 0}, // lower left leg
    {9,  0, -ARM_LEN, 0}, // left foot
    {0,  0, TORSO_LEN/2, 0}, // head
  };

  struct ShapeDesc {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
    {0,  0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
    {1,  ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
    {2,  ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
    {3,  -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper left arm
    {4,  -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
    {5,  0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // upper right leg
    {6,  0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // lower right leg
    {7,  0, -ARM_THICK/2, -ARM_LEN/2 + ARM_THICK/2, ARM_THICK, ARM_THICK, ARM_LEN, g_cube}, // right foot
    {8,  0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // upper left leg
    {9,  0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // lower left leg
    {10, 0, -ARM_THICK/2, -ARM_LEN/2 + ARM_THICK/2, ARM_THICK, ARM_THICK, ARM_LEN, g_cube}, // left foot
    {11, 0, HEAD_RADIUS, 0, HEAD_RADIUS/2, HEAD_RADIUS, HEAD_RADIUS/2, g_sphere}, // head
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));;
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i) {
    shared_ptr<MyShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      color,
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
  constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

  g_currentPickedRbtNode = g_robot1Node;
  g_viewNode = g_skyNode;

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
}

static void reset() {
  g_view = 0;
  g_skyFrame = 0;
  initScene();
  g_viewNode = g_skyNode;
  g_currentPickedRbtNode = g_robot1Node;
  g_renderArcball = true;
  g_keyFrames.clear();
  g_nodes.clear();
  g_keyFrameIter = g_keyFrames.begin();
  g_animating = false;
  g_ms = 0;
  g_msBetweenKeyFrames = 2000;
  cout << "Reset objects and modes to defaults" << endl;
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  // Reset v1
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    g_v1 = Cvec3(0,0,0);

    if (g_picking) {
      pick();
      g_picking = false;
    }
  }
  glutPostRedisplay();
}

static void sceneGraphToKeyFrame(vector<RigTForm>& frame) {
  for (int i = 0; i < g_nodes.size(); i++) {
    frame[i] = g_nodes[i]->getRbt();
  }
}


static void keyFrameToSceneGraph() {
  if (!g_keyFrames.empty()) {
    g_nodes.clear();
    dumpSgRbtNodes(g_world, g_nodes);
    for (int i = 0; i < g_nodes.size(); i++) {
      g_nodes[i]->setRbt((*g_keyFrameIter)[i]);
    }
  }
  else {
    cout << "Current keyFrame is not defined" << endl;
  }
}

static void newKeyFrame() {
  // Increment the iterator if the the current frame is defined.
  if (!g_keyFrames.empty()) {
    g_keyFrameIter++;
  }
  // create the new frame.
  dumpSgRbtNodes(g_world,g_nodes);
  vector<RigTForm> newFrame(g_nodes.size());
  sceneGraphToKeyFrame(newFrame);
  // Add the new frame.
  g_keyFrames.insert(g_keyFrameIter, newFrame);
  g_keyFrameIter--;
  cout << "Added new keyframe" << endl;
}

static void updateKeyFrame() {
  // If the keyframe is defined, pop the RBT nodes and push the new ones.
  if (!g_keyFrames.empty()) {
    g_nodes.clear();
    dumpSgRbtNodes(g_world,g_nodes);
    sceneGraphToKeyFrame(*g_keyFrameIter);
    cout << "Updated keyframe" << endl;
  }
  else {
    newKeyFrame();
  }
}

static void deleteKeyFrame() {
  // Don't do anything if the list is empty.
  if (g_keyFrames.empty()) {
    cout << "Keyframe is not defined" << endl;
    return;
  }
  // Set the next keyframe.
  list<vector<RigTForm> >::iterator iter;
  iter = g_keyFrameIter;
  if (g_keyFrameIter == g_keyFrames.begin()) {
    iter++;
  }
  else {
    iter--;
  }
  // Erase the keyframe and move the iterator.
  g_keyFrames.erase(g_keyFrameIter);
  g_keyFrameIter = iter;
  cout << "Keyframe deleted" << endl;
  // Copy RBT nodes to the scenegraph if the keyframe is defined.
  if (g_keyFrameIter != g_keyFrames.end()) {
    keyFrameToSceneGraph();
  }
}

static void previousKeyFrame() {
  if (g_keyFrameIter != g_keyFrames.begin()) {
    g_keyFrameIter--;
    keyFrameToSceneGraph();
    cout << "Previous keyframe" << endl;
  }
  else {
    if (g_keyFrameIter == g_keyFrames.end()) {
      cout << "Keyframe is not defined" << endl;
    }
    else {
      cout << "Keyframe is at beginning" << endl;
    }
  }
}

static void nextKeyFrame() {
  if (g_keyFrameIter != g_keyFrames.end()) {
    g_keyFrameIter++;
    if (g_keyFrameIter != g_keyFrames.end()) {
      keyFrameToSceneGraph();
      cout << "Next keyframe" << endl;
    }
    else {
      g_keyFrameIter--;
      cout << "Keyframe is the last keyframe" << endl;
    }
  }
  else {
    cout << "Keyframe is not defined" << endl;
  }
}

static void inputKeyFrames(char* filename) {
  ifstream file(filename);
  if (!file.good())
  {
    cout << "No readable file available" << endl;
    return;
  }
  int numFrames;
  int rbtPerFrame;
  file >> numFrames >> rbtPerFrame;
  g_keyFrames.clear();
  for (int i = 0; i < numFrames; i++) {
      vector<RigTForm> newFrame(rbtPerFrame);
    for (int j = 0; j < rbtPerFrame; j++) {
      Cvec3 trans = Cvec3();
      Quat rotation = Quat();
      file >> trans[0] >> trans[1] >> trans[2] >> rotation[0] >> rotation[1] >> rotation[2] >> rotation[3];
      newFrame[j] = RigTForm(trans, rotation);
    }
    g_keyFrames.push_back(newFrame);
  }
  file.close();
  g_keyFrameIter = g_keyFrames.begin();
  g_nodes.clear();
  dumpSgRbtNodes(g_world, g_nodes);
  keyFrameToSceneGraph();
  cout << "KeyFrames loaded" << endl;
}

static void outputKeyFrames(char *filename) {
  // Line 0: Number of frames and number of RBTs per frame.
  // Line N, N>0: Traslation factor and Rotation factor of frame N-1
  // Values delimited by ' '
  if (g_keyFrames.empty()) {
    cout << "No keyframes to store" << endl;
    return;
  }
  ofstream file(filename);
  int numFrames = g_keyFrames.size();
  int rbtPerFrame = g_nodes.size();
  file << numFrames << " " << rbtPerFrame << endl;
  for (list<vector<RigTForm> >::iterator iter = g_keyFrames.begin(); iter != g_keyFrames.end(); iter++) {
    for (int i = 0; i < rbtPerFrame; i++) {
      Cvec3 trans = (*iter)[i].getTranslation();
      Quat rotation = (*iter)[i].getRotation();
      file << trans[0] << " " << trans[1] << " " << trans[2] << " " << rotation[0] << " " << rotation[1] << " " << rotation[2] << " " << rotation[3] << "\n";
    }
  }
  file.close();
  cout << "Keyframes saved" << endl;
}

bool interpolateAndDisplay(float t) {
  // 0 to n - 1 are shown.  n is number of frames - 1.  n - 1 is number of frames - 2.  Iter 2 goes up to number of frames - 2, so iter 1 goes up to number of frames - 3
  if (t == g_keyFrames.size() - 3) {
    return true;
  }
  else {
    t++;
    double alpha = t - floor(t);
    list<vector<RigTForm> >::iterator iter1 = g_keyFrames.begin();
    for (int i = 0; i < floor(t); i++) {
      iter1++;
    }
    list<vector<RigTForm> >::iterator iter2 = iter1;
    iter2++;
    g_nodes.clear();
    dumpSgRbtNodes(g_world, g_nodes);
    for (int i = 0; i < g_nodes.size(); i++) {
      g_nodes[i]->setRbt(lerp((*iter1)[i], (*iter2)[i], alpha));
    }
    glutPostRedisplay();
    return false;
  }
}

static void animateTimerCallback(int ms) {
  float t = (float) ms/(float) g_msBetweenKeyFrames;
  if (g_animating) {
    g_ms = ms;
    bool endReached = interpolateAndDisplay(t);
    if (!endReached) {
      glutTimerFunc(1000/g_animateFramesPerSecond, animateTimerCallback, ms + 1000/g_animateFramesPerSecond);
    }
    else {
      cout << "Animation complete" << endl;
      g_keyFrameIter = g_keyFrames.end();
      g_keyFrameIter--;
      g_animating = false;
      g_ms = 0;
    }
  }
}

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "v\t\tCycle view\n"
    << "a\t\tToggle arcball rendering\n"
    << "m\t\tToggle sky frame"
    << "p\t\tPicker mode\n"
    << "space\t\tCopy current keyframe to the scenegraph\n"
    << "u\t\tUpdate keyframe\n"
    << ">\t\tNext keyframe\n"
    << "<\t\tPrevious keyframe\n"
    << "d\t\tDelte keyframe\n"
    << "n\t\tCreate new keyframe\n"
    << "i\t\tInput keyframes from file\n"
    << "w\t\tOutput keyframes to file\n"
    << "y\t\tPlay/stop animation\n"
    << "+\t\tIncrease animation speed\n"
    << "-\t\tDecrease animation speed\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  case 'v':
    g_view = (g_view + 1) % 3;
    if (g_view == 0) {
      cout << "Sky node view" << endl;
    }
    else if (g_view == 1) {
      cout << "Robot 1 view" << endl;
    }
    else {
      cout << "Robot 2 view" << endl;
    }
    break;
  case 'm':
    if (g_viewNode == g_skyNode && g_currentPickedRbtNode == g_skyNode) {
      g_skyFrame = (g_skyFrame + 1) % 2;
      if (g_skyFrame == 0) {
        cout << "World-sky frame" << endl;
      }
      else {
        cout << "Sky-sky frame" << endl;
      }
    }
    break;
  case 'r':
    reset();
    break;
  case 'a':
    g_renderArcball = !g_renderArcball;
    if (g_renderArcball) {
      cout << "Render arcball on " << endl;
    }
    else {
      cout << "Render arcball off" << endl;
    }
    break;
  case 'p':
    g_picking = !g_picking;
    if (g_picking) {
      cout << "Pick mode on" << endl;
    }
    else {
      cout << "Pick mode off" << endl;
    }
    break;
  case ' ':
    keyFrameToSceneGraph();
    break;
  case 'u':
    updateKeyFrame();
    break;
  case '>':
    nextKeyFrame();
    break;
  case '<':
    previousKeyFrame();
    break;
  case 'd':
    deleteKeyFrame();
    break;
  case 'n':
    newKeyFrame();
    break;
  case 'i':
    inputKeyFrames("keyFrames.txt");
    break;
  case 'w':
    outputKeyFrames("keyFrames.txt");
    break;
  case 'y':
    if (g_keyFrames.size() < 4) {
      cout << "Not enough keyframes" << endl;
    }
    else {
        g_animating = !g_animating;
        if (g_animating) {
          cout << "Animation Playing" << endl;
          animateTimerCallback(g_ms);
        }
        else {
          cout << "Animation paused" << endl;
        }
    }
    break;
  case '+':
    if (g_msBetweenKeyFrames - 100 >= 100) {
      g_msBetweenKeyFrames -= 100;
      cout << "Speed increased: time between frames decreased by 100 ms" << endl;
    }
    else {
      cout << "Speed cannot be increased: time between frames remains at 100 ms" << endl;
    }
    break;
  case '-':
    g_msBetweenKeyFrames += 100;
    cout << "Speed decreased: time between frames increased by 100 ms" << endl;
    break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 5");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initCubes();
  initArcball();
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();
    initScene();
    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
