#include "main_widget.h"

#include <fstream>
#include <iostream>
// #include <locale.h>
// #include <math.h>
#include <Eigen/Dense>
#include <QMouseEvent>
#include <OpenGL/glu.h>

#include "image_renderer.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector3f;
using std::cerr;
using std::endl;
using std::ifstream;
using std::max;
using std::min;
using std::string;
using std::vector;

namespace {

Matrix3d RotationX(const double radian) {
  Matrix3d rotation;
  rotation <<
    1, 0, 0,
    0, cos(radian), -sin(radian),
    0, sin(radian), cos(radian);
  return rotation;
}

Matrix3d RotationY(const double radian) {
  Matrix3d rotation;
  rotation <<
    cos(radian), 0, sin(radian),
    0, 1, 0,
    -sin(radian), 0, cos(radian);
  return rotation;
}

Matrix3d RotationZ(const double radian) {
  Matrix3d rotation;
  rotation <<
    cos(radian), -sin(radian), 0,
    sin(radian), cos(radian), 0,
    0, 0, 1;
  return rotation;
}

void SetColor(const int index, Vector3f& diffuse, Vector3f& ambient) {
  switch (index) {
    case 0: {
      ambient = Vector3f(0.2, 0.2, 0.2);
      diffuse = Vector3f(0.8, 0.8, 0.8);
      break;
    }
    case 1: {
      ambient = Vector3f(0.2, 0.0, 0.0);
      diffuse = Vector3f(0.8, 0.0, 0.0);
      break;
    }
    case 2: {
      ambient = Vector3f(0.0, 0.2, 0.0);
      diffuse = Vector3f(0.0, 0.8, 0.0);
      break;
    }
    case 3: {
      ambient = Vector3f(0.0, 0.0, 0.2);
      diffuse = Vector3f(0.0, 0.0, 0.8);
      break;
    }
    case 4: {
      ambient = Vector3f(0.2, 0.2, 0.0);
      diffuse = Vector3f(0.8, 0.8, 0.0);
      break;
    }
    case 5: {
      ambient = Vector3f(0.0, 0.2, 0.2);
      diffuse = Vector3f(0.0, 0.8, 0.8);
      break;
    }
    case 6: {
      ambient = Vector3f(0.2, 0.0, 0.2);
      diffuse = Vector3f(0.8, 0.0, 0.8);
      break;
    }
    default: {
      ambient = Vector3f(0.2, 0.2, 0.2);
      diffuse = Vector3f(0.5, 0.5, 0.5);
    }
  }
}

}  // namespace

MainWidget::MainWidget(const Configuration& configuration,
                       ImageRenderer* image_renderer,
                       QWidget *parent)
  : QGLWidget(parent),
    image_renderer_(image_renderer),
    configuration_(configuration) {
  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);

  background_ = Vector3f(0, 0, 0);
  mouse_down_ = false;

  if (!ReadMeshes())
    return;

  /*
  cerr << meshes_.size() << " files read." << endl;
  for (const auto& mesh : meshes_) {
    cerr << mesh.vertices().size() << ' ' << mesh.faces().size() << endl;
  }
  */

  InitializeCameraProbe();
}

void MainWidget::InitializeCameraProbe() {
  Vector3d center(0, 0, 0);
  {
    int count = 0;
    for (const auto& mesh : meshes_) {
      const auto& vertices = mesh.vertices();
      for (const auto& face : mesh.faces()) {
        for (int i = 0; i < 3; ++i) {
          center += vertices[face[i]];
          ++count;
        }
      }
    }
    if (count != 0)
      center /= count;
  }
  double radius = 0.0;
  {
    int count = 0;
    for (const auto& mesh : meshes_) {
      const auto& vertices = mesh.vertices();
      for (const auto& face : mesh.faces()) {
        for (int i = 0; i < 3; ++i) {
          const double diff = (center - vertices[face[i]]).norm();
          radius += diff * diff;
          ++count;
        }
      }
    }
    if (count != 0) {
      radius /= count;
      radius = sqrt(radius);
      const double kScale = 3.0;
      radius *= kScale;
    }
  }
  const Vector3d kInitialViewDegree(0, 0, 0);
  const double kPerspectiveAngleDegree = 90.0;
  camera_.Initialize(center,
                     radius,
                     kInitialViewDegree,
                     kPerspectiveAngleDegree);

  //----------------------------------------------------------------------
  probe_.lat_lng_degree      = Vector2d(0, 0);
  probe_.rotation_degree     = Vector3d(0, 0, 0);
  probe_.vertical_fov_degree = 90.0;
}

MainWidget::~MainWidget() {
  FreeResources();
}

bool MainWidget::ReadMeshes() {
  ifstream ifstr;
  ifstr.open(configuration_.ListFilename().c_str());
  if (!ifstr.is_open()) {
    return false;
  }

  int num_files;
  ifstr >> num_files;

  meshes_.resize(num_files);
  for (int i = 0; i < num_files; ++i) {
    string filename;
    ifstr >> filename;
    const string full_path = configuration_.MeshFilename(filename);
    if (!meshes_[i].Read(full_path))
      return false;
  }

  ifstr.close();
  return true;
}

void MainWidget::AllocateResources() {
}

void MainWidget::FreeResources() {
}

void MainWidget::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(background_[0], background_[1], background_[2], 0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
}

void MainWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  if (w != current_width_ || h != current_height_) {
    current_width_  = w;
    current_height_ = h;
    FreeResources();
    AllocateResources();
  }
}

void MainWidget::SetMatrices() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  const double kMinRadiusScale = 0.01;
  const double kMaxRadiusScale = 10.0;
  gluPerspective(camera_.perspective_angle_degree_,
                 width() / static_cast<double>(height()),
                 camera_.radius_ * kMinRadiusScale,
                 camera_.radius_ * kMaxRadiusScale);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  Vector3d from, to, up;
  GetFromToUp(from, to, up);
  gluLookAt(from[0], from[1], from[2],
            to[0], to[1], to[2],
            up[0], up[1], up[2]);
}

void MainWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  // glEnable(GL_AUTO_NORMAL);
  // glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  {
    glShadeModel (GL_SMOOTH);
    GLfloat mat_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    glLightfv(GL_LIGHT0, GL_AMBIENT, mat_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, mat_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, mat_specular);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  }

  SetMatrices();

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!cross_section_or_heart) PaintHeart();
  else PaintCrossSection();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  PaintProbe();
}

void MainWidget::PaintHeart() {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glBegin(GL_TRIANGLES);
  for (int mesh_index = 0; mesh_index < (int)meshes_.size(); ++mesh_index) {
    const auto& mesh = meshes_[mesh_index];

    Vector3f diffuse, ambient;
    SetColor(mesh_index, diffuse, ambient);

    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, &diffuse[0]);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, &ambient[0]);

    const auto& vertices = mesh.vertices();
    for (const auto& face : mesh.faces()) {
      const auto& diff0 = vertices[face[1]] - vertices[face[0]];
      const auto& diff1 = vertices[face[2]] - vertices[face[0]];
      const auto& normal = diff0.cross(diff1).normalized();
      for (int i = 0; i < 3; ++i) {
        glNormal3d(normal[0], normal[1], normal[2]);
        glVertex3d(vertices[face[i]][0], vertices[face[i]][1], vertices[face[i]][2]);
      }
    }
  }
  glEnd();
  glPopMatrix();
}

float MainWidget::DistFromPlane( Vector3d P, Vector3d N, float planeD ) {
    return N.dot(P) + planeD;
}

bool MainWidget::GetSegmentPlaneIntersection( Vector3d P1, Vector3d P2, Vector3d& outP,
                                   Vector3d N, float planeD) {
  float d1 = DistFromPlane(P1, N, planeD),
        d2 = DistFromPlane(P2, N, planeD);
  if (d1*d2 > 0) // points on the same side of plane
    return false;

  float t = d1/(d1-d2); // 'time' of intersection point on the segment
  outP = P1 + t * (P2 - P1);

  return true;
}

void MainWidget::TrianglePlaneIntersection(Vector3d triA, Vector3d triB, Vector3d triC,
                               vector<Vector3d>& outSegTips, Vector3d N, float planeD)
{
   Vector3d IntersectionPoint;
   if( GetSegmentPlaneIntersection( triA, triB, IntersectionPoint, N, planeD))
     outSegTips.push_back(IntersectionPoint);

   if( GetSegmentPlaneIntersection( triB, triC, IntersectionPoint, N, planeD))
     outSegTips.push_back(IntersectionPoint);

   if( GetSegmentPlaneIntersection( triC, triA, IntersectionPoint, N, planeD))
     outSegTips.push_back(IntersectionPoint);
}

void MainWidget::PaintCrossSection() {
  Vector3d center, top, bottom;
  GetProbeTriangle(center, top, bottom);

  Vector3d N = (top - bottom).cross(top - center).normalized();
  float planeD = -top.dot(N);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLineWidth(10);

  glBegin(GL_LINES);
  for (int mesh_index = 0; mesh_index < (int)meshes_.size(); ++mesh_index) {
    const auto& mesh = meshes_[mesh_index];

    Vector3f diffuse, ambient;
    SetColor(mesh_index, diffuse, ambient);

    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, &diffuse[0]);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, &ambient[0]);

    const auto& vertices = mesh.vertices();
    for (const auto& face : mesh.faces()) {
      const auto& diff0 = vertices[face[1]] - vertices[face[0]];
      const auto& diff1 = vertices[face[2]] - vertices[face[0]];
      const auto& normal = diff0.cross(diff1).normalized();
      Vector3d triA = vertices[face[0]];
      Vector3d triB = vertices[face[1]];
      Vector3d triC = vertices[face[2]];
      vector<Vector3d> outSegTips;
      TrianglePlaneIntersection(triA, triB, triC, outSegTips, N, planeD);
      if (outSegTips.size() == 2) {
        for (int i = 0; i < 2; ++i) {
          glNormal3d(normal[0], normal[1], normal[2]);
          glVertex3d(outSegTips[i][0], outSegTips[i][1], outSegTips[i][2]);
        }
      }
    }
  }
  glEnd();
  glPopMatrix();

}

void MainWidget::PaintProbe() {
  Vector3d center, top, bottom;
  GetProbeTriangle(center, top, bottom);

  GLfloat gray[] = { 0.2, 0.2, 0.2 };
  GLfloat white[] = { 1.0, 1.0, 1.0 };

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, white);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, gray);

  glBegin(GL_TRIANGLES);

  glVertex3d(center[0], center[1], center[2]);
  glVertex3d(top[0], top[1], top[2]);
  glVertex3d(bottom[0], bottom[1], bottom[2]);

  glVertex3d(center[0], center[1], center[2]);
  glVertex3d(bottom[0], bottom[1], bottom[2]);
  glVertex3d(top[0], top[1], top[2]);

  glEnd();

  /*
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glTranslated(camera_.center_[0], camera_.center_[1], camera_.center_[2]);

  glRotated(probe_.lat_lng_degree[0], 1, 0, 0);
  glRotated(probe_.lat_lng_degree[1], 0, 1, 0);

  const double kScale = 0.50;
  glTranslated(0, 0, - camera_.radius_ * kScale);

  glRotated(probe_.rotation_degree[0], 1, 0, 0);
  glRotated(probe_.rotation_degree[1], 0, 1, 0);
  glRotated(probe_.rotation_degree[2], 0, 0, 1);

  // GLUquadricObj* quadratic = gluNewQuadric();
  // gluSphere(quadratic, camera_.radius_ * 0.05, 10, 10);
  // gluDeleteQuadric(quadratic);
  {
    const double scale = 2.0 * camera_.radius_;
    const double angle = probe_.vertical_fov_degree / 2.0 * M_PI / 180.0;
    glScaled(scale, scale, scale);

    GLfloat gray[] = { 0.2, 0.2, 0.2 };
    GLfloat white[] = { 1.0, 1.0, 1.0 };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, gray);

    glBegin(GL_TRIANGLES);

    glVertex3d(0, 0, 0);
    glVertex3d(0, sin(angle), cos(angle));
    glVertex3d(0, -sin(angle), cos(angle));

    glVertex3d(0, 0, 0);
    glVertex3d(0, -sin(angle), cos(angle));
    glVertex3d(0, sin(angle), cos(angle));

    glEnd();
  }

  glPopMatrix();
  */
}

void MainWidget::GetProbeTriangle(Eigen::Vector3d& center,
                                  Eigen::Vector3d& top,
                                  Eigen::Vector3d& bottom) const {
  const Matrix3d global_rotation_x = RotationX(probe_.lat_lng_degree[0] * M_PI / 180.0);
  const Matrix3d global_rotation_y = RotationY(probe_.lat_lng_degree[1] * M_PI / 180.0);
  const Matrix3d global_rotation = global_rotation_x * global_rotation_y;

  const double kScale = 0.50;
  Vector3d offset(0, 0, -camera_.radius_ * kScale);
  // center = camera_.center_ + global_rotation * offset;

  const Matrix3d rotation_x = RotationX(probe_.rotation_degree[0] * M_PI / 180.0);
  const Matrix3d rotation_y = RotationY(probe_.rotation_degree[1] * M_PI / 180.0);
  const Matrix3d rotation_z = RotationZ(probe_.rotation_degree[2] * M_PI / 180.0);
  const Matrix3d rotation = rotation_x * rotation_y * rotation_z;

  const double angle = probe_.vertical_fov_degree / 2.0 * M_PI / 180.0;
  const double scale = 2.0 * camera_.radius_;

  center = Vector3d(0, 0, 0);
  top    = Vector3d(0, sin(angle), cos(angle)) * scale;
  bottom = Vector3d(0, -sin(angle), cos(angle)) * scale;

  center = global_rotation * (rotation * center + offset) + camera_.center_;
  top    = global_rotation * (rotation * top    + offset) + camera_.center_;
  bottom = global_rotation * (rotation * bottom + offset) + camera_.center_;
}

void MainWidget::GetFromToUp(Eigen::Vector3d& from, Eigen::Vector3d& to, Eigen::Vector3d& up) const {
  const Matrix3d rotation_x =
      RotationX(camera_.view_degree_[0] * M_PI / 180.0);
  const Matrix3d rotation_y =
      RotationY(camera_.view_degree_[1] * M_PI / 180.0);
  const Matrix3d rotation_z =
      RotationZ(camera_.view_degree_[2] * M_PI / 180.0);

  const Matrix3d transformation = rotation_z * rotation_y * rotation_x;
  from = Vector3d(0, 0, -camera_.radius_);
  to   = Vector3d(0, 0, 0);
  up   = Vector3d(0, 1, 0);

  from = transformation * from;
  to   = transformation * to;
  up   = transformation * up;

  from += camera_.center_;
  to   += camera_.center_;
}

//----------------------------------------------------------------------
// GUI
//----------------------------------------------------------------------
void MainWidget::mousePressEvent(QMouseEvent *e) {
  mouse_press_position_ = QVector2D(e->localPos());
  mouse_down_ = true;

  if (e->button() == Qt::RightButton) {
  }
}

void MainWidget::mouseReleaseEvent(QMouseEvent*) {
  mouse_down_ = false;
}

void MainWidget::mouseMoveEvent(QMouseEvent* e) {
  QVector2D diff = QVector2D(e->localPos()) - mouse_move_position_;
  mouse_move_position_ = QVector2D(e->localPos());

  if (mouse_down_) {
    if (e->modifiers() & Qt::ShiftModifier) {
      diff /= 5.0;
      probe_.lat_lng_degree[0] -= diff[1];
      probe_.lat_lng_degree[1] += diff[0];
    } else if (e->modifiers() & Qt::AltModifier) {
      diff /= 5.0;
      probe_.rotation_degree[0] += diff[1];
      probe_.rotation_degree[1] += diff[0];
    } else {
      diff /= 5.0;
      camera_.view_degree_[0] += diff[1];
      camera_.view_degree_[1] -= diff[0];
    }
  }

  updateGL();

  //  update plane parameters
  Vector3d center, top, bottom;
  GetProbeTriangle(center, top, bottom);

  

  image_renderer_->PassMeshPlane(center, top, bottom, meshes_);

  image_renderer_->updateGL();
  image_renderer_->first_time = false;

}

void MainWidget::mouseDoubleClickEvent(QMouseEvent*) {
  mouse_down_ = true;
}

void MainWidget::keyPressEvent(QKeyEvent* e) {
  //----------------------------------------------------------------------
  // Arrows.
  const double kFast = 5.0;
  const double kSlow = 0.5;

  if (e->key() == Qt::Key_H) {
    cerr << "Try the following UIs." << endl
         << "1. Mouse-drag." << endl
         << "2. Shift + Mouse-drag." << endl
         << "3. Alt + Mouse-drag." << endl
         << "4. Left,Right,Up,Down array keys." << endl
         << "5. Key A." << endl;
  } else if (e->key() == Qt::Key_Up) {
    probe_.rotation_degree[2] += kSlow;
  } else if (e->key() == Qt::Key_Down) {
    probe_.rotation_degree[2] -= kSlow;
  } else if (e->key() == Qt::Key_Left) {
    probe_.rotation_degree[2] -= kFast;
  } else if (e->key() == Qt::Key_Right) {
    probe_.rotation_degree[2] += kFast;
  } else if (e->key() == Qt::Key_A) {
    cross_section_or_heart = !cross_section_or_heart;
  } else if (e->key() == Qt::Key_D) {
  }
  updateGL();
}

void MainWidget::keyReleaseEvent(QKeyEvent*) {
}

void MainWidget::wheelEvent(QWheelEvent* e) {
  const double kMinDegree = 30.0;
  const double kMaxDegree = 150.0;
  if (e->orientation() == Qt::Vertical) {
    camera_.perspective_angle_degree_ =
      max(kMinDegree, min(kMaxDegree, camera_.perspective_angle_degree_ + e->delta()));
    updateGL();
  }
}
