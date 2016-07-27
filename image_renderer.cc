#include "image_renderer.h"

#include <fstream>
#include <iostream>
#include <QMouseEvent>

#include <OpenGL/glu.h>

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

ImageRenderer::ImageRenderer(QWidget* parent) :
  QGLWidget(parent) {
  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);

  background_ = Vector3f(0, 0, 0);
}

ImageRenderer::~ImageRenderer() {
}

void ImageRenderer::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(background_[0], background_[1], background_[2], 0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  
}

void ImageRenderer::paintGL() {

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
  

  UseData();
}

void ImageRenderer::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  if (w != current_width_ || h != current_height_) {
    current_width_  = w;
    current_height_ = h;
  }
}

void ImageRenderer::GetFromToUp(Eigen::Vector3d& from, Eigen::Vector3d& to, Eigen::Vector3d& up) const {
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

void ImageRenderer::SetMatrices() {
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

void ImageRenderer::UseData() {
  if (first_time) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    Vector3d center(0.5, 0.0, 0.0);
    Vector3d top(0.0, 0.5, 0.0);
    Vector3d bottom(0.0, 0.0, 0.0);

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
  } else {
    //std::cout << "haha" << std::endl;
    SetMatrices();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    Vector3d N = (top_ - bottom_).cross(top_ - center_).normalized();
    float planeD = -top_.dot(N);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLineWidth(5);

    glBegin(GL_LINES);
    for (int mesh_index = 0; mesh_index < (int)meshes_->size(); ++mesh_index) {
      std::cout << mesh_index << std::endl;
      const auto& mesh = (*meshes_)[mesh_index];

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
}

void ImageRenderer::PassMeshPlane(Eigen::Vector3d center, Eigen::Vector3d top, 
  Eigen::Vector3d bottom, std::vector<Mesh>& meshes){
  center_ = center;
  top_ = top;
  bottom_ = bottom;
  //std::cout << meshes << std::flush;
  meshes_  = std::auto_ptr<vector<Mesh>>(new vector<Mesh>);
  meshes_->resize(meshes.size());
  for (int mesh_index = 0; mesh_index < (int)meshes.size(); ++mesh_index) {
    meshes_->at(mesh_index) = meshes[mesh_index];
  }
  std::cout << "hey" << std::flush;
}

float ImageRenderer::DistFromPlane( Vector3d P, Vector3d N, float planeD ) {
    return N.dot(P) + planeD;
}

bool ImageRenderer::GetSegmentPlaneIntersection( Vector3d P1, Vector3d P2, Vector3d& outP,
                                   Vector3d N, float planeD) {
  float d1 = DistFromPlane(P1, N, planeD),
        d2 = DistFromPlane(P2, N, planeD);
  if (d1*d2 > 0) // points on the same side of plane
    return false;

  float t = d1/(d1-d2); // 'time' of intersection point on the segment
  outP = P1 + t * (P2 - P1);

  return true;
}

void ImageRenderer::TrianglePlaneIntersection(Vector3d triA, Vector3d triB, Vector3d triC,
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