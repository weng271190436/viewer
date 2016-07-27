#ifndef VIEWER_IMAGE_RENDERER_H_
#define VIEWER_IMAGE_RENDERER_H_

#include <Eigen/Dense>
#include <QGLWidget>
#include <QOpenGLFunctions>
#include "../base/mesh.h"
#include "camera.h"
#include "probe.h"

using Eigen::Vector3d;
using std::vector;

class ImageRenderer : public QGLWidget, protected QOpenGLFunctions {
  Q_OBJECT
public:
  ImageRenderer(QWidget* parent = 0);
  ~ImageRenderer();
  void PassMeshPlane(Vector3d center, Vector3d top, 
    Vector3d bottom, vector<Mesh>& meshes_);
  bool first_time = true;

private:
  virtual void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void GetFromToUp(Eigen::Vector3d& from, Eigen::Vector3d& to, Eigen::Vector3d& up) const;
  void SetMatrices();
  void UseData();
  float DistFromPlane( Vector3d P, Vector3d N, float planeD );
  bool GetSegmentPlaneIntersection( Vector3d P1, Vector3d P2, Vector3d& outP,
                                   Vector3d N, float planeD);
  void TrianglePlaneIntersection(Vector3d triA, Vector3d triB, Vector3d triC,
    vector<Vector3d>& outSegTips, Vector3d N, float planeD);

  Eigen::Vector3d center_;
  Eigen::Vector3d top_;
  Eigen::Vector3d bottom_;
  std::auto_ptr<std::vector<Mesh>> meshes_;
 
  int current_width_;
  int current_height_;
  Eigen::Vector3f background_;
  Camera camera_;
  Probe probe_;
};

#endif  // VIEWER_IMAGE_RENDERER_H_
