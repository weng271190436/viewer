#ifndef VIEWER_MAIN_WIDGET_H__
#define VIEWER_MAIN_WIDGET_H__

#include <QGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QTime>

#include "Eigen/Dense"
#include <map>
#include <string>
#include <vector>

#include "../base/mesh.h"
#include "camera.h"
#include "probe.h"

using Eigen::Vector3d;
using std::vector;

class ImageRenderer;

struct Configuration {
  std::string data_directory;

  std::string ListFilename() const {
    return data_directory + "/list.txt";
  }

  std::string MeshFilename(const std::string& filename) const {
    return data_directory + "/" + filename;
  }
};

class MainWidget : public QGLWidget, protected QOpenGLFunctions {
Q_OBJECT
public:
    explicit MainWidget(const Configuration& configuration,
                        ImageRenderer* image_renderer,
                        QWidget *parent = 0);
    ~MainWidget();

 private:
    bool ReadMeshes();
    void InitializeCameraProbe();

    void AllocateResources();
    void FreeResources();

    virtual void initializeGL();
    void resizeGL(int w, int h);
    void SetMatrices();
    void paintGL();
    void PaintHeart();
    float DistFromPlane( Vector3d P, Vector3d N, float planeD );
    bool GetSegmentPlaneIntersection( Vector3d P1, Vector3d P2, Vector3d& outP,
                                   Vector3d N, float planeD);
    void TrianglePlaneIntersection(Vector3d triA, Vector3d triB, Vector3d triC,
                               vector<Vector3d>& outSegTips, Vector3d N, float planeD);
    void PaintCrossSection();
    void PaintProbe();

    //------------------------------------------------------------------------------------------
    // Wei, this is the function for you to use.
    void GetProbeTriangle(Eigen::Vector3d& center, Eigen::Vector3d& top, Eigen::Vector3d& bottom) const;
    //------------------------------------------------------------------------------------------
    void GetFromToUp(Eigen::Vector3d& from, Eigen::Vector3d& to, Eigen::Vector3d& up) const;

    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void wheelEvent(QWheelEvent* e);
    // void timerEvent(QTimerEvent *e);

    //----------------------------------------------------------------------
    ImageRenderer* image_renderer_;
    Configuration configuration_;

    QVector2D mouse_press_position_;
    QVector2D mouse_move_position_;
    bool mouse_down_;

    Eigen::Vector3f background_;
    int current_width_;
    int current_height_;

    std::vector<Mesh> meshes_;

    Camera camera_;
    Probe probe_;

    // when set to true, PaintCrossSection() is called
    // when set to false, PaintHeart() is called
    bool cross_section_or_heart = false;
};

#endif // VIEWER_MAIN_WIDGET_H__
