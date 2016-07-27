# Feel free to add more paths here. This should just
# give warnings without errors.

QT       += core gui widgets
CONFIG   += c++11
TARGET   = viewer
TEMPLATE = app

QMAKE_CXXFLAGS += '-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare -Wno-overloaded-virtual -Wno-deprecated-register'

QT += opengl

SOURCES += viewer.cc main_widget.cc image_renderer.cc camera.cc ../base/mesh.cc
HEADERS += main_widget.h image_renderer.h camera.h ../base/mesh.h

#    RESOURCES += \
#        shaders.qrc

INCLUDEPATH += '/Users/dinghba/stash/third_party'
INCLUDEPATH += '/usr/local/include/'
INCLUDEPATH += '/usr/local/Cellar/eigen/3.2.8/include/eigen3'

LIBS += '-F/Users/dinghba/Qt/5.7/clang_64/lib/QtOpenGL.framework/Versions/5/'
LIBS += '-L/usr/local/lib'
LIBS += '-lopencv_core'
LIBS += '-lopencv_imgproc'
LIBS += '-lopencv_highgui'

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target
