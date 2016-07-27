#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QGridLayout>

#include <iostream>
#include <fstream>
#include <string>

#include "image_renderer.h"
#include "main_widget.h"

using namespace std;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    exit (1);
  }
  QApplication app(argc, argv);
  app.setApplicationName("viewer");

  QWidget window;
  // window.resize(1024, 720);
   window.resize(1440, 720);
  // window.resize(1000, 600);

  Configuration configuration;
  {
    configuration.data_directory = argv[1];
  }

  ImageRenderer* image_renderer = new ImageRenderer();
  MainWidget* main_widget = new MainWidget(configuration, image_renderer);
  ImageRenderer* image_renderer2 = new ImageRenderer();
  MainWidget* main_widget2 = new MainWidget(configuration, image_renderer);

  //QHBoxLayout* layout = new QHBoxLayout();
  QGridLayout* layout = new QGridLayout();
  layout->addWidget(main_widget, 0, 0);
  layout->addWidget(image_renderer, 0, 1);
  layout->addWidget(main_widget2, 1, 0);
  layout->addWidget(image_renderer2, 1, 1);
  window.setLayout(layout);
  window.show();

  return app.exec();
}
