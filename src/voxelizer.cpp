#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"

int main(int argc, char** argv) {
  if (argc == 1) {
    QApplication app(argc, argv);

    // app.setQuitOnLastWindowClosed(false);

    Mainframe frame;
    frame.show();

    return app.exec();
  } else {
    // run standalone for batch generation.
  }
}
