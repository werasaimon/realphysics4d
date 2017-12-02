#include "widget.h"
#include <QApplication>

#include <GL/freeglut.h>

int main(int argc, char *argv[])
{

    glutInit( &argc , argv );

    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}
