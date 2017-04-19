#ifndef FORMRUNSCRIPT_H
#define FORMRUNSCRIPT_H

#include <QWidget>

namespace Ui
{
  class FormRunScript;
}

class FormRunScript : public QWidget
{
    Q_OBJECT

public:
    explicit FormRunScript(QWidget *parent = 0);
            ~FormRunScript();



#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)
     void keyPressEvent(QKeyEvent *event);
     void keyReleaseEvent(QKeyEvent *event);
#endif


private:

    Ui::FormRunScript *ui;
};

#endif // FORMRUNSCRIPT_H
