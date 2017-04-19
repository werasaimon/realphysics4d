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

private:

    Ui::FormRunScript *ui;
};

#endif // FORMRUNSCRIPT_H
