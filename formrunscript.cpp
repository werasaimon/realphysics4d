#include "formrunscript.h"
#include "ui_formrunscript.h"

FormRunScript::FormRunScript(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FormRunScript)
{
    ui->setupUi(this);
}

FormRunScript::~FormRunScript()
{
    delete ui;
}


#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)

void FormRunScript::keyPressEvent(QKeyEvent *event)
{
    ui->widget->keyPressEvent(event);
}

void FormRunScript::keyReleaseEvent(QKeyEvent *event)
{
    ui->widget->keyReleaseEvent(event);
}

#endif
