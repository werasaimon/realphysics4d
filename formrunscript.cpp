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
