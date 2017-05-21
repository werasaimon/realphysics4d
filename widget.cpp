#include "widget.h"
#include "ui_widget.h"



#include <QFile>
#include <QTextStream>
#include <QMessageBox>




Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);


    connect( ui->pushButton   , SIGNAL(clicked(bool)) , this , SLOT(runScript()) );
    connect( ui->pushButton_2 , SIGNAL(clicked(bool)) , this , SLOT(fileDialog()) );
    connect( ui->pushButton_3 , SIGNAL(clicked(bool)) , this , SLOT(saveFile()) );
}




Widget::~Widget()
{
    delete ui;
}



//void Widget::keyPressEvent(QKeyEvent *event)
//{
//}

//void Widget::keyReleaseEvent(QKeyEvent *event)
//{
//}


void Widget::runScript()
{

    char fileName[] = "file1.lua";
    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    file.write( ui->textEdit->toPlainText().toUtf8() );
    file.close();


    // this->hide();
   // this->close();

    if( !mIsActiveButtonRunScript )
    {

       widget = new FormRunScript();
       widget->setWindowFlags( Qt::Dialog |
                               Qt::CustomizeWindowHint |
                               Qt::WindowTitleHint |
                               Qt::WindowMinMaxButtonsHint );
       widget->show();
       mIsActiveButtonRunScript = true;
       ui->pushButton->setText("stop");
    }
    else
    {

       widget->close();
       delete widget;
       mIsActiveButtonRunScript = false;
       ui->pushButton->setText("run..");
    }

   file.remove();

}

void Widget::fileDialog()
{

    QString filename = QFileDialog::getOpenFileName( this ,
                                                     tr("Open File") ,
                                                     "//" ,
                                                     "All files (*.*);; Lua-File (*.lua *.txt)");


    mStringFilePathName = filename;
    QFile FileRead(filename);
    FileRead.open( QFile::ReadOnly );
    QTextStream stream( &FileRead );

    ui->textEdit->setText( stream.readAll().toStdString().c_str() );

    //mLuaMashine.runString(stream.readAll().toStdString().c_str());
    //QMessageBox::information( this , tr("File Name") , filename );

}

void Widget::saveFile()
{

    QFile file(mStringFilePathName);
    file.open(QIODevice::WriteOnly);
    file.write( ui->textEdit->toPlainText().toUtf8() );
    file.close();

}
