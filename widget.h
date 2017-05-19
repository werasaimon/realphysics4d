#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "formrunscript.h"
#include <QFileDialog>


namespace Ui
{
 class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
            ~Widget();

//    void keyPressEvent(QKeyEvent *event);
//    void keyReleaseEvent(QKeyEvent *event);



private slots:

     void runScript();
     void fileDialog();
     void saveFile();


private:

    //------------- Attribute --------------//

    Ui::Widget *ui;
    FormRunScript *widget;


    QString mStringFilePathName;
    bool    mIsActiveButtonRunScript = false;
};

#endif // WIDGET_H
