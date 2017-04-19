#include "UnitSceneLuaInterpretationSDK.h"
#include <QFile>



UnitSceneLuaInterpretationSDK::UnitSceneLuaInterpretationSDK()
{

}

///-------------------------------------------------------///
/// Initilization lua-interpritator
///-------------------------------------------------------///
bool UnitSceneLuaInterpretationSDK::initLua()
{

    mLuaMashine.initialization();
    mLuaMashine.parserClassesLibrary();

    mLuaMashine.importToMethodScope( luabind::class_<UnitSceneLuaInterpretationSDK>("scene")
                                     .def(luabind::constructor<>())
                                     .def("keyDown", &UnitSceneLuaInterpretationSDK::keyDown)
                                     // value
                                     .def_readwrite("width"  , &UnitSceneLuaInterpretationSDK::width)
                                     .def_readwrite("height" , &UnitSceneLuaInterpretationSDK::height));




    ///---------------------- Register all key --------------------------------///
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_Q"]  = int(Qt::Key_Q);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_W"]  = int(Qt::Key_W);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_E"]  = int(Qt::Key_E);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_R"]  = int(Qt::Key_R);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_T"]  = int(Qt::Key_T);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_Y"]  = int(Qt::Key_Y);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_U"]  = int(Qt::Key_U);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_I"]  = int(Qt::Key_I);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_O"]  = int(Qt::Key_O);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_P"]  = int(Qt::Key_P);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_A"]  = int(Qt::Key_A);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_S"]  = int(Qt::Key_S);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_D"]  = int(Qt::Key_D);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_F"]  = int(Qt::Key_F);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_G"]  = int(Qt::Key_G);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_H"]  = int(Qt::Key_H);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_J"]  = int(Qt::Key_J);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_K"]  = int(Qt::Key_K);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_L"]  = int(Qt::Key_L);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_Z"]  = int(Qt::Key_Z);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_X"]  = int(Qt::Key_X);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_C"]  = int(Qt::Key_C);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_V"]  = int(Qt::Key_V);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_B"]  = int(Qt::Key_B);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_N"]  = int(Qt::Key_N);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["Key_M"]  = int(Qt::Key_M);



}

///------------------- initilization GL --------------------///
/// \brief UnitSceneLuaInterpretationSDK::initGL
/// \return
bool UnitSceneLuaInterpretationSDK::initGL()
{

    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[luabind::def( "glProjection" , &UtilOpenGL::glProject_)]);
    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[luabind::def( "glModelView"  , &UtilOpenGL::glModelView_)]);
    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[luabind::def( "glIdentity"   , &UtilOpenGL::glLoadIdentity_)]);
    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[luabind::def( "glClear"      , &UtilOpenGL::glClear_)]);
    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[luabind::def( "glViewport"   , &UtilOpenGL::glViewport_) ]);



    mLuaMashine.importToMethodScope( luabind::namespace_("GL")[ luabind::def( "loadTexture" , &loadTexture)]);

}



///---------------- initilization -------------------------///
/// \brief UnitSceneLuaInterpretationSDK::initialization
/// \return
bool UnitSceneLuaInterpretationSDK::initialization()
{

    ///-------------------------------------------///

        //        const char fileName0[] = "plane.3DS";
        //        QFile mFile0(fileName0);
        //        if(!CopyFileResources( mFile0 , ":/Files/plane.3DS" )) return false;


        //        const char fileName1[] = "box.bmp";
        //        QFile mFile1(fileName1);
        //        if(!CopyFileResources( mFile1 , ":/Files/box.bmp" )) return false;


        //        const char fileName2[] = "vshader2.glsl";
        //        QFile mFile2(fileName2);
        //        if(!CopyFileResources( mFile2 , ":/shaders/vshader2.glsl" )) return false;


        //        const char fileName3[] = "fshader2.glsl";
        //        QFile mFile3(fileName3);
        //        if(!CopyFileResources( mFile3 , ":/shaders/fshader2.glsl" )) return false;

    ///-------------------------------------------///

    NullAllKey();

    initLua();
    initGL();

    //------------------- loader lua-script -----------------------------//

    char fileName[] = "file1.lua";
    QFile FileRead(fileName);
    FileRead.open( QFile::ReadOnly );

    QTextStream stream( &FileRead );
    mLuaMashine.runString(stream.readAll().toStdString().c_str());

    FileRead.close();
    FileRead.remove();

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "setup" , this);
    }
    catch(...)
    {
       // cout<< "is not initilization function setup" <<endl;
    }

}

///------------------  render  -------------------------///
/// \brief UnitSceneLuaInterpretationSDK::render
/// \param FrameTime
void UnitSceneLuaInterpretationSDK::render(float FrameTime)
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glLoadIdentity();


    //    glFrontFace(GL_CCW);
    //    glCullFace(GL_BACK);
    //    glEnable(GL_CULL_FACE);


    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "render" , this);
    }
    catch(...)
    {
       // cout<< "is not initilization function render" <<endl;
    }

}


///-----------------  update  -------------------------///
/// \brief UnitSceneLuaInterpretationSDK::update
void UnitSceneLuaInterpretationSDK::update()
{

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "update" , this);
    }
    catch(...)
    {
       // cout<< "is not initilization function update" <<endl;
    }

}

///----------------  resize  -------------------------///
/// \brief UnitSceneLuaInterpretationSDK::resize
/// \param w
/// \param h
void UnitSceneLuaInterpretationSDK::resize(float w, float h)
{
    width  = w;
    height = h;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "resize" , this);
    }
    catch(...)
    {
       // cout<< "is not initilization function resize" <<endl;
    }

}

///---------------------- mouseMove ------------------------///
/// \brief UnitSceneLuaInterpretationSDK::mouseMove
/// \param x
/// \param y
void UnitSceneLuaInterpretationSDK::mouseMove(float x, float y)
{

}

///--------------------- mousePress -----------------------///
/// \brief UnitSceneLuaInterpretationSDK::mousePress
/// \param x
/// \param y
void UnitSceneLuaInterpretationSDK::mousePress(float x, float y)
{

}


///-------------------- mouseWheel ------------------------///
/// \brief UnitSceneLuaInterpretationSDK::mouseWheel
/// \param delta
void UnitSceneLuaInterpretationSDK::mouseWheel(float delta)
{

}

///------------------- keyboard---------------------------///
/// \brief UnitSceneLuaInterpretationSDK::keyboard
/// \param key
void UnitSceneLuaInterpretationSDK::keyboard(int key)
{

}

///-------------------- destroy --------------------------///
/// \brief UnitSceneLuaInterpretationSDK::destroy
void UnitSceneLuaInterpretationSDK::destroy()
{

}

//---------------------------------------------------------------//
///----------------------------- run-script --------------------///
/// \brief UnitSceneLuaInterpretationSDK::runScriptLua
/// \param _str
void UnitSceneLuaInterpretationSDK::runScriptLua(const char *_str)
{
   mLuaMashine.runString(_str);
}
