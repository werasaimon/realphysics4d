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


    ///------------------- Loader Libary SDK-lua --------------------------------///

    LoadLibaryLuaValue                 libaryValue( mLuaMashine.getVirtualMashinLua() );
    LoadLibaryLuaPhysicsEngine libaryPhysicsEngine( mLuaMashine.getVirtualMashinLua() );
    LoadLibaryLuaUIEngine           libaryUIEngine( mLuaMashine.getVirtualMashinLua() );
    LoadLibaryLuaShader               libaryShader( mLuaMashine.getVirtualMashinLua() );
    LoadLibaryLuaOpenGL               liberyOpenGL( mLuaMashine.getVirtualMashinLua() );

    libaryValue.LoadLibary();
    libaryPhysicsEngine.LoadLibary();
    libaryUIEngine.LoadLibary();
    libaryShader.LoadLibary();
    liberyOpenGL.LoadLibary();


    ///--------------------------------------------------------------------------///


    mLuaMashine.importToScope( luabind::class_<mouse>("mouse")
                                     // value
                                     .def_readwrite("x"       , &mouse::m_x)
                                     .def_readwrite("y"       , &mouse::m_y)
                                     .def_readwrite("button"  , &mouse::m_button));


    mLuaMashine.importToScope( luabind::class_<UnitSceneLuaInterpretationSDK>("scene")
                                     .def(luabind::constructor<>())
                                     .def("keyDown", &UnitSceneLuaInterpretationSDK::keyDown)
                                     // value
                                     .def_readwrite("width"   , &UnitSceneLuaInterpretationSDK::width)
                                     .def_readwrite("height"  , &UnitSceneLuaInterpretationSDK::height)
                                     .def_readwrite("mouse"   , &UnitSceneLuaInterpretationSDK::mMouse)
                                     .def_readwrite("Z_wheel" , &UnitSceneLuaInterpretationSDK::mZ_Wheel)
                                     .def_readwrite("hitKey"  , &UnitSceneLuaInterpretationSDK::mHitKey));




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


    luabind::globals(mLuaMashine.getVirtualMashinLua())["MOUSE_RIGHT"] = (int)(Qt::MouseButton::RightButton);
    luabind::globals(mLuaMashine.getVirtualMashinLua())["MOUSE_LEFT"]  = (int)(Qt::MouseButton::LeftButton);



}

///------------------- initilization GL --------------------///
/// \brief UnitSceneLuaInterpretationSDK::initGL
/// \return
bool UnitSceneLuaInterpretationSDK::initGL()
{

    mLuaMashine.importToScope( luabind::namespace_("GL")[ luabind::def( "loadTexture" , &loadTexture)]);

}



///---------------- initilization -------------------------///
/// \brief UnitSceneLuaInterpretationSDK::initialization
/// \return
///
bool UnitSceneLuaInterpretationSDK::initialization()
{

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

    mZ_Wheel = 0;

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
///
void UnitSceneLuaInterpretationSDK::render(float FrameTime)
{

    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //    glLoadIdentity();


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
///
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
///
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
///
void UnitSceneLuaInterpretationSDK::mouseMove(float x , float y , int button)
{
    mMouse.m_x = x;
    mMouse.m_y = y;
    mMouse.m_button = button;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "mouseMove" , this);
    }
    catch(...)
    {

    }
}

///--------------------- mousePress -----------------------///
/// \brief UnitSceneLuaInterpretationSDK::mousePress
/// \param x
/// \param y
/// \param button
///
void UnitSceneLuaInterpretationSDK::mousePress(float x , float y , int button)
{
    mMouse.m_x = x;
    mMouse.m_y = y;
    mMouse.m_button = button;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "mousePress" , this);
    }
    catch(...)
    {

    }
}


///--------------------- mouseReleasePress -----------------------///
/// \brief UnitSceneLuaInterpretationSDK::mouseReleasePress
/// \param x
/// \param y
/// \param button
///
void UnitSceneLuaInterpretationSDK::mouseReleasePress(float x, float y, int button)
{
    mMouse.m_x = x;
    mMouse.m_y = y;
    mMouse.m_button = button;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "mouseRelease" , this);
    }
    catch(...)
    {

    }
}


///-------------------- mouseWheel ------------------------///
/// \brief UnitSceneLuaInterpretationSDK::mouseWheel
/// \param delta
///
void UnitSceneLuaInterpretationSDK::mouseWheel(float delta)
{
    mZ_Wheel += delta;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "mouseWheel" , this);
    }
    catch(...)
    {

    }
}

///------------------- keyboard---------------------------///
/// \brief UnitSceneLuaInterpretationSDK::keyboard
/// \param key
///
void UnitSceneLuaInterpretationSDK::keyboard(int key)
{
    mHitKey = key;

    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "keyboard" , this);
    }
    catch(...)
    {

    }
}

///-------------------- destroy --------------------------///
/// \brief UnitSceneLuaInterpretationSDK::destroy
///
void UnitSceneLuaInterpretationSDK::destroy()
{
    try
    {
        luabind::call_function<void>(mLuaMashine.getVirtualMashinLua(), "destroy" , this);
    }
    catch(...)
    {

    }

    mLuaMashine.closet();
}

//---------------------------------------------------------------//
///----------------------------- run-script --------------------///
/// \brief UnitSceneLuaInterpretationSDK::runScriptLua
/// \param _str
///
void UnitSceneLuaInterpretationSDK::runScriptLua(const char *_str)
{
   mLuaMashine.runString(_str);
}
