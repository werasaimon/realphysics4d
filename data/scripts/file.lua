puase=false;

local p = {};
local m = {};
local n = 40;

local eye        =  vector3(0,0,100)
local center     =  vector3(0,0,-60)
local up         =  vector3(0,1,0)

local cam    = camera();
local shader = shaderProgram()

local physWorld = dynamics_world( vector3(0,-30,0) )



--****** initilization ********--
function setup( scene )

    shader:addSourceFile( shaderProgram.vertex    , "shaders/vshader2.glsl"  )
    shader:addSourceFile( shaderProgram.fragment  , "shaders/fshader2.glsl"  )
    shader:link()

    scene.width  = 600;
    scene.height = 400;

    aspect = scene.width / scene.height
    zNear  = 3.0
    zFar   = 512
    fov    = 45.0;

    cam:project( fov , aspect , zNear , zFar );


    texture = GL.loadTexture("Files/box.bmp")

    for i=0,n do

        type = ultimate_physics.dynamic;
        if i==0 then
             type = ultimate_physics.static;
         end;

        m[i] = mesh_3ds("Files/plane.3DS");
        m[i]:identity();
        m[i]:texture(texture,0);
        m[i]:translate( vector3( math.sin(i) * 2.1, -10 + 20*(i),-60) );

        p[i] = physWorld:RigidBody( m[i]:getMatrix() )
        p[i]:type( type )

        m[i]:identity();
        p[i]:addHull( m[i] , 2.0 )

    end;


end;


--******* render *********--
function render( scene )


    GL.glViewport( 0 , 0 , scene.width , scene.height );

    shader:bind()

    cam:lookAt( eye , center  , up )

    shader:UniformValue( "ProjectionMatrix" , cam:project() )
    shader:UniformValue( "ViewMatrix"       , cam:modelView() )

    for i = 0 , n do
      shader:UniformValue( "ModelMatrix"  , m[i]:getMatrix() )
      m[i]:draw(shader);
    end;

    shader:release()


    if scene:keyDown(Key_T) then
        --eye = eye + vector3(0,0,0.1) * 0.5
          pause=true;
    end;


    if scene:keyDown(Key_Y) then
       eye = eye - vector3(0,0,0.1) * 0.5
    end;

end;


--******* update *********--
function update( scene )
 
    if pause then
          physWorld:update( 1.0/60.0 )
          for i=0,n do
          p[i]:update()
          end;
    end;
 
end


--******* resize *********--
function resize( scene )

    aspect = scene.width / scene.height
    zNear  = 3.0
    zFar   = 512
    fov    = 45.0;

    cam:project( fov , aspect , zNear , zFar );

end




function destroy( scene ) 
   physWorld:destroy(); 
   delete( physWorld ); 
end;




