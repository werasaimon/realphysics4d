local eye        =  vector3(0,0,40)
local center     =  vector3(0,0,0)
local up         =  vector3(0,1,0)

local cam        = camera();

local n_size = 4;
local primitives = {};

--****** initilization ********--
function setup( scene )

    scene.width  = 600;
    scene.height = 400;

    aspect = scene.width / scene.height
    fov = 45.0;  zNear = 3.0 zFar = 512
    cam:project( fov , aspect , zNear , zFar );

    for i=0 , n_size do

        primitives[i] = mesh_box( vector3(3,3,3) );
        primitives[i]:identity();
        primitives[i]:translate( vector3(0, -10 + i * 5 ,0)  );

    end;


end;


--******* render *********--
function render( scene )

    GL.glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
    GL.glViewport( 0 , 0 , scene.width , scene.height );

    cam:lookAt( eye , center  , up )

    GL.glProjection( cam:project())
    GL.glModelView(cam:modelView())

    for i=0 , n_size do
        primitives[i]:draw()
    end;

end;


--******* update *********--
function update( scene )
 


end


--******* resize *********--
function resize( scene )

    aspect = scene.width / scene.height
    fov = 45.0;  zNear = 3.0 zFar = 512
    cam:project( fov , aspect , zNear , zFar );

end


oldX = 0.0;
oldY = 0.0;
function mouseMove( scene )

    x = scene.mouse.x
    y = scene.mouse.y
    speedX = (x - oldX);
    speedY = (y - oldY);
    oldX = x;
    oldY = y;

    M = matrix4()
    M:identity()
    M = M.rotate( vector3(0,1,0)  , speedX  *  0.01) * M
    M = M.rotate( vector3(1,0,0)  , speedY  *  0.01) * M

    eye = M * eye;

end



function mousePress( scene )

     oldX = scene.mouse.x
     oldY = scene.mouse.y

end


