local eye        =  vector3(0,0,40)
local center     =  vector3(0,0,0)
local up         =  vector3(0,1,0)

local fov   = 45.0;
local zNear = 3.0
local zFar  = 512 * 2;

local camera = camera();

local mouseAngleX = 0.0;
local mouseAngleY = 0.0;

local n_size = 0;
local primitives = {};

local Z_wheel = 0.0;

--****** initilization ********--
function setup( scene )

    scene.width  = 600;
    scene.height = 400;

    aspect = scene.width / scene.height
    camera:project( fov , aspect , zNear , zFar );


    primitives[n_size] = mesh_plane( 2.0 , 2.0 );
    primitives[n_size]:identity();
    primitives[n_size]:translate( vector3(0, -7 + 5 ,0)  );
    primitives[n_size]:vColor(color4(0,1,0,0) )
    n_size = n_size + 1;

    primitives[n_size] = mesh_box( vector3(3,3,3) );
    primitives[n_size]:identity();
    primitives[n_size]:translate( vector3(0, -5 + 10 ,0)  );
    primitives[n_size]:vColor(color4(1,0,0,0) )
    n_size = n_size + 1;


end;


--******* render *********--
function render( scene )



    GL.glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
    GL.glViewport( 0 , 0 , scene.width , scene.height );


    M = matrix4()
    M:identity()
    M = M.rotate( vector3(0,1,0)  , mouseAngleX) * M
    M = M.rotate( vector3(1,0,0)  , mouseAngleY) * M

    eye = M * vector3(0,0, 40 + Z_wheel);



    camera:lookAt( eye , center  , up )


    GL.glProjection( camera:project())
    GL.glModelView(camera:modelView())



    Model = matrix4();
    Model = Model.rotate( vector3(0,1,0)  , 0.01 );

    for i=0 , n_size do
        primitives[i]:setMatrix( Model * primitives[i]:getMatrix() );
        primitives[i]:draw();
    end;

end;


--******* update *********--
function update( scene )
 


end


--******* resize *********--
function resize( scene )

    aspect = scene.width / scene.height
    camera:project( fov , aspect , zNear , zFar );

end



--****** mouse_move *******--
oldX = 0.0;
oldY = 0.0;
function mouseMove( scene )

    speedX = (scene.mouse.x - oldX);
    speedY = (scene.mouse.y - oldY);
    oldX = scene.mouse.x;
    oldY = scene.mouse.y;
    mouseAngleX = mouseAngleX + speedX  *  0.01;
    mouseAngleY = mouseAngleY + speedY  *  0.01;

end


--****** mouse_prees *******--
function mousePress( scene )

    oldX = scene.mouse.x
    oldY = scene.mouse.y

end


--****** mouse_wheel *******--
function mouseWheel( scene )

     Z_wheel = scene.Z_wheel * 0.01;

end


