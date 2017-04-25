local fov   = 45.0;
local zNear = 3.0
local zFar  = 512 * 2;

local eye    =  vector3(0,0,40)
local center =  vector3(0,0,0)
local up     =  vector3(0,1,0)

local camera = camera();

local mouseAngleX = 0.0;
local mouseAngleY = 0.0;

local n_size = 0;
local primitives = {};

local Z_wheel = 0.0;

---------------------------------------
local pause=false;

local gravity       = vector3(0,-30,0);
local DynamicsWorld = dynamics_world( gravity )

local NbBodies = 0;
local bodies = {};



--****** initilization ********--
function setup( scene )

    scene.width  = 600;
    scene.height = 400;

    aspect = scene.width / scene.height
    camera:project( fov , aspect , zNear , zFar );



    for i = 0 , 14 do

        halfSize = vector3(3,3,3);

        if( i == 0 ) then halfSize = vector3(30,3,30); end;

        primitives[n_size] = mesh_box( halfSize );
        primitives[n_size]:identity();
        primitives[n_size]:translate( vector3(math.sin(i) , -10 + 4 * i ,0)  );
        primitives[n_size]:vColor( color4(1,0,1,0) )

        type = ultimate_physics.dynamic;

        if( i == 0 ) then  type = ultimate_physics.static; end;
        if( i == 0 ) then  primitives[n_size]:vColor( color4(1,1,1,0) );end;



        bodies[NbBodies] = DynamicsWorld:RigidBody( primitives[n_size]:getMatrix() )
        bodies[NbBodies]:type( type )
        
        primitives[n_size]:identity(); 

        bodies[NbBodies]:addHull( primitives[n_size] , 2.0 )

        NbBodies = NbBodies + 1;
        n_size   = n_size   + 1;

    end;


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


    for i=0 , n_size do
        primitives[i]:draw();
    end;

end;


--******* update *********--
timeStep = (1.0/60.0);
function update( scene )
 
    if pause then
        DynamicsWorld:update(timeStep);
        for i = 0 , NbBodies do
           bodies[i]:update()
        end;
    end;



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


--****** keyboard *******--
function keyboard( scene )

    if ( scene.hitKey == Key_P ) then
        if( pause ) then pause = false else pause = true end;
    end;

end


