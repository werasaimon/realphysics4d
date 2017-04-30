local fov   = 45.0;
local zNear = 3.0
local zFar  = 512 * 2;

local eye    =  vector3(0,0,-40)
local center =  vector3(0,0,0)
local up     =  vector3(0,1,0)

local M = matrix4()

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
local joints = {};



--****** initilization ********--
function setup( scene )

    scene.width  = 600;
    scene.height = 400;

    aspect = scene.width / scene.height
    camera:project( fov , aspect , zNear , zFar );

    M:identity()

    for i = 0 , 10 do

        halfSize = vector3(2,2,2);
        pos      = vector3( 10 , -5 + i * 4 , 0 );

        if( n_size == 0 ) then halfSize = vector3(100,3,100); pos = vector3(0,-10,0) end;

        primitives[n_size] = mesh_box( halfSize );
        primitives[n_size]:identity();
        primitives[n_size]:translate( pos );
        primitives[n_size]:vColor( color4(0,0,1,0) )

        if( n_size == 0 ) then  primitives[n_size]:vColor( color4(1,1,1,1) );end;

        n_size   = n_size   + 1;

    end;


end;


--******* render *********--
function render( scene )

    GL.glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
    GL.glViewport( 0 , 0 , scene.width , scene.height );


    camera:setMatrix( M:transpose() );
    camera:translate( eye );


    GL.glProjection(  camera:project() );
    GL.glModelView( camera:modelView() );


    for i=0 , n_size do
        primitives[i]:draw();
    end;

end;


--******* update *********--
timeStep = (1.0/60.0);
function update( scene )


        if pause then
            DynamicsWorld:update(timeStep);
        end;


        if pause then
             for i = 0 , NbBodies do
                bodies[i]:update();
             end;
        end;

end


--******* resize *********--
function resize( scene )

    aspect = scene.width / scene.height;
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


    M:identity()
    M = M.rotate( vector3(0,1,0)  , -mouseAngleX) * M;
    M = M.rotate( vector3(1,0,0)  , -mouseAngleY) * M;

end


--****** mouse_prees *******--
function mousePress( scene )

    oldX = scene.mouse.x
    oldY = scene.mouse.y

    if ( scene.mouse.button == MOUSE_RIGHT ) then

          halfSize = vector3(1,1,1);

          primitives[n_size] = mesh_box( halfSize );
          primitives[n_size]:identity();
          primitives[n_size]:setMatrix( camera:getMatrix():inverse() );
          primitives[n_size]:vColor( color4(1,0,1,1) );


          bodies[NbBodies] = DynamicsWorld:RigidBody( primitives[n_size]:getMatrix() )
          bodies[NbBodies]:type( ultimate_physics.dynamic );

          primitives[n_size]:identity();
          bodies[NbBodies]:addBox(  primitives[n_size] , halfSize * 0.5 , 20.0 );

          bodies[NbBodies]:applyForceToCenter( M * vector3(0,0,1) * -50000.0 );

          n_size   = n_size   + 1;
          NbBodies = NbBodies + 1;

    end;

end


--****** mouse_wheel *******--
function mouseWheel( scene )

      Z_wheel = scene.Z_wheel * 0.01;

end


--****** keyboard *******--
function keyboard( scene )


    if ( scene.hitKey == Key_O ) then

           if( pause ) then pause = false else pause = true end;
    end;


    -------------------------------------------------

    if ( scene.hitKey == Key_P ) then

            pause = true;

            for i = 0 , n_size-1 do

                type = ultimate_physics.dynamic;

                if( i == 0 ) then  type = ultimate_physics.static; end;
                if( i == n_size-1 ) then  type = ultimate_physics.static; end;


                bodies[NbBodies] = DynamicsWorld:RigidBody( primitives[i]:getMatrix() )
                bodies[NbBodies]:type( type )

                primitives[i]:identity();

                halfSize = primitives[i]:halfSize() * 0.5;
                bodies[NbBodies]:addHull(  primitives[i] , primitives[i]:getMatrix() , 2.0 )


                NbBodies = NbBodies + 1;


            end

            for i = 2 , n_size-1 do

                   anchor = vector3( 10 , -5 + i * 4 , 0 );

                   joint_info  =  ball_joint_info( bodies[i-1] , bodies[i] , anchor );
                   joints[i-2] =  DynamicsWorld:Joint( joint_info );
            end;


    end;

    -------------------------------------------------

    local speedMove = 0.5;


    if ( scene.hitKey == Key_W ) then
         eye = eye + (M * vector3(0,0,1)) * speedMove;
    end;

    if ( scene.hitKey == Key_S ) then
         eye = eye - (M * vector3(0,0,1)) * speedMove;
    end;

    if ( scene.hitKey == Key_A ) then
         eye = eye + (M * vector3(1,0,0)) * speedMove;
    end;

    if ( scene.hitKey == Key_D ) then
         eye = eye - (M * vector3(1,0,0)) * speedMove;
    end;

end


