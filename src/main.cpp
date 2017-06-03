/*
 * main.cpp
 *
 *  Created on: 1 июн. 2017 г.
 *      Author: werqa
 */

//#include <stdio.h>
//#include <stdlib.h>
//
//int main(void)
//{
//	puts("!!!Hello World!!!");
//	return EXIT_SUCCESS;
//}

#include <iostream>

#include "examples/GLViewer.h"
#include "examples/UnitScene.h"
#include "examples/UnitSceneDrawMesh.h"

using namespace std;
using namespace utility_engine;


int Width  = 600;
int Height = 400;

int   world_size = 20;

float mouse_x , mouse_y ;
int   mouse_b = 0;

float timeStep = 1.0f / 60.0f;



UnitScene       *Scene;
Viewer          *viewer;



// Declarations
void simulate();
void display();
void reshape(int width, int height);
void mouseButton(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboard(unsigned char key , int x  , int y);
void special(int key , int x  , int y);
void init();
void Timer(int t);

// Main function
int main(int argc, char** argv)
{
		// Initialize GLUT
		viewer = new Viewer();
		viewer->init( argc , argv , "Test" , Vector2(Width,Height) , Vector2(0,0) , false );
		viewer->initilisation();


		/**/
		Scene = new  UnitSceneDrawMesh( viewer );

		// Create the scene
		if(!Scene->Init())
		{
			cout<< "error initialization" <<endl;
		}

		// Create and initialize the Viewer
		bool initOK = true;
		if (!initOK) return 1;

		init();

		// Glut Idle function that is continuously called
		glutIdleFunc(simulate);
		glutDisplayFunc(display);
		glutReshapeFunc(reshape);
		glutKeyboardFunc(keyboard);
		glutSpecialFunc(special);
		glutMouseFunc(mouseButton);
		glutMotionFunc(mouseMotion);

		//glutTimerFunc(0, Timer, (int) 100.0f / 60.0f);

		// Glut main looop
		glutMainLoop();

		//delete scene;
		Scene->Destroy();

		delete Scene;
		delete viewer;

		/**/
		return 0;
}

// Simulate function
void simulate()
{
    // Display the scene
    display();
}

// Initialization
void init()
{
    // Define the background color (black)
    glClearColor(0.0, 0.0, 0.0, 1.0);

}

// Reshape function
void reshape(int width, int height)
{
     viewer->reshape(width, height);
}



void keyboard(unsigned char key , int x  , int y)
{
     viewer->keyboard(key , x , y);
     Scene->keyboard(key);
}

void special(int key , int x  , int y)
{
     viewer->special(key, x, y);
}

// Called when a mouse button event occurs
void mouseButton(int button, int state, int x, int y)
{
	viewer->mouseButtonEvent(button, state, x , y);
}

// Called when a mouse motion event occurs
void mouseMotion(int x, int y)
{
	viewer->motion(x, y);
}

// Display the scene
void display()
{

	Scene->Render(timeStep);

    glutSwapBuffers();
}


void Timer(int t)
{
	display();
	glutTimerFunc(t , Timer, (int) 1.0f / 60.0f);
}
