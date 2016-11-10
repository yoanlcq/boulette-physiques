#include "Main.h"

int GWidth  = 800; //Graphics resolution
int GHeight = 600;

int MouseX = 0; //Mouse coordinates
int MouseY = 0;

Vertex* DragVertex = 0; //The vertex that is currently dragged around with the mouse

Physics World( 0.0f, 0.5f, 10 ); //Create a new physics instance with gravity pointing downwards and using 10 iterations

void Render() {
	glClear( GL_COLOR_BUFFER_BIT ); //Clear screen

	World.Render(); //Draw world

	glFlush(); //Force rendering commands to be executed as soon as possible
}

void DispatchKeyboard( unsigned char Key, int X, int Y ) { //Exit program if the escape key is pressed
	if( Key == 27 ) exit( 0 );
}

void DispatchTimer( int ID ) { //Updates and renders everything at 60 FPS (or less, if the update method takes longer)
	World.Update();

	Render();

	if( DragVertex != 0 ) 
		DragVertex->Position = Vec2( (float)MouseX, (float)MouseY ); //Sets the position of the DragVertex to the mouse position to drag it around

	glutTimerFunc( (int)( 1000.0f/60.0f ), DispatchTimer, 0 ); //Call this function again in 16 ms
}

void DispatchMouseMotion( int X, int Y ) { //Records mouse movement
	MouseX = X;
	MouseY = Y;
}

void DispatchMouseClick( int Button, int State, int X, int Y ) {
	if( Button == GLUT_LEFT_BUTTON ) {
		if( State == GLUT_DOWN ) { //Set a new vertex to be dragged around if the left mouse button is hit
			if( !DragVertex )
				DragVertex = World.FindVertex( X, Y );
			else
				DragVertex = 0;
		}
	} else if( Button == GLUT_RIGHT_BUTTON && State == GLUT_DOWN ) { //Create a box if the right mouse button is hit
		( new PhysicsBody() )->CreateBox( X - 25, Y - 25, 50, 50 );
	}
}

void InitPhysics() {
#if 0
	for( int X = 20; X < GWidth; X += 100 ) {
		for( int Y = 50; Y < GHeight - 50; Y += 100 )
			( new PhysicsBody() )->CreateBox( X, Y, 50, 50 ); //Create a few boxes
	}

	for( int X = 50; X < GWidth - 50; X += 130 ) {
		PhysicsBody* Body = new PhysicsBody(); //Create a few triangles. Only boxes would get boring, right?
	
		Vertex* V1 = new Vertex( Body, X      , 45 );
		Vertex* V2 = new Vertex( Body, X +  50, 0  );
		Vertex* V3 = new Vertex( Body, X + 100, 45 );
	
		new Edge( Body, V1, V2 );
		new Edge( Body, V2, V3 );
		new Edge( Body, V3, V1 );
	}
#endif
    ( new PhysicsBody() )->CreateBox(GWidth/60.f, GHeight-GHeight/8.f, GWidth-GWidth/60.f, GHeight/8.f); //Create a few boxes
    for( int Y = 50; Y < GHeight - 50; Y += 200 )
        ( new PhysicsBody() )->CreateBox( GWidth/2, Y, 50, 50 ); //Create a few boxes

}

void InitGL() {
	glMatrixMode( GL_PROJECTION ); //Setup the projection and modelview matrix
	glLoadIdentity();
	
	glOrtho( 0, GWidth, GHeight, 0, -1, 1 );
	
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
}

int main( int argc, char **argv ) {
	glutInit( &argc, argv ); //Init glut (basic window setup, callback function setup etc.)

	glutInitDisplayMode( GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA );

	glutInitWindowPosition( 100, 100 );
	glutInitWindowSize    ( GWidth, GHeight );

	glutCreateWindow( "Verlet Physics Demo" );

	glutDisplayFunc      ( Render );
	glutTimerFunc        ( (int)( 100.0f/60.0f ), DispatchTimer, 0 );
	glutPassiveMotionFunc( DispatchMouseMotion );
	glutMouseFunc        ( DispatchMouseClick );
	glutKeyboardFunc     ( DispatchKeyboard );

	InitGL(); //Init OGL and physics
	InitPhysics();

	glutMainLoop(); //Everything's set up - let's enter the mainloop, yay!
	
	return 0;
}
