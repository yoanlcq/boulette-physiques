#include "Main.h"

void Physics::AddBody( PhysicsBody* Body ) {
	Bodies[ BodyCount++ ] = Body;
}

void Physics::AddVertex( Vertex* V ) {
	Vertices[ VertexCount++ ] = V;
}

void Physics::AddEdge( Edge* E ) {
	Edges[ EdgeCount++ ] = E;
}

//Sets the force on each vertex to the gravity force. You could of course apply other forces like magnetism etc.
void Physics::UpdateForces() {
	for( int I = 0; I < VertexCount; I++ )
		Vertices[ I ]->Acceleration = Gravity;
}

void Physics::UpdateVerlet() { //Updates the vertex position as explained in the article
	for( int I = 0; I < VertexCount; I++ ) {
		Vertex& V = *Vertices[ I ];

		Vec2 Temp = V.Position;
		V.Position += V.Position - V.OldPosition + V.Acceleration*Timestep*Timestep;
		V.OldPosition = Temp;
	}
}

void Physics::UpdateEdges() {
	for( int I = 0; I < EdgeCount; I++ ) {
		Edge& E = *Edges[ I ];

		Vec2 V1V2 = E.V2->Position - E.V1->Position; //Calculate the vector between the two vertices

		float V1V2Length = V1V2.Length(); //Calculate the current distance
		float Diff       = V1V2Length - E.Length; //Calculate the difference from the original length
		
		V1V2.Normalize();

		E.V1->Position += V1V2*Diff*0.5f; //Push both vertices apart by half of the difference respectively so the distance between them equals the original length
		E.V2->Position -= V1V2*Diff*0.5f;
	}
}

void Physics::IterateCollisions() {
	for( int I = 0; I < Iterations; I++ ) { //Repeat this a few times to give more exact results

		//A small 'hack' that keeps the vertices inside the screen. You could of course implement static objects and create
		//four to serve as screen boundaries, but the max/min method is faster
		for( int T = 0; T < VertexCount; T++ ) {
			Vec2& Pos = Vertices[ T ]->Position;
	
			Pos.X = MAX( MIN( Pos.X, (float)GWidth  ), 0.0f );
			Pos.Y = MAX( MIN( Pos.Y, (float)GHeight ), 0.0f );
		}

		UpdateEdges(); //Edge correction step

		for( int I = 0; I < BodyCount; I++ ) {
			Bodies[ I ]->CalculateCenter(); //Recalculate the center
		}

		for( int B1 = 0; B1 < BodyCount; B1++ ) { //Iterate trough all bodies
			for( int B2 = 0; B2 < BodyCount; B2++ ) {
				if( B1 != B2 )
					if( BodiesOverlap( Bodies[ B1 ], Bodies[ B2 ] ) ) //Test the bounding boxes
						if( DetectCollision( Bodies[ B1 ], Bodies[ B2 ] ) ) //If there is a collision, respond to it
							ProcessCollision();
			}
		}
	}
}

bool Physics::DetectCollision( PhysicsBody* B1, PhysicsBody* B2 ) {
	float MinDistance = 10000.0f; //Initialize the length of the collision vector to a relatively large value
	for( int I = 0; I < B1->EdgeCount + B2->EdgeCount; I++ ) { //Just a fancy way of iterating through all of the edges of both bodies at once
		Edge* E;

		if( I < B1->EdgeCount )
			E = B1->Edges[ I ];
		else
			E = B2->Edges[ I - B1->EdgeCount ];

		//This will skip edges that lie totally inside the bodies, as they don't matter.
		//The boundary flag has to be set manually and defaults to true
		if( !E->Boundary )
			continue;

		Vec2 Axis( E->V1->Position.Y - E->V2->Position.Y, E->V2->Position.X - E->V1->Position.X ); //Calculate the perpendicular to this edge and normalize it
		Axis.Normalize();

		float MinA, MinB, MaxA, MaxB; //Project both bodies onto the perpendicular
		B1->ProjectToAxis( Axis, MinA, MaxA );
		B2->ProjectToAxis( Axis, MinB, MaxB );

		float Distance = IntervalDistance( MinA, MaxA, MinB, MaxB ); //Calculate the distance between the two intervals

		if( Distance > 0.0f ) //If the intervals don't overlap, return, since there is no collision
			return false;
		else if( abs( Distance ) < MinDistance ) {
			MinDistance = abs( Distance );

			CollisionInfo.Normal = Axis; //Save collision information for later
			CollisionInfo.E      = E;    //Store the edge, as it is the collision edge
		}
	}

	CollisionInfo.Depth = MinDistance;

	if( CollisionInfo.E->Parent != B2 ) { //Ensure that the body containing the collision edge lies in B2 and the one conatining the collision vertex in B1
		PhysicsBody* Temp = B2;
		B2 = B1;
		B1 = Temp;
	}

	int Sign = SGN( CollisionInfo.Normal*( B1->Center - B2->Center ) ); //This is needed to make sure that the collision normal is pointing at B1
	
	//Remember that the line equation is N*( R - R0 ). We choose B2->Center as R0; the normal N is given by the collision normal

	if( Sign != 1 )
		CollisionInfo.Normal = -CollisionInfo.Normal; //Revert the collision normal if it points away from B1

	Vec2 CollisionVector = CollisionInfo.Normal*CollisionInfo.Depth;

	float SmallestD = 10000.0f; //Initialize the smallest distance to a large value
	for( int I = 0; I < B1->VertexCount; I++ ) {
		float Distance = CollisionInfo.Normal*( B1->Vertices[ I ]->Position - B2->Center ); //Measure the distance of the vertex from the line using the line equation

		if( Distance < SmallestD ) { //If the measured distance is smaller than the smallest distance reported so far, set the smallest distance and the collision vertex
			SmallestD = Distance;
			CollisionInfo.V = B1->Vertices[ I ];
		}
	}

	return true; //There is no separating axis. Report a collision!
}

void Physics::ProcessCollision() {
	Vertex* E1 = CollisionInfo.E->V1;
	Vertex* E2 = CollisionInfo.E->V2;

	Vec2 CollisionVector = CollisionInfo.Normal*CollisionInfo.Depth;

	float T;
	if( abs( E1->Position.X - E2->Position.X ) > abs( E1->Position.Y - E2->Position.Y ) )
		T = ( CollisionInfo.V->Position.X - CollisionVector.X - E1->Position.X )/(  E2->Position.X - E1->Position.X );
	else
		T = ( CollisionInfo.V->Position.Y - CollisionVector.Y - E1->Position.Y )/(  E2->Position.Y - E1->Position.Y );

	float Lambda = 1.0f/( T*T + ( 1 - T )*( 1 - T ) );

	E1->Position -= CollisionVector*( 1 - T )*0.5f*Lambda;
	E2->Position -= CollisionVector*      T  *0.5f*Lambda;
	CollisionInfo.V->Position += CollisionVector*0.5f;
}

float Physics::IntervalDistance( float MinA, float MaxA, float MinB, float MaxB ) {
	if( MinA < MinB )
		return MinB - MaxA;
	else
		return MinA - MaxB;
}

bool Physics::BodiesOverlap( PhysicsBody* B1, PhysicsBody* B2 ) { //Simple bounding box overlapping test
	return ( B1->MinX <= B2->MaxX ) && ( B1->MinY <= B2->MaxY ) && ( B1->MaxX >= B2->MinX ) && ( B2->MaxY >= B1->MinY );
}

void Physics::Update() {
	UpdateForces();
	UpdateVerlet();
	IterateCollisions();
}

void Physics::Render() { //Basic OGL rendering. Nothing fancy :)
	glColor3f( 1.0f, 0.0f, 0.0f );

	glBegin( GL_LINES );
		for( int I = 0; I < EdgeCount; I++ ) {
			glVertex2f( Edges[ I ]->V1->Position.X, Edges[ I ]->V1->Position.Y );
			glVertex2f( Edges[ I ]->V2->Position.X, Edges[ I ]->V2->Position.Y );
		}
	glEnd();

	glPointSize( 2.0f );
	glColor3f( 1.0f, 1.0f, 1.0f );

	glBegin( GL_POINTS );
		for( int I = 0; I < VertexCount; I++ ) {
			glVertex2f( Vertices[ I ]->Position.X, Vertices[ I ]->Position.Y );
		}
	glEnd();
}

Vertex* Physics::FindVertex( int X, int Y ) { //Helper function that finds the vertex laying closest to a given point
	Vertex* NearestVertex = 0;
	float MinDistance = 1000.0f;

	Vec2 Coords( (float)X, (float)Y );

	for( int I = 0; I < VertexCount; I++ ) {
		float Distance = ( Vertices[ I ]->Position - Coords ).LengthSq();

		if( Distance < MinDistance ) {
			NearestVertex = Vertices[ I ];
			MinDistance = Distance;
		}
	}

	return NearestVertex;
}

Edge::Edge( PhysicsBody* Body, Vertex* pV1, Vertex* pV2, int pBoundary ) { //Constructor
	V1 = pV1; //Set boundary vertices
	V2 = pV2;

	Length   = ( pV2->Position - pV1->Position ).Length(); //Calculate the original length
	Boundary = pBoundary;

	Parent = Body;

	Body->AddEdge( this ); //Add the edge to the given body and to the physics simulator
	World.AddEdge( this );
}

Vertex::Vertex( PhysicsBody* Body, float PosX, float PosY ) {
	Position    = Vec2( PosX, PosY );
	OldPosition = Vec2( PosX, PosY );

	Parent = Body;

	Body->AddVertex( this ); //Add the vertex to the given body and to the physics simulator
	World.AddVertex( this );
}

PhysicsBody::PhysicsBody() { //Constructor
	 VertexCount = EdgeCount = 0;

	 World.AddBody( this ); //Add body to the physics simulator
}

void PhysicsBody::AddEdge( Edge* E ) {
	Edges[ EdgeCount++ ] = E;
}

void PhysicsBody::AddVertex( Vertex *V ) {
	Vertices[ VertexCount++ ] = V;
}

void PhysicsBody::ProjectToAxis( Vec2& Axis, float& Min, float& Max ) {
	float DotP = Axis*Vertices[ 0 ]->Position;

	Min = Max = DotP; //Set the minimum and maximum values to the projection of the first vertex

	for( int I = 0; I < VertexCount; I++ ) {
		DotP = Axis*Vertices[ I ]->Position; //Project the rest of the vertices onto the axis and extend the interval to the left/right if necessary

		Min = MIN( DotP, Min );
		Max = MAX( DotP, Max );
	}
}

void PhysicsBody::CalculateCenter() { //Recalculates the center of mass and the bounding box
	Center = Vec2( 0.0f, 0.0f );

	MinX = MinY =  10000.0f;
	MaxX = MaxY = -10000.0f;

	for( int I = 0; I < VertexCount; I++ ) {
		Center += Vertices[ I ]->Position;

		MinX = MIN( MinX, Vertices[ I ]->Position.X );
		MinY = MIN( MinY, Vertices[ I ]->Position.Y );
		MaxX = MAX( MaxX, Vertices[ I ]->Position.X );
		MaxY = MAX( MaxX, Vertices[ I ]->Position.Y );
	}

	Center /= VertexCount;
}

void PhysicsBody::CreateBox( int X, int Y, int Width, int Height ) { //Creates a box primitive
	Vertex* V1 = new Vertex( this, X        , Y          );
	Vertex* V2 = new Vertex( this, X + Width, Y          );
	Vertex* V3 = new Vertex( this, X + Width, Y + Height );
	Vertex* V4 = new Vertex( this, X        , Y + Height );

	new Edge( this, V1, V2, true );
	new Edge( this, V2, V3, true );
	new Edge( this, V3, V4, true );
	new Edge( this, V4, V1, true );

	new Edge( this, V1, V3, false );
	new Edge( this, V2, V4, false );
}
