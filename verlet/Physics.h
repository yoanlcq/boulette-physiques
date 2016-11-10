#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <iostream>

#include "Vector.h"

#define MAX_BODIES   512  //Maximum body/vertex/edgecount the physics simulation can handle
#define MAX_VERTICES 1024
#define MAX_EDGES    1024
#define MAX_BODY_VERTICES 64 //Maximum body/edge count a body can contain
#define MAX_BODY_EDGES    64

struct PhysicsBody; //Prototypes
struct Vertex;
struct Edge;

class Physics {
	Vec2 Gravity; //Most of this should be clear after reading the article

	int BodyCount;
	int VertexCount;
	int EdgeCount;

	Vertex*      Vertices[ MAX_VERTICES ];
	Edge*        Edges   [ MAX_EDGES    ];
	PhysicsBody* Bodies  [ MAX_BODIES   ];

	float Timestep;
	int Iterations;

	void  UpdateForces();
	void  UpdateVerlet();
	void  UpdateEdges ();
	void  IterateCollisions();
	bool  DetectCollision( PhysicsBody* B1, PhysicsBody* B2 );
	void  ProcessCollision();
	float IntervalDistance( float MinA, float MaxA, float MinB, float MaxB );
	bool  BodiesOverlap( PhysicsBody* B1, PhysicsBody* B2 ); //Used for optimization to test if the bounding boxes of two bodies overlap

	struct {
		float Depth;
		Vec2  Normal;

		Edge*   E;
		Vertex* V;
	} CollisionInfo;

public:
	void Update();
	void Render();

	void AddBody  ( PhysicsBody* Body ); //Adds new elements to the simulation
	void AddEdge  ( Edge* E );
	void AddVertex( Vertex* V );

	Vertex* FindVertex( int X, int Y );

	Physics( float GravitationX = 0.0f, float GravitationY = 0.0f, int pIterations = 1 ) : //Constructor
					BodyCount( 0 ), VertexCount( 0 ), EdgeCount( 0 ),
					Gravity( Vec2( GravitationX, GravitationY ) ),
					Iterations( pIterations ), Timestep( 0.25f ) {}
};

struct PhysicsBody {
	Vec2 Center; //Center of mass

	int MinX, MinY, MaxX, MaxY; //Min/max coordinates of the bounding box

	int VertexCount;
	int EdgeCount;

	Vertex* Vertices[ MAX_BODY_VERTICES ];
	Edge*   Edges   [ MAX_BODY_EDGES    ];

	PhysicsBody(); //The constructor is empty

	void AddEdge  ( Edge*   E );
	void AddVertex( Vertex* V );
	void ProjectToAxis( Vec2& Axis, float& Min, float& Max );
	void CalculateCenter(); //Calculates the venter of mass

	void CreateBox( int X, int Y, int Width, int Height ); //Helper function to create a box primitive
};

struct Vertex {
	Vec2 Position;
	Vec2 OldPosition;
	Vec2 Acceleration;

	PhysicsBody* Parent;

	Vertex( PhysicsBody* Body, float PosX, float PosY ); //Constructor
};

struct Edge {
	Vertex* V1;
	Vertex* V2;

	float Length;
	int Boundary; //Value used for optimization - see Physics::DetectCollision for more information

	PhysicsBody* Parent;

	Edge( PhysicsBody* Body, Vertex* pV1, Vertex* pV2, int pBoundary = true ); //Constructor
};

#endif
