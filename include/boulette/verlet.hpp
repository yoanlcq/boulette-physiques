#ifndef BOULETTE_VERLET_HPP
#define BOULETTE_VERLET_HPP

#include <boulette.hpp>

namespace boulette {

#ifdef THIS_AINT_GONNA_BE_DEFINED
// T is the space type, RT is expected to be a real-number type.
template <typename T, typename RT>
struct VerletPhysicsSystem {
    vec2<T> gravity;
    RT timestep;
    void update() {}
};

#endif










#ifdef THIS_AINT_GONNA_BE_DEFINED_EITHER
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
					if( AABBsOverlap( Bodies[ B1 ], Bodies[ B2 ] ) ) //Test the bounding boxes
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


    float friction_cst = 0.005f;
    Vec2 EdgeVector = Vec2(E2->Position - E1->Position);
    EdgeVector.Normalize();

	E1->Position -= CollisionVector*( 1 - T )*0.5f*Lambda;
	E2->Position -= CollisionVector*      T  *0.5f*Lambda;
	CollisionInfo.V->Position += CollisionVector*0.5f;
    CollisionInfo.V->Position += (CollisionInfo.V->OldPosition - CollisionInfo.V->Position)*friction_cst;
    // More Friction -> Vertices closer to the OldPosition.
    // CollisionInfo.V->Position = CollisionInfo.V->OldPosition;
}

float Physics::IntervalDistance( float MinA, float MaxA, float MinB, float MaxB ) {
	if( MinA < MinB )
		return MinB - MaxA;
	else
		return MinA - MaxB;
}


void VerletPhysicsSys::update() {
	updateForces();
	updateVerlet();
	iterateCollisions();
}

VerletVertex VerletPhysicsSys::findClosestVertex(vec2<T> cursor);

vec2<T>    VerletPhysicsBody::computeCenterOfMass();
aabb_2d<T> VerletPhysicsBody::computeAABB();
vec2<T>    VerletPhysicsBody::projectToAxis();

#endif


}

#endif//BOULETTE_VERLET_HPP
