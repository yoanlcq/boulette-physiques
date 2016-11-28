#ifndef BOULETTE_VERLET_HPP
#define BOULETTE_VERLET_HPP

#include <boulette.hpp>
#include <x86intrin.h>

namespace boulette {

#define THIS_AINT_GONNA_BE_DEFINED
#ifdef THIS_AINT_GONNA_BE_DEFINED
// T is the space type, RT is expected to be a real-number type.
template <typename T, typename RT>
struct VerletPhysicsSystem {
    vec2<T>  screen_size; // Keep vertices inside for debugging
    vec2<T>  gravity;  // Rather see it as a force that affects all bodies
    RT       timestep; // The simulation's speed - 1 is the norm.
    size_t   vcount;   // Total number of vertices.
    vec2<T> *vaccel;   // All vertex accelerations.
    vec2<T> *vpos;     // All vertex positions.
    vec2<T> *vprevpos; // All vertex previous positions.
    size_t   ecount;   // Total number of edges.
    typedef struct {size_t v1, v2;} evert_s;
    evert_s *evert; // For each edge, two indices to vertices.
    T       *elength;     // For each edge, its length.
    // Each array that contains data for the edges is split into
    // 2 sections : edges that are occluded, and the others.
    // Here, an edge is said to be occluded if we know that it is totally
    // inside a body, therefore we can avoid testing it for collisions.
    // Thus, to iterate over all lengths of edges that are occluded, one would
    // do it like this :
    //     for(size_t i=e_occluded_start ; i<ecount ; ++i)
    // And over all those that aren't :
    //     for(size_t i=0 ; i<e_occluded_start ; ++i)
    size_t  e_occluded_start;

    VerletPhysicsSystem(vec2<T> screen_size) 
      : screen_size(screen_size),
        gravity(0, .98),
        timestep(1)
    {
        vcount   = 4;
        vaccel   = (vec2<T>*) _mm_malloc(vcount*sizeof(vec2<T>), 16);
        vpos     = (vec2<T>*) _mm_malloc(vcount*sizeof(vec2<T>), 16);
        vprevpos = (vec2<T>*) _mm_malloc(vcount*sizeof(vec2<T>), 16);

        memset(vaccel, 0, vcount*sizeof(*vaccel));

        vpos[0] = vec2<T>(50, 50);
        vpos[1] = vec2<T>(90, 50);
        vpos[2] = vec2<T>(90, 90);
        vpos[3] = vec2<T>(50, 90);

        static_assert(sizeof *vpos == sizeof *vprevpos, "");
        memcpy(vprevpos, vpos, vcount*sizeof *vpos);

        ecount  = 6;
        evert   = (evert_s*) _mm_malloc(ecount*sizeof(evert_s), 16);
        elength =       (T*) _mm_malloc(ecount*sizeof(T), 16);
        evert[0].v1 = 0, evert[0].v2 = 1;
        evert[1].v1 = 1, evert[1].v2 = 2;
        evert[2].v1 = 2, evert[2].v2 = 3;
        evert[3].v1 = 3, evert[3].v2 = 0;
        e_occluded_start = 4;
        evert[4].v1 = 0, evert[4].v2 = 2;
        evert[5].v1 = 1, evert[5].v2 = 3;

        for(size_t i=0 ; i<ecount ; ++i)
            elength[i] = norm(vpos[evert[i].v2]-vpos[evert[i].v1]);
    }

    ~VerletPhysicsSystem() {
        _mm_free(vaccel);
        _mm_free(vpos);
        _mm_free(vprevpos);
        _mm_free(evert);
        _mm_free(elength);
    }

    void integrateNewPositions() {
        const RT sq_timestep = timestep*timestep;
        for(size_t i=0 ; i<vcount ; ++i) {
            vaccel[i] = gravity; // Apply all forces
            // Then integrate.
            vec2<T> tmp = vpos[i];
            vpos[i] += vpos[i] - vprevpos[i] + vaccel[i]*sq_timestep;
            vprevpos[i] = tmp;
        }
    }
#define min(a,b) (a<b ? a : b)
#define max(a,b) (a>b ? a : b)
#define clamp(x,l,h) (min(h,max(l,x)))
    void keepVerticesInsideScreen() {
        for(size_t i=0 ; i<vcount ; ++i) {
            vpos[i].x = clamp(vpos[i].x, T(0), screen_size.x);
            vpos[i].y = clamp(vpos[i].y, T(0), screen_size.y);
        }
    }
    void edgeCorrectionStep() {
        //UpdateEdges();
        for(size_t i=0 ; i<ecount ; ++i) {
        
        }
    }
    void recomputeCentersOfMass() {
        /*
        for(size_t i=0 ; i<vcount ; ++i) {
            CalculateCenter();
        }
        */
    }
    void processBodyCollisions() {
        /*
        if(DetectCollision(B1,B2))
            ProcessCollision();
        */
    }
    void iterateCollisions() {
        // No specific reason for it to stay the same. Could change dynamically.
        static const size_t iteration_count(10);
        for(size_t i=0 ; i<iteration_count ; ++i) {
            keepVerticesInsideScreen();
            edgeCorrectionStep();
            recomputeCentersOfMass();
            processBodyCollisions();
        }
    }
    void update() {
        integrateNewPositions();
        iterateCollisions();
        //iterateCollisions();
    }

    void renderSDL2(SDL_Renderer *rdr, RT interp=1) const {
        SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
        //Render each edge...
        for(size_t i=0 ; i<ecount ; ++i) {
            size_t v1 = evert[i].v1, v2 = evert[i].v2;
            vec2<int> a, b;
            a.x = RT(vprevpos[v1].x) + interp*RT(vpos[v1].x - vprevpos[v1].x);
            a.y = RT(vprevpos[v1].y) + interp*RT(vpos[v1].y - vprevpos[v1].y);
            b.x = RT(vprevpos[v2].x) + interp*RT(vpos[v2].x - vprevpos[v2].x);
            b.y = RT(vprevpos[v2].y) + interp*RT(vpos[v2].y - vprevpos[v2].y);
            SDL_RenderDrawLine(rdr, a.x, a.y, b.x, b.y);
        }
        SDL_SetRenderDrawColor(rdr,   0, 255, 0, 255);
        // Ugly way of rendering vertices.
        for(size_t i=0 ; i<vcount ; ++i)
            for(int y=-1 ; y<=1 ; ++y)
                for(int x=-1 ; x<=1 ; ++x) {
                    vec2<int> ipos;
                    ipos.x = RT(vprevpos[i].x) + interp*RT(vpos[i].x - vprevpos[i].x);
                    ipos.y = RT(vprevpos[i].y) + interp*RT(vpos[i].y - vprevpos[i].y);
                    SDL_RenderDrawPoint(rdr, ipos.x+x, ipos.y+y);
                }
    }
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
}

VerletVertex VerletPhysicsSys::findClosestVertex(vec2<T> cursor);

vec2<T>    VerletPhysicsBody::computeCenterOfMass();
aabb_2d<T> VerletPhysicsBody::computeAABB();
vec2<T>    VerletPhysicsBody::projectToAxis();

#endif


}

#endif//BOULETTE_VERLET_HPP
