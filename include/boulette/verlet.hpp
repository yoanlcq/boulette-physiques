#ifndef BOULETTE_VERLET_HPP
#define BOULETTE_VERLET_HPP

#include <boulette.hpp>
#include <x86intrin.h>

namespace boulette {

// This system is implemented using gamedev.net's article 
// "A Verlet based approach for 2D game physics" as a reference.
// http://www.gamedev.net/page/resources/_/technical/math-and-physics/a-verlet-based-approach-for-2d-game-physics-r2714
//
// I rewrote the code in order to match better the 
// Data-Oriented Design mindset.
//
// This implementation brings the following improvements :
// - Sets of data that are not often accessed together are put into separate
//   arrays.
// - Arrays are aligned on a 16-byte boundary, thanks to _mm_malloc().
//   The compiler can thus generate aligned SSE2 instructions (such as movdqa, 
//   instead of the slower movdqu). However, seeing the disassembly,
//   the compiler still has to perform some checks.
//   TODO use GCC vector extensions.
// - Edges which are guaranteed to be fully occluded by other edges don't need to be
//   taken into account for collision detection.
//   For instance, edges which are inside a body in order to keep it solid will never
//   collide with anything on the outside (and those are numerous).
//   In the example code, this "optimization" was done with an "if()" for each edge,
//   at each update step ("int boudary;" member). 
//   In my code, the edge data arrays are split in two sections :
//   - The edges that are always completely occluded;
//   - The others. We only iterate over this one section when detecting collisions.
// - In the example, collision was stateful (see use of CollisionInfo), so not parallelizable.
// - The Add*() methods focused on adding one object at a time, but when there's one,
//   there's many.
//   The addRigidBodies() method allows to efficiently add any number of rigid bodies
//   in the system at once, taking advantage of data locality.
// - Each vertex used to store a back pointer to the body they're in, which is needlessly
//   expensive.
//
// TODO Why not valarray ? Or Vector ?
//
// TODO
// - Fix objects not behaving well
// - malloc_align16()
// - Fix fake occluded edges optimization
// - Growing vertices (vertices as spheres)
// - Sprites
// - Coordinate shift from simulation-space to view-space
// - Port to Emscripten ??


// XXX Actually implement these !
#define malloc_align16(...) malloc(__VA_ARGS__)
#define realloc_align16(...) realloc(__VA_ARGS__)
#define free_align16(...) free(__VA_ARGS__)

// T is the space type, RT is expected to be a real-number type.
template <typename T, typename RT>
struct VerletPhysicsSystem {

    typedef uint_fast32_t index;
    struct rgba32 {
        uint32_t r:8, g:8, b:8, a:8; 
        rgba32(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a) {}
    };
    struct evert_s {
        index v1, v2;
        evert_s(index v1, index v2) : v1(v1), v2(v2) {}
    };

    vec2<T>  screen_size; // Keep vertices inside for debugging
    vec2<T>  gravity;     // Rather see it as a force that affects all bodies
    RT       timestep;    // The simulation's speed. 1 is the norm.
    index    vcount;      // Total number of vertices.
    vec2<T> *vaccel;      // For each vertex, its acceleration.
    vec2<T> *vpos;        // For each vertex, its position.
    vec2<T> *vprevpos;    // For each vertex, its previous position.
    rgba32  *vcolor;      // For each vertex, its color (for visual debugging).
    index    ecount;      // Total number of edges.
    evert_s *evert;       // For each edge, two indices to vertices.
    T       *elength;     // For each edge, its length.
    // Each array that contains data for the edges is split into
    // 2 sections : edges that are occluded, and the others.
    // Here, an edge is said to be occluded if we know that it is totally
    // inside a body, therefore we can avoid testing it for collisions.
    // Thus, to iterate over all lengths of edges that are occluded, one would
    // do it like this :
    //     for(index i=e_occluded_start ; i<ecount ; ++i)
    // And over all those that aren't :
    //     for(index i=0 ; i<e_occluded_start ; ++i)
    index    e_occluded_start;
    index    bcount;      // Total number of Bodies.
    vec2<T> *bcenter;     // For each body, its center of mass.
    index   *bvertcount;  // For each body, its vertex count.
    index  **bvert;       // For each body, its vertex indices.
    index   *bedgecount;  // For each body, its edge count.
    index  **bedge;       // For each body, its edge indices.


    VerletPhysicsSystem(vec2<T> screen_size, RT timestep=1) 
      : screen_size(screen_size),
        gravity(0, 1), // Not .98, cuz it is rounded to zero if T is integer.
        timestep(timestep),
        vcount    (0),
        vaccel    (nullptr),
        vpos      (nullptr),
        vprevpos  (nullptr),
        vcolor    (nullptr),
        ecount    (0),
        evert     (nullptr),
        elength   (nullptr),
        e_occluded_start(0),
        bcount    (0),
        bcenter   (nullptr),
        bvertcount(nullptr),
        bvert     (nullptr),
        bedgecount(nullptr),
        bedge     (nullptr)
    {}

    ~VerletPhysicsSystem() {
        free_align16(vaccel);
        free_align16(vpos);
        free_align16(vprevpos);
        free_align16(vcolor);
        free_align16(evert);
        free_align16(elength);

        for(index i=0 ; i<bcount ; ++i) {
            free_align16(bvert[i]);
            free_align16(bedge[i]);
        }
        free_align16(bvert     );
        free_align16(bedge     );
        free_align16(bcenter   );
        free_align16(bvertcount);
        free_align16(bedgecount);
    }

    struct VertexDescriptor {
        vec2<T> position;
        rgba32 color;
        VertexDescriptor(vec2<T> pos, rgba32 rgba) : position(pos), color(rgba) {}
    };
    struct BodyDescriptor {
        std::vector<VertexDescriptor> vertices;
        std::vector<evert_s> edges;
        std::vector<evert_s> occluded_edges;
    };

    static BodyDescriptor describeBox(vec2<T> half_size, const std::vector<rgba32> &colors) {
        BodyDescriptor box;
        uint_fast32_t i=0;
        for(int y=-1 ; y<=1 ; y+=2) {
            for(int x=-1 ; x<=1 ; x+=2) {
                box.vertices.push_back(VertexDescriptor(half_size*vec2<T>(x, y), colors[i%colors.size()]));
                box.edges.push_back(evert_s(i, (i+1)%4));
                ++i;
            }
        }
        box.occluded_edges.push_back(evert_s(0, 2));
        box.occluded_edges.push_back(evert_s(1, 3));
        return box;
    }

    static BodyDescriptor describeRigidDisk(float radius, size_t vertex_count, const std::vector<rgba32> &colors) {
        BodyDescriptor disk;
        for(uint_fast32_t i=0 ; i<vertex_count ; ++i) {
            // (i+0.5) for more fun : otherwise the bottom edge falls flat 
            // on the ground and we don't get to see the body bouncing a bit ._.
            float theta = float(M_PI*2*(i+0.5))/float(vertex_count);
            vec2<T> pos(radius*cos(theta), radius*sin(theta));
            disk.vertices.push_back(VertexDescriptor(pos, colors[i%colors.size()]));
            disk.edges.push_back(evert_s(i, (i+1)%vertex_count));
        }
        for(uint_fast32_t i=0 ; i<vertex_count-2 ; ++i)
            for(uint_fast32_t j=0 ; j<(i ? vertex_count-2-i : vertex_count-3); ++j)
                disk.occluded_edges.push_back(evert_s(i, i+2+j));
        return disk;
    }
    static BodyDescriptor describeSlimyDisk(float radius, size_t vertex_count, const std::vector<rgba32> &colors) {
        BodyDescriptor disk;
        for(uint_fast32_t i=0 ; i<vertex_count ; ++i) {
            // (i+0.5) for more fun : otherwise the bottom edge falls flat 
            // on the ground and we don't get to see the body bouncing a bit ._.
            float theta = float(M_PI*2*(i+0.5))/float(vertex_count);
            vec2<T> pos(radius*cos(theta), radius*sin(theta));
            disk.vertices.push_back(VertexDescriptor(pos, colors[i%colors.size()]));
            disk.edges.push_back(evert_s(i, (i+1)%vertex_count));
            disk.occluded_edges.push_back(evert_s(i, (i+(vertex_count/2))%vertex_count));
        }
        return disk;
    }
    static BodyDescriptor describeBob() {
        rgba32 color(000, 255, 000, 255);
        BodyDescriptor bob;
        // Head
        bob.vertices.push_back(VertexDescriptor(vec2<T>(-10,  10), color));
        bob.vertices.push_back(VertexDescriptor(vec2<T>( 10,  10), color));
        bob.vertices.push_back(VertexDescriptor(vec2<T>( 10, -10), color));
        bob.vertices.push_back(VertexDescriptor(vec2<T>(-10, -10), color));
        // Spine
        bob.vertices.push_back(VertexDescriptor(vec2<T>(0,  10), color));
        bob.vertices.push_back(VertexDescriptor(vec2<T>(0,  30), color));
        bob.vertices.push_back(VertexDescriptor(vec2<T>(0,  40), color));
    }

    void addRigidBodies(const BodyDescriptor &b, size_t cnt, vec2<T> *centers) {

        vpos       = (vec2<T>*) realloc_align16(vpos      , (vcount+cnt*b.vertices.size())*sizeof(*vpos      ));
        vprevpos   = (vec2<T>*) realloc_align16(vprevpos  , (vcount+cnt*b.vertices.size())*sizeof(*vprevpos  ));
        vaccel     = (vec2<T>*) realloc_align16(vaccel    , (vcount+cnt*b.vertices.size())*sizeof(*vaccel    ));
        vcolor     = (rgba32*)  realloc_align16(vcolor    , (vcount+cnt*b.vertices.size())*sizeof(*vcolor    ));
        evert      = (evert_s*) realloc_align16(evert     , (ecount+cnt*(b.edges.size()+b.occluded_edges.size()))*sizeof(*evert     ));
        elength    = (T*)       realloc_align16(elength   , (ecount+cnt*(b.edges.size()+b.occluded_edges.size()))*sizeof(*elength   ));
        bcenter    = (vec2<T>*) realloc_align16(bcenter   , (bcount+cnt)  *sizeof(*bcenter   ));
        bvertcount = (index*)   realloc_align16(bvertcount, (bcount+cnt)  *sizeof(*bvertcount));
        bedgecount = (index*)   realloc_align16(bedgecount, (bcount+cnt)  *sizeof(*bedgecount));
        bvert      = (index**)  realloc_align16(bvert     , (bcount+cnt)  *sizeof(*bvert     ));
        bedge      = (index**)  realloc_align16(bedge     , (bcount+cnt)  *sizeof(*bedge     ));

        for(index i=0 ; i<cnt ; ++i)
            for(index v=0 ; v<b.vertices.size() ; ++v) {
                vpos  [vcount + i*b.vertices.size() + v] = b.vertices[v].position;
                vcolor[vcount + i*b.vertices.size() + v] = b.vertices[v].color;
            }
        memcpy(vprevpos+vcount, vpos+vcount, cnt*b.vertices.size()*sizeof(*vpos));
        memset(vaccel+vcount, 0, cnt*b.vertices.size()*sizeof(*vaccel));

        index new_occluded_start = e_occluded_start + cnt*b.edges.size();
        memmove(evert + new_occluded_start, evert + e_occluded_start, (ecount-e_occluded_start)*sizeof(*evert));
        memmove(elength + new_occluded_start, elength + e_occluded_start, (ecount-e_occluded_start)*sizeof(*elength));
        for(index i=0 ; i<cnt ; ++i) {
            for(index e=0 ; e<b.edges.size() ; ++e) {
                index v1 = vcount + i*b.vertices.size() + b.edges[e].v1;
                index v2 = vcount + i*b.vertices.size() + b.edges[e].v2;
                evert  [e_occluded_start + b.edges.size()*i + e] = evert_s(v1, v2);
                elength[e_occluded_start + b.edges.size()*i + e] = norm(vpos[v2] - vpos[v1]);
            }
            for(index e=0 ; e<b.occluded_edges.size() ; ++e) {
                index v1 = vcount + i*b.vertices.size() + b.occluded_edges[e].v1;
                index v2 = vcount + i*b.vertices.size() + b.occluded_edges[e].v2;
                evert  [new_occluded_start + (ecount-e_occluded_start) + b.occluded_edges.size()*i + e] = evert_s(v1, v2);
                elength[new_occluded_start + (ecount-e_occluded_start) + b.occluded_edges.size()*i + e] = norm(vpos[v2] - vpos[v1]);
            }
        }

        for(index i=0 ; i<cnt ; ++i) {
            bvertcount[bcount+i] = b.vertices.size();
            bedgecount[bcount+i] = b.edges.size()+b.occluded_edges.size();
            bvert[bcount+i]      = (index*) malloc_align16(bvertcount[bcount+i]*sizeof(index));
            bedge[bcount+i]      = (index*) malloc_align16(bedgecount[bcount+i]*sizeof(index));

            for(index v=0 ; v<bvertcount[bcount+i] ; ++v)
                bvert[bcount+i][v] = vcount + i*b.vertices.size() + v;
            for(index e=0 ; e<b.edges.size() ; ++e)
                bedge[bcount+i][e] =   e_occluded_start + i*b.edges.size() + e;
            for(index e=0 ; e<b.occluded_edges.size() ; ++e)
                bedge[bcount+i][b.edges.size()+e] = new_occluded_start + (ecount-e_occluded_start) + i*b.occluded_edges.size() + e;
        }
        for(index i=0 ; i<cnt ; ++i) {
            bcenter[bcount+i] = computeCenterOfMass(bcount+i);
            for(index v=0 ; v<b.vertices.size() ; ++v) {
                vpos    [vcount + i*b.vertices.size() + v] -= bcenter[bcount+i];
                vprevpos[vcount + i*b.vertices.size() + v] -= bcenter[bcount+i];
                vpos    [vcount + i*b.vertices.size() + v] += centers[i];
                vprevpos[vcount + i*b.vertices.size() + v] += centers[i];
            }
        }

        bcount += cnt;
        vcount += cnt*b.vertices.size();
        ecount += cnt*(b.edges.size()+b.occluded_edges.size());
        e_occluded_start = new_occluded_start;
    }


    void integrateNewPositions() {
        const RT sq_timestep = timestep*timestep;
        for(index i=0 ; i<vcount ; ++i) {
            vaccel[i] = gravity; // Apply all forces
            // Then integrate.
            vec2<T> tmp = vpos[i];
            vpos[i] += vpos[i] - vprevpos[i] + vaccel[i]*sq_timestep;
            vprevpos[i] = tmp;
        }
    }
#define clamp(x,l,h) (std::min(h,std::max(l,x)))
    void keepVerticesInsideScreen() {
        for(index i=0 ; i<vcount ; ++i) {
            vpos[i].x = clamp(vpos[i].x, T(0), screen_size.x);
            vpos[i].y = clamp(vpos[i].y, T(0), screen_size.y);
        }
    }
    void edgeCorrectionStep() {
        // XXX
        // Don't keep it like this in the final code !!
        // This is required when one vertex (here, the first)
        // needs to follow the mouse : We need to reset its position
        // to the desired location at each edge correction step.
        // If we don't, edges behave like elastic bands.
        int x, y;
        SDL_GetMouseState(&x, &y);
        assert(vcount);
        vpos[0] = vec2<T>(x, y);

        for(index i=0 ; i<ecount ; ++i) {
            vec2<T> &v1pos = vpos[evert[i].v1];
            vec2<T> &v2pos = vpos[evert[i].v2];
            vec2<T>  v1v2  = v2pos - v1pos;
            T v1v2_len = norm(v1v2);
            T diff = v1v2_len - elength[i];
            v1pos += (v1v2*diff)/(T(2)*v1v2_len);
            v2pos -= (v1v2*diff)/(T(2)*v1v2_len);
        }
    }
    vec2<T> computeCenterOfMass(index i) {
        vec2<T> center(0,0);
        for(index v=0 ; v<bvertcount[i] ; ++v)
            center += vpos[bvert[i][v]];
        return center / bvertcount[i];
    }

    typedef struct {
        T depth;
        vec2<T> normal;
        index vert, edge;
    } collision_info;

    T interval_distance(vec2<T> a, vec2<T> b) {
        return (a.x<b.x ? b.x-a.y : a.x-b.y);
    }

    bool bodyHasEdge(index b, index e) { 
        for(index i=0 ; i<bedgecount[b] ; ++i)
            if(bedge[b][i] == e)
                return true;
        return false; 
    }
    vec2<T> projectBodyToAxis(index b, vec2<T> axis) {
        T dotp(dot(axis, vpos[bvert[b][0]]));
        vec2<T> proj(dotp, dotp);
        for(index i=0 ; i<bvertcount[b] ; ++i) {
            dotp = dot(axis, vpos[bvert[b][i]]);
            proj = vec2<T>(std::min(dotp, proj.x), std::max(dotp, proj.y));
        }
        return proj;
    }

    bool detectCollision(collision_info &ci, index b1, index b2) {
        // XXX Get MAX instead.
        ci.depth = T(10000);
        // Iterate through edges of both bodies.
        index currentbody = b1;
        for(index eindex=0 ;  ; ++eindex) 
        {
            if(currentbody==b1 && eindex >= bedgecount[b1])
                currentbody = b2;
            if(currentbody==b2 && eindex >= bedgecount[b2])
                break;
            index e = bedge[currentbody][eindex];
            /// XXX Optimization is pointless o_o
            if(e >= e_occluded_start)
                continue;
            index v1 = evert[e].v1, v2 = evert[e].v2;
            // axis = normalize(perpendicular to edge).
            vec2<T> axis = normalize(vec2<T>(vpos[v1].y - vpos[v2].y, vpos[v2].x - vpos[v1].x));
            vec2<T> b1proj = projectBodyToAxis(b1, axis);
            vec2<T> b2proj = projectBodyToAxis(b2, axis);
            T distance = interval_distance(b1proj, b2proj);
            if(distance > T(0))
                return false;
            distance = abs(distance);
            if(distance >= ci.depth)
                continue;
            ci.depth  = distance;
            ci.normal = axis;
            ci.edge   = e;
        }
        if(!bodyHasEdge(b2, ci.edge))
             std::swap(b1, b2); // so b1 contains the vertex.
        if(dot(ci.normal, bcenter[b1] - bcenter[b2]) < T(0))
             ci.normal = -ci.normal; // so normal points to b1.
        // find ci.vert
        T smallest_distance(10000); // XXX Get MAX instead.
        for(index v=0 ; v<bvertcount[b1] ; ++v) {
            T distance = dot(ci.normal, vpos[bvert[b1][v]] - bcenter[b1]);
            if(distance >= smallest_distance)
                continue;
            smallest_distance = distance;
            ci.vert = bvert[b1][v];
        }
        return true;
    }

    void processCollision(const collision_info &ci, index b1, index b2) {
        vec2<T> collision_vector = ci.normal*ci.depth;
        T t;
        index v  = ci.vert;
        index v1 = evert[ci.edge].v1;
        index v2 = evert[ci.edge].v2;
        if(abs(vpos[v1].x - vpos[v2].x ) > abs(vpos[v1].y - vpos[v2].y))
		    t = (vpos[v].x - collision_vector.x - vpos[v1].x) / (vpos[v2].x - vpos[v1].x);
	    else
		    t = (vpos[v].y - collision_vector.y - vpos[v1].y) / (vpos[v2].y - vpos[v1].y);
        T lambda = t*t + (T(1)-t)*(T(1)-t);
        vpos[v1] -= collision_vector*(T(1)-t)/(T(2)*lambda);
        vpos[v2] -= collision_vector*      t /(T(2)*lambda);
        vpos[v]  += collision_vector/T(2);

        vec2<T> mv = vpos[v] - vprevpos[v];
        vec2<T> v1v2 = vpos[v1] - vpos[v2];
        T dotp = dot(normalize(mv), normalize(v1v2));
        if(abs(dotp) > T(.9)) {
            //vec2<T> mv_proj = v1v2*dot(mv, v1v2)/norm(v1v2);
            T friction(.8); // XXX allow per-body friction.
            vpos[v] -= mv*friction;
        }
    }

    void processBodyCollisions() {
        for(index b1=0 ; b1<bcount ; ++b1) 
            for(index b2=0 ; b2<bcount ; ++b2) {
                if(b1 == b2)
                    continue;
                collision_info ci({});
                if(detectCollision(ci, b1, b2))
                    processCollision(ci, b1, b2);
            }
    }
    void iterateCollisions() {
        // No specific reason for it to stay the same. Could change dynamically.
        static const uint_fast32_t iteration_count(10);
        for(uint_fast32_t i=0 ; i<iteration_count ; ++i) {
            keepVerticesInsideScreen();
            edgeCorrectionStep();
            for(index b=0 ; b<bcount ; ++b)
                bcenter[b] = computeCenterOfMass(b);
            processBodyCollisions();
        }
    }
    void update() {
        integrateNewPositions();
        iterateCollisions();
    }

    index pickClosestScreenSpaceVertex(vec2<T> cursor) const {
        // Brute-force linear search FTW.
        // It is an error to call this function if there are no vertices.
        assert(vcount > 0);
        index closest_i = 0;
        T min_len = norm(cursor - vpos[closest_i]);
        for(index i=1 ; i<vcount ; ++i) {
            T next_len = norm(cursor - vpos[i]);
            if(min_len <= next_len)
                continue;
            min_len = next_len;
            closest_i = i;
        }
        return closest_i;
    }

    void renderSDL2(SDL_Renderer *rdr, RT interp=1) const {
        SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);

        //Render each edge...
        for(index i=0 ; i<ecount ; ++i) {
            index v1 = evert[i].v1, v2 = evert[i].v2;
            vec2<int> a, b;
            a.x = RT(vprevpos[v1].x) + interp*RT(vpos[v1].x - vprevpos[v1].x);
            a.y = RT(vprevpos[v1].y) + interp*RT(vpos[v1].y - vprevpos[v1].y);
            b.x = RT(vprevpos[v2].x) + interp*RT(vpos[v2].x - vprevpos[v2].x);
            b.y = RT(vprevpos[v2].y) + interp*RT(vpos[v2].y - vprevpos[v2].y);
            SDL_RenderDrawLine(rdr, a.x, a.y, b.x, b.y);
        }
        SDL_SetRenderDrawColor(rdr,   0, 255, 0, 255);
        // Ugly way of rendering vertices.
        for(index i=0 ; i<vcount ; ++i) {
            SDL_SetRenderDrawColor(rdr, vcolor[i].r, vcolor[i].g, vcolor[i].b,
                                        vcolor[i].a);
            for(int y=-1 ; y<=1 ; ++y) {
                for(int x=-1 ; x<=1 ; ++x) {
                    vec2<int> ipos;
                    ipos.x = RT(vprevpos[i].x) + interp*RT(vpos[i].x - vprevpos[i].x);
                    ipos.y = RT(vprevpos[i].y) + interp*RT(vpos[i].y - vprevpos[i].y);
                    SDL_RenderDrawPoint(rdr, ipos.x+x, ipos.y+y);
                }
            }
        }
    }
};










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
