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
//   arrays (goal : minimise cache trashing).
// - Arrays are aligned on a 16-byte boundary, thanks to _mm_malloc().
//   The compiler can thus generate aligned SSE2 instructions (such as movdqa, 
//   instead of the slower movdqu). However, seeing the disassembly,
//   the compiler still has to perform some checks.
//   Using GCC's __attribute__((vectorsize(16))) would do the trick, but then
//   our code cannot be generic anymore.
// - Edges which are guaranteed to be fully occluded by other edges don't need to be
//   taken into account for collision detection.
//   For instance, edges which are inside a body in order to keep it solid will never
//   collide with anything on the outside (and those are numerous).
//   In the example code, this "optimization" was done with an "if()" for each edge,
//   at each update step ("int boundary;" member). 
//   In my code, the edge data arrays are split in two sections :
//   - The edges that are always completely occluded;
//   - The others. We only iterate over this one section when detecting collisions.
// - In the example, collision was stateful (see use of CollisionInfo), so not parallelizable.
// - The Add*() methods focused on adding one object at a time, but when there's one,
//   there's many.
//   The addRigidBodies() method allows to efficiently add any number of rigid bodies
//   in the system at once, taking advantage of data locality.
// - Each vertex used to store a back pointer to the body they're in, which is needlessly
//   expensive memory-wise.
//
// About the arrays : I acknowledge they don't quite fit the C++ mindset, but they're the basic tools in DOD.
// I could have looked into std::valarray, std::vector, and extending std::allocator.
// I also considered implementing my own (small) aligned vector type, but chose not to do it - the 
// rationale being that each vector has to store is element count (to implement resize()),
// which would be redundant for most cases here.
// For instance, as the below VerletPhysicsSystem shows, it wouldn't make sense for the 'vpos' and 'vprevpos'
// members to each maintain their own 'element count' member : only one is needed and its is the element count
// for both arrays.
//
// TODO
// - Fix vertex picking deforming edges
// - Fix fake occluded edges optimization
// - Fix objects not behaving well


// I initially planned to have these actually return 16-byte-aligned memory,
// but I can't simply because there's no aligned realloc() function.
// Actually, it doesn't make a big difference because the compiler
// generates the same assembly output either way, which is still not that
// bad.
// So, misleading names, but if one day they have to be implemented, it'd be here.
#define malloc_align16(...)  malloc(__VA_ARGS__)
#define realloc_align16(...) realloc(__VA_ARGS__)
#define free_align16(...)    free(__VA_ARGS__)


// T is the space type, RT is expected to be a real-number type.
template <typename T, typename RT>
struct VerletPhysicsSystem {

    typedef aabb_2d<T> aabb;
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
    aabb    *baabb;       // For each body, its Axis-Aligned Bounding Box.

    // This flag is pretty self-explanatory and a bit hackish.
    // Basically any good physics engine would allow for creating
    // "physics materials" which, among others, can specify a friction
    // coefficient.
    // In the original implementation, all bodies were frictionless, and
    // never stopped sliding on stuff. So I tried to implement it myself,
    // initially wanting friction to be per-body. Now I haven't got much
    // time, so this setting affects all bodies at once.
    // Also, I suspect that enabling it causes some noticeable 
    // problems to the bodies' behaviours.
    bool enable_experimental_friction;
    // This one ranges from 0 (no friction) to 1 (full friction).
    // Avoid setting it to 1, prefer something like 0.9 instead.
    // Only takes effect when enable_experimental_friction == true.
    T friction_coefficient;


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
        bedge     (nullptr),
        baabb     (nullptr),
        enable_experimental_friction(false),
        friction_coefficient(.8)
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
        free_align16(baabb);
    }

    void reset() {
        // Just reconstruct ourselves, and destroy our past self in the process.
        // Part of me feels bad for this.
        *this = VerletPhysicsSystem(screen_size);
    }

    void reshape(vec2<T> p_screen_size) {
        screen_size = p_screen_size;
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

    static BodyDescriptor describeNecklace(float radius, size_t vertex_count, const std::vector<rgba32> &colors) {
        BodyDescriptor necklace;
        for(uint_fast32_t i=0 ; i<vertex_count ; ++i) {
            // (i+0.5) for more fun : otherwise the bottom edge falls flat 
            // on the ground and we don't get to see the body bouncing a bit ._.
            float theta = float(M_PI*2*(i+0.5))/float(vertex_count);
            vec2<T> pos(radius*cos(theta), radius*sin(theta));
            necklace.vertices.push_back(VertexDescriptor(pos, colors[i%colors.size()]));
            necklace.edges.push_back(evert_s(i, (i+1)%vertex_count));
        }
        return necklace;
    }

    static BodyDescriptor describeRigidDisk(float radius, size_t vertex_count, const std::vector<rgba32> &colors) {
        BodyDescriptor disk = describeNecklace(radius, vertex_count, colors);
        for(uint_fast32_t i=0 ; i<vertex_count-2 ; ++i)
            for(uint_fast32_t j=0 ; j<(i ? vertex_count-2-i : vertex_count-3); ++j)
                disk.occluded_edges.push_back(evert_s(i, i+2+j));
        return disk;
    }
    static BodyDescriptor describeSlimyDisk(float radius, size_t vertex_count, const std::vector<rgba32> &colors) {
        BodyDescriptor disk = describeNecklace(radius, vertex_count, colors);
        for(uint_fast32_t i=0 ; i<vertex_count ; ++i)
            disk.occluded_edges.push_back(evert_s(i, (i+(vertex_count/2))%vertex_count));
        return disk;
    }


    // This method was easily the hardest to implement, and it is definitely not easy to grasp at first glance.
    // I'd like to make it simpler, but this would imply creating more variables and splitting it into
    // helper methods, which I feel would be counter-productive.
    //
    // I feel sorry for any reader of this method - seems like it's one place where DOD falls short.
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
        baabb      = (aabb*)    realloc_align16(baabb     , (bcount+cnt)  *sizeof(*baabb     ));

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
            baabb  [bcount+i] = computeAabb(bcount+i);
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

    aabb computeAabb(index i) const {
        T minx(std::numeric_limits<T>::max());
        T miny(std::numeric_limits<T>::max());
        T maxx(std::numeric_limits<T>::min());
        T maxy(std::numeric_limits<T>::min());
        for(index v=0 ; v<bvertcount[i] ; ++v) {
            minx = std::min(minx, vpos[bvert[i][v]].x);
            miny = std::min(miny, vpos[bvert[i][v]].y);
            maxx = std::max(maxx, vpos[bvert[i][v]].x);
            maxy = std::max(maxy, vpos[bvert[i][v]].y);
        }
        vec2<T> halfSize(abs(maxx-minx)/T(2), abs(maxy-miny)/T(2));
        vec2<T> center(minx+halfSize.x, miny+halfSize.y);
        return aabb(center, halfSize);
    }

    vec2<T> computeCenterOfMass(index i) const {
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
        ci.depth = std::numeric_limits<T>::max();
        //ci.depth = T(10000);
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
        T smallest_distance(std::numeric_limits<T>::max());
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

        if(!enable_experimental_friction)
            return;
        vec2<T> mv = vpos[v] - vprevpos[v];
        vec2<T> v1v2 = vpos[v1] - vpos[v2];
        T dotp = dot(normalize(mv), normalize(v1v2));
        if(abs(dotp) > T(.9))
            vpos[v] -= mv*friction_coefficient;
    }

    void processBodyCollisions() {
        for(index b1=0 ; b1<bcount ; ++b1) 
            for(index b2=0 ; b2<bcount ; ++b2) {
                if(b1 == b2)
                    continue;
                if(!baabb[b1].intersects(baabb[b2]))
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
            for(index b=0 ; b<bcount ; ++b) {
                // XXX They should be packed together
                bcenter[b] = computeCenterOfMass(b);
                baabb  [b] = computeAabb(b);
            }
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

        SDL_SetRenderDrawColor(rdr, 0, 255, 0, 255);
        //Render each AABB... No pretty interpolaton here, we don't care that much.
        for(index i=0 ; i<bcount ; ++i)
            baabb[i].renderSDL2Wireframe(rdr);
    }
};



}

#endif//BOULETTE_VERLET_HPP
