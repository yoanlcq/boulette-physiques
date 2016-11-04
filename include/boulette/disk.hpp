#ifndef BOULETTE_DISK_HPP
#define BOULETTE_DISK_HPP

#include <boulette.hpp>
#include <SDL2/SDL.h>

namespace boulette {

template <typename T>
struct disk_2d { 
    vec2<T> center;
    T       radius;
    disk_2d(vec2<T> center=vec2<T>(0,0), T radius=.5) 
        : center(center), radius(radius) {}
    ~disk_2d() {}
    void renderSDL2Wireframe(SDL_Renderer *rdr, size_t line_count=32) const {
        SDL_Point *points = new SDL_Point[line_count+1];
        for(size_t i=0 ; i<line_count ; ++i) {
            points[i].x = round(float(center.x)+float(radius)*cos(2*M_PI*i/(float)line_count));
            points[i].y = round(float(center.y)+float(radius)*sin(2*M_PI*i/(float)line_count));
        }
        points[line_count] = points[0];
        SDL_RenderDrawLines(rdr, points, line_count+1);
        delete[] points;
    }
    bool intersects(const disk_2d &other) const {
        return radius+other.radius > norm(center - other.center);
    }
    bool intersects(const aabb_2d<T> &other) const {
        aabb_2d<T> my_aabb(center, vec2<T>(radius, radius));
        bool aabb_intersects = other.intersects(my_aabb);
        if(!aabb_intersects)
            return false;
        for(int ys=-1 ; ys<=1 ; ys+=2) for(int xs=-1 ; xs<=1 ; xs+=2)
            if(intersects(disk_2d(vec2<T>(
                other.center.x + (xs>0 ? other.halfSize.x : -other.halfSize.x),
                other.center.y + (ys>0 ? other.halfSize.y : -other.halfSize.y)
                ), 0)))
            {
                std::cout << "at " << xs << "," << ys << std::endl;
                return true;
            }
        return false;
    }
};

}

#endif//BOULETTE_DISK_HPP
