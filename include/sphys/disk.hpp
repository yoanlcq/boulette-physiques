#ifndef SPHYS_DISK_HPP
#define SPHYS_DISK_HPP

#include <sphys.hpp>
#include <SDL2/SDL.h>

namespace sphys {

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
        return radius+other.radius < norm(center - other.center);
    }
    bool intersects(const aabb_2d<T> &other) const {
        vec2<T> dvec = abs(center - other.center);
        vec2<T> a(0,0);
        if(!dvec.x)
            return radius+other.halfSize.y > dvec.y;
        if(!dvec.y)
            return radius+other.halfSize.x > dvec.x;
        if(dvec.x/dvec.y > other.halfSize.x/other.halfSize.y)
            a = dvec * other.halfSize.x / dvec.x;
        else
            a = dvec * other.halfSize.y / dvec.y;
        //std::cout << "  dvec : " << dvec << "(norm:" << norm(dvec) << ")" << std::endl;
        //std::cout << "     a : " <<    a << "(norm:" << norm(   a) << ")" << std::endl;
        //std::cout << "radius : " << radius << std::endl;
        return radius+norm(a) >= norm(dvec);
    }
};

}

#endif//SPHYS_DISK_HPP
