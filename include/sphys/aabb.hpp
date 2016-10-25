#ifndef SPHYS_AABB_HPP
#define SPHYS_AABB_HPP

#include <sphys.hpp>
#include <SDL2/SDL.h>

namespace sphys {

template <typename T>
struct aabb_2d { 
    vec2<T> center, halfSize;
    aabb_2d() : center(0,0), halfSize(.5,.5) {}
    aabb_2d(vec2<T> center, vec2<T>halfSize) 
        : center(center), halfSize(halfSize) {}
    ~aabb_2d() {}
    void renderSDL2(SDL_Renderer *rdr) const {
        SDL_Rect rect;
        rect.x = center.x - halfSize.x;
        rect.y = center.y - halfSize.y;
        rect.w = 2*halfSize.x;
        rect.h = 2*halfSize.y;
        SDL_RenderFillRects(rdr, &rect, 1);
    }
    bool intersects(const aabb_2d &other) const {
        vec2<T> dist = vec2<T>::abs(center - other.center);
        vec2<T> intersect_dist = halfSize + other.halfSize;
        return dist < intersect_dist;
    }
};

}

#endif//SPHYS_AABB_HPP
