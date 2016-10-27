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
    SDL_Rect toSDL2Rect() const {
        SDL_Rect rect;
        rect.x = center.x - halfSize.x;
        rect.y = center.y - halfSize.y;
        rect.w = T(2)*halfSize.x;
        rect.h = T(2)*halfSize.y;
        return rect;
    }
    void renderSDL2(SDL_Renderer *rdr) const {
        SDL_Rect rect = toSDL2Rect();
        SDL_RenderFillRects(rdr, &rect, 1);
    }
    void renderSDL2Wireframe(SDL_Renderer *rdr) const {
        SDL_Rect rect = toSDL2Rect();
        SDL_RenderDrawRects(rdr, &rect, 1);
    }
    bool intersects(const aabb_2d &other) const {
        vec2<T> dist = abs(center - other.center);
        vec2<T> intersect_dist = halfSize + other.halfSize;
        return dist < intersect_dist;
    }
};

}

#endif//SPHYS_AABB_HPP
