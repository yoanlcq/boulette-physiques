#ifndef TEST1_HPP
#define TEST1_HPP

#include <sphys.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>

namespace Test1 {

typedef sphys::q<16,16> q;
typedef sphys::aabb_2d<q> aabb_2d;
typedef sphys::vec2<q> vec2;

class Test1 {
    bool m_shouldQuit;
    std::vector<aabb_2d> aabbs;
public:
    Test1();
    ~Test1();
    bool shouldQuit() const;
    void handleSDL2Event(const SDL_Event *e);
    void renderSDL2(SDL_Renderer *rdr) const;
};

} // namespace Test1

#endif//TEST1_HPP
