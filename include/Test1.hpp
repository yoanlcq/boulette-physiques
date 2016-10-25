#ifndef TEST1_HPP
#define TEST1_HPP

#include <sphys.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace Test1 {

extern const uintptr_t updateFixedStepSimulationBit;

typedef sphys::q<16,16> q;
typedef sphys::aabb_2d<q> aabb_2d;
typedef sphys::vec2<q> vec2;

class Test1 {
protected:
    bool m_shouldQuit;
    uint64_t tick;
    bool wasPreparedToRender;
    TTF_Font *font;
    SDL_Texture *guiTex;
    std::vector<aabb_2d> aabbs;
    void renderSDL2_GUI(SDL_Renderer *rdr) const;
public:
    Test1();
    ~Test1();
    bool shouldQuit() const;
    void prepareRenderSDL2(SDL_Renderer *rdr);
    void updateFixedStepSimulation();
    void handleSDL2Event(const SDL_Event *e);
    void renderSDL2(SDL_Renderer *rdr) const;
};

} // namespace Test1

#endif//TEST1_HPP
