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
typedef sphys::disk_2d<q> disk_2d;
typedef sphys::vec2<q> vec2;

struct rgba32 {
    uint32_t r:8;
    uint32_t g:8;
    uint32_t b:8;
    uint32_t a:8;
};

#if 0
typedef sphys::VelocitySys<q>         VelocitySys;
typedef sphys::VelocityCpt<q>         VelocityCpt;
typedef sphys::Aabb2dRenderableSys<q> Aabb2dRenderableSys;
typedef sphys::Aabb2dRenderableCpt<q> Aabb2dRenderableCpt;
typedef sphys::Aabb2dColliderSys<q>   Aabb2dColliderSys;
typedef sphys::Aabb2dColliderCpt<q>   Aabb2dColliderCpt;


// C'est quoi mon problème ?
// 
// - HandleEvents()
// - UserPreUpdateSim()
//   - hero.vel = ...;
// - UpdateSim()
//   - (centers,vels) = (next_centers, next_vels)
//   - SimCull()
//   - (next_centers[,next_vels]) <= TestAabbsVsAabbs() <-- tick, (colliders <-- (centers,sizes[,vels]))
// - UserPostUpdateSim()
// - Render() // arrive 4x de suite environ, car plus fréquent que UpdateSim().
//   - RenderCull()
//   - RenderAabbs() <-- tick, (renderables <-- (lerp(centers, next_centers),sizes[,lerp(vels, next_vels)]))

struct AabbEntity {
    rgba32 color;
    VelocityCpt vel;
    Aabb2d aabb;
    Aabb2dRenderableCpt renderable;
    Aabb2dColliderCpt collider;
};
AabbEntity::AabbEntity() : color(255,0,0,255), aabb(vec2(20,20),vec2(5,5)) {
    renderable.setColor(color);
    renderable.useVelocityCpt(vel);
    renderable.setAabb2d(aabb);
    collider  .useVelocityCpt(vel);
    collider  .setAabb2d(aabb);
}
void AabbEntity::giveInput(const Keyboard &keyboard) {
    static const int speedmul = 5;
    if(keyboard.right && !keyboard.left)
        vel.x = speedmul;
    if(keyboard.left && !keyboard.right)
        vel.x = -speedmul;
    if(keyboard.up && !keyboard.down)
        vel.y = -speedmul;
    if(keyboard.down && !keyboard.up)
        vel.y = speedmul;
}
#endif

struct Mouse {
    bool down;
    sphys::vec2<int32_t> wheel;
};

struct Keyboard {
    bool up    : 1;
    bool down  : 1;
    bool left  : 1;
    bool right : 1;
};

struct SimState {
    bool intersects;
    bool aabb_disk_intersects;
};

class Test1 {
protected:
    bool m_shouldQuit;
    uint64_t tick;
    bool wasPreparedToRender;
    TTF_Font *font;
    SDL_Texture *guiTex;
    Mouse mouse;
    Keyboard keyboard;
    SimState simstate;
    std::vector<aabb_2d> aabbs;
    std::vector<disk_2d> disks;
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
