#ifndef TEST1_HPP
#define TEST1_HPP

#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <TextGui.hpp>

namespace Test1 {

extern const uintptr_t updateFixedStepSimulationBit;

typedef boulette::q<24,8> q;
typedef boulette::aabb_2d<q> aabb_2d;
typedef boulette::disk_2d<q> disk_2d;
typedef boulette::vec2<q> vec2;
typedef boulette::vec2<q> unitv2;
typedef boulette::vec2<int32_t> i32v2;

#if 0
typedef boulette::VelocitySys<q>         VelocitySys;
typedef boulette::VelocityCpt<q>         VelocityCpt;
typedef boulette::Aabb2dRenderableSys<q> Aabb2dRenderableSys;
typedef boulette::Aabb2dRenderableCpt<q> Aabb2dRenderableCpt;
typedef boulette::Aabb2dColliderSys<q>   Aabb2dColliderSys;
typedef boulette::Aabb2dColliderCpt<q>   Aabb2dColliderCpt;


// C'est quoi mon problème ?
// 
// - HandleEvents()
// - UserPreUpdateSim()
//   - hero.next_vel += ...; // shouldn't actually have to type "next_"
// - UpdateSim()
//   - (centers,vels) = (next_centers, next_vels)
//   - SimCull() // Broad phase
//   - (next_centers[,next_vels]) <= TestAabbsVsAabbs() <-- tick, (colliders <-- (centers,sizes[,vels]))
//     // ^ Narrow phase
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
    i32v2 wheel;
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
    TextGui text_gui;
    Mouse mouse;
    Keyboard keyboard;
    SimState simstate;
    disk_2d disk;
    vec2 vel, accel;
public:
    Test1(SDL_Renderer *rdr, unitv2 screen_size);
    ~Test1();
    bool shouldQuit() const;
    void updateFixedStepSimulation();
    void handleSDL2Event(const SDL_Event *e);
    void renderSDL2(SDL_Renderer *rdr) const;
};

} // namespace Test1

#endif//TEST1_HPP
