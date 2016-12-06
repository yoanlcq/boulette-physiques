#ifndef TESTVERLET_HPP
#define TESTVERLET_HPP

#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace TestVerlet {

extern const uintptr_t updateFixedStepSimulationBit;

typedef boulette::q<24,8> q;
typedef q unit;
typedef q real;
typedef boulette::vec2<unit> unitv2;
typedef boulette::VerletPhysicsSystem<unit,real> VerletSys;

class TestVerlet {
protected:
    bool m_shouldQuit;
    uint64_t tick;
    bool wasPreparedToRender;
    VerletSys verletSys;
public:
     TestVerlet(unitv2 screen_size);
    ~TestVerlet();
    bool shouldQuit() const;
    void prepareRenderSDL2(SDL_Renderer *rdr);
    void updateFixedStepSimulation();
    void handleSDL2Event(const SDL_Event *e);
    void renderSDL2(SDL_Renderer *rdr) const;
};

} // namespace TestVerlet

#endif//TESTVERLET_HPP
