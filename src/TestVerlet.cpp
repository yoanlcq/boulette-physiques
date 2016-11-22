#include <TestVerlet.hpp>
#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

using namespace std;
using namespace boulette;

namespace TestVerlet {

const uintptr_t updateFixedStepSimulationBit(0x800);

#define hope assert

TestVerlet::TestVerlet(vec2<q> screen_size) : 
    m_shouldQuit(false), tick(0), wasPreparedToRender(false),
    verletSys(screen_size) {}

TestVerlet::~TestVerlet() {}

bool TestVerlet::shouldQuit() const { return m_shouldQuit; }

void TestVerlet::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: case SDL_APP_TERMINATING: m_shouldQuit = true; break;
    case SDL_USEREVENT:
        if((uint32_t)(uintptr_t)e->user.data1 & updateFixedStepSimulationBit)
            updateFixedStepSimulation(); 
        break;
#define HANDLE_KEY_EVT(is_down) \
        switch(e->key.keysym.sym) { \
        case SDLK_RIGHT: break; \
        case SDLK_UP:    break; \
        case SDLK_LEFT:  break; \
        case SDLK_DOWN:  break; \
        }
    case SDL_KEYDOWN: HANDLE_KEY_EVT(true);  break;
    case SDL_KEYUP:   HANDLE_KEY_EVT(false); break;
#undef HANDLE_KEY_EVT
    case SDL_MOUSEBUTTONDOWN: break;
    case SDL_MOUSEBUTTONUP:   break;
    case SDL_MOUSEWHEEL:       break;
    }
}

void TestVerlet::updateFixedStepSimulation() {
    ++tick;
    verletSys.update();
}

void TestVerlet::prepareRenderSDL2(SDL_Renderer *rdr) {
    wasPreparedToRender = true;
}

void TestVerlet::renderSDL2(SDL_Renderer *rdr) const {
    assert(wasPreparedToRender);
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    verletSys.renderSDL2(rdr);
}

} // namespace TestVerlet
