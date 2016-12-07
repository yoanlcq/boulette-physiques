#include <TestVerlet.hpp>
#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

using namespace std;
using namespace boulette;

extern uint32_t g_update_dt_ms;

namespace TestVerlet {

const uintptr_t updateFixedStepSimulationBit(0x800);

#define hope assert

TestVerlet::TestVerlet(unitv2 screen_size) : 
    m_shouldQuit(false), tick(0), wasPreparedToRender(false),
    verletSys(screen_size) {}

TestVerlet::~TestVerlet() {}

bool TestVerlet::shouldQuit() const { return m_shouldQuit; }

static uint32_t last_update_time_ms = 0;

void TestVerlet::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: case SDL_APP_TERMINATING: m_shouldQuit = true; break;
    case SDL_USEREVENT:
        if((uint32_t)(uintptr_t)e->user.data1 & updateFixedStepSimulationBit) {
            updateFixedStepSimulation(); 
            last_update_time_ms = SDL_GetTicks();
        }
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

    int x, y;
    SDL_GetMouseState(&x, &y);
    for(uint_fast32_t i=0 ; i<verletSys.vcount ; ++i)
        verletSys.vcolor[i].r = 0;
    size_t idx = verletSys.pickClosestScreenSpaceVertex(unitv2(x,y));
    if(norm(verletSys.vpos[idx] - unitv2(x,y)) < unit(300)) {
        verletSys.vcolor[idx].r = 255;
        verletSys.vpos[idx]  = unitv2(x,y);
        //verletSys.vprevpos[idx]  = unitv2(x,y);
    }
}

void TestVerlet::prepareRenderSDL2(SDL_Renderer *rdr) {
    wasPreparedToRender = true;
}

void TestVerlet::renderSDL2(SDL_Renderer *rdr) const {
    assert(wasPreparedToRender);
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    uint32_t time_ms = SDL_GetTicks();
    uint32_t frame_dt = time_ms - last_update_time_ms;
    float interp = frame_dt/float(g_update_dt_ms);
    //cout << interp << endl;
    verletSys.renderSDL2(rdr, interp);
}

} // namespace TestVerlet
