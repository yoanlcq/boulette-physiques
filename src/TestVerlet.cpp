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
    verletSys(screen_size, 1) {

#define COLCOUNT 4
    VerletSys::rgba32 cols[COLCOUNT] = {
        VerletSys::rgba32(255, 000, 000, 255),
        VerletSys::rgba32(255, 255, 000, 255),
        VerletSys::rgba32(000, 255, 000, 255),
        VerletSys::rgba32(000, 000, 255, 255)
    };

    VerletSys::VerletRigidBody box;
    box.vertices.push_back(VerletSys::VerletVertex(unitv2(-30, -30), cols[0]));
    box.vertices.push_back(VerletSys::VerletVertex(unitv2( 30, -30), cols[1]));
    box.vertices.push_back(VerletSys::VerletVertex(unitv2( 30,  30), cols[2]));
    box.vertices.push_back(VerletSys::VerletVertex(unitv2(-30,  30), cols[3]));
    box.edges.push_back(VerletSys::evert_s(0, 1));
    box.edges.push_back(VerletSys::evert_s(1, 2));
    box.edges.push_back(VerletSys::evert_s(2, 3));
    box.edges.push_back(VerletSys::evert_s(3, 0));
    box.occluded_edges.push_back(VerletSys::evert_s(0, 2));
    box.occluded_edges.push_back(VerletSys::evert_s(1, 3));
#define BOXCOUNT 4
    unitv2 centers[BOXCOUNT];
    for(uint_fast32_t i=0 ; i<BOXCOUNT ; ++i)
        centers[i] = unitv2(60 + i*150, 60);

#define DISKSEGCOUNT 42
    VerletSys::VerletRigidBody disk;
    for(uint_fast32_t i=0 ; i<DISKSEGCOUNT ; ++i) {
        float r = 60.f, theta = M_PI*2.f*i/(float)DISKSEGCOUNT;
        unitv2 pos(r*cosf(theta), r*sinf(theta));
        disk.vertices.push_back(VerletSys::VerletVertex(pos, cols[i%COLCOUNT]));
        disk.edges.push_back(VerletSys::evert_s(i, (i+1)%DISKSEGCOUNT));
        //disk.occluded_edges.push_back(VerletSys::evert_s(i, (i+(DISKSEGCOUNT/2))%DISKSEGCOUNT));
        disk.edges.push_back(VerletSys::evert_s(i, (i+(DISKSEGCOUNT/2))%DISKSEGCOUNT));
        //for(uint_fast32_t j=0 ; j<DISKSEGCOUNT ; ++j)
            //if(j != i)
                //disk.edges.push_back(VerletSys::evert_s(i, j));
    }

    verletSys.addRigidBodies(disk, BOXCOUNT, centers);
    verletSys.addRigidBodies( box, BOXCOUNT, centers);
}

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
