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

TestVerlet::TestVerlet(SDL_Renderer *rdr, unitv2 screen_size) : 
    m_shouldQuit(false), tick(0),
    text_gui(rdr, screen_size.x, screen_size.y),
    verletSys(screen_size, 1) 
{
    vector<VerletSys::rgba32> cols;
    cols.push_back(VerletSys::rgba32(255, 000, 000, 255));
    cols.push_back(VerletSys::rgba32(255, 255, 000, 255));
    cols.push_back(VerletSys::rgba32(000, 255, 000, 255));
    cols.push_back(VerletSys::rgba32(000, 000, 255, 255));
    unitv2 centers[4];
    for(uint_fast32_t y=0 ; y<4 ; ++y) {
        for(uint_fast32_t x=0 ; x<4 ; ++x)
            centers[x] = unitv2(60+x*130+y*20, 60+y*70);
        VerletSys::BodyDescriptor body;
        if(y<2)
            body = VerletSys::describeRigidDisk(30, 4+y, cols);
        else
            body = VerletSys::describeSlimyDisk(30+y*5, 22+y, cols);
        verletSys.addRigidBodies(body, 4, centers);
    }
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
    stringstream ss;
    int x, y;
    SDL_GetMouseState(&x, &y);
    ss 
        << "Mouse : (" << x << ", " << y << ")" 
        << " | Timestep : " << verletSys.timestep
        << " | Gravity : "  << verletSys.gravity
        << std::endl
        << verletSys.bcount << " bodies"
        << " | " << verletSys.vcount << " vertices"
        << " | " << verletSys.ecount << " edges"
        << " ("  << (verletSys.ecount-verletSys.e_occluded_start)
        << " ("  << (uint32_t)(100.f*(verletSys.ecount-verletSys.e_occluded_start)/(float)verletSys.ecount) << "%)"
        << " occluded)"
        << std::endl
        << "...";
    /*
    for(uint_fast32_t i=0 ; i<verletSys.bcount ; ++i) {
        ss  << "V:" << verletSys.bvertcount[i] << " | "
            << "E:" << verletSys.bedgecount[i] << " | "
            << std::endl;
    }
    */
    text_gui.text = ss.str();
    text_gui.update();
}

void TestVerlet::renderSDL2(SDL_Renderer *rdr) const {
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    uint32_t time_ms = SDL_GetTicks();
    uint32_t frame_dt = time_ms - last_update_time_ms;
    float interp = frame_dt/float(g_update_dt_ms);
    //cout << interp << endl;
    verletSys.renderSDL2(rdr, interp);
    text_gui.renderSDL2(rdr);
}

} // namespace TestVerlet
