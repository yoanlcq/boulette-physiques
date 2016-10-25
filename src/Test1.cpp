#include <Test1.hpp>
#include <sphys.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

using namespace std;
using namespace sphys;

template<typename T> static void testRealNumbers(T a, T b) {
    cout << "a   = " <<   a << endl;
    cout << "b   = " <<   b << endl;
    cout << "a+b = " << a+b << endl;
    cout << "a-b = " << a-b << endl;
    cout << "a*b = " << a*b << endl;
    cout << "a/b = " << a/b << endl;
}

static void testQ() {
    static const double a(1.25), b(2.25);
#define WRAP_TEST_REAL_NUMBER(...) \
    cout << endl << "Testing " << #__VA_ARGS__ << " :" << endl; \
    testRealNumbers<__VA_ARGS__>(__VA_ARGS__(a),__VA_ARGS__(b))
    WRAP_TEST_REAL_NUMBER(double);
    WRAP_TEST_REAL_NUMBER(q<32,0>);
    WRAP_TEST_REAL_NUMBER(q<28,4>);
    WRAP_TEST_REAL_NUMBER(q<26,6>);
    WRAP_TEST_REAL_NUMBER(q<6,26>);
    WRAP_TEST_REAL_NUMBER(q<4,28>);
#undef WRAP_TEST_REAL_NUMBER
}

namespace Test1 {

const uintptr_t updateFixedStepSimulationBit(0x800);

#define hope assert

Test1::Test1() : m_shouldQuit(false), tick(0), font(NULL), guiTex(NULL) {
    testQ();
    font = TTF_OpenFont("res/basis33/basis33.ttf", 16);
    hope(font);
    aabbs.push_back(aabb_2d(vec2(50,50), vec2(10,10)));
    aabbs.push_back(aabb_2d(vec2(200,200), vec2(50,20)));
}
Test1::~Test1() {
    SDL_DestroyTexture(guiTex);
}
bool Test1::shouldQuit() const { return m_shouldQuit; }
void Test1::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: m_shouldQuit = true; break;
    case SDL_USEREVENT:
        if((uint32_t)(uintptr_t)e->user.data1 & updateFixedStepSimulationBit)
            updateFixedStepSimulation(); 
        break;
    }
}
void Test1::updateFixedStepSimulation() {
    ++tick;
    cout << "Tick: " << tick << endl;
}
void Test1::renderSDL2_GUI(SDL_Renderer *rdr) const {
    SDL_Color color = {255, 255, 0};
    ostringstream oss;
    oss << "Tick: " << tick << endl;
    SDL_Surface *s = TTF_RenderText_Solid(
        font, oss.str().c_str(), color
    );
    SDL_Rect rect = {10, 10, s->w, s->h};
    // XXX Use SDL_LockTexture() ???
    SDL_UpdateTexture(guiTex, &rect, s->pixels, s->pitch);
    SDL_RenderCopy(rdr, guiTex, &rect, &rect);
    SDL_FreeSurface(s);
}
void Test1::prepareRenderSDL2(SDL_Renderer *rdr) {
    wasPreparedToRender = true;
    guiTex = SDL_CreateTexture(rdr, 
        SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, 
        800, 600);
    hope(guiTex);
}

void Test1::renderSDL2(SDL_Renderer *rdr) const {
    assert(wasPreparedToRender);
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    for(size_t i=0 ; i<aabbs.size() ; ++i)
        aabbs[i].renderSDL2(rdr);
    SDL_SetRenderDrawColor(rdr, 255, 0, 255, 255);
    renderSDL2_GUI(rdr);
}

} // namespace Test1
