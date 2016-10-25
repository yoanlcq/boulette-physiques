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
    hope(font = TTF_OpenFont("res/basis33/basis33.ttf", 16));
    aabbs.push_back(aabb_2d(vec2(50,50), vec2(10,10)));
    aabbs.push_back(aabb_2d(vec2(200,200), vec2(50,20)));
}
Test1::~Test1() {
    SDL_DestroyTexture(guiTex);
    TTF_CloseFont(font);
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
}
struct rgba32 {
    uint32_t r:8;
    uint32_t g:8;
    uint32_t b:8;
    uint32_t a:8;
};

void Test1::renderSDL2_GUI(SDL_Renderer *rdr) const {
    SDL_Color color = {0, 128, 0, 0};
    ostringstream oss;
    oss << "Tick: " << tick;
    SDL_Surface *s = TTF_RenderUTF8_Solid(
        font, oss.str().c_str(), color
    );
    hope(s);
    rgba32 *rgba = new rgba32[s->w*s->h];
    for(long y=0 ; y<s->h ; ++y) {
        for(long x=0 ; x<s->w ; ++x) {
             if(!((const uint8_t*)s->pixels)[y*s->pitch + x]) {
                 memset(&rgba[y*s->w + x], 0, sizeof(rgba32));
                 continue;
             }
            rgba[y*s->w + x].r = 255;
            rgba[y*s->w + x].g = 200;
            rgba[y*s->w + x].b = 10;
            rgba[y*s->w + x].a = 255;
        }
    }
    SDL_Rect rect = {0, 0, s->w, s->h};
    // XXX Use SDL_LockTexture() ???
    SDL_UpdateTexture(guiTex, &rect, rgba, s->w*sizeof(rgba32));
    SDL_RenderCopy(rdr, guiTex, &rect, &rect);
    SDL_FreeSurface(s);
    delete[] rgba;
}
void Test1::prepareRenderSDL2(SDL_Renderer *rdr) {
    wasPreparedToRender = true;
    hope(guiTex = SDL_CreateTexture(rdr, 
        SDL_PIXELFORMAT_ABGR8888, 
        SDL_TEXTUREACCESS_STREAMING, 
        800, 600
    ));
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
