#include <Test1.hpp>
#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

using namespace std;
using namespace boulette;

double ln(double x) { return log(x); }

template<typename T> static void testRealNumbers(T x, T y, T c) {
    cout << "x        = " <<   x      << endl;
    cout << "y        = " <<   y      << endl;
    cout << "x+y      = " << x+y      << endl;
    cout << "x-y      = " << x-y      << endl;
    cout << "x*y      = " << x*y      << endl;
    cout << "x/y      = " << x/y      << endl;
    cout << "cos(x)   = " << cos(x)   << endl;
    cout << "sin(x)   = " << sin(x)   << endl;
    cout << "tan(x)   = " << tan(x)   << endl;
    cout << "exp(x)   = " << exp(x)   << endl;
    cout << "ln(x)    = " << ln(x)    << endl;
    cout << "sqrt(x)  = " << sqrt(x)  << endl;
    cout << "pow(x,y) = " << pow(x,y) << endl;
    assert(c>=T(-1) && c<=T(1));
    cout << "c          = " << c        << endl;
    cout << "acos(c)    = " << acos(c)  << endl;
    cout << "asin(c)    = " << asin(c)  << endl;
    cout << "atan(x)    = " << atan(x)  << endl;
    cout << "atan2(y,x) = " << atan2(y,x) << endl;
}

static void testQ() {
    static const double a(1.25), b(2.25), clamped(0.5);
    cout.precision(64);
#define WRAP_TEST_REAL_NUMBER(...) \
    cout << endl << "Testing " << #__VA_ARGS__ << " :" << endl; \
    testRealNumbers<__VA_ARGS__>(__VA_ARGS__(a),__VA_ARGS__(b),__VA_ARGS__(clamped))
    WRAP_TEST_REAL_NUMBER(double);
    WRAP_TEST_REAL_NUMBER(q<16,16>);
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

Test1::Test1() : 
    m_shouldQuit(false), tick(0), wasPreparedToRender(false), 
    font(NULL), guiTex(NULL), mouse({}), keyboard({}), simstate({}), 
    aabbs({}), disks({})
{
    testQ();
    //exit(0);
    hope(font = TTF_OpenFont("res/basis33/basis33.ttf", 16));
    aabbs.push_back(aabb_2d(vec2(50,50), vec2(10,10)));
    aabbs.push_back(aabb_2d(vec2(300,300), vec2(163,100)));
    disks.push_back(disk_2d(vec2(80,80), 200));
}
Test1::~Test1() {
    SDL_DestroyTexture(guiTex);
    TTF_CloseFont(font);
}
bool Test1::shouldQuit() const { return m_shouldQuit; }
void Test1::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: case SDL_APP_TERMINATING: m_shouldQuit = true; break;
    case SDL_USEREVENT:
        if((uint32_t)(uintptr_t)e->user.data1 & updateFixedStepSimulationBit)
            updateFixedStepSimulation(); 
        break;
#define HANDLE_KEY_EVT(is_down) \
        switch(e->key.keysym.sym) { \
        case SDLK_RIGHT: keyboard.right = is_down; break; \
        case SDLK_UP:    keyboard.up    = is_down; break; \
        case SDLK_LEFT:  keyboard.left  = is_down; break; \
        case SDLK_DOWN:  keyboard.down  = is_down; break; \
        }
    case SDL_KEYDOWN: HANDLE_KEY_EVT(true);  break;
    case SDL_KEYUP:   HANDLE_KEY_EVT(false); break;
#undef HANDLE_KEY_EVT
    case SDL_MOUSEBUTTONDOWN: mouse.down = true;  break;
    case SDL_MOUSEBUTTONUP:   mouse.down = false; break;
    case SDL_MOUSEWHEEL:      mouse.wheel.y += e->wheel.y; break;
    }
}

void Test1::updateFixedStepSimulation() {
    ++tick;
    static const int speedmul = 5;
    // TODO velocity
    if(keyboard.right && !keyboard.left)
        aabbs[0].center.x += speedmul,
        disks[0].center.x += speedmul;
    if(keyboard.left && !keyboard.right)
        aabbs[0].center.x -= speedmul,
        disks[0].center.x -= speedmul;
    if(keyboard.up && !keyboard.down)
        aabbs[0].center.y -= speedmul,
        disks[0].center.y -= speedmul;
    if(keyboard.down && !keyboard.up)
        aabbs[0].center.y += speedmul,
        disks[0].center.y += speedmul;
    simstate.intersects = aabbs[0].intersects(aabbs[1]);
    simstate.aabb_disk_intersects = disks[0].intersects(aabbs[1]);
    mouse.wheel.y = 0;
}

static void rgba_px_from_bool8(
                rgba32 *rgba, 
                const uint8_t *pixels, 
                size_t w, size_t h, size_t row_size, 
                uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    for(size_t y=0 ; y<h ; ++y) {
        for(size_t x=0 ; x<w ; ++x) {
             if(!pixels[y*row_size + x]) {
                 memset(&rgba[y*w + x], 0, sizeof(rgba32));
                 continue;
             }
            rgba[y*w + x].r = r;
            rgba[y*w + x].g = g;
            rgba[y*w + x].b = b;
            rgba[y*w + x].a = a;
        }
    }
}

void Test1::renderSDL2_GUI(SDL_Renderer *rdr) const {
    SDL_Color color = {0, 128, 0, 0};
    ostringstream oss;
    oss << "Tick: " << tick << " (Red-Blue intersection: " << (simstate.intersects ? "yes" : "no") << ")";
    SDL_Surface *s = TTF_RenderUTF8_Solid(
        font, oss.str().c_str(), color
    );
    hope(s);
    rgba32 *rgba = new rgba32[s->w*s->h];
    rgba_px_from_bool8(rgba, (uint8_t*)s->pixels, s->w, s->h, s->pitch, 255, 200, 10, 255);
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
    assert(aabbs.size() == 2);
    SDL_SetRenderDrawColor(rdr, 128*simstate.intersects, 0, 255, 255);
    aabbs[1].renderSDL2(rdr);
    SDL_SetRenderDrawColor(rdr, 255, 0, 128*simstate.intersects, 255);
    aabbs[0].renderSDL2(rdr);
    SDL_SetRenderDrawColor(rdr, 255, 128, 0, 255);
    aabbs[0].renderSDL2Wireframe(rdr);
    SDL_SetRenderDrawColor(rdr, 255, 0, 255*simstate.aabb_disk_intersects, 255);
    disks[0].renderSDL2Wireframe(rdr);
    renderSDL2_GUI(rdr);
}

} // namespace Test1
