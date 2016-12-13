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

Test1::Test1(SDL_Renderer *rdr, unitv2 screen_size) : 
    m_shouldQuit(false), tick(0),  
    text_gui(rdr, screen_size.x, screen_size.y), mouse({}), keyboard({}), simstate({}), 
    disk(vec2(80,80),20), vel(0,0), accel(0,0)
{
    testQ();
    text_gui.text = "Foo !!\nBar !!";
    text_gui.update();
    //exit(0);
}
Test1::~Test1() {}
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
    accel.x = accel.y = 0;
    if(keyboard.right && !keyboard.left)
        accel.x = speedmul;
    if(keyboard.left && !keyboard.right)
        accel.x = -speedmul;
    if(keyboard.up && !keyboard.down)
        accel.y = -speedmul;
    if(keyboard.down && !keyboard.up)
        accel.y = speedmul;
    mouse.wheel.y = 0;
}


void Test1::renderSDL2(SDL_Renderer *rdr) const {
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    //renderworld.disk.pos = lerp(oldpos, newpos, (td-tf1)/float(tf2-tf1));
    disk.renderSDL2Wireframe(rdr);
    text_gui.renderSDL2(rdr);
}

} // namespace Test1
