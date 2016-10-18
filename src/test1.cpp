#include <Test1.hpp>
#include <sphys.hpp>
#include <stdcxx.hpp>

using namespace std;
using namespace sphys;

static void test_q() {
    q<8,24> a(1.25), b(2.25);
    double   x(1.25), y(2.25);
    cout << "a  : " <<   a << "(expected " <<   x << ")" << endl;
    cout << "b  : " <<   b << "(expected " <<   y << ")" << endl;
    cout << "a+b: " << a+b << "(expected " << x+y << ")" << endl;
    cout << "a-b: " << a-b << "(expected " << x-y << ")" << endl;
    cout << "a*b: " << a*b << "(expected " << x*y << ")" << endl;
    cout << "a/b: " << a/b << "(expected " << x/y << ")" << endl;
}

Test1::Test1() {
    test_q();
}
Test1::~Test1() {}
bool Test1::shouldQuit() const { return m_shouldQuit; }
void Test1::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: m_shouldQuit = true; break;
    }
}
void Test1::renderToSDL2Renderer(SDL_Renderer *rdr) const {}
