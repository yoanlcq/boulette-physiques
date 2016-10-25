#include <Test1.hpp>
#include <sphys.hpp>
#include <stdcxx.hpp>

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

Test1::Test1() {
    testQ();
    typedef q<16,16> qtype;
    typedef aabb_2d<qtype> qaabb_2d;
    typedef vec2<qtype> qvec2;
    aabbs.push_back(qaabb_2d(qvec2(50,50), qvec2(100,100)));
}
Test1::~Test1() {}
bool Test1::shouldQuit() const { return m_shouldQuit; }
void Test1::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: m_shouldQuit = true; break;
    }
}
void Test1::renderSDL2(SDL_Renderer *rdr) const {
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    aabbs[0].renderSDL2(rdr);
}
