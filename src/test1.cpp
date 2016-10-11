#include <Test1.hpp>

Test1::Test1() {}
Test1::~Test1() {}
bool Test1::shouldQuit() const { return m_shouldQuit; }
void Test1::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT: m_shouldQuit = true; break;
    }
}
void Test1::renderToSDL2Renderer(SDL_Renderer *rdr) const {}
