#ifndef TEST1_HPP
#define TEST1_HPP

#include <SDL2/SDL.h>

class Test1 {
    bool m_shouldQuit;
public:
    Test1();
    ~Test1();
    bool shouldQuit() const;
    void handleSDL2Event(const SDL_Event *e);
    void renderToSDL2Renderer(SDL_Renderer *rdr) const;
};

#endif//TEST1_HPP
