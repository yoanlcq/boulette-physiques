#include <sphys/sphys.hpp>

namespace sphys {

Window::Window(const std::string &title, const u16_aabb &box) {
    SDL_Init(SDL_INIT_VIDEO);
    win = SDL_CreateWindow(title.c_str(), box.pos.x, box.pos.y, 
            box.size.x, box.size.y, 0);
    rdr = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
}

Window::~Window() {
    SDL_DestroyRenderer(rdr);
    SDL_DestroyWindow(win);
}

}
