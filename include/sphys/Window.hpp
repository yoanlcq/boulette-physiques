#include <sphys/sphys.hpp>

namespace sphys {

class Window {
private:
    SDL_Window *win;
    SDL_Renderer *rdr;
public:
    explicit Window(const std::string &title, const u16_aabb &box);
};

}
