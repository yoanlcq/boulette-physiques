#pragma once

#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

class TextGui {
    TTF_Font *font;
    SDL_Texture *tex;
    size_t fontsize;
public:
    TextGui(
        SDL_Renderer *rdr, 
        size_t screen_w, size_t screen_h, 
        std::string fontpath = "res/basis33/basis33.ttf", 
        size_t fontsize = 16,
        SDL_Color fg_color = {220, 220, 220, 255},
        SDL_Color bg_color = { 40,  40,  40, 255}
    );
    ~TextGui();
    void renderSDL2(SDL_Renderer *rdr) const;
    SDL_Color fg_color, bg_color;
    std::string text;
    void update();
};

