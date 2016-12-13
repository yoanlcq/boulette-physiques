#include <TextGui.hpp>

using namespace std;

struct rgba32 {
    uint32_t r:8;
    uint32_t g:8;
    uint32_t b:8;
    uint32_t a:8;
};

#define hope assert

TextGui::TextGui(SDL_Renderer *rdr, size_t screen_w, size_t screen_h, string fontpath, size_t fontsize, SDL_Color fg_color, SDL_Color bg_color) 
    :
    font(TTF_OpenFont(fontpath.c_str(), fontsize)),
    tex(SDL_CreateTexture(rdr,
        SDL_PIXELFORMAT_ABGR8888, 
        SDL_TEXTUREACCESS_STREAMING, 
        screen_w, screen_h
    )),
    fontsize(fontsize),
    fg_color(fg_color), 
    bg_color(bg_color), 
    text("")
{
    hope(font);
    hope(tex);
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
}
TextGui::~TextGui() {
    SDL_DestroyTexture(tex);
    TTF_CloseFont(font);
}

typedef uint8_t bool8;

static void rgba_px_from_bool8(
                rgba32 *rgba, 
                const bool8 *pixels, 
                int dst_x_off, int dst_y_off,
                size_t dst_w, size_t dst_h,
                size_t src_w, size_t src_h, size_t src_row_size,
                const SDL_Color &color)
{
    for(size_t y=0 ; y<src_h ; ++y) {
        for(size_t x=0 ; x<src_w ; ++x) {
            if(y+dst_y_off < 0 || y+dst_y_off >= dst_h
            || x+dst_x_off < 0 || x+dst_x_off >= dst_w
            || !pixels[y*src_row_size + x])
                continue;
            rgba[(y+dst_y_off)*dst_w + x+dst_x_off].r = color.r;
            rgba[(y+dst_y_off)*dst_w + x+dst_x_off].g = color.g;
            rgba[(y+dst_y_off)*dst_w + x+dst_x_off].b = color.b;
            rgba[(y+dst_y_off)*dst_w + x+dst_x_off].a = color.a;
        }
    }
}

void TextGui::renderSDL2(SDL_Renderer *rdr) const {
    SDL_RenderCopy(rdr, tex, NULL, NULL);
}

void TextGui::update() {
    uint32_t format;
    int access, w, h;
    SDL_QueryTexture(tex, &format, &access, &w, &h);
    rgba32 *rgba = new rgba32[w*h]();

    stringstream ss;
    ss.str(text);
    string line;

    size_t y = 0;
    while(getline(ss, line)) {
        SDL_Surface *s = TTF_RenderUTF8_Solid(font, line.c_str(), fg_color);
        rgba_px_from_bool8(rgba, (bool8*)s->pixels, 1, 1+y, w, h,
                           s->w, s->h, s->pitch, bg_color);
        rgba_px_from_bool8(rgba, (bool8*)s->pixels, 0, 0+y, w, h,
                           s->w, s->h, s->pitch, fg_color); 
        y += fontsize;
        SDL_FreeSurface(s);
    }
    // XXX Use SDL_LockTexture() ???
    SDL_UpdateTexture(tex, NULL, rgba, w*sizeof(rgba32));
    delete[] rgba;
}


