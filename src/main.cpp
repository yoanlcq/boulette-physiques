#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <Test1.hpp>
#include <TestVerlet.hpp>

static uint32_t timer_callback(uint32_t interval, void *param) {
    SDL_Event e;
    e.type       = SDL_USEREVENT;
    e.user.type  = SDL_USEREVENT;
    e.user.code  = 0;
    e.user.data1 = param;
    e.user.data2 = NULL;
    SDL_PushEvent(&e);
    return interval;
}

uint32_t g_update_dt_ms = 0;

int main(int argc, char *argv[]) {

    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        std::cerr << "SDL_Init(): " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }
    if(TTF_Init() < 0) {
        std::cerr << "TTF_Init(): " << TTF_GetError() << std::endl;
        return EXIT_FAILURE;
    }
    size_t win_w=600, win_h=400;
    SDL_Window *win = SDL_CreateWindow(
        "test1", 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        win_w, win_h, 0
    );
    SDL_Renderer *rdr = SDL_CreateRenderer(win, -1, 
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

#define TEST TestVerlet
    TEST::TEST test1(TEST::unitv2(win_w, win_h));
    g_update_dt_ms = 50;
    SDL_AddTimer(g_update_dt_ms, timer_callback, (void*)TEST::updateFixedStepSimulationBit);

    test1.prepareRenderSDL2(rdr);
    do {
        SDL_Event e;
        while(SDL_PollEvent(&e))
            test1.handleSDL2Event(&e);
        SDL_SetRenderDrawColor(rdr, 0, 0, 0, 255);
        SDL_RenderClear(rdr);
        test1.renderSDL2(rdr);
        SDL_RenderPresent(rdr);
    } while(!test1.shouldQuit());

    SDL_DestroyRenderer(rdr);
    SDL_DestroyWindow(win);
    SDL_Quit();

    return EXIT_SUCCESS;
}

