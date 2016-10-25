#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <Test1.hpp>

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

int main(int argc, char *argv[]) {

    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        std::cerr << "SDL_Init(): " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }
    if(TTF_Init() < 0) {
        std::cerr << "TTF_Init(): " << TTF_GetError() << std::endl;
        return EXIT_FAILURE;
    }
    SDL_Window *win = SDL_CreateWindow(
        "test1", 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        800, 600, 0
    );
    SDL_Renderer *rdr = SDL_CreateRenderer(win, -1, 
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );


    Test1::Test1 test1;
    uint32_t tick_delay = 50;
    SDL_AddTimer(tick_delay, timer_callback, (void*)Test1::updateFixedStepSimulationBit);

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

