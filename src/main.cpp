#include <SDL2/SDL.h>
#include <Test1.hpp>

int main(int argc, char *argv[]) {

    SDL_Init(SDL_INIT_VIDEO);
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

