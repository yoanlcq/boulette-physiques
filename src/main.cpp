#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
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

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

// Si on se trouve dans le répertoire de l'exécutable,
// on ne trouvera pas les ressources - il faut se mettre
// dans son dossier parent.
// Cette solution n'est pas très propre, mais c'est peu
// probable qu'elle pose des problèmes.
static void fix_cwd(const char *argv0) {
#ifdef _WIN32
    const char *argv0_last = strrchr(argv0, '\\');
#else
    const char *argv0_last = strrchr(argv0, '/');
#endif
    argv0 = argv0_last ? argv0_last+1 : argv0;
    bool success = true;
#ifdef _WIN32
    if(GetFileAttributesA(argv0) != INVALID_FILE_ATTRIBUTES)
        success = SetCurrentDirectoryA("..");
#else
    if(!access(argv0, F_OK))
        success = !chdir("..");
#endif
    assert(success);
}

#ifdef _WIN32
int CALLBACK WinMain(
  _In_ HINSTANCE hInstance,
  _In_ HINSTANCE hPrevInstance,
  _In_ LPSTR     lpCmdLine,
  _In_ int       nCmdShow
)
#else
int main(int argc, char *argv[]) 
#endif
{
#ifdef _WIN32
    fix_cwd(__argv[0]);
#else
    fix_cwd(argv[0]);
#endif

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
        "Boulette :: Verlet Integration", 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        win_w, win_h, SDL_WINDOW_RESIZABLE
    );
    SDL_Renderer *rdr = SDL_CreateRenderer(win, -1, 
        SDL_RENDERER_PRESENTVSYNC
    );
    SDL_SetRenderDrawBlendMode(rdr, SDL_BLENDMODE_BLEND);

    uint32_t update_dt_ms = 50;
#define TEST TestVerlet
    TEST::TEST test1(rdr, TEST::unitv2(win_w, win_h), update_dt_ms);
    SDL_AddTimer(update_dt_ms, timer_callback, (void*)TEST::updateFixedStepSimulationBit);

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

