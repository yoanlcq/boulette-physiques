#ifndef TESTVERLET_HPP
#define TESTVERLET_HPP

#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <TextGui.hpp>

namespace TestVerlet {

extern const uintptr_t updateFixedStepSimulationBit;

typedef boulette::vec2<int> vec2i;
typedef boulette::q<24,8> q;



// You can change these two typedef below if you want (this is pretty much the point 
// of boulette's heavy use of templates). 
//
// Here are the constraints :
// - 'unit' reads as "spatial unit". It can be any integer or real type,
//   though integers are not guaranteed to work.
//   Full support for integral spatial units COULD be provided (replacing fixed-point's 
//   determinism and consistency), but it'd require extra care I didn't put time into.
// - 'real' is pretty self-explanatory. Can be any fixed-point or floating-point type.
typedef q unit;
typedef q real;


typedef boulette::vec2<unit> unitv2;
typedef boulette::VerletPhysicsSystem<unit,real> VerletSys;

// held : "is the key currently held down ?"
// clicked : "was the key pressed once ?"
struct InputKey { bool held, clicked; };

struct Input {
    vec2i mousepos, mouseprevpos;
    int mousescroll;
    struct { 
        bool mouseleft, mouseright, mousemiddle;
        bool right, up, left, down, plus, minus, n, s, d, h, r, f, a;
    } held, clicked;

    // Pretty much chose this layout so this method scales well
    // with the number of supported keys.
    // Otherwise I agree that 'input.left.clicked' would look better than 'input.clicked.left'.
    void clear_clicked() { 
        memset(&clicked, 0, sizeof clicked); 
        mousescroll = 0;
    }
    void handleSDL2Event(const SDL_Event *e);
};

struct CreationTool {
    size_t vertex_count, radius;
    CreationTool() : vertex_count(2), radius(42) {}
    ~CreationTool() {}
};

class TestVerlet {
    bool should_quit;
    bool does_display_extras;
    bool does_display_aabbs;
    bool is_grabbing_a_vertex;
    size_t grabbed_vertex_index;
    uint32_t update_dt_ms;
    uint64_t tick;
    Input input;
    CreationTool creation_tool;
    TextGui text_gui;
    VerletSys verletSys;
    SDL_Renderer *rdr; // Wanted to avoid having this as a member, but 
                       // necessary for resizing the TextGui's texture.
                       // Please forget that it's here .__. Use renderSDL2()
                       // as usual.
public:
     TestVerlet(SDL_Renderer *rdr, unitv2 screen_size, uint32_t update_dt_ms);
    ~TestVerlet();
    bool shouldQuit() const;
    void updateFixedStepSimulation();
    void handleSDL2Event(const SDL_Event *e);
    void renderSDL2(SDL_Renderer *rdr) const;
};

} // namespace TestVerlet

#endif//TESTVERLET_HPP
