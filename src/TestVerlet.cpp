#include <TestVerlet.hpp>
#include <boulette.hpp>
#include <stdcxx.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

using namespace std;
using namespace boulette;

double ln(double x) { return log(x);  }
float  ln(float  x) { return logf(x); }

template<typename T> static void testRealNumbers(T x, T y, T c) {
    cout << "x        = " <<   x      << endl;
    cout << "y        = " <<   y      << endl;
    cout << "x+y      = " << x+y      << endl;
    cout << "x-y      = " << x-y      << endl;
    cout << "x*y      = " << x*y      << endl;
    cout << "x/y      = " << x/y      << endl;
    cout << "cos(x)   = " << cos(x)   << endl;
    cout << "sin(x)   = " << sin(x)   << endl;
    cout << "tan(x)   = " << tan(x)   << endl;
    cout << "exp(x)   = " << exp(x)   << endl;
    cout << "ln(x)    = " << ln(x)    << endl;
    cout << "sqrt(x)  = " << sqrt(x)  << endl;
    cout << "pow(x,y) = " << pow(x,y) << endl;
    assert(c>=T(-1) && c<=T(1));
    cout << "c          = " << c        << endl;
    cout << "acos(c)    = " << acos(c)  << endl;
    cout << "asin(c)    = " << asin(c)  << endl;
    cout << "atan(x)    = " << atan(x)  << endl;
    cout << "atan2(y,x) = " << atan2(y,x) << endl;
    cout << "min value  = " << std::numeric_limits<T>::min() << endl;
    cout << "max value  = " << std::numeric_limits<T>::max() << endl;
}

static void testQ() {
    static const double a(1.25), b(2.25), clamped(0.5);
    cout.precision(64);
// Doing it via a macro because of the automatic "stringify" operator.
// Using __VA_ARGS__ because of the comma in template arguments.
#define WRAP_TEST_REAL_NUMBER(...) \
    cout << endl << "Testing " << #__VA_ARGS__ << " :" << endl; \
    testRealNumbers<__VA_ARGS__>(__VA_ARGS__(a),__VA_ARGS__(b),__VA_ARGS__(clamped))
    WRAP_TEST_REAL_NUMBER(double);
    WRAP_TEST_REAL_NUMBER(q<16,16>);
    WRAP_TEST_REAL_NUMBER(q<32,0>);
    WRAP_TEST_REAL_NUMBER(q<28,4>);
    WRAP_TEST_REAL_NUMBER(q<26,6>);
    WRAP_TEST_REAL_NUMBER(q<6,26>);
    WRAP_TEST_REAL_NUMBER(q<4,28>);
#undef WRAP_TEST_REAL_NUMBER
    cout << endl << "-- DONE TESTING FIXED-POINT NUMBERS --" << endl << endl;
}




namespace TestVerlet {

const uintptr_t updateFixedStepSimulationBit(0x800);


void Input::handleSDL2Event(const SDL_Event *e) {
#define HANDLE_KEY_EVT(is_down) \
        switch(e->key.keysym.sym) { \
        case SDLK_RIGHT:    held.right = is_down; if(is_down) clicked.right = true; break; \
        case SDLK_UP:       held.up    = is_down; if(is_down) clicked.up    = true; break; \
        case SDLK_LEFT:     held.left  = is_down; if(is_down) clicked.left  = true; break; \
        case SDLK_DOWN:     held.down  = is_down; if(is_down) clicked.down  = true; break; \
        case SDLK_PLUS: \
        case SDLK_KP_PLUS:  held.plus  = is_down; if(is_down) clicked.plus  = true; break; \
        case SDLK_MINUS: \
        case SDLK_KP_MINUS: held.minus = is_down; if(is_down) clicked.minus = true; break; \
        case SDLK_n:        held.n     = is_down; if(is_down) clicked.n     = true; break; \
        case SDLK_s:        held.s     = is_down; if(is_down) clicked.s     = true; break; \
        case SDLK_d:        held.d     = is_down; if(is_down) clicked.d     = true; break; \
        case SDLK_h:        held.h     = is_down; if(is_down) clicked.h     = true; break; \
        case SDLK_r:        held.r     = is_down; if(is_down) clicked.r     = true; break; \
        case SDLK_f:        held.f     = is_down; if(is_down) clicked.f     = true; break; \
        case SDLK_a:        held.a     = is_down; if(is_down) clicked.a     = true; break; \
        }
#define HANDLE_MBT_EVT(is_down) \
        switch(e->button.button) { \
        case SDL_BUTTON_LEFT:   held.mouseleft   = is_down; if(is_down) clicked.mouseleft   = true; break; \
        case SDL_BUTTON_MIDDLE: held.mousemiddle = is_down; if(is_down) clicked.mousemiddle = true; break; \
        case SDL_BUTTON_RIGHT:  held.mouseright  = is_down; if(is_down) clicked.mouseright  = true; break; \
        }
    switch(e->type) {
    case SDL_KEYDOWN:         HANDLE_KEY_EVT(true);      break;
    case SDL_KEYUP:           HANDLE_KEY_EVT(false);     break;
    case SDL_MOUSEBUTTONDOWN: HANDLE_MBT_EVT(true);      break;
    case SDL_MOUSEBUTTONUP:   HANDLE_MBT_EVT(false);     break;
    case SDL_MOUSEWHEEL:      mousescroll += e->wheel.y; break;
    case SDL_MOUSEMOTION:
        mouseprevpos = mousepos;
        mousepos = vec2i(e->motion.x, e->motion.y);
        break;
    }
#undef HANDLE_KEY_EVT
#undef HANDLE_MBT_EVT
}


TestVerlet::TestVerlet(SDL_Renderer *rdr, unitv2 screen_size, uint32_t update_dt_ms) :  
    should_quit(false), does_display_extras(false), does_display_aabbs(false),
    is_grabbing_a_vertex(false), grabbed_vertex_index(0),
    update_dt_ms(update_dt_ms), tick(0),
    input({}), creation_tool(),
    text_gui(rdr, screen_size.x, screen_size.y),
    verletSys(screen_size, 1),
    rdr(rdr)
{

    testQ();

    vector<VerletSys::rgba32> cols;
    cols.push_back(VerletSys::rgba32(255, 000, 000, 255));
    cols.push_back(VerletSys::rgba32(255, 255, 000, 255));
    cols.push_back(VerletSys::rgba32(000, 255, 000, 255));
    cols.push_back(VerletSys::rgba32(000, 000, 255, 255));
    unitv2 centers[4];
    for(uint_fast32_t y=0 ; y<4 ; ++y) {
        for(uint_fast32_t x=0 ; x<4 ; ++x)
            centers[x] = unitv2(60+x*130+y*20, 60+y*70);
        VerletSys::BodyDescriptor body;
        if(y<2)
            body = VerletSys::describeRigidDisk(30, 4+y, cols);
        else
            body = VerletSys::describeSlimyDisk(30+y*5, 22+y, cols);
        verletSys.addRigidBodies(body, 4, centers);
    }
}

TestVerlet::~TestVerlet() {}

bool TestVerlet::shouldQuit() const { return should_quit; }

static uint32_t last_update_time_ms = 0;

void TestVerlet::handleSDL2Event(const SDL_Event *e) {
    switch(e->type) {
    case SDL_QUIT:
    case SDL_APP_TERMINATING: 
        should_quit = true; 
        break;
    case SDL_USEREVENT:
        if((uint32_t)(uintptr_t)e->user.data1 & updateFixedStepSimulationBit) {
            updateFixedStepSimulation(); 
            last_update_time_ms = SDL_GetTicks();
        }
        break;
    case SDL_WINDOWEVENT: 
        switch(e->window.event) {
        case SDL_WINDOWEVENT_RESIZED:
        case SDL_WINDOWEVENT_SIZE_CHANGED: 
            {
                int w(e->window.data1), h(e->window.data2);
                verletSys.reshape(unitv2(w, h));
                text_gui.reshape(rdr, w, h);
            }
            break;
        }
        break;
    default: input.handleSDL2Event(e); break;
    }
}


void TestVerlet::updateFixedStepSimulation() {
    ++tick;

    if(input.clicked.r)
        verletSys.reset();

    if(input.held.left)
        verletSys.timestep = max(real(0), verletSys.timestep - real(.01));
    if(input.held.right) 
        verletSys.timestep += real(.01);

    if(input.held.plus)
        creation_tool.vertex_count += 1;
    if(input.held.minus && creation_tool.vertex_count >= 3)
        creation_tool.vertex_count -= 1;

    if(input.held.up)
        creation_tool.radius += 1;
    if(input.held.down && creation_tool.radius >= 2)
        creation_tool.radius -= 1;

    if(input.clicked.h)
        does_display_extras = !does_display_extras;
    if(input.clicked.a)
        does_display_aabbs = !does_display_aabbs;


    if(input.clicked.f)
        verletSys.enable_experimental_friction = !verletSys.enable_experimental_friction;

    unitv2 mouseworldpos(input.mousepos.x, input.mousepos.y);

    vector<VerletSys::rgba32> colors;
    colors.push_back(VerletSys::rgba32(255, 255, 255, 255));

    if(input.clicked.n)
        verletSys.addRigidBodies(VerletSys::describeNecklace(creation_tool.radius, creation_tool.vertex_count, colors), 1, &mouseworldpos);
    if(input.clicked.s)
        verletSys.addRigidBodies(VerletSys::describeSlimyDisk(creation_tool.radius, creation_tool.vertex_count, colors), 1, &mouseworldpos);
    if(input.clicked.d)
        verletSys.addRigidBodies(VerletSys::describeRigidDisk(creation_tool.radius, creation_tool.vertex_count, colors), 1, &mouseworldpos);

    if(input.clicked.mouseleft) {
        if(!is_grabbing_a_vertex)
            grabbed_vertex_index = verletSys.pickClosestScreenSpaceVertex(mouseworldpos);
        is_grabbing_a_vertex = !is_grabbing_a_vertex;
    }

    verletSys.update();

    if(is_grabbing_a_vertex) {
        if(grabbed_vertex_index >= verletSys.vcount)
            is_grabbing_a_vertex = false;
        else
            verletSys.vpos[grabbed_vertex_index] = mouseworldpos;
    }


    stringstream ss;
    ss 
        << "Welcome to Boulette Physiques !" << endl
        << "(Press H to toggle stats, commands, etc)";
    if(does_display_extras) {
    ss  << endl
        << "Mouse : " << input.mousepos
        << " | Timestep : " << verletSys.timestep
        << " | Gravity : "  << verletSys.gravity
        << endl
        << "Experimental friction : " 
        << (verletSys.enable_experimental_friction ? "on" : "off")
        << " (coeff: " << verletSys.friction_coefficient <<  ")"
        << endl
        << verletSys.bcount << " bodies"
        << " | " << verletSys.vcount << " vertices"
        << " | " << verletSys.ecount << " edges"
        << " ("  << (verletSys.ecount-verletSys.e_occluded_start)
        << " ("  << (uint32_t)(100.f*(verletSys.ecount-verletSys.e_occluded_start)/(float)verletSys.ecount) << "%)"
        << " occluded)" << endl
        << endl
        << "--- NEXT BODY INSTANCE'S SETTINGS ---" << endl
        << "Vertex count : " << creation_tool.vertex_count 
        << " | Radius : " << creation_tool.radius << endl
        << endl
        << "--- COMMANDS ---" << endl
        << "Left click : drag/drop the closest vertex" << endl
        << "Left and Right arrow keys : Change timestep" << endl
        << "Up and Down arrow keys : Change new body instance's radius" << endl
        << "+ and - keys : Change new body instance's vertex count" << endl
        << "N : Instanciate Necklace (pretty!)" << endl
        << "S : Instanciate Slimy Disk (funny!)" << endl
        << "D : Instanciate Rigid Disk (expensive!)" << endl
        << "F : Toggle experimental friction" << endl
        << "A : Toggle AABB display" << endl
        << "R : Remove all bodies mercilessly" << endl
    ;
    }
    text_gui.text = ss.str();
    text_gui.update();

    input.clear_clicked();
}

void TestVerlet::renderSDL2(SDL_Renderer *rdr) const {
    SDL_SetRenderDrawColor(rdr, 255, 0, 0, 255);
    uint32_t time_ms = SDL_GetTicks();
    uint32_t frame_dt = time_ms - last_update_time_ms;
    float interp = frame_dt/float(update_dt_ms);
    //cout << interp << endl;
    if(does_display_aabbs)
        verletSys.renderSDL2WithAabbs(rdr, interp);
    else
        verletSys.renderSDL2(rdr, interp);
    text_gui.renderSDL2(rdr);
}

} // namespace TestVerlet
