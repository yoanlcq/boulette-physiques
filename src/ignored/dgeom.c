#include <SDL2/SDL.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

typedef int64_t dg_spacial_unit, dg_p;
typedef int32_t dg_spacial_scale, dg_s; /* Used to bscale() things. */
/* Q number to store tan() values. */
typedef int32_t dg_q16_16;
typedef dg_q16_16 dg_qtan;
/* One must avoid having to compute the atan2() of values that take more
 * than the number of bits of a #dg_qtan.
 * This is similar to how #dg_spacial_scale limits the size of physics 
 * simulations. */
typedef uint64_t dg_temporal_unit, dg_tick;
typedef uint16_t dg_angular_unit, dg_angle;
#define DG_ANGLE_PI ((~(dg_angle)0)/2)

typedef struct { dg_p x, y; } dg_pvec2;
typedef struct { dg_s x, y; } dg_svec2;
typedef struct { uint8_t r, g, b, a; } dg_rgba32;

typedef struct {
    dg_rgba32 color;
    dg_pvec2  center;
    dg_svec2  half_size;
    dg_angle  orientation;
    dg_pvec2  vertices[4]; /*!< 
    * Generated from the above members : see #dg_rect_recompute_vertices(). */
} dg_rect;

void dg_rect_render_sdl2(const dg_rect *rect, SDL_Renderer *rd) {
    const dg_pvec2 *v = rect->vertices;
    const dg_rgba32 *c = &rect->color;
    SDL_SetRenderDrawColor(rd, c->r, c->g, c->b, c->a);
    SDL_RenderDrawLine(rd, v[0].x, v[0].y, v[1].x, v[1].y);
    SDL_SetRenderDrawColor(rd, c->r/2, c->g/2, c->b/2, c->a);
    SDL_RenderDrawLine(rd, v[1].x, v[1].y, v[2].x, v[2].y);
    SDL_SetRenderDrawColor(rd, c->r/4, c->g/4, c->b/4, c->a);
    SDL_RenderDrawLine(rd, v[2].x, v[2].y, v[3].x, v[3].y);
    SDL_SetRenderDrawColor(rd, c->r/8, c->g/8, c->b/8, c->a);
    SDL_RenderDrawLine(rd, v[3].x, v[3].y, v[0].x, v[0].y);
}

#include <math.h>
/* v is assumed to be a direction vector (that is, starts from the origin). 
 * Right now, angle is in degrees. */
void dg_pvec2_rotate_using_doubles(dg_pvec2 *r, const dg_pvec2 *v, dg_angle angle) {
    double angled = atan2(v->y, v->x);
    angled += (angle*M_PI*2.)/360.;
    double len = sqrt(v->x*v->x + v->y*v->y);
    r->x = len*cos(angled);
    r->y = len*sin(angled);
}
dg_p dg_p_sqrt(dg_p x) {return 0;}
dg_p dg_p_bscale(dg_p x, dg_s s) {
    return (((int64_t)x)*((int64_t)s))>>31;
}
dg_s dg_angle_cos(dg_angle angle) {return 0;}
dg_s dg_angle_sin(dg_angle angle) {return 0;}
dg_qtan dg_angle_tan(dg_angle angle) {return 0;}
dg_qtan dg_qtan_div(dg_qtan a, dg_qtan b) {return 0;}
dg_angle dg_qtan_atan(dg_qtan x) {return 0;}
#define warn_if(cond)
dg_angle dg_p_atan2(dg_p y, dg_p x) {
    warn_if(x & ~(dg_p)~(dg_qtan)0 
         || y & ~(dg_p)~(dg_qtan)0
         && "Whoops, y, x or both use more bits than dg_qtan can take!\n"
            "(Your physics simulation probably needs too many units).\n"
            "You should take care that it doesn't happen.");
    return dg_qtan_atan(dg_qtan_div(y, x));
}
void dg_pvec2_rotate_stable(dg_pvec2 *r, const dg_pvec2 *v, dg_angle angle) {
    angle += dg_p_atan2(v->y, v->x);
    dg_p len = dg_p_sqrt(v->x*v->x + v->y*v->y);
    r->x = dg_p_bscale(len, dg_angle_cos(angle));
    r->y = dg_p_bscale(len, dg_angle_sin(angle));
}
#define dg_pvec2_rotate dg_pvec2_rotate_using_doubles

void dg_rect_recompute_vertices(dg_rect *r) {
    dg_pvec2 *v = r->vertices;
#define COMPUTE_VERTICE(i,xsign,ysign) \
    /* First, get direction vector */ \
    v[i].x = xsign r->half_size.x; \
    v[i].y = ysign r->half_size.y; \
    /* Rotate the direction vector by r->orientation */ \
    dg_pvec2_rotate(v+i, v+i, r->orientation); \
    /* Turn it into a position */ \
    v[i].x += r->center.x; \
    v[i].y += r->center.y
    COMPUTE_VERTICE(0,-,-);
    COMPUTE_VERTICE(1,+,-);
    COMPUTE_VERTICE(2,+,+);
    COMPUTE_VERTICE(3,-,+);
#undef COMPUTE_VERTICE
}

typedef struct {
    bool quit;
    dg_svec2 wsize;
    SDL_Window *win;
    SDL_Renderer *rdr;
    size_t rect_count;
    dg_rect rects[2];
} dg_sim;

void dg_sim_init(dg_sim *sim) {
    sim->quit = false;
    sim->wsize.x = 320;
    sim->wsize.y = 240;
    sim->win = SDL_CreateWindow("Stable simulation", 
        0, 0, sim->wsize.x, sim->wsize.y, 0
    );
    sim->rdr = SDL_CreateRenderer(sim->win, -1, SDL_RENDERER_PRESENTVSYNC);
    sim->rect_count = 2;
    sim->rects[0].center.x    = sim->wsize.x/2;
    sim->rects[0].center.y    = sim->wsize.y/2;
    sim->rects[0].half_size.x = 30;
    sim->rects[0].half_size.y = 10;
    sim->rects[0].orientation = 0;
    sim->rects[0].color.r     = 255;
    sim->rects[0].color.g     = 0;
    sim->rects[0].color.b     = 0;
    sim->rects[0].color.a     = 255;
    sim->rects[1].center.x    = 50;
    sim->rects[1].center.y    = 60;
    sim->rects[1].half_size.x = 30;
    sim->rects[1].half_size.y = 10;
    sim->rects[1].orientation = 0;
    sim->rects[1].color.r     = 0;
    sim->rects[1].color.g     = 0;
    sim->rects[1].color.b     = 255;
    sim->rects[1].color.a     = 255;
    dg_rect_recompute_vertices(sim->rects);
    dg_rect_recompute_vertices(sim->rects+1);
}
void dg_sim_deinit(dg_sim *sim) {
    SDL_DestroyRenderer(sim->rdr);
    SDL_DestroyWindow(sim->win);
}
void dg_sim_read_events(dg_sim *sim) {
    SDL_Event e;
    while(SDL_PollEvent(&e)) 
    switch(e.type) {
    case SDL_QUIT: sim->quit = true; break;
    }
}
void dg_sim_render(dg_sim *sim) {
    size_t i;
    ++(sim->rects[0].orientation);
    dg_rect_recompute_vertices(sim->rects);
    SDL_SetRenderDrawColor(sim->rdr, 0, 0, 0, 255);
    SDL_RenderClear(sim->rdr);
    for(i=0 ; i<sim->rect_count ; ++i)
        dg_rect_render_sdl2(sim->rects+i, sim->rdr);
    SDL_RenderPresent(sim->rdr);
}

int main(int argc, char *argv[]) {
    SDL_Init(SDL_INIT_VIDEO);

    dg_sim sim;
    dg_sim_init(&sim);
    for(;;) {
        dg_sim_read_events(&sim);
        if(sim.quit)
            break;
        dg_sim_render(&sim);
    }
    dg_sim_deinit(&sim);

    SDL_Quit();
    return 0;
}
