#ifndef BOULETTE_HPP
#define BOULETTE_HPP

#ifdef __GNUC__
#define debug_break() __asm__("int 3":::"memory")
#define NOINLINE __attribute__((noinline))
#else
#error "Please use a GCC-compatible compiler."
#endif

#include <boulette/fixed.hpp>
#include <boulette/mathutils.hpp>
#include <boulette/vec.hpp>
#include <boulette/aabb.hpp>
#include <boulette/disk.hpp>
#include <boulette/box.hpp>
#include <boulette/tri.hpp>
#include <boulette/verlet.hpp>

#endif//BOULETTE_HPP
