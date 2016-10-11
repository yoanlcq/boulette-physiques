#ifndef SPHYS_AABB_HPP
#define SPHYS_AABB_HPP

#include <sphys.hpp>

namespace sphys {

template <typename T>
struct aabb_2d { 
    T x,y,w,h; 
};

typedef aabb_2d<int16_t> s16_aabb_2d;

}

#endif//SPHYS_AABB_HPP
