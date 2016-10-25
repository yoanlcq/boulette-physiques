#ifndef SPHYS_VEC_HPP
#define SPHYS_VEC_HPP

#include <sphys.hpp>
#include <stdcxx.hpp>

namespace sphys {

template <typename T> 
struct vec2 { 
    T x, y; 
    vec2() : x(0), y(0) {}
    vec2(T x, T y) : x(x), y(y) {}
    ~vec2() {}
    vec2& operator+=(const vec2& rhs);
    vec2& operator-=(const vec2& rhs);
    vec2& operator*=(const vec2& rhs);
    vec2& operator/=(const vec2& rhs);
    friend std::ostream& operator<<(std::ostream& lhs, const vec2 &rhs) {return lhs << "(" << rhs.x << ", " << rhs.y << ")";}
    friend vec2 operator+ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x+rhs.x, lhs.y+rhs.y);}
    friend vec2 operator- (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x-rhs.x, lhs.y-rhs.y);}
    friend vec2 operator* (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x*rhs.x, lhs.y*rhs.y);}
    friend vec2 operator/ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x/rhs.x, lhs.y/rhs.y);}
    friend bool operator==(const vec2 &lhs, const vec2 &rhs) {return lhs.x==rhs.x && lhs.y==rhs.y;}
    friend bool operator!=(const vec2 &lhs, const vec2 &rhs) {return !operator==(lhs,rhs);}
    friend bool operator< (const vec2 &lhs, const vec2 &rhs) {return lhs.x<rhs.x && lhs.y<rhs.y;}
    friend bool operator> (const vec2 &lhs, const vec2 &rhs) {return  operator< (rhs,lhs);}
    friend bool operator<=(const vec2 &lhs, const vec2 &rhs) {return !operator> (lhs,rhs);}
    friend bool operator>=(const vec2 &lhs, const vec2 &rhs) {return !operator< (lhs,rhs);}
};

typedef vec2<float> f32v2;

}

#endif//SPHYS_VEC_HPP
