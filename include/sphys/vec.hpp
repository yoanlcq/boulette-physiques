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
    //vec2 abs() const { return vec2(abs(x), abs(y)); }
    friend vec2 abs(const vec2 &v) { return vec2(abs(v.x), abs(v.y)); }
    friend T dot(const vec2 &u, const vec2 &v) { return u.x*v.x + u.y*v.y; }
    friend T norm(const vec2 &u) { 
        T sqnorm = sqrNorm(u);
        T sq = sqrt(sqnorm);
        std::cout << "norm(): sqrt(" << sqnorm << ") = " << sq << std::endl;
        return sq; 
    }
    friend T sqrNorm(const vec2 &u) { 
        std::cout << "sqrNorm():u.x=" << u.x << std::endl;
        std::cout << "sqrNorm():u.x*u.x=" << u.x*u.x << std::endl;
        std::cout << "sqrNorm():u.y=" << u.y << std::endl;
        std::cout << "sqrNorm():u.y*u.y=" << u.y*u.y << std::endl;
        std::cout << "sqrNorm():       =" << u.x*u.x + u.y*u.y << std::endl;
        return u.x*u.x + u.y*u.y; 
    }
    friend std::ostream& operator<<(std::ostream& lhs, const vec2 &rhs) {return lhs << "(" << rhs.x << ", " << rhs.y << ")";}
    friend vec2 operator+ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x+rhs.x, lhs.y+rhs.y);}
    friend vec2 operator- (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x-rhs.x, lhs.y-rhs.y);}
    friend vec2 operator* (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x*rhs.x, lhs.y*rhs.y);}
    friend vec2 operator/ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x/rhs.x, lhs.y/rhs.y);}
    friend vec2 operator+ (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x+rhs,   lhs.y+rhs);  }
    friend vec2 operator- (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x-rhs,   lhs.y-rhs);  }
    friend vec2 operator* (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x*rhs,   lhs.y*rhs);  }
    friend vec2 operator/ (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x/rhs,   lhs.y/rhs);  }
    friend bool operator==(const vec2 &lhs, const vec2 &rhs) {return lhs.x==rhs.x && lhs.y==rhs.y;}
    friend bool operator!=(const vec2 &lhs, const vec2 &rhs) {return !operator==(lhs,rhs);}
    friend bool operator< (const vec2 &lhs, const vec2 &rhs) {return lhs.x<rhs.x && lhs.y<rhs.y;}
    friend bool operator> (const vec2 &lhs, const vec2 &rhs) {return lhs.x>rhs.x && lhs.y<rhs.y;}
    friend bool operator<=(const vec2 &lhs, const vec2 &rhs) {return lhs.x<=rhs.x && lhs.y<=rhs.y;}
    friend bool operator>=(const vec2 &lhs, const vec2 &rhs) {return lhs.x>=rhs.x && lhs.y<=rhs.y;}
};

typedef vec2<float> f32v2;

}

#endif//SPHYS_VEC_HPP
