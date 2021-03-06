#ifndef BOULETTE_VEC_HPP
#define BOULETTE_VEC_HPP

#include <boulette.hpp>
#include <stdcxx.hpp>

namespace boulette {

template <typename T> 
struct vec2 { 
    T x, y; 
    vec2() : x(0), y(0) {}
    vec2(T x, T y) : x(x), y(y) {}
    ~vec2() {}
    //vec2 abs() const { return vec2(abs(x), abs(y)); }
    friend vec2 abs(const vec2 &v) { return vec2(abs(v.x), abs(v.y)); }
    friend T dot(const vec2 &u, const vec2 &v) { return u.x*v.x + u.y*v.y; }
    friend T norm(const vec2 &u) { 
        T sqnorm = sqrNorm(u);
        T sq = sqrt(sqnorm);
        //std::cout << "norm(): sqrt(" << sqnorm << ") = " << sq << std::endl;
        return sq; 
    }
    friend vec2 normalize(const vec2 &u) { return u/norm(u); }
    friend T sqrNorm(const vec2 &u) { 
        /*
        std::cout << "sqrNorm():u.x=" << u.x << std::endl;
        std::cout << "sqrNorm():u.x*u.x=" << u.x*u.x << std::endl;
        std::cout << "sqrNorm():u.y=" << u.y << std::endl;
        std::cout << "sqrNorm():u.y*u.y=" << u.y*u.y << std::endl;
        std::cout << "sqrNorm():       =" << u.x*u.x + u.y*u.y << std::endl;
        */
        return u.x*u.x + u.y*u.y; 
    }
    friend std::ostream& operator<<(std::ostream& lhs, const vec2 &rhs) {return lhs << "(" << rhs.x << ", " << rhs.y << ")";}
    friend vec2 operator+ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x+rhs.x, lhs.y+rhs.y);}
    friend vec2 operator- (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x-rhs.x, lhs.y-rhs.y);}
    friend vec2 operator* (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x*rhs.x, lhs.y*rhs.y);}
    friend vec2 operator/ (const vec2 &lhs, const vec2 &rhs) {return vec2(lhs.x/rhs.x, lhs.y/rhs.y);}
    vec2  operator- () const { return vec2(0,0)-*this;}
    vec2& operator+=(const vec2& rhs) {*this = (*this)+rhs; return *this;}
    vec2& operator-=(const vec2& rhs) {*this = (*this)-rhs; return *this;}
    vec2& operator*=(const vec2& rhs) {*this = (*this)*rhs; return *this;}
    vec2& operator/=(const vec2& rhs) {*this = (*this)/rhs; return *this;}
    friend vec2 operator+ (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x+rhs,   lhs.y+rhs);  }
    friend vec2 operator- (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x-rhs,   lhs.y-rhs);  }
    friend vec2 operator* (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x*rhs,   lhs.y*rhs);  }
    friend vec2 operator/ (const vec2 &lhs, const    T &rhs) {return vec2(lhs.x/rhs,   lhs.y/rhs);  }
    vec2& operator+=(const T& rhs) {*this = (*this)+rhs; return *this;}
    vec2& operator-=(const T& rhs) {*this = (*this)-rhs; return *this;}
    vec2& operator*=(const T& rhs) {*this = (*this)*rhs; return *this;}
    vec2& operator/=(const T& rhs) {*this = (*this)/rhs; return *this;}
    friend bool operator==(const vec2 &lhs, const vec2 &rhs) {return lhs.x==rhs.x && lhs.y==rhs.y;}
    friend bool operator!=(const vec2 &lhs, const vec2 &rhs) {return !operator==(lhs,rhs);}
    friend bool operator< (const vec2 &lhs, const vec2 &rhs) {return lhs.x<rhs.x && lhs.y<rhs.y;}
    friend bool operator> (const vec2 &lhs, const vec2 &rhs) {return lhs.x>rhs.x && lhs.y<rhs.y;}
    friend bool operator<=(const vec2 &lhs, const vec2 &rhs) {return lhs.x<=rhs.x && lhs.y<=rhs.y;}
    friend bool operator>=(const vec2 &lhs, const vec2 &rhs) {return lhs.x>=rhs.x && lhs.y<=rhs.y;}
};

typedef vec2<float> f32v2;

}

#endif//BOULETTE_VEC_HPP
