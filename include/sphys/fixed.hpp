#ifndef SPHYS_FIXED_HPP
#define SPHYS_FIXED_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace sphys {

typedef  int32_t qsint;
typedef uint32_t quint;
typedef  int32_t qlsint;
typedef uint32_t qluint;

template <size_t dbits, size_t fbits> 
struct q {
    qsint raw; // This member is kept public. Edit only if you know what you're doing.
    q(float f);
    q(double f);
    q(int64_t i);
    ~q();
    static q one, two, pi, half_pi, two_pi, e;
    static quint fmask;
    qlsint lraw() const;
    std::string toString(int max_dec=-2) const;
    explicit operator float();
    explicit operator double();
    explicit operator int64_t();
    q& operator+=(const q& rhs);
    q& operator-=(const q& rhs);
    q& operator*=(const q& rhs);
    q& operator/=(const q& rhs);
};

template <size_t d, size_t f> inline q<d,f> operator+ (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline q<d,f> operator- (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline q<d,f> operator* (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline q<d,f> operator/ (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator==(const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator!=(const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator< (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator> (const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator<=(const q<d,f> &lhs, const q<d,f> &rhs);
template <size_t d, size_t f> inline bool   operator>=(const q<d,f> &lhs, const q<d,f> &rhs);


typedef q<16,16> q16_16;
}

#endif//SPHYS_FIXED_HPP
