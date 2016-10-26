#ifndef SPHYS_FIXED_HPP
#define SPHYS_FIXED_HPP

#include <stdcxx.hpp>

namespace sphys {

typedef  int32_t qsint;
typedef uint32_t quint;
typedef  int64_t qlsint;
typedef uint64_t qluint;

template <size_t d, size_t f> 
struct q {
    qsint raw; // This member is kept public. Edit only if you know what you're doing.
    ~q() {}
    q()           : raw(0) {}
    q(float    x) : raw(x*q(1).raw + (x>=0.f ? 0.5f : -0.5f)) {}
    q(double   x) : raw(x*q(1).raw + (x>=0.  ? 0.5  : -0.5 )) {}
    q(int64_t  i) : raw(i<<f) {}
    q(int32_t  i) : raw(i<<f) {}
    q(int16_t  i) : raw(i<<f) {}
    q(int8_t   i) : raw(i<<f) {}
    q(uint64_t i) : raw(i<<f) {}
    q(uint32_t i) : raw(i<<f) {}
    q(uint16_t i) : raw(i<<f) {}
    q(uint8_t  i) : raw(i<<f) {}

    static const q pi;
    static const q e;
    static const quint fmask;

    std::string toString(int max_dec=-2) const {
        char str[32];
        int ndec = 0, slen = 0;
        char tmp[12] = {0};
        qluint fr, ip;
        auto rawcpy = raw;

        static_assert(d>1, "Needs decimal_bits > 1, Because we need more than the sign bit of signed integers.");
        static_assert(d>0, "Needs decimal_bits > 0, otherwise we'll print garbage.");
        static_assert(d+f<=64, "Too big width.");
        if (max_dec == -1)
            max_dec = (d+f<=32 ? 2 : 10);
        else if (max_dec == -2)
            max_dec = 15;

        if (rawcpy < 0) {
            str[slen++] = '-';
            rawcpy *= -1;
        }

        ip = rawcpy>>f;
        do {
            tmp[ndec++] = '0' + ip % 10;
            ip /= 10;
        } while (ip);

        while (ndec > 0)
            str[slen++] = tmp[--ndec];
        str[slen++] = '.';

        fr = rawcpy&fmask;
        do {
            fr *= 10;
            str[slen++] = '0' + (fr >> f);
            fr &= (1 << f) - 1;
            ndec++;
        } while (fr && ndec < max_dec);

        if (ndec > 1 && str[slen-1] == '0')
            str[slen-1] = '\0'; /* cut off trailing 0 */
        else
            str[slen] = '\0';
        return std::string(str);
    }

    qlsint lraw() const {return raw;}
    operator    float() {return raw/(float)fmask;}
    operator   double() {return raw/(double)fmask;}
    operator  int64_t() {return raw>>f;}
    operator  int32_t() {return raw>>f;}
    operator  int16_t() {return raw>>f;}
    operator  int8_t()  {return raw>>f;}
    operator uint64_t() {return raw>>f;}
    operator uint32_t() {return raw>>f;}
    operator uint16_t() {return raw>>f;}
    operator uint8_t()  {return raw>>f;}
    q  operator- () const { return q(0)-*this;}
    q& operator+=(const q& rhs) {*this = (*this)+rhs; return *this;}
    q& operator-=(const q& rhs) {*this = (*this)-rhs; return *this;}
    q& operator*=(const q& rhs) {*this = (*this)*rhs; return *this;}
    q& operator/=(const q& rhs) {*this = (*this)/rhs; return *this;}
    friend std::ostream& operator<<(std::ostream& lhs, const q &rhs) {return lhs << rhs.toString();}
    friend q    operator+ (const q &lhs, const q &rhs) {q r; r.raw = lhs.raw+rhs.raw; return r;}
    friend q    operator- (const q &lhs, const q &rhs) {q r; r.raw = lhs.raw-rhs.raw; return r;}
    friend q    operator* (const q &lhs, const q &rhs) {q r; r.raw = (lhs.lraw()*rhs.lraw()) >> f; return r;}
    friend q    operator/ (const q &lhs, const q &rhs) {q r; r.raw = (lhs.lraw() << f)/rhs.lraw(); return r;}
    friend bool operator==(const q &lhs, const q &rhs) {return lhs.raw==rhs.raw;    }
    friend bool operator!=(const q &lhs, const q &rhs) {return !operator==(lhs,rhs);}
    friend bool operator< (const q &lhs, const q &rhs) {return lhs.raw<rhs.raw;     }
    friend bool operator> (const q &lhs, const q &rhs) {return  operator< (rhs,lhs);}
    friend bool operator<=(const q &lhs, const q &rhs) {return !operator> (lhs,rhs);}
    friend bool operator>=(const q &lhs, const q &rhs) {return !operator< (lhs,rhs);}

    friend q           abs(const q &x)                 {return x<q(0) ? -x : x;}
};

template<size_t d, size_t f> const q<d,f> q<d,f>::pi(3.14159265358979323846);
template<size_t d, size_t f> const q<d,f> q<d,f>::e(2.71828182845904523536);
template<size_t d, size_t f> const quint q<d,f>::fmask((1<<f)-1);


} // namespace sphys


#endif//SPHYS_FIXED_HPP
