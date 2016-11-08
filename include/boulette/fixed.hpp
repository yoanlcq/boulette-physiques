#ifndef BOULETTE_FIXED_HPP
#define BOULETTE_FIXED_HPP

#include <stdcxx.hpp>

namespace boulette {

typedef  int32_t qsint;
typedef uint32_t quint;
typedef  int64_t qlsint;
typedef uint64_t qluint;

#define LOCAL_E  2.71828182845904523536
#define LOCAL_PI 3.14159265358979323846

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
    static q  fromRaw(qsint raw) {q r; r.raw = raw; return r;}

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
    operator    float() const {return raw/(float)fmask;}
    operator   double() const {return raw/(double)fmask;}
    operator  int64_t() const {return raw>>f;}
    operator  int32_t() const {return raw>>f;}
    operator  int16_t() const {return raw>>f;}
    operator  int8_t()  const {return raw>>f;}
    operator uint64_t() const {return raw>>f;}
    operator uint32_t() const {return raw>>f;}
    operator uint16_t() const {return raw>>f;}
    operator uint8_t()  const {return raw>>f;}
    operator    bool()  const {return !!raw;}
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

    friend q  abs(const q &x) {return x<q(0) ? -x : x;}
    friend q  sin(q fp) {
        int sign = 1;
        q sqr, result;
        const q SK[2] = {
            q(7.61e-03),
            q(1.6605e-01)
        };

        fp.raw %= 2*pi.raw;
        if(fp < q(0))
            fp += q(2)*pi;
        if(fp > pi/q(2) && fp <= pi) 
            fp = pi-fp;
        else if(fp > pi && fp <= pi + pi/q(2)) {
            fp -= pi;
            sign = -1;
        } else if(fp > pi + pi/q(2)) {
            fp = (pi.raw << 1) - fp.raw;
            sign = -1;
        }
        sqr = fp*fp;
        result = SK[0];
        result *= sqr;
        result -= SK[1];
        result *= sqr;
        result += q(1);
        result *= fp;
        return q(sign) * result;
    }
    friend q  cos(const q &x) {
        return sin(pi/q(2) - x);
    }
    friend q  tan(q x) {/*assert(false && "This was not implemented yet!");*/ return q(0);}
    friend q  acos(q x) {/*assert(false && "This was not implemented yet!");*/ return q(0);}
    friend q  asin(q x) {/*assert(false && "This was not implemented yet!");*/ return q(0);}
    friend q  atan(q x) {/*assert(false && "This was not implemented yet!");*/ return q(0);}
    friend q  atan2(q y, q x) {/*assert(false && "This was not implemented yet!");*/ return q(0);}
    friend q  ln (const q &x) {
        q log2, xi, ff, s, z, w, R;
        const q LN2(0.69314718055994530942);
        const q LG[7] = {
            q(6.666666666666735130e-01),
            q(3.999999999940941908e-01),
            q(2.857142874366239149e-01),
            q(2.222219843214978396e-01),
            q(1.818357216161805012e-01),
            q(1.531383769920937332e-01),
            q(1.479819860511658591e-01)
        };

        if (x.raw < 0)
            return (0);
        if (x.raw == 0)
            return 0xffffffff;

        log2 = 0;
        xi = x;
        while (xi > q(2)) {
            xi.raw >>= 1;
            (log2.raw)++;
        }
        ff = xi - q(1);
        s = ff/(q(2) + ff);
        z = s*s;
        w = z*z;
        R = w*(LG[1] + w*(LG[3] + w*LG[5]))
          + z*(LG[0] + w*(LG[2] + w*(LG[4] + w*LG[6])));
        return LN2*fromRaw(log2.raw << f) + ff - s*(ff-R);
    }
#if 0
    friend q  ufact(size_t n) {
        q r(1);
        for(size_t i=2 ; i<=n ; ++i)
            r *= i;
        return r;
    }
    friend q  upow(const q &x, size_t p) {
        q r(1);
        for(size_t i=0 ; i<p ; ++i)
            r *= x;
        return r;
    }
#endif
    friend q  exp(q x, size_t n=10) {
        /*
        q r(1);
        for(size_t i=1 ; i<=n ; ++i)
            r += upow(x,i)/ufact(i);
        return r;
        */
        q xabs, k, z, R, xp;
        const q LN2(0.69314718055994530942);
        const q LN2_INV(1.4426950408889634074);
        const q EXP_P[5] = {
            q(1.66666666666666019037e-01),
            q(-2.77777777770155933842e-03),
            q(6.61375632143793436117e-05),
            q(-1.65339022054652515390e-06),
            q(4.13813679705723846039e-08),
        };

        if(x == q(0))
            return q(1);
        xabs = abs(x);
        k = xabs*LN2_INV;
        k += q(0.5);
        k.raw &= ~fmask;
        if(x < q(0))
            k = -k;
        x -= k*LN2;
        z = x*x;
        /* Taylor */
        R = q(2) + z*(EXP_P[0] + z*(EXP_P[1] + z*(EXP_P[2] + z*(EXP_P[3] + (z*EXP_P[4])))));

        xp = q(1) + (x*q(2))/(R - x);
        if (k < q(0))
            k.raw = q(1).raw >> (-k.raw >> f);
        else
            k.raw = q(1).raw << (k.raw >> f);

        return k*xp;
    }
    friend q  log(const q &x, const q base) {return ln(x)/ln(base);}
    friend q  pow(const q &x, const q &p)   {return exp(ln(x)*p);}
    friend q sqrt(const q &x)               {
        if(x < q(0))
            std::cerr << "Warning : Square root is not defined for " << x << std::endl;
        return pow(x, q(0.5));
    }
};

template<size_t d, size_t f> const q<d,f> q<d,f>::pi(LOCAL_PI);
template<size_t d, size_t f> const q<d,f> q<d,f>::e(LOCAL_E);
template<size_t d, size_t f> const quint  q<d,f>::fmask((1<<f)-1);

#undef LOCAL_E 
#undef LOCAL_PI

} // namespace boulette


#endif//BOULETTE_FIXED_HPP
