#include <sphys.hpp>

namespace sphys {

template <size_t dbits, size_t fbits>
q<dbits,fbits>::q(float f) : raw(f*one + (f>=0.f ? 0.5f : -0.5f)) {}

template <size_t dbits, size_t fbits>
q<dbits,fbits>::q(double f) : raw(f*one + (f>=0. ? 0.5 : -0.5)) {}

template <size_t dbits, size_t fbits>
q<dbits,fbits>::q(int64_t i) : raw(i<<fbits) {}

template <size_t dbits, size_t fbits>
q<dbits,fbits>::~q() {}

template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::one(1);
template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::two(2);
template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::pi(3.14159265358979323846);
template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::two_pi(3.14159265358979323846*2.);
template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::half_pi(3.14159265358979323846/2.);
template <size_t dbits, size_t fbits> q<dbits,fbits> q<dbits,fbits>::e(2.71828182845904523536);
template <size_t dbits, size_t fbits> quint q<dbits,fbits>::fmask((1<<fbits)-1);

template <size_t dbits, size_t fbits>
q<dbits,fbits>::operator float() {
    return raw/(float)fmask;
}
template <size_t dbits, size_t fbits>
q<dbits,fbits>::operator double() {
    return raw/(double)fmask;
}
template <size_t dbits, size_t fbits>
q<dbits,fbits>::operator int64_t() {
    return raw>>fbits;
}

template <size_t dbits, size_t fbits>
std::string q<dbits,fbits>::toString(int max_dec) const {
    char str[32];
	int ndec = 0, slen = 0;
	char tmp[12] = {0};
	qluint fr, ip;

    static_assert(dbits+fbits<=64, "Too big width.");
	if (max_dec == -1)
		max_dec = (dbits+fbits<=32 ? 2 : 10);
	else if (max_dec == -2)
		max_dec = 15;

	if (raw < 0) {
		str[slen++] = '-';
		raw *= -1;
	}

	ip = raw>>fbits;
	do {
		tmp[ndec++] = '0' + ip % 10;
		ip /= 10;
	} while (ip);

	while (ndec > 0)
		str[slen++] = tmp[--ndec];
	str[slen++] = '.';

	fr = raw&fmask;
	do {
		fr *= 10;
		str[slen++] = '0' + (fr >> fbits);
		fr &= (1 << fbits) - 1;
		ndec++;
	} while (fr && ndec < max_dec);

	if (ndec > 1 && str[slen-1] == '0')
		str[slen-1] = '\0'; /* cut off trailing 0 */
	else
		str[slen] = '\0';
    return std::string(str);
}



template <size_t d, size_t f>
inline qlsint q<d,f>::lraw() const { return raw; }

template <size_t d, size_t f> inline q<d,f> operator+ (const q<d,f> &lhs, const q<d,f> &rhs) {q<d,f> q; q.raw = lhs.raw+rhs.raw; return q;}
template <size_t d, size_t f> inline q<d,f> operator- (const q<d,f> &lhs, const q<d,f> &rhs) {q<d,f> q; q.raw = lhs.raw-rhs.raw; return q;}
template <size_t d, size_t f> inline q<d,f> operator* (const q<d,f> &lhs, const q<d,f> &rhs) {
	return (lhs.lraw()*rhs.lraw()) >> f;
}
template <size_t d, size_t f>        q<d,f> operator/ (const q<d,f> &lhs, const q<d,f> &rhs) {
	return (lhs.lraw() << f) / rhs.lraw();
}
template <size_t d, size_t f> inline bool   operator==(const q<d,f> &lhs, const q<d,f> &rhs) {return lhs.raw==rhs.raw;    }
template <size_t d, size_t f> inline bool   operator!=(const q<d,f> &lhs, const q<d,f> &rhs) {return !operator==(lhs,rhs);}
template <size_t d, size_t f> inline bool   operator< (const q<d,f> &lhs, const q<d,f> &rhs) {return lhs.raw<rhs.raw;     }
template <size_t d, size_t f> inline bool   operator> (const q<d,f> &lhs, const q<d,f> &rhs) {return  operator< (rhs,lhs);}
template <size_t d, size_t f> inline bool   operator<=(const q<d,f> &lhs, const q<d,f> &rhs) {return !operator> (lhs,rhs);}
template <size_t d, size_t f> inline bool   operator>=(const q<d,f> &lhs, const q<d,f> &rhs) {return !operator< (lhs,rhs);}


}
