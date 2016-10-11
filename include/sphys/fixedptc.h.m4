changecom(`<<<<<<<<',`>>>>>>>>')dnl
#ifndef FE_Q`'WBITS`'_`'FBITS`'_H
#define FE_Q`'WBITS`'_`'FBITS`'_H

/*! \file fate/fixedpt/q`'WBITS`'_`'FBITS.h
 *  \brief q`'WBITS`'_`'FBITS
 *  \ingroup fixedpt
 *
 * Note : I am not the original author of the fixed-point-related code within
 * this file. Please find the credits and preserved copyright notice at the
 * end of this description.
 *
 * First, some links : 
 * - <a href="http://gafferongames.com/networking-for-game-programmers/floating-point-determinism/" target="_blank">This must-read article from Gaffer on Games about consistency of the operations on floating-point numbers across various devices</a>;
 * - <a href="http://www.tundragames.com/minimizing-the-pain-of-lockstep-multiplayer/" target="_blank">Precious article from Tundra Games, in which they explain how they managed to make their game's simulation yield the same results on their target devices</a>;
 * - <a href="https://en.wikipedia.org/wiki/Q_`'(number_format)" target="_blank">Wikipedia's description of Q numbers</a>.
 *
 * F.A.T.E provides a template for generating arbitrary Q numbers, provided
 * their total size is either 32 or 64 bits. Long story short : Q numbers 
 * allow representing rational numbers using an in-memory format which
 * differs from floating-point numbers, but unlike them, operations are 
 * guaranteed to yield the same results across various CPU architectures, 
 * devices, compilers, compiler settings, and operating systems.
 * 
 * This is possible because Q numbers are based on integers - and since most
 * operations on them require very few instructions, they are pretty fast too.
 *
 * GCC has built-in support for fixed-point numbers, but not for all targets -
 * I dropped it as soon as I saw it wasn't supported on my regular laptop.
 * 
 * Now, consistency is only a concern if we're talking about implicitly shared 
 * state, but for otherwise client-specific behaviour, such as rendering, we 
 * can freely (and cheaply!) convert to floats/doubles.
 *
 * Regarding the current implementation : If a Q number uses a total size
 * greater than 32 bits but less than or equal to 64 bits, the implementation
 * will attempt to use the #__int128_t type if the compiler provides it 
 * (seemingly, if SSE is supported by the target architecture). Otherwise
 * it will fall back to performing some extra operations on 64-bit integers.
 *
 * All operations on q`'WBITS`'_`'FBITS types also work on uq`'WBITS`'_`'FBITS
 * types : their unsigned version.
 *
 * TODO : I'm a bit skeptical about the #rconst() macro. Does it yield the same
 * results on every platform, with different compilers and compiler settings ?
 *
 * 
 * This file has been generated by using GNU M4 on GENERATOR. 
 * Therefore, it's pointless to edit it directly. Edit GENERATOR 
 * instead.
 *
 *
 * GENERATOR itself is a modified version of :
 * https://github.com/andrewray/mirage-kfreebsd/blob/master/fixpt/fixedptc.h
 * Please find the preserved copyright notice after this description. 
 * 
 * Endless thanks to the original authors and contributors of the fixedptc  
 * library.
 *
 * -- ORIGINAL COPYRIGHT NOTICE --
 *
 * Copyright (c) 2010-2012 Ivan Voras <ivoras@freebsd.org>
 * Copyright (c) 2012 Tim Hartrick <tim@edgecast.com>
 * Copyright (c) 2013 Gabor Pali <pgj@freebsd.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * -- END OF COPYRIGHT NOTICE --
 * @{
 */

#include <stdint.h>

#define FE_Q`'WBITS`'_`'FBITS`'_WBITS WBITS
#define FE_Q`'WBITS`'_`'FBITS`'_FBITS FBITS
#define FE_Q`'WBITS`'_`'FBITS`'_BITS  (WBITS+FBITS)

#if FE_Q`'WBITS`'_`'FBITS`'_BITS == 32
typedef int32_t fe_q`'WBITS`'_`'FBITS;
typedef	int64_t	fe_q`'WBITS`'_`'FBITS`'d;
typedef	uint32_t fe_uq`'WBITS`'_`'FBITS`';
typedef	uint64_t fe_uq`'WBITS`'_`'FBITS`'d;
#elif FE_Q`'WBITS`'_`'FBITS`'_BITS == 64

#if !defined(__SIZEOF_INT128__)
typedef struct {
    uint64_t hi;
    uint64_t lo;
    char     sign;
} fe_q`'WBITS`'_`'FBITS`'_int128;
#endif

typedef int64_t  fe_q`'WBITS`'_`'FBITS;
typedef	uint64_t fe_uq`'WBITS`'_`'FBITS`';

#if defined(__SIZEOF_INT128__)
typedef	__int128_t fe_q`'WBITS`'_`'FBITS`'d;
typedef	__uint128_t fe_uq`'WBITS`'_`'FBITS`'d;
#endif

#else
#error "FE_Q`'WBITS`'_`'FBITS`'_BITS must be equal to 32 or 64"
#endif


#if FE_Q`'WBITS`'_`'FBITS`'_WBITS >= FE_Q`'WBITS`'_`'FBITS`'_BITS
#error "FE_Q`'WBITS`'_`'FBITS`'_WBITS must be less than FE_Q`'WBITS`'_`'FBITS`'_BITS"
#endif

/*! \brief The bit mask covering the fractional part of a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_FMASK	(((fe_q`'WBITS`'_`'FBITS)1 << FE_Q`'WBITS`'_`'FBITS`'_FBITS) - 1)

/*! \brief Converts a float/double to a Q`'WBITS`'.`'FBITS.
 *         It is named \c rconst because the macro is type-agnostic.
 *         TODO: Does it yield the result across various platforms and compiler settings ? */
#define fe_q`'WBITS`'_`'FBITS`'_rconst(R) ((fe_q`'WBITS`'_`'FBITS)((R) * FE_Q`'WBITS`'_`'FBITS`'_ONE + ((R) >= 0 ? 0.5 : -0.5)))
/*! \brief Converts a float to a Q`'WBITS`'.`'FBITS. 
 *         TODO: Does it yield the result across various platforms and compiler settings ? */
#define fe_q`'WBITS`'_`'FBITS`'_from_float(R) fe_q`'WBITS`'_`'FBITS`'_rconst(R)
/*! \brief Converts a double to a Q`'WBITS`'.`'FBITS.
 *         TODO: Does it yield the result across various platforms and compiler settings ? */
#define fe_q`'WBITS`'_`'FBITS`'_from_double(R) fe_q`'WBITS`'_`'FBITS`'_rconst(R)

/*! \brief Converts a Q`'WBITS`'.`'FBITS to a float. 
 *         Warning : the result is not supposed to be consistent across platforms. */
#define fe_q`'WBITS`'_`'FBITS`'_to_float(F) ((F)/(float)FE_Q`'WBITS`'_`'FBITS`'_FMASK)
/*! \brief Converts a Q`'WBITS`'.`'FBITS to a double.
 *         Warning : the result is not supposed to be consistent across platforms. */
#define fe_q`'WBITS`'_`'FBITS`'_to_double(F) ((F)/(double)FE_Q`'WBITS`'_`'FBITS`'_FMASK)

/*! \brief Converts an integer of any width to a Q`'WBITS`'.`'FBITS. */
#define fe_q`'WBITS`'_`'FBITS`'_from_int(I) (((fe_q`'WBITS`'_`'FBITS)(I)) << FE_Q`'WBITS`'_`'FBITS`'_FBITS)
/*! \brief Converts a Q`'WBITS`'.`'FBITS to an integer. */
#define fe_q`'WBITS`'_`'FBITS`'_to_int(F) ((F) >> FE_Q`'WBITS`'_`'FBITS`'_FBITS)


/*! \brief Returns the addition of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. */
#define fe_q`'WBITS`'_`'FBITS`'_add(A,B) ((A) + (B))
/*! \brief Returns the subtraction of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. */
#define fe_q`'WBITS`'_`'FBITS`'_sub(A,B) ((A) - (B))

#if defined(__DOXYGEN__) \
 || FE_Q`'WBITS`'_`'FBITS`'_BITS == 32 \
 || (FE_Q`'WBITS`'_`'FBITS`'_BITS == 64 && defined(__SIZEOF_INT128__))
/*! \brief Returns the product of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. 
 *
 * Unlike #fe_q`'WBITS`'_`'FBITS`'_mul(), it is implemented as a macro
 * when possible.
 */
#define fe_q`'WBITS`'_`'FBITS`'_xmul(A,B)						\
	((fe_q`'WBITS`'_`'FBITS)(((fe_q`'WBITS`'_`'FBITS`'d)(A) * (fe_q`'WBITS`'_`'FBITS`'d)(B)) >> FE_Q`'WBITS`'_`'FBITS`'_FBITS))
/*! \brief Returns the division of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. 
 *
 * Unlike #fe_q`'WBITS`'_`'FBITS`'_div(), it is implemented as a macro
 * when possible.
 */
#define fe_q`'WBITS`'_`'FBITS`'_xdiv(A,B)						\
	((fe_q`'WBITS`'_`'FBITS)(((fe_q`'WBITS`'_`'FBITS`'d)(A) << FE_Q`'WBITS`'_`'FBITS`'_FBITS) / (fe_q`'WBITS`'_`'FBITS`'d)(B)))
#else
#define fe_q`'WBITS`'_`'FBITS`'_xmul(A,B) \
        fe_q`'WBITS`'_`'FBITS`'_mul(A,B)
#define fe_q`'WBITS`'_`'FBITS`'_xdiv(A,B) \
        fe_q`'WBITS`'_`'FBITS`'_div(A,B)
#endif

/*! \brief Extracts the fractional part from a Q`'WBITS`'.`'FBITS. */
#define fe_q`'WBITS`'_`'FBITS`'_fracpart(A) ((fe_q`'WBITS`'_`'FBITS)(A) & FE_Q`'WBITS`'_`'FBITS`'_FMASK)

/*! \brief 1, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_ONE	((fe_q`'WBITS`'_`'FBITS)((fe_q`'WBITS`'_`'FBITS)1 << FE_Q`'WBITS`'_`'FBITS`'_FBITS))
/*! \brief 0.5, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_ONE_HALF (FE_Q`'WBITS`'_`'FBITS`'_ONE >> 1)
/*! \brief 2, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_TWO	(FE_Q`'WBITS`'_`'FBITS`'_ONE + FE_Q`'WBITS`'_`'FBITS`'_ONE)
/*! \brief Pi, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_PI	fe_q`'WBITS`'_`'FBITS`'_rconst(3.14159265358979323846)
/*! \brief Pi*2, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_TWO_PI	fe_q`'WBITS`'_`'FBITS`'_rconst(2 * 3.14159265358979323846)
/*! \brief Pi/2, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_HALF_PI	fe_q`'WBITS`'_`'FBITS`'_rconst(3.14159265358979323846 / 2)
/*! \brief e, as a Q`'WBITS`'.`'FBITS. */
#define FE_Q`'WBITS`'_`'FBITS`'_E	fe_q`'WBITS`'_`'FBITS`'_rconst(2.7182818284590452354)

/*! \brief Returns a Q`'WBITS`'.`'FBITS's absolute value. */
#define fe_q`'WBITS`'_`'FBITS`'_abs(A) ((A) < 0 ? -(A) : (A))


/*! \brief Returns the division of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_div(fe_q`'WBITS`'_`'FBITS A, fe_q`'WBITS`'_`'FBITS B)
{
#if FE_Q`'WBITS`'_`'FBITS`'_BITS == 64 && !defined(__SIZEOF_INT128__)
	/* A bit complicated but non-SSE version. */
	fe_q`'WBITS`'_`'FBITS`'_int128 tmp;
	fe_q`'WBITS`'_`'FBITS result, rem;
	uint64_t s;
	int i;
	const uint64_t hib = 0x8000000000000000ULL;

	tmp.sign = 0;

	if (0 > A) {
		tmp.sign = !tmp.sign;
		A = -A;
	}

	tmp.hi = A >> (FE_Q`'WBITS`'_`'FBITS`'_BITS - FE_Q`'WBITS`'_`'FBITS`'_FBITS);
	tmp.lo = A << FE_Q`'WBITS`'_`'FBITS`'_FBITS;

	if (0 > B) {
		tmp.sign = !tmp.sign;
		B = -B;
	}

	rem = 0;
	for (i = 0; i < 128; i++) {
		rem <<= 1;

		if (tmp.hi & hib)
			rem |= 1;

		s = tmp.lo & hib;
		tmp.hi <<= 1;
		tmp.lo <<= 1;

		if (s)
			tmp.hi |= 1;

		if (rem >= B) {
			rem -= B;
			tmp.lo |= 1;
		}
	}

	result = tmp.lo;
	result = tmp.sign ? -result : result;

	return result;
#else
	return (((fe_q`'WBITS`'_`'FBITS`'d)A << FE_Q`'WBITS`'_`'FBITS`'_FBITS) / (fe_q`'WBITS`'_`'FBITS`'d)B);
#endif
}

/*! \brief Returns the product of two Q`'WBITS`'.`'FBITS`'s as a Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_mul(fe_q`'WBITS`'_`'FBITS A, fe_q`'WBITS`'_`'FBITS B)
{
#if 0 && FE_Q`'WBITS`'_`'FBITS`'_BITS == 64
    /* This overrides the original non-SSE implementation, because it
     * didn't yield the same results as the SSE implementation. 
     * Besides, it returned garbage when fbits > wbits (GCC warns about
     * a left-shift overflow).
     * Have this lazy division-based implementation instead which, at least,
     * yields the same results everywhere. This freaks out my 
     * estimated-performance-o-meter though. We also lose a bit of
     * precision in the process. */
    return fe_q`'WBITS`'_`'FBITS`'_div(
               A, 
               fe_q`'WBITS`'_`'FBITS`'_div(
                   FE_Q`'WBITS`'_`'FBITS`'_ONE,
                   B
               )
           );
#else
#if FE_Q`'WBITS`'_`'FBITS`'_BITS == 64 && !defined(__SIZEOF_INT128__)
	/* A bit complicated but non-SSE version. */
	fe_q`'WBITS`'_`'FBITS`'_int128 tmp;
	uint64_t a0, a1;
	uint64_t b0, b1;
	uint64_t d, d0, d1;
	uint64_t e, e0, e1;
	uint64_t f, f0, f1;
	uint64_t g, g0, g1;
	uint64_t sum, carry, roll, pmax;
	fe_q`'WBITS`'_`'FBITS result;

	tmp.sign = 0;

	if (0 > A) {
		tmp.sign = !tmp.sign;
		A = -A;
	}

	if (0 > B) {
		tmp.sign = !tmp.sign;
		B = -B;
	}

	a1 = A >> 32;
	a0 = A - (a1 << 32);

	b1 = B >> 32;
	b0 = B - (b1 << 32);

	d = a0 * b0;
	d1 = d >> 32;
	d0 = d - (d1 << 32);

	e = a0 * b1;
	e1 = e >> 32;
	e0 = e - (e1 << 32);

	f = a1 * b0;
	f1 = f >> 32;
	f0 = f - (f1 << 32);

	g = a1 * b1;
	g1 = g >> 32;
	g0 = g - (g1 << 32);

	sum = d1 + e0 + f0;
	carry = 0;
	roll = 1 << 30;
	roll <<= 2;

	pmax = roll - 1;
	for (; pmax < sum; sum -= roll, carry++);

	tmp.lo = d0 + (sum << 32);
	tmp.hi = carry + e1 + f1 + g0 + (g1 << 32);

	result = ((tmp.hi >> FE_Q`'WBITS`'_`'FBITS`'_FBITS) << (2 * FE_Q`'WBITS`'_`'FBITS`'_FBITS)) +
	    ((tmp.hi & FE_Q`'WBITS`'_`'FBITS`'_FMASK) << FE_Q`'WBITS`'_`'FBITS`'_FBITS) + (tmp.lo >> FE_Q`'WBITS`'_`'FBITS`'_FBITS);
	result = tmp.sign ? -result : result;

	return result;

#else
	return (((fe_q`'WBITS`'_`'FBITS`'d)A * (fe_q`'WBITS`'_`'FBITS`'d)B) >> FE_Q`'WBITS`'_`'FBITS`'_FBITS);
#endif
#endif
}



/*! \brief Converts a Q`'WBITS`'.`'FBITS to string. The string buffer should be at least
 *         25-characters wide.
 *
 * The max_dec argument specifies how many decimal digits to the right
 * of the decimal point to generate. If set to -1, the "default" number
 * of decimal digits will be used (2 for 32-bit fe_q`'WBITS`'_`'FBITS width, 10 for
 * 64-bit fe_q`'WBITS`'_`'FBITS width); If set to -2, "all" of the digits will
 * be returned, meaning there will be invalid, bogus digits outside the
 * specified precisions.
 */
static inline void
fe_q`'WBITS`'_`'FBITS`'_str(fe_q`'WBITS`'_`'FBITS A, char *str, int max_dec)
{
	int ndec = 0, slen = 0;
	char tmp[12] = {0};
	fe_uq`'WBITS`'_`'FBITS`' fr, ip;

	if (max_dec == -1)
#if FE_Q`'WBITS`'_`'FBITS`'_BITS == 32
		max_dec = 2;
#elif FE_Q`'WBITS`'_`'FBITS`'_BITS == 64
		max_dec = 10;
#else
#error Invalid width
#endif
	else if (max_dec == -2)
		max_dec = 15;

	if (A < 0) {
		str[slen++] = '-';
		A *= -1;
	}

	ip = fe_q`'WBITS`'_`'FBITS`'_to_int(A);
	do {
		tmp[ndec++] = '0' + ip % 10;
		ip /= 10;
	} while (ip != 0);

	while (ndec > 0)
		str[slen++] = tmp[--ndec];
	str[slen++] = '.';

	fr = fe_q`'WBITS`'_`'FBITS`'_fracpart(A);
	do {
		fr *= 10;
		str[slen++] = '0' + (fr >> FE_Q`'WBITS`'_`'FBITS`'_FBITS);
		fr &= (((fe_q`'WBITS`'_`'FBITS)1 << FE_Q`'WBITS`'_`'FBITS`'_FBITS) - 1);
		ndec++;
	} while (fr != 0 && ndec < max_dec);

	if (ndec > 1 && str[slen-1] == '0')
		str[slen-1] = '\0'; /* cut off trailing 0 */
	else
		str[slen] = '\0';
}


/*! \brief Returns the sine of the given Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_sin(fe_q`'WBITS`'_`'FBITS fp)
{
	int sign = 1;
	fe_q`'WBITS`'_`'FBITS sqr, result;
	const fe_q`'WBITS`'_`'FBITS SK[2] = {
		fe_q`'WBITS`'_`'FBITS`'_rconst(7.61e-03),
		fe_q`'WBITS`'_`'FBITS`'_rconst(1.6605e-01)
	};

	fp %= 2 * FE_Q`'WBITS`'_`'FBITS`'_PI;
	if (fp < 0)
		fp = FE_Q`'WBITS`'_`'FBITS`'_PI * 2 + fp;
	if ((fp > FE_Q`'WBITS`'_`'FBITS`'_HALF_PI) && (fp <= FE_Q`'WBITS`'_`'FBITS`'_PI)) 
		fp = FE_Q`'WBITS`'_`'FBITS`'_PI - fp;
	else if ((fp > FE_Q`'WBITS`'_`'FBITS`'_PI) && (fp <= (FE_Q`'WBITS`'_`'FBITS`'_PI + FE_Q`'WBITS`'_`'FBITS`'_HALF_PI))) {
		fp = fp - FE_Q`'WBITS`'_`'FBITS`'_PI;
		sign = -1;
	} else if (fp > (FE_Q`'WBITS`'_`'FBITS`'_PI + FE_Q`'WBITS`'_`'FBITS`'_HALF_PI)) {
		fp = (FE_Q`'WBITS`'_`'FBITS`'_PI << 1) - fp;
		sign = -1;
	}
	sqr = fe_q`'WBITS`'_`'FBITS`'_mul(fp, fp);
	result = SK[0];
	result = fe_q`'WBITS`'_`'FBITS`'_mul(result, sqr);
	result -= SK[1];
	result = fe_q`'WBITS`'_`'FBITS`'_mul(result, sqr);
	result += FE_Q`'WBITS`'_`'FBITS`'_ONE;
	result = fe_q`'WBITS`'_`'FBITS`'_mul(result, fp);
	return sign * result;
}


/*! \brief Returns the cosine of the given Q`'WBITS`'.`'FBITS. 
 * 
 * I've observed that the error is about 1/10000.
 */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_cos(fe_q`'WBITS`'_`'FBITS A)
{
	return (fe_q`'WBITS`'_`'FBITS`'_sin(FE_Q`'WBITS`'_`'FBITS`'_HALF_PI - A));
}


/*! \brief Returns the tangens of the given Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_tan(fe_q`'WBITS`'_`'FBITS A)
{
	return fe_q`'WBITS`'_`'FBITS`'_div(fe_q`'WBITS`'_`'FBITS`'_sin(A), fe_q`'WBITS`'_`'FBITS`'_cos(A));
}


/*! \brief Returns the value exp(x), i.e. e^x of the given Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_exp(fe_q`'WBITS`'_`'FBITS fp)
{
	fe_q`'WBITS`'_`'FBITS xabs, k, z, R, xp;
	const fe_q`'WBITS`'_`'FBITS LN2 = fe_q`'WBITS`'_`'FBITS`'_rconst(0.69314718055994530942);
	const fe_q`'WBITS`'_`'FBITS LN2_INV = fe_q`'WBITS`'_`'FBITS`'_rconst(1.4426950408889634074);
	const fe_q`'WBITS`'_`'FBITS EXP_P[5] = {
		fe_q`'WBITS`'_`'FBITS`'_rconst(1.66666666666666019037e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(-2.77777777770155933842e-03),
		fe_q`'WBITS`'_`'FBITS`'_rconst(6.61375632143793436117e-05),
		fe_q`'WBITS`'_`'FBITS`'_rconst(-1.65339022054652515390e-06),
		fe_q`'WBITS`'_`'FBITS`'_rconst(4.13813679705723846039e-08),
	};

	if (fp == 0)
		return (FE_Q`'WBITS`'_`'FBITS`'_ONE);
	xabs = fe_q`'WBITS`'_`'FBITS`'_abs(fp);
	k = fe_q`'WBITS`'_`'FBITS`'_mul(xabs, LN2_INV);
	k += FE_Q`'WBITS`'_`'FBITS`'_ONE_HALF;
	k &= ~FE_Q`'WBITS`'_`'FBITS`'_FMASK;
	if (fp < 0)
		k = -k;
	fp -= fe_q`'WBITS`'_`'FBITS`'_mul(k, LN2);
	z = fe_q`'WBITS`'_`'FBITS`'_mul(fp, fp);
	/* Taylor */
	R = FE_Q`'WBITS`'_`'FBITS`'_TWO +
	    fe_q`'WBITS`'_`'FBITS`'_mul(z, EXP_P[0] + fe_q`'WBITS`'_`'FBITS`'_mul(z, EXP_P[1] +
	    fe_q`'WBITS`'_`'FBITS`'_mul(z, EXP_P[2] + fe_q`'WBITS`'_`'FBITS`'_mul(z, EXP_P[3] +
	    fe_q`'WBITS`'_`'FBITS`'_mul(z, EXP_P[4])))));
	xp = FE_Q`'WBITS`'_`'FBITS`'_ONE + fe_q`'WBITS`'_`'FBITS`'_div(fe_q`'WBITS`'_`'FBITS`'_mul(fp, FE_Q`'WBITS`'_`'FBITS`'_TWO), R - fp);
	if (k < 0)
		k = FE_Q`'WBITS`'_`'FBITS`'_ONE >> (-k >> FE_Q`'WBITS`'_`'FBITS`'_FBITS);
	else
		k = FE_Q`'WBITS`'_`'FBITS`'_ONE << (k >> FE_Q`'WBITS`'_`'FBITS`'_FBITS);
	return (fe_q`'WBITS`'_`'FBITS`'_mul(k, xp));
}


/*! \brief Returns the natural logarithm of the given Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_ln(fe_q`'WBITS`'_`'FBITS x)
{
	fe_q`'WBITS`'_`'FBITS log2, xi;
	fe_q`'WBITS`'_`'FBITS f, s, z, w, R;
	const fe_q`'WBITS`'_`'FBITS LN2 = fe_q`'WBITS`'_`'FBITS`'_rconst(0.69314718055994530942);
	const fe_q`'WBITS`'_`'FBITS LG[7] = {
		fe_q`'WBITS`'_`'FBITS`'_rconst(6.666666666666735130e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(3.999999999940941908e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(2.857142874366239149e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(2.222219843214978396e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(1.818357216161805012e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(1.531383769920937332e-01),
		fe_q`'WBITS`'_`'FBITS`'_rconst(1.479819860511658591e-01)
	};

	if (x < 0)
		return (0);
	if (x == 0)
		return 0xffffffff;

	log2 = 0;
	xi = x;
	while (xi > FE_Q`'WBITS`'_`'FBITS`'_TWO) {
		xi >>= 1;
		log2++;
	}
	f = xi - FE_Q`'WBITS`'_`'FBITS`'_ONE;
	s = fe_q`'WBITS`'_`'FBITS`'_div(f, FE_Q`'WBITS`'_`'FBITS`'_TWO + f);
	z = fe_q`'WBITS`'_`'FBITS`'_mul(s, s);
	w = fe_q`'WBITS`'_`'FBITS`'_mul(z, z);
	R = fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[1] + fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[3]
	    + fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[5]))) + fe_q`'WBITS`'_`'FBITS`'_mul(z, LG[0]
	    + fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[2] + fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[4]
	    + fe_q`'WBITS`'_`'FBITS`'_mul(w, LG[6]))));
	return (fe_q`'WBITS`'_`'FBITS`'_mul(LN2, (log2 << FE_Q`'WBITS`'_`'FBITS`'_FBITS)) + f
	    - fe_q`'WBITS`'_`'FBITS`'_mul(s, f - R));
}
	

/*! \brief Returns the logarithm of the given base of the given Q`'WBITS`'.`'FBITS. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_log(fe_q`'WBITS`'_`'FBITS x, fe_q`'WBITS`'_`'FBITS base)
{
	return (fe_q`'WBITS`'_`'FBITS`'_div(fe_q`'WBITS`'_`'FBITS`'_ln(x), fe_q`'WBITS`'_`'FBITS`'_ln(base)));
}


/*! \brief Return the power value (n^exponent) of the given Q`'WBITS`'.`'FBITS`'s. 
 *
 * The implementation returns 0 if \p n is negative.
 */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_pow(fe_q`'WBITS`'_`'FBITS n, fe_q`'WBITS`'_`'FBITS exp)
{
	if (exp == 0)
		return (FE_Q`'WBITS`'_`'FBITS`'_ONE);
	if (n < 0)
		return 0;
	return (fe_q`'WBITS`'_`'FBITS`'_exp(fe_q`'WBITS`'_`'FBITS`'_mul(fe_q`'WBITS`'_`'FBITS`'_ln(n), exp)));
}

/*! \brief Returns the square root of the given Q`'WBITS`'.`'FBITS, 
 *         or -1 in case of error. */
static inline fe_q`'WBITS`'_`'FBITS
fe_q`'WBITS`'_`'FBITS`'_sqrt(fe_q`'WBITS`'_`'FBITS A)
{
    /* Commented out the original implementation because it doesn't work.
     * Seemingly, it wasn't tested on Q16.48 numbers. */
    return fe_q`'WBITS`'_`'FBITS`'_pow(A, FE_Q`'WBITS`'_`'FBITS`'_ONE_HALF);
    /*
	int invert = 0;
	int iter = FE_Q`'WBITS`'_`'FBITS`'_FBITS;
	int l, i;

	if (A < 0)
		return (-1);
	if (A == 0 || A == FE_Q`'WBITS`'_`'FBITS`'_ONE)
		return (A);
	if (A < FE_Q`'WBITS`'_`'FBITS`'_ONE && A > 6) {
		invert = 1;
		A = fe_q`'WBITS`'_`'FBITS`'_div(FE_Q`'WBITS`'_`'FBITS`'_ONE, A);
	}
	if (A > FE_Q`'WBITS`'_`'FBITS`'_ONE) {
		int s = A;

		iter = 0;
		while (s > 0) {
			s >>= 2;
			iter++;
		}
	}

	// Newton's iterations
	l = (A >> 1) + 1;
	for (i = 0; i < iter; i++)
		l = (l + fe_q`'WBITS`'_`'FBITS`'_div(A, l)) >> 1;
	if (invert)
		return (fe_q`'WBITS`'_`'FBITS`'_div(FE_Q`'WBITS`'_`'FBITS`'_ONE, l));
	return (l);
    */
}



/*! @} */

#endif /* FE_Q`'WBITS`'_`'FBITS`'_H */