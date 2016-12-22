#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <assert.h>
#ifndef __GNUC__
#error "Please use a GCC-compatible compiler."
#endif
#include <x86intrin.h>

// Compilation :
//     gcc -g -std=c11 -msse -msse2 -O3 prove_simd.c
// Disassembly files (disas_*.txt) generated using :
//     gdb a.out -q -ex "set disassembly-flavor intel" \
//                  -ex "disas /m u32_mass_add" \
//                  -ex "quit" \
//                  > disas_*.txt
//
// 'disas_1.txt' contains the output when NAIVE is defined.
// 'disas_3.txt' contains the output when it isn't.
//
// When comparing both files, the difference in code size and efficiency
// becomes immediately alarming.
// Here's my explanation for disas_1.txt :
//     Even though the program is compiled with -O3, there are things the
//     compiler cannot prove, which cause it to perform a lot of operations
//     we might not actually need.
//     We can see the pshufd, movdqa, paddd and movaps instructions, 
//     but they need many preceding instructions because :
//     - The compiler cannot prove that our array starts at a 16-byte boundary.
//       I believe it tries to manually align data.
//     - The compiler cannot prove that the array's element count is a 
//       multiple of 4.
//       Since it is a uint32 array, and XMM registers can hold 4 uint32s at 
//       a time, if the element count is not guaranteed to be a multiple 
//       of 4 then XMM loads may load extra data that is not part of the array,
//       so the generated code has to perform the appropriate checks.
//
// By using GCC's __attribute__((vector_size)), we can declare to GCC a
// vector type for 4 uint32s (automatically, GCC infers that it is 16-byte 
// aligned, so pointers to them are assumed to be multiples of 16).
// The resulting assembly is in 'disas_3.txt', and it doesn't take an expert
// to understand that it is much, much better.
//
// We are now constrained to allocate our integers by groups of 4, but apart
// from the care it requires, it's not much of a problem - at worse, we
// have 3 extra integers that are not treated as part of the array.


#define MALLOC_ALIGN16

#ifndef MALLOC_ALIGN16
#define MALLOC(x) malloc(x)
#define FREE(x) free(x)
#else
#define MALLOC(x) _mm_malloc((x), 16)
#define FREE(x) _mm_free(x)
#endif

//#define NAIVE
#ifdef NAIVE

void u32_mass_add(uint32_t *ary, size_t cnt, uint32_t val) {
    for(uint_fast32_t i=0 ; i<cnt ; ++i)
        ary[i] += val;
}
void u32_mass_print(uint32_t const *ary, size_t cnt) {
    for(uint_fast32_t i=0 ; i<cnt ; ++i)
        printf("%"PRIu32"\n", ary[i]);
}
void u32_mass_set_with_rand(uint32_t *ary, size_t cnt) {
    for(uint_fast32_t i=0 ; i<cnt ; ++i)
        ary[i] = rand();
}

int main(void) {

    srand(time(NULL));

#define CNT 4096
    uint32_t *array = MALLOC(CNT*sizeof(uint32_t));
    u32_mass_set_with_rand(array, CNT);    
    u32_mass_add(array, CNT, rand());
    u32_mass_print(array, CNT);
    FREE(array);
    return EXIT_SUCCESS;
}

#else

typedef uint32_t __attribute__((vector_size(16))) u32v4;

void u32_mass_add(u32v4 *ary, size_t cnt, uint32_t val) {
    u32v4 val4 = { val, val, val, val };
    for(uint_fast32_t i=0 ; i<(cnt+3)/4 ; ++i)
        ary[i] += val4;
}
void u32_mass_print(u32v4 const *ary, size_t cnt) {
    for(uint_fast32_t i=0 ; i<(cnt+3)/4 ; ++i)
        printf("%"PRIu32"\n%"PRIu32"\n%"PRIu32"\n%"PRIu32"\n", 
               ary[i][0], ary[i][1], ary[i][2], ary[i][3]);
}
void u32_mass_set_with_rand(u32v4 *ary, size_t cnt) {
    for(uint_fast32_t i=0 ; i<(cnt+3)/4 ; ++i)
        ary[i] = (u32v4){rand(), rand(), rand(), rand()};
}

int main(void) {

    srand(time(NULL));

#define CNT 4096
    u32v4 *array = _mm_malloc(((CNT+3)/4)*sizeof(u32v4), 16);
    u32_mass_set_with_rand(array, CNT);    
    u32_mass_add(array, CNT, rand());
    u32_mass_print(array, CNT);
    _mm_free(array);
    return EXIT_SUCCESS;
}


#endif
