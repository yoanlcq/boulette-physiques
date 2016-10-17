#include <cstddef>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

uint64_t fact(uint64_t x) {
    uint64_t prod;
    for(prod=1 ; x>=1 ; --x)
        prod *= x;
    return prod;
}

inline double cos_taylor(double x, size_t order) {
    size_t i;
    double res=1.0, sign = -1.0;
    for(i=1 ; i<order ; ++i) {
        res += sign*pow(x,i*2)/fact(i*2);
        sign = -sign;
    }
    return res;
}
inline double sin_taylor(double x, size_t order) {
    return cos_taylor(x-M_PI/2.0, order);
}

int main(void) {
    double foo = 1.5;
    size_t i;
    for(i=0 ; i<10 ; ++i)
        std::cout << cos_taylor(foo, i) << std::endl;
    std::cout << cos(foo) << std::endl;
    return EXIT_SUCCESS;
}
