#ifndef BOULETTE_MATHUTILS_HPP
#define BOULETTE_MATHUTILS_HPP

/* t must be in [0;1]. */
template<typename T> T lerp(T left, T right, T t) {
    return left + (right-left)*t;
}

#endif /* BOULETTE_MATHUTILS_HPP */
