#include <sphys/sphys.hpp>

namespace sphys {

template <typename T>
struct aabb { 
    vec2<T> pos, size; 
    aabb(T x, T y, T w, T h) : pos.x(x), pos.y(y), size.x(w), size.y(h) {}
}

typedef aabb<uint16_t> u16_aabb;

}
