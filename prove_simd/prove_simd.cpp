#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <x86intrin.h>

using namespace std;
#if 0
typedef uint32_t /*alignas(16)*/ u32_align16;

    typedef uint32_t __attribute__((vector_size(16))) u32v4;

struct PackedU32Set {
    uint32_t *array;
    size_t count;
    PackedU32Set(size_t cnt) : array((uint32_t*)_mm_malloc(cnt*sizeof*array, 16)), count(cnt) {}
    ~PackedU32Set() {
        _mm_free(array);
    }
    __attribute__ ((noinline))
    void add(uint32_t val) {
        for(uint_fast32_t i=0 ; i<count ; ++i)
            array[i] += val;
    }
    friend std::ostream& operator<<(std::ostream &lhs, const PackedU32Set &rhs) {
        for(uint_fast32_t i=0 ; i<rhs.count ; ++i)
            cout << rhs.array[i] << " ";
        return cout;
    }
    void set_with_rand() {
        for(uint_fast32_t i=0 ; i<count ; ++i)
            array[i] += rand();
    }
};
#endif

#if 0
__attribute__((alloc_align(1), alloc_size()))
__attribute__((assume_aligned(16)))
T* aligned_ptr(T* ary) { return __builtin_assume_aligned(ary, align); }
#endif

#define NOINLINE __attribute__((noinline))

template<typename T>
class sse_array {
private:
    static_assert(sizeof(T)==4, "");
    typedef uint32_t __attribute__((vector_size(16))) xmm;
    xmm *array;
    size_t count;
public:
    sse_array(size_t cnt=0) : array((xmm*)_mm_malloc(cnt*sizeof(T), 16)), count(cnt) {}
    ~sse_array() { _mm_free(array); }
    T* get_array() __attribute__((assume_aligned(16))) {
        return (T*)__builtin_assume_aligned(array, 16);
    }
    NOINLINE void add(T val) {
        for(uint_fast32_t i=0 ; i<(count+3)/4 ; ++i)
            array[i] += (xmm){val,val,val,val};
    }
    friend std::ostream& operator<<(std::ostream &lhs, sse_array &rhs) {
        for(uint_fast32_t i=0 ; i<rhs.count ; ++i)
            cout << rhs.get_array()[i] << " ";
        return cout;
    }
    void set_with_rand() {
        for(uint_fast32_t i=0 ; i<count ; ++i)
            get_array()[i] += rand()%14;
    }
};

#include <valarray>
int main(void) {
    //srand(time(NULL));
    /*
    PackedU32Set pu(4096);
    pu.set_with_rand();
    pu.add(rand());
    cout << pu << endl;
    typedef std::valarray<uint32_t> StdPackedU32Set;
    StdPackedU32Set spu = StdPackedU32Set(4096);
    for(auto& i : spu) 
        i = rand();
    spu += rand();
    for(auto& i : spu) 
        cout << i << endl;
    */
    sse_array<uint32_t> ss(4096);
    ss.set_with_rand();
    ss.add(rand()%14);
    cout << ss << endl;
    return 0;
}
