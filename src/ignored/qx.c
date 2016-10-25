#include <stdio.h>
#include <stdint.h>
#include <assert.h>

typedef int8_t  qint;
typedef int16_t qlint; /* Must be at least  2*sizeof qint. */

typedef union {
     qint    raw;
    uint16_t db0, db1, db2, db3, db4;
    uint8_t  db5, db6, db7;
    uint8_t  fb0, fb1, fb2, fb3, fb4;
    uint16_t fb5, fb6, fb7;
} q5_3;

#define qx_priv_xbits(q,x) \
    ( sizeof((q).x##b0)/2 \
    + sizeof((q).x##b1)/2 \
    + sizeof((q).x##b2)/2 \
    + sizeof((q).x##b3)/2 \
    + sizeof((q).x##b4)/2 \
    + sizeof((q).x##b5)/2 \
    + sizeof((q).x##b6)/2 \
    + sizeof((q).x##b7)/2 \
    )
#define qx_dbits(q) qx_priv_xbits(q,d)
#define qx_fbits(q) qx_priv_xbits(q,f)
#define qx_bits(q)  (8*sizeof(q))

#define qx_sametype(a,b) (qx_dbits(a)==qx_dbits(b) && qx_fbits(a)==qx_fbits(b))

#define qx_add(r,a,b) ((r)->raw = ((a.raw)+(b.raw)))
#define qx_mul(r,a,b) do { \
        static_assert(qx_sametype(a,b), "Not same type!"); \
        static_assert(qx_sametype(*r,a), "Not same type!"); \
        (r)->raw = qx_mul_inl((a).raw, (b).raw, qx_fbits(a)); \
    } while(0)
static inline qlint qx_mul_inl(qlint a, qlint b, size_t fbits) {
    return (a*b)>>fbits;
}

int main(void) {
    q5_3 q;
    printf("%zu, %zu, %zu\n", sizeof q, sizeof q.db0, sizeof q.db5);
    printf("q%zu.%zu\n", qx_dbits(q), qx_fbits(q));
    qx_mul(&q,q,q); 
    qx_add(&q,q,q);
    return 0;
}
