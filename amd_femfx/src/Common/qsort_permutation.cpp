/*-
 * Copyright (c) 1992, 1993
 *    The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

//#include <sys/cdefs.h>
#if defined(LIBC_SCCS) && !defined(lint)
#if 0
static char sccsid[] = "@(#)qsort.c    8.1 (Berkeley) 6/4/93";
#else
__RCSID("$NetBSD: qsort.c,v 1.20 2009/06/01 06:37:40 yamt Exp $");
#endif
#endif /* LIBC_SCCS and not lint */

#include <sys/types.h>

#include <assert.h>
#include <errno.h>
#include <stdlib.h>

//static inline char    *med3 QS__P((char *, char *, char *,
//    int (*)(const void *, const void *)));
//static inline void     swapfunc QS__P((char *, char *, size_t, int));

#define QS_MIN(a, b)    (a) < (b) ? a : b

#define QS__P(x) x
#define QS_DIAGASSERT assert

/*
 * Qsort routine from Bentley & McIlroy's "Engineering a Sort Function".
 */
#define QS_SWAPCODE(TYPE, parmi, parmj, n) {         \
    size_t i = (n) / sizeof (TYPE);         \
    TYPE *pi = (TYPE *)(void *)(parmi);         \
    TYPE *pj = (TYPE *)(void *)(parmj);         \
    do {                         \
        TYPE    t = *pi;            \
        *pi++ = *pj;                \
        *pj++ = t;                \
        } while (--i > 0);                \
}

#define QS_SWAPINIT(a, es) swaptype = ((char *)a - (char *)0) % sizeof(long) || \
    es % sizeof(long) ? 2 : es == sizeof(long)? 0 : 1;

static inline void
swapfunc(char *a, char *b, size_t n, int swaptype)
{

    if (swaptype <= 1) 
        QS_SWAPCODE(long, a, b, n)
    else
        QS_SWAPCODE(char, a, b, n)
}

static inline void
swapfunc_int(int *a, int *b, size_t n)
{
    size_t i = n;
    int *pi = a;
    int *pj = b;
    do {
        int t = *pi;
        *pi++ = *pj;
        *pj++ = t;
        } while (--i > 0);
}

#define QS_SWAP(a, b)                        \
    if (swaptype == 0) {                    \
        long t = *(long *)(void *)(a);            \
        *(long *)(void *)(a) = *(long *)(void *)(b);    \
        *(long *)(void *)(b) = t;            \
    } else                            \
        swapfunc(a, b, es, swaptype)

#define QS_SWAP_INT(a, b)                        \
     {  int t = *(int *)(void *)(a);            \
        *(int *)(void *)(a) = *(int *)(void *)(b);    \
        *(int *)(void *)(b) = t; }       

#define QS_VECSWAP(a, b, n) if ((n) > 0) swapfunc((a), (b), (size_t)(n), swaptype)
//#define QS_VECSWAP_INT(a, b, n) if ((n) > 0) swapfunc((char*)(a), (char*)(b), (size_t)(n)*sizeof(int), 0)
#define QS_VECSWAP_INT(a, b, n) if ((n) > 0) swapfunc_int(a, b, (size_t)(n))

static inline char *
med3(char *a, char *b, char *c,
    int (*cmp) QS__P((const void *, const void *)))
{

    return cmp(a, b) < 0 ?
           (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a ))
              :(cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c ));
}

void
qsort_permutation(void *a, int* permutation, size_t n, size_t es,
    int (*cmp) QS__P((const void *, const void *)))
{
    char *pa, *pb, *pc, *pd, *pl, *pm, *pn;
    int *ppa, *ppb, *ppc, *ppd, *ppl, *ppm, *ppn;
    size_t d, r;
    size_t rperm;
    int swaptype, cmp_result;

    QS_DIAGASSERT(a != nullptr);
    QS_DIAGASSERT(cmp != nullptr);

loop:    QS_SWAPINIT(a, es);
    if (n < 7) {
        for (pm = (char *) a + es, ppm = permutation + 1; pm < (char *) a + n * es; pm += es, ppm++)
            for (pl = pm, ppl = ppm; pl > (char *) a && cmp(pl - es, pl) > 0; pl -= es, ppl--)
            {
                QS_SWAP(pl, pl - es);
                QS_SWAP_INT(ppl, ppl - 1);
            }
        return;
    }
    pm = (char *) a + (n / 2) * es;
    ppm = permutation + (n / 2);
    if (n > 7) {
        pl = (char *) a;
        pn = (char *) a + (n - 1) * es;
        if (n > 40) {
            d = (n / 8) * es;
            pl = med3(pl, pl + d, pl + 2 * d, cmp);
            pm = med3(pm - d, pm, pm + d, cmp);
            pn = med3(pn - 2 * d, pn - d, pn, cmp);
        }
        pm = med3(pl, pm, pn, cmp);
        ppm = permutation + (size_t)(pm - (char *) a) / es;
    }
    QS_SWAP((char*)a, pm);
    QS_SWAP_INT(permutation, ppm);
    pa = pb = (char *) a + es;
    ppa = ppb = permutation + 1;

    pc = pd = (char *) a + (n - 1) * es;
    ppc = ppd = permutation + (n - 1);
    for (;;) {
        while (pb <= pc && (cmp_result = cmp(pb, a)) <= 0) {
            if (cmp_result == 0) {
                QS_SWAP(pa, pb);
                QS_SWAP_INT(ppa, ppb);
                pa += es;
                ppa++;
            }
            pb += es;
            ppb++;
        }
        while (pb <= pc && (cmp_result = cmp(pc, a)) >= 0) {
            if (cmp_result == 0) {
                QS_SWAP(pc, pd);
                QS_SWAP_INT(ppc, ppd);
                pd -= es;
                ppd--;
            }
            pc -= es;
            ppc--;
        }
        if (pb > pc)
            break;
        QS_SWAP(pb, pc);
        QS_SWAP_INT(ppb, ppc);
        pb += es;
        ppb++;
        pc -= es;
        ppc--;
    }

    pn = (char *) a + n * es;
    ppn = permutation + n;
    r = QS_MIN(pa - (char *) a, pb - pa);
    rperm = QS_MIN(ppa - permutation, ppb - ppa);
    QS_VECSWAP((char*)a, pb - r, r);
    QS_VECSWAP_INT(permutation, ppb - rperm, rperm);
    r = QS_MIN((size_t)(pd - pc), pn - pd - es);
    rperm = QS_MIN((ppd - ppc), ppn - ppd - 1);
    QS_VECSWAP(pb, pn - r, r);
    QS_VECSWAP_INT(ppb, ppn - rperm, rperm);
    rperm = ppb - ppa;
    if ((r = pb - pa) > es)
        qsort_permutation(a, permutation, r / es, es, cmp);
    rperm = ppd - ppc;
    if ((r = pd - pc) > es) { 
        /* Iterate rather than recurse to save stack space */
        a = pn - r;
        permutation = ppn - rperm;
        n = r / es;
        goto loop;
    }
/*        qsort(pn - r, r / es, es, cmp);*/
}