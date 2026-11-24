// A two-lane SSE2 exp after SLEEF's xexp (Cody-Waite reduction by ln 2 in two parts, a degree-12 polynomial: 1 + s +
// s^2/2 + s^3 P(s), P of degree 9 by Estrin, 2^q in two halves): its error against std::exp in ulp over 20 million
// arguments across the solver's range (-1000..80), and its speed against std::exp, a value at a time.
//
// Kept, not used (CoolSynth's D346): within 1 ulp of std::exp everywhere (8.7 % of results 1 ulp off, denormals too),
// but 3.9 ns a value against MSVC's 4.25: some 0.7 % of the TB-303 voice, before gathering the devices' arguments
// into lanes. SLEEF (Boost licence) read for the method; nothing of it is built in. Build alone:
//   cl /O2 /std:c++20 /EHsc exp_sse2.cpp
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <emmintrin.h>
#include <random>
#include <vector>

// 2^q for an integer q (as a double, -1022..1023) in each lane: its bits built from q + 1023 by the 1.5 * 2^52 trick
static inline __m128d pow2(__m128d q)
{
    const __m128d magic = _mm_set1_pd(6755399441055744.0);
    const __m128i bits = _mm_sub_epi64(_mm_castpd_si128(_mm_add_pd(_mm_add_pd(q, _mm_set1_pd(1023.0)), magic)),
                                       _mm_castpd_si128(magic));
    return _mm_castsi128_pd(_mm_slli_epi64(bits, 52));
}

static inline __m128d mla(__m128d x, __m128d y, __m128d z) { return _mm_add_pd(_mm_mul_pd(x, y), z); }

static inline __m128d exp2lanes(__m128d d)
{
    const __m128d magic = _mm_set1_pd(6755399441055744.0); // round to nearest even for |x| < 2^51
    const __m128d u = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(d, _mm_set1_pd(1.442695040888963407359924681001892137426645954152985934135449406931)), magic), magic);
    __m128d s = _mm_sub_pd(d, _mm_mul_pd(u, _mm_set1_pd(0.69314718055966295651160180568695068359375)));
    s = _mm_sub_pd(s, _mm_mul_pd(u, _mm_set1_pd(0.28235290563031577122588448175013436025525412068e-12)));
    const __m128d s2 = _mm_mul_pd(s, s), s4 = _mm_mul_pd(s2, s2), s8 = _mm_mul_pd(s4, s4);
    const auto c = [](double x) { return _mm_set1_pd(x); };
    // SLEEF's POLY10(x, x2, x4, x8, c9 .. c0) = x8 POLY2(x, c9, c8) + POLY8(x, x2, x4, c7 .. c0), Estrin
    const __m128d hi = mla(s, c(0.2081276378237164457e-8), c(0.2511210703042288022e-7));
    const __m128d a01 = mla(s, c(0.2755762628169491192e-6), c(0.2755723402025388239e-5));
    const __m128d a23 = mla(s, c(0.2480158687479686264e-4), c(0.1984126989855865850e-3));
    const __m128d a45 = mla(s, c(0.1388888888914497797e-2), c(0.8333333333314938210e-2));
    const __m128d a67 = mla(s, c(0.4166666666666602598e-1), c(0.1666666666666669072e+0));
    const __m128d lo = mla(s4, mla(s2, a01, a23), mla(s2, a45, a67));
    __m128d r = mla(s8, hi, lo);
    r = mla(r, s, c(0.5));
    r = mla(r, s, c(1.0));
    r = mla(r, s, c(1.0));
    // 2^u in two halves (u down to -1443 at d = -1000), each within the normal range
    const __m128d h = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(u, c(0.5)), magic), magic);
    r = _mm_mul_pd(_mm_mul_pd(r, pow2(h)), pow2(_mm_sub_pd(u, h)));
    return _mm_andnot_pd(_mm_cmplt_pd(d, c(-1000.0)), r); // below -1000: zero, as SLEEF
}

static double ulps(double a, double b)
{
    if (a == b)
        return 0.0;
    std::int64_t ia, ib;
    std::memcpy(&ia, &a, 8);
    std::memcpy(&ib, &b, 8);
    return std::abs(static_cast<double>(ia - ib));
}

int main()
{
    std::mt19937_64 rng(1);
    std::uniform_real_distribution<double> wide(-1000.0, 80.0), narrow(-40.0, 40.0);
    const int n = 20000000;
    std::vector<double> x(n), mine(n);
    for (int i = 0; i < n; ++i)
        x[static_cast<size_t>(i)] = (i & 1) ? wide(rng) : narrow(rng);
    for (int i = 0; i < n; i += 2)
        _mm_storeu_pd(&mine[static_cast<size_t>(i)], exp2lanes(_mm_loadu_pd(&x[static_cast<size_t>(i)])));
    double worst = 0.0, worstAt = 0.0, worstDenormal = 0.0;
    long off = 0, offBy2 = 0;
    for (int i = 0; i < n; ++i)
    {
        const double want = std::exp(x[static_cast<size_t>(i)]);
        const double e = ulps(mine[static_cast<size_t>(i)], want);
        off += e > 0.0;
        offBy2 += e > 1.0;
        if (want < 2.2250738585072014e-308)
            worstDenormal = std::max(worstDenormal, e);
        else if (e > worst)
        {
            worst = e;
            worstAt = x[static_cast<size_t>(i)];
        }
    }
    std::printf("against std::exp: %.3f %% differ at all, %ld by more than 1 ulp; the worst %.0f ulp (at %.9f); denormal results within %.0f ulp\n",
                100.0 * off / n, offBy2, worst, worstAt, worstDenormal);
    using clock = std::chrono::steady_clock;
    double sink = 0.0;
    for (int round = 0; round < 3; ++round)
    {
        auto a = clock::now();
        for (int i = 0; i < n; ++i)
            sink += std::exp(x[static_cast<size_t>(i)] * 0.05);
        const double tStd = std::chrono::duration<double, std::nano>(clock::now() - a).count() / n;
        a = clock::now();
        __m128d acc = _mm_setzero_pd();
        const __m128d k = _mm_set1_pd(0.05);
        for (int i = 0; i < n; i += 2)
            acc = _mm_add_pd(acc, exp2lanes(_mm_mul_pd(_mm_loadu_pd(&x[static_cast<size_t>(i)]), k)));
        double two[2];
        _mm_storeu_pd(two, acc);
        sink += two[0] + two[1];
        const double tMine = std::chrono::duration<double, std::nano>(clock::now() - a).count() / n;
        std::printf("std::exp %.2f ns a value; two lanes %.2f ns a value\n", tStd, tMine);
    }
    std::printf("%g\n", sink);
}
