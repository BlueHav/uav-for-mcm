#include <cmath>
#include <cstdint>
#include <algorithm>

// Identical half-cell ray samples to RollingLocalMap's NumPy fallback.
// Occupied endpoint retention is applied by the caller after carving.
template<bool HalfMetre> __attribute__((always_inline)) inline void carve_impl(std::uint8_t* state, const std::int64_t* shape,
                      const std::int64_t* origin, double resolution,
                      const double* position, const double* rays, int count) {
    const double maximum=resolution*std::sqrt(double(shape[0]*shape[0]+shape[1]*shape[1]+shape[2]*shape[2]));
    for(int r=0;r<count;++r){
        const double* ray=rays+3*r;
        const double length=std::sqrt(ray[0]*ray[0]+ray[1]*ray[1]+ray[2]*ray[2]);
        if(!std::isfinite(length)||length<=0.)continue;
        const double limit=std::min(length,maximum);
        const double ux=ray[0]/length,uy=ray[1]/length,uz=ray[2]/length;
        bool entered=false;
        for(int step=0;step*resolution*.5<limit;++step){
            const double distance=step*resolution*.5;
            if(length<=distance+resolution*.75)break;
            const double px=position[0]+ux*distance,py=position[1]+uy*distance,pz=position[2]+uz*distance;
            // Division by exactly0.5 is exactly multiplication by2 for these
            // bounded map coordinates. Other resolutions keep original math.
            const auto x=std::int64_t(std::floor(HalfMetre?px*2.:px/resolution))-origin[0];
            const auto y=std::int64_t(std::floor(HalfMetre?py*2.:py/resolution))-origin[1];
            const auto z=std::int64_t(std::floor(HalfMetre?pz*2.:pz/resolution))-origin[2];
            const bool inside=x>=0&&x<shape[0]&&y>=0&&y<shape[1]&&z>=0&&z<shape[2];
            if(inside){
                state[(x*shape[1]+y)*shape[2]+z]=1;
                entered=true;
            }else if(entered){
                // A straight ray cannot re-enter a convex box after leaving it.
                // Keep original sample coordinates, including outside starts.
                break;
            }
        }
    }
}

#if defined(__x86_64__) && defined(__GNUC__)
// Runtime dispatch retains the original portable build. SSE4.1 provides exact
// hardware floor instead of the generic x86 rounding sequence; no fast-math.
__attribute__((target("sse4.1"))) static void carve_sse(std::uint8_t* state, const std::int64_t* shape,
                      const std::int64_t* origin, double resolution,
                      const double* position, const double* rays, int count) {
    if(resolution==.5) carve_impl<true>(state,shape,origin,.5,position,rays,count);
    else carve_impl<false>(state,shape,origin,resolution,position,rays,count);
}
#endif

extern "C" void carve(std::uint8_t* state, const std::int64_t* shape,
                      const std::int64_t* origin, double resolution,
                      const double* position, const double* rays, int count) {
#if defined(__x86_64__) && defined(__GNUC__)
    if(__builtin_cpu_supports("sse4.1")) {
        carve_sse(state,shape,origin,resolution,position,rays,count);return;
    }
#endif
    if(resolution==.5) carve_impl<true>(state,shape,origin,.5,position,rays,count);
    else carve_impl<false>(state,shape,origin,resolution,position,rays,count);
}
