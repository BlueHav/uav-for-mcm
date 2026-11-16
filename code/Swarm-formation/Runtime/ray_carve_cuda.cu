#include <cuda_runtime.h>
#include <cstdint>
#include <vector>

namespace {
struct Grid {long long shape[3],origin[3];double p[3],resolution;};
__global__ void carve_mask(const double* rays,int ray_count,Grid grid,unsigned* mask){
  int r=blockDim.x*blockIdx.x+threadIdx.x;
  if(r>=ray_count)return;
  const double* q=rays+3*r;
  double length=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]);
  if(!isfinite(length)||length<=0.)return;
  const auto* n=grid.shape;
  const double scale=grid.resolution;
  const double limit=fmin(length,scale*sqrt(double(n[0]*n[0]+n[1]*n[1]+n[2]*n[2])));
  const double ux=q[0]/length,uy=q[1]/length,uz=q[2]/length;
  bool entered=false;
  for(int step=0;step*scale*.5<limit;++step){
    const double d=step*scale*.5;
    if(length<=d+scale*.75)break;
    long long x=static_cast<long long>(floor((grid.p[0]+ux*d)/scale))-grid.origin[0];
    long long y=static_cast<long long>(floor((grid.p[1]+uy*d)/scale))-grid.origin[1];
    long long z=static_cast<long long>(floor((grid.p[2]+uz*d)/scale))-grid.origin[2];
    if(x>=0&&x<n[0]&&y>=0&&y<n[1]&&z>=0&&z<n[2]){
      unsigned index=(x*n[1]+y)*n[2]+z;
      atomicOr(mask+(index>>5),1U<<(index&31));
      entered=true;
    }else if(entered)break;
  }
}

struct Workspace {
    double* rays=nullptr;
    unsigned* mask=nullptr;
    size_t ray_capacity=0, mask_capacity=0;
    cudaStream_t stream=nullptr;
    std::vector<unsigned> host;

    ~Workspace() {
        if (rays) cudaFree(rays);
        if (mask) cudaFree(mask);
        if (stream) cudaStreamDestroy(stream);
    }
};

template<class T>
cudaError_t reserve(T*& buffer,size_t& capacity,size_t required) {
    if (capacity>=required) return cudaSuccess;
    T* replacement=nullptr;
    auto status=cudaMalloc(&replacement,required*sizeof(T));
    if (status!=cudaSuccess) return status;
    if (buffer) cudaFree(buffer);
    buffer=replacement;
    capacity=required;
    return cudaSuccess;
}
} // namespace

static int carve_cuda_impl(std::uint8_t* state,const std::int64_t* shape,
                         const std::int64_t* origin,double resolution,
                         const double* position,const double* rays,int count,bool on_device) {
    if (count<=0) return 0;
    static thread_local Workspace workspace;
    const Grid grid{{shape[0],shape[1],shape[2]},{origin[0],origin[1],origin[2]},
                    {position[0],position[1],position[2]},resolution};
    const size_t cells=static_cast<size_t>(shape[0])*shape[1]*shape[2];
    const size_t words=(cells+31)/32;
    const size_t ray_values=static_cast<size_t>(count)*3;
    try { workspace.host.resize(words); }
    catch (...) { return cudaErrorMemoryAllocation; }
    cudaError_t status=cudaSuccess;
    if (!workspace.stream) status=cudaStreamCreateWithFlags(&workspace.stream,cudaStreamNonBlocking);
    if (status==cudaSuccess&&!on_device) status=reserve(workspace.rays,workspace.ray_capacity,ray_values);
    if (status==cudaSuccess) status=reserve(workspace.mask,workspace.mask_capacity,words);
    if (status!=cudaSuccess) return status;

    const auto stream=workspace.stream;
    if (!on_device) status=cudaMemcpyAsync(workspace.rays,rays,ray_values*sizeof(double),cudaMemcpyHostToDevice,stream);
    if (status==cudaSuccess) status=cudaMemsetAsync(workspace.mask,0,words*sizeof(unsigned),stream);
    if (status==cudaSuccess) {
        carve_mask<<<(count+127)/128,128,0,stream>>>(on_device?rays:workspace.rays,count,grid,workspace.mask);
        status=cudaGetLastError();
    }
    if (status==cudaSuccess)
        status=cudaMemcpyAsync(workspace.host.data(),workspace.mask,words*sizeof(unsigned),cudaMemcpyDeviceToHost,stream);
    const auto synchronized=cudaStreamSynchronize(stream);
    if (status!=cudaSuccess) return status;
    if (synchronized!=cudaSuccess) return synchronized;

    // Apply the completed mask only after success. The caller can use the
    // CPU backend on any CUDA error without a partially modified map.
    for (size_t word=0;word<words;++word) {
        unsigned bits=workspace.host[word];
        while (bits) {
            const size_t index=word*32+__builtin_ctz(bits);
            state[index]=1;
            bits &= bits-1;
        }
    }
    return 0;
}


extern "C" int carve_cuda(std::uint8_t* state,const std::int64_t* shape,const std::int64_t* origin,
                         double resolution,const double* position,const double* rays,int count) {
    return carve_cuda_impl(state,shape,origin,resolution,position,rays,count,false);
}

extern "C" int carve_cuda_device(std::uint8_t* state,const std::int64_t* shape,const std::int64_t* origin,
                                double resolution,const double* position,const double* rays,int count) {
    return carve_cuda_impl(state,shape,origin,resolution,position,rays,count,true);
}
