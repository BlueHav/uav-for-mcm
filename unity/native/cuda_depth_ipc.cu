#include <cuda_runtime.h>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <cmath>
#include <dlfcn.h>
#include <sys/file.h>
#include <unistd.h>

extern "C" cudaError_t cudaGraphicsGLRegisterImage(cudaGraphicsResource_t*,unsigned,unsigned,unsigned);
extern "C" cudaError_t cudaGLGetDevices(unsigned*,int*,unsigned,int);

namespace {
constexpr int width=480, height=270, cameras=30;
constexpr size_t row_bytes=width*sizeof(float);
constexpr size_t frame_bytes=row_bytes*height;
struct Submission {
    int pending, camera;
    uintptr_t texture, render_target;
    uint64_t stamp_ns;
    float pose[7];
};
struct Header {
    uint64_t stamp_ns;
    int32_t pid;
    float pose[7];
    cudaIpcMemHandle_t memory;
};
struct Camera {
    unsigned texture=0;
    cudaGraphicsResource_t resource=nullptr;
    float* pixels=nullptr;
    cudaIpcMemHandle_t memory{};
    int fd=-1;
};
Camera sources[cameras];
int renderer_device=-1;
cudaStream_t capture_stream=nullptr;

bool path_for(char* path,size_t size,int camera) {
    const char* dir=std::getenv("YOPO_CUDA_DEPTH_IPC_DIR");
    if (!dir||!*dir||camera<0||camera>=cameras) return false;
    int n=std::snprintf(path,size,"%s/camera-%02d",dir,camera);
    return n>0&&static_cast<size_t>(n)<size;
}

bool prepare(Camera& state,const Submission& frame) {
    if (renderer_device<0) {
        unsigned count=0;
        int device=-1;
        if (cudaGLGetDevices(&count,&device,1,1)!=cudaSuccess||count!=1) return false;
        if (cudaSetDevice(device)!=cudaSuccess||
            cudaStreamCreateWithFlags(&capture_stream,cudaStreamNonBlocking)!=cudaSuccess) return false;
        renderer_device=device;
    }
    if (state.texture!=frame.texture) {
        if (state.resource) {
            if (cudaGraphicsUnregisterResource(state.resource)!=cudaSuccess) return false;
            state.resource=nullptr;
        }
        if (cudaGraphicsGLRegisterImage(&state.resource,frame.texture,0x0de1,1)!=cudaSuccess) return false;
        state.texture=frame.texture;
    }
    if (!state.pixels) {
        if (cudaMalloc(&state.pixels,frame_bytes)!=cudaSuccess) return false;
        if (cudaIpcGetMemHandle(&state.memory,state.pixels)!=cudaSuccess) {
            cudaFree(state.pixels);state.pixels=nullptr;return false;
        }
    }
    if (state.fd<0) {
        char path[1024];
        if (!path_for(path,sizeof(path),frame.camera)) return false;
        state.fd=open(path,O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
    }
    return state.fd>=0;
}

constexpr int sample_columns=40, sample_rows=23;
__global__ void gather_depth(const float* image,float* values) {
    const int i=blockDim.x*blockIdx.x+threadIdx.x;
    if (i>=sample_columns*sample_rows) return;
    const int x=12*(i%sample_columns), y=height-1-12*(i/sample_columns);
    values[i]=image[y*width+x];
}

void compare_render_target(const Submission& frame,const Camera& state) {
    static const bool enabled=std::getenv("YOPO_CUDA_DEPTH_PARITY")!=nullptr;
    if (!enabled||frame.camera<18||frame.camera>23) return;
    using GetTextureSubImage=void(*)(unsigned,int,int,int,int,int,int,int,unsigned,unsigned,int,void*);
    static auto get_texture=reinterpret_cast<GetTextureSubImage>(dlsym(RTLD_DEFAULT,"glGetTextureSubImage"));
    if (!get_texture) {
        static void* gl=dlopen("libGL.so.1",RTLD_LAZY|RTLD_LOCAL);
        if(gl)get_texture=reinterpret_cast<GetTextureSubImage>(dlsym(gl,"glGetTextureSubImage"));
    }
    if (!get_texture||!frame.render_target) return;
    static float* samples=nullptr;
    constexpr int count=sample_columns*sample_rows;
    if (!samples&&cudaMalloc(&samples,count*sizeof(float))!=cudaSuccess) return;
    gather_depth<<<(count+127)/128,128,0,capture_stream>>>(state.pixels,samples);
    float values[count];
    if (cudaMemcpyAsync(values,samples,sizeof(values),cudaMemcpyDeviceToHost,capture_stream)!=cudaSuccess||
        cudaStreamSynchronize(capture_stream)!=cudaSuccess) return;
    int closest=-1;
    for (int i=0;i<count;++i)
        if (std::isfinite(values[i])&&values[i]>.05f&&(closest<0||values[i]<values[closest])) closest=i;
    static uint64_t last[cameras]{};
    if (closest<0||(values[closest]>=1.5f&&frame.stamp_ns-last[frame.camera]<1000000000UL)) return;
    last[frame.camera]=frame.stamp_ns;
    const int x=12*(closest%sample_columns), y=height-1-12*(closest/sample_columns);
    float original=NAN,snapshot=NAN;
    get_texture(frame.render_target,0,x,y,0,1,1,1,0x1903,0x1406,sizeof(float),&original);
    get_texture(frame.texture,0,x,y,0,1,1,1,0x1903,0x1406,sizeof(float),&snapshot);
    std::fprintf(stderr,"CUDA_DEPTH_PARITY camera=%d stamp_ns=%llu x=%d y=%d unity=%.8g snapshot=%.8g cuda=%.8g\n",
        frame.camera,static_cast<unsigned long long>(frame.stamp_ns),x,height-1-y,original,snapshot,values[closest]);
    std::fflush(stderr);
}

void capture(int,void* data) {
    auto& frame=*static_cast<Submission*>(data);
    if (frame.camera>=0&&frame.camera<cameras&&frame.texture) {
        auto& state=sources[frame.camera];
        if (prepare(state,frame)&&flock(state.fd,LOCK_EX|LOCK_NB)==0) {
            auto status=cudaGraphicsMapResources(1,&state.resource,capture_stream);
            if (status==cudaSuccess) {
                cudaArray_t array=nullptr;
                status=cudaGraphicsSubResourceGetMappedArray(&array,state.resource,0,0);
                if (status==cudaSuccess)
                    status=cudaMemcpy2DFromArrayAsync(state.pixels,row_bytes,array,0,0,row_bytes,height,
                                                      cudaMemcpyDeviceToDevice,capture_stream);
                const auto unmapped=cudaGraphicsUnmapResources(1,&state.resource,capture_stream);
                auto finished=cudaStreamSynchronize(capture_stream);
                if (status==cudaSuccess&&unmapped==cudaSuccess&&finished==cudaSuccess) {
                    compare_render_target(frame,state);
                    Header header{};
                    header.stamp_ns=frame.stamp_ns;
                    header.pid=getpid();
                    for (int i=0;i<7;++i) header.pose[i]=frame.pose[i];
                    header.memory=state.memory;
                    if (pwrite(state.fd,&header,sizeof(header),0)!=sizeof(header)) ftruncate(state.fd,0);
                } else {
                    // Readers must not associate a partially copied texture with an older timestamp.
                    ftruncate(state.fd,0);
                }
            }
            flock(state.fd,LOCK_UN);
        }
    }
    __atomic_store_n(&frame.pending,0,__ATOMIC_RELEASE);
}
}

extern "C" void* yopo_depth_capture_callback() { return reinterpret_cast<void*>(&capture); }

struct Reader {
    int fd=-1;
    int pid=-1;
    float* memory=nullptr;
    cudaStream_t stream=nullptr;
};

extern "C" void* yopo_depth_open(int camera) {
    char path[1024];
    if (!path_for(path,sizeof(path),camera)) return nullptr;
    int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
    if (fd<0) return nullptr;
    auto* reader=new Reader;
    reader->fd=fd;
    if (cudaStreamCreateWithFlags(&reader->stream,cudaStreamNonBlocking)!=cudaSuccess) {
        close(fd);delete reader;return nullptr;
    }
    return reader;
}

extern "C" int yopo_depth_read(void* source,float* output,uint64_t* stamp,float* pose) {
    auto& reader=*static_cast<Reader*>(source);
    if (flock(reader.fd,LOCK_SH|LOCK_NB)!=0) return 0;
    Header header{};
    int result=0;
    if (pread(reader.fd,&header,sizeof(header),0)==sizeof(header)&&header.stamp_ns>*stamp) {
        if (reader.pid!=header.pid||!reader.memory) {
            if (reader.memory) cudaIpcCloseMemHandle(reader.memory);
            reader.memory=nullptr;
            auto error=cudaIpcOpenMemHandle(reinterpret_cast<void**>(&reader.memory),header.memory,cudaIpcMemLazyEnablePeerAccess);
            reader.pid=error==cudaSuccess?header.pid:-1;
        }
        if (reader.memory&&cudaMemcpyAsync(output,reader.memory,frame_bytes,cudaMemcpyDeviceToDevice,reader.stream)==cudaSuccess
            &&cudaStreamSynchronize(reader.stream)==cudaSuccess) {
            *stamp=header.stamp_ns;
            for (int i=0;i<7;++i) pose[i]=header.pose[i];
            result=1;
        }
    }
    flock(reader.fd,LOCK_UN);
    return result;
}

extern "C" void yopo_depth_close(void* source) {
    if (!source) return;
    auto* reader=static_cast<Reader*>(source);
    if (reader->memory) cudaIpcCloseMemHandle(reader->memory);
    cudaStreamDestroy(reader->stream);
    close(reader->fd);
    delete reader;
}
