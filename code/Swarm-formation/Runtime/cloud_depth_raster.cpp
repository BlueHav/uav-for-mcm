// Server-only build:
// g++ -O3 -std=c++11 -shared -fPIC cloud_depth_raster.cpp -o cloud_depth_raster_native.so
// Depth-only CPU projection of the exact same oriented triangle mesh as Open3D.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace {
struct Vec { double x, y, z; };
Vec sub(Vec a, Vec b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec cross(Vec a, Vec b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
double dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
struct Triangle { Vec a, b, c, normal, low, high; };
struct Scene { std::vector<Triangle> triangles; };
bool finite(Vec a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
Vec transform(Vec v, Vec origin, const float* r) {
    v=sub(v, origin);
    return {v.x*r[0]+v.y*r[3]+v.z*r[6], v.x*r[1]+v.y*r[4]+v.z*r[7], v.x*r[2]+v.y*r[5]+v.z*r[8]};
}
struct Plane { double x,y,z,w; };
double distance(Plane p, Vec v) { return p.x*v.x+p.y*v.y+p.z*v.z+p.w; }
int clip(const Vec* input, int count, Vec* output, Plane plane) {
    if (!count) return 0;
    int size=0;
    Vec previous=input[count-1]; double dp=distance(plane,previous);
    for (int i=0;i<count;++i) {
        Vec current=input[i]; double dc=distance(plane,current);
        if ((dp>=0)!=(dc>=0)) {
            double t=dp/(dp-dc);
            output[size++]={previous.x+t*(current.x-previous.x), previous.y+t*(current.y-previous.y), previous.z+t*(current.z-previous.z)};
        }
        if (dc>=0) output[size++]=current;
        previous=current; dp=dc;
    }
    return size;
}
struct Projected { double u,v,inv; };
double edge(Projected a, Projected b, double u, double v) { return (b.u-a.u)*(v-a.v)-(b.v-a.v)*(u-a.u); }
void raster(Vec a, Vec b, Vec c, double fx, double fy, double cx, double cy,
            int width, int height, double near_clip, double far_clip, float* output) {
    if(!finite(a) || !finite(b) || !finite(c) || a.x<=0 || b.x<=0 || c.x<=0) return;
    Projected p[3]={{cx-fx*a.y/a.x,cy-fy*a.z/a.x,1/a.x},
                    {cx-fx*b.y/b.x,cy-fy*b.z/b.x,1/b.x},
                    {cx-fx*c.y/c.x,cy-fy*c.z/c.x,1/c.x}};
    for(const Projected& point : p) if(!std::isfinite(point.u) || !std::isfinite(point.v) || !std::isfinite(point.inv)) return;
    double area=edge(p[0],p[1],p[2].u,p[2].v);
    if (std::abs(area)<1e-12) return;
    if (area<0) { std::swap(p[1],p[2]); area=-area; }
    double xmin=std::max(0.,std::min({p[0].u,p[1].u,p[2].u})-.5);
    double xmax=std::min(double(width-1),std::max({p[0].u,p[1].u,p[2].u})-.5);
    double ymin=std::max(0.,std::min({p[0].v,p[1].v,p[2].v})-.5);
    double ymax=std::min(double(height-1),std::max({p[0].v,p[1].v,p[2].v})-.5);
    if(xmax<xmin || ymax<ymin) return;
    int x0=int(std::ceil(xmin)),x1=int(std::floor(xmax));
    int y0=int(std::ceil(ymin)),y1=int(std::floor(ymax));
    double inv_area=1/area;
    double du0=-(p[2].v-p[1].v),du1=-(p[0].v-p[2].v),du2=-(p[1].v-p[0].v);
    // A small roundoff allowance covers shared edges without geometric dilation.
    double tolerance=area*1e-12;
    for(int y=y0;y<=y1;++y) {
        double e0=edge(p[1],p[2],x0+.5,y+.5);
        double e1=edge(p[2],p[0],x0+.5,y+.5);
        double e2=edge(p[0],p[1],x0+.5,y+.5);
        for(int x=x0;x<=x1;++x,e0+=du0,e1+=du1,e2+=du2) {
            if(e0 < -tolerance || e1 < -tolerance || e2 < -tolerance) continue;
            double reciprocal=(e0*p[0].inv+e1*p[1].inv+e2*p[2].inv)*inv_area;
            if(reciprocal<=0) continue;
            double depth=1/reciprocal;
            if(depth >= near_clip-1e-9 && depth <= far_clip+1e-9) {
                float value=static_cast<float>(depth);
                float& old=output[y*width+x];
                if(value<old) old=value;
            }
        }
    }
}
}

extern "C" int cloud_depth_raster_abi() { return 1; }

extern "C" void* cloud_depth_raster_create(const float* vertices, int64_t vertex_count,
                                            const uint32_t* indices, int64_t triangle_count) {
    if(!vertices || !indices || vertex_count<3 || vertex_count>10000000 || triangle_count<1 || triangle_count>20000000) return nullptr;
    try {
        std::unique_ptr<Scene> scene(new Scene);
        scene->triangles.reserve(triangle_count);
        for(int64_t i=0;i<triangle_count;++i) {
            Vec v[3];
            for(int j=0;j<3;++j) {
                uint32_t index=indices[3*i+j]; if(index>=vertex_count) return nullptr;
                v[j]={vertices[3*index],vertices[3*index+1],vertices[3*index+2]};
                if(!finite(v[j])) return nullptr;
            }
            Triangle t; t.a=v[0];t.b=v[1];t.c=v[2];t.normal=cross(sub(v[1],v[0]),sub(v[2],v[0]));
            t.low={std::min({v[0].x,v[1].x,v[2].x}),std::min({v[0].y,v[1].y,v[2].y}),std::min({v[0].z,v[1].z,v[2].z})};
            t.high={std::max({v[0].x,v[1].x,v[2].x}),std::max({v[0].y,v[1].y,v[2].y}),std::max({v[0].z,v[1].z,v[2].z})};
            scene->triangles.push_back(t);
        }
        return scene.release();
    } catch(...) { return nullptr; }
}

extern "C" void cloud_depth_raster_destroy(void* pointer) { delete static_cast<Scene*>(pointer); }

extern "C" int cloud_depth_raster_render(void* pointer, const float* position, const float* rotation,
    int width,int height,double fx,double fy,double cx,double cy,double near_clip,double far_clip,float* output) {
    if(!pointer || !position || !rotation || !output || width<1 || height<1 || width>8192 || height>8192
       || !std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy)
       || fx<=0 || fy<=0 || cx<0 || cy<0 || cx>width || cy>height
       || !std::isfinite(near_clip) || !std::isfinite(far_clip) || near_clip<=0 || far_clip<=near_clip) return -1;
    Vec origin={position[0],position[1],position[2]}; if(!finite(origin)) return -2;
    for(int i=0;i<9;++i) if(!std::isfinite(rotation[i])) return -2;
    std::fill(output,output+int64_t(width)*height,std::numeric_limits<float>::infinity());
    Vec lo={1e300,1e300,1e300},hi={-1e300,-1e300,-1e300};
    // Conservative world AABB of the complete camera frustum. This inexpensive
    // test discards distant terrain before per-triangle clipping/projection.
    for(double x : {near_clip,far_clip}) for(double u : {0.,double(width)}) for(double v : {0.,double(height)}) {
        double y=(cx-u)*x/fx,z=(cy-v)*x/fy;
        Vec p={origin.x+x*rotation[0]+y*rotation[1]+z*rotation[2],
               origin.y+x*rotation[3]+y*rotation[4]+z*rotation[5],
               origin.z+x*rotation[6]+y*rotation[7]+z*rotation[8]};
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    Plane planes[6]={{1,0,0,-near_clip},{-1,0,0,far_clip},
                     {cx,-fx,0,0},{width-cx,fx,0,0},{cy,0,-fy,0},{height-cy,0,fy,0}};
    const Scene* scene=static_cast<Scene*>(pointer);
    for(const Triangle& t : scene->triangles) {
        if(t.high.x<lo.x || t.low.x>hi.x || t.high.y<lo.y || t.low.y>hi.y || t.high.z<lo.z || t.low.z>hi.z) continue;
        if(dot(t.normal,sub(origin,t.a))<=0) continue;
        Vec a[16],b[16];
        a[0]=transform(t.a,origin,rotation);a[1]=transform(t.b,origin,rotation);a[2]=transform(t.c,origin,rotation);
        int count=3;Vec* input=a;Vec* other=b;
        for(Plane plane : planes) {
            count=clip(input,count,other,plane);std::swap(input,other);
            if(count<3) break;
        }
        if(count<3) continue;
        for(int i=1;i<count-1;++i) raster(input[0],input[i],input[i+1],fx,fy,cx,cy,width,height,near_clip,far_clip,output);
    }
    return 0;
}
