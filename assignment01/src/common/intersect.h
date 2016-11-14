#ifndef _INTERSECT_H_
#define _INTERSECT_H_

#include "scene.h"

// intersection record
struct intersection3f {
    bool        hit;        // whether it hits something
    float       ray_t;      // ray parameter for the hit
    vec3f       pos;        // hit position
    vec3f       norm;       // hit normal
    vec2f       texcoord;   // hit texture coordinates
    Material*   mat;        // hit material
    
    // constructor (defaults to no intersection)
    intersection3f() : hit(false) { }
    
    // constructor to override default intersection
    explicit intersection3f(bool hit) : hit(hit) { }
};

#define ray3f_epsilon 0.0005f
#define ray3f_rayinf 1000000.0f

// 3D Ray
struct ray3f {
    vec3f e;        // origin
    vec3f d;        // direction
    float tmin;     // min t value
    float tmax;     // max t value
    
    // Default constructor
    ray3f() : e(zero3f), d(z3f), tmin(ray3f_epsilon), tmax(ray3f_rayinf) { }
    
    // Element-wise constructor
    ray3f(const vec3f& e, const vec3f& d) :
    e(e), d(d), tmin(ray3f_epsilon), tmax(ray3f_rayinf) { }
    
    // Element-wise constructor
    ray3f(const vec3f& e, const vec3f& d, float tmin, float tmax) :
    e(e), d(d), tmin(tmin), tmax(tmax) { }
    
    // Eval ray at a specific t
    vec3f eval(float t) const { return e + d * t; }
    
    // Create a ray from point a to point b
    static ray3f make_segment(const vec3f& a, const vec3f& b) { return ray3f(a,normalize(b-a),ray3f_epsilon,dist(a,b)-2*ray3f_epsilon); }
};

// transform a ray by a frame
inline ray3f transform_ray(const frame3f& f, const ray3f& v) { return ray3f(transform_point(f,v.e), transform_vector(f,v.d), v.tmin, v.tmax); }
inline ray3f transform_ray_from_local(const frame3f& f, const ray3f& v) { return ray3f(transform_point(f,v.e), transform_vector(f,v.d), v.tmin, v.tmax); }

// transform a ray by a frame inverse
inline ray3f transform_ray_inverse(const frame3f& f, const ray3f& v) { return ray3f(transform_point_inverse(f,v.e),transform_vector_inverse(f,v.d),v.tmin,v.tmax); }
inline ray3f transform_ray_to_local(const frame3f& f, const ray3f& v) { return ray3f(transform_point_inverse(f,v.e),transform_vector_inverse(f,v.d),v.tmin,v.tmax); }

// prepare scene acceleration and triangulate meshes
void accelerate(Scene* scene);

// intersects the scene and return the first intrerseciton
intersection3f intersect(Scene* scene, ray3f ray);

// intersects the scene and return any intrerseciton
bool intersect_shadow(Scene* scene, ray3f ray);


//////////////////////////////
#include <algorithm>

#define BVHAccelerator_min_prims 4
#define BVHAccelerator_epsilon ray3f_epsilon
#define BVHAccelerator_build_maxaxis false

// bvh accelerator node
struct BVHNode {
    bool leaf;      // leaf node
    range3f bbox;   // bounding box
#ifndef _WIN32
    union {
        struct { int start, end; }; // for leaves: start and end primitive
        struct { int n0, n1; };     // for internal: left and right node
    };
#else
    int start, end; // for leaves: start and end primitive
    int n0, n1;     // for internal: left and right node
#endif
};

// bvh accelerator
struct BVHAccelerator {
    vector<int>     prims;  // sorted primitices
    vector<BVHNode> nodes;  // bvh nodes
};

// split the list of nodes according to a policy
int make_accelerator_split_kd_tree(vector<pair<range3f,int>>& boxed_prims, int start, int end, int depth = 0) {
    unsigned int axis = depth % 3;
    
    auto mid = (start+end)/2;
    std::sort(boxed_prims.begin()+start,boxed_prims.begin()+end,
              [axis](const pair<range3f,int>& i, const pair<range3f,int>& j) {
                  return center(i.first)[axis] < center(j.first)[axis]; });
    return mid;
}

// split the list of nodes according to a policy
int make_accelerator_split(vector<pair<range3f,int>>& boxed_prims, int start, int end, const range3f& bbox, bool maxaxis) {
    auto axis = 0;
    if(maxaxis) {
        auto s = size(bbox);
        if(s.x >= s.y and s.x >= s.z) axis = 0;
        else if (s.y >= s.x and s.y >= s.z) axis = 1;
        else axis = 2;
    } else {
        auto d = zero3f;
        for(auto a : range(3)) {
            auto mid = (start+end) / 2;
            std::sort(boxed_prims.begin()+start,boxed_prims.begin()+end,
                      [a](const pair<range3f,int>& i, const pair<range3f,int>& j) {
                          return center(i.first)[a] < center(j.first)[a]; });
            auto bbox0 = range3f(), bbox1 = range3f();
            for(auto i : range(start,mid)) bbox0 = runion(bbox0, boxed_prims[i].first);
            for(auto i : range(mid,end)) bbox1 = runion(bbox1, boxed_prims[i].first);
            auto s0 = size(bbox0), s1 = size(bbox1);
            d[a] = s0.x+s0.y+s0.z+s1.x+s1.y+s1.z;
        }
        if(d.x <= d.y and d.x <= d.z) axis = 0;
        else if(d.y <= d.x and d.y <= d.z) axis = 1;
        else axis = 2;
    }
    auto mid = (start+end)/2;
    std::sort(boxed_prims.begin()+start,boxed_prims.begin()+end,
              [axis](const pair<range3f,int>& i, const pair<range3f,int>& j) {
                  return center(i.first)[axis] < center(j.first)[axis]; });
    return mid;
}

// recursively add a node to an accelerator using KD tree
void make_accelerator_node_kd_tree(int nodeid,
                                   vector<pair<range3f,int>>& boxed_prims,
                                   vector<BVHNode>& nodes,
                                   int start, int end, int depth=0)
{
    range3f bbox;
    auto node = BVHNode();
    for(auto i : range(start, end)) bbox = runion(bbox,boxed_prims[i].first);
    if(end-start <= BVHAccelerator_min_prims) {
        node.bbox = bbox;
        node.leaf = true;
        node.start = start;
        node.end = end;
    } else {
        int middle = make_accelerator_split_kd_tree(boxed_prims,start,end, depth);
        node.bbox = bbox;
        node.leaf = false;
        nodes.push_back(BVHNode());
        node.n0 = nodes.size();
        nodes.push_back(BVHNode());
        node.n1 = nodes.size();
        nodes.push_back(BVHNode());
        make_accelerator_node_kd_tree(node.n0,boxed_prims,nodes,start,middle, depth+1);
        make_accelerator_node_kd_tree(node.n1,boxed_prims,nodes,middle,end, depth+1);
    }
    nodes[nodeid] = node;
}

// recursively add a node to an accelerator
void make_accelerator_node(int nodeid,
                           vector<pair<range3f,int>>& boxed_prims,
                           vector<BVHNode>& nodes,
                           int start, int end) {
    range3f bbox;
    auto node = BVHNode();
    for(auto i : range(start, end)) bbox = runion(bbox,boxed_prims[i].first);
    if(end-start <= BVHAccelerator_min_prims) {
        node.bbox = bbox;
        node.leaf = true;
        node.start = start;
        node.end = end;
    } else {
        int middle = make_accelerator_split(boxed_prims,start,end,bbox,BVHAccelerator_build_maxaxis);
        node.bbox = bbox;
        node.leaf = false;
        nodes.push_back(BVHNode());
        node.n0 = nodes.size();
        nodes.push_back(BVHNode());
        node.n1 = nodes.size();
        nodes.push_back(BVHNode());
        make_accelerator_node(node.n0,boxed_prims,nodes,start,middle);
        make_accelerator_node(node.n1,boxed_prims,nodes,middle,end);
    }
    nodes[nodeid] = node;
}

// intersect bounding box
inline bool intersect_bbox(const ray3f& ray, const range3f& bbox, float& t0, float& t1) {
    t0 = ray.tmin; t1 = ray.tmax;
    for (int i = 0; i < 3; ++i) {
        auto invRayDir = 1.f / ray.d[i];
        auto tNear = (bbox.min[i] - ray.e[i]) * invRayDir;
        auto tFar  = (bbox.max[i] - ray.e[i]) * invRayDir;
        if (tNear > tFar) std::swap(tNear, tFar);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar  < t1 ? tFar  : t1;
        if (t0 > t1) return false;
    }
    return true;
}

// intersect bounding box without returning bounds
inline bool intersect_bbox(const ray3f& ray, const range3f& bbox) {
    float t0, t1; return intersect_bbox(ray,bbox,t0,t1);
}

// intersect triangle
inline bool intersect_triangle(const ray3f& ray, const vec3f& v0, const vec3f& v1, const vec3f& v2, float& t, float& u, float& v) {
    //put_your_code_here("Implement triangle-ray intersection here");
//    auto p = ray.e;
//    auto d = ray.d;
//    auto n = normalize(cross((v1-v0),(v2-v1)));
//    auto t = dot((v0-p),n)/(dot(d,n));
//    if(t>ray.tmax or t<ray.tmin) return false;
//    auto x = ray.eval(t);
//    if(dot(cross((b-a),(x-a)),n)<0 or
//       dot(cross((c-b),(x-b)),n)<0 or
//       dot(cross((a-c),(x-c)),n)<0) return false;
//    return true;

    auto a = v0-v2;
    auto b = v1-v2;
    auto d = ray.d;
    auto e = ray.e - v2;
    auto denominator = dot(cross(d,b),a);
    if (denominator ==0) return false;
    t = dot(cross(e,a),b)/denominator;
    u = dot(cross(d,b),e)/denominator;
    v = dot(cross(e,a),d)/denominator;
    if (t>ray.tmax or t<ray.tmin) return false;
    if (u<0 or v<0 or u+v>1) return false;
    return true;
}

inline bool intersect_triangle(const ray3f& ray, const vec3f& v0, const vec3f& v1, const vec3f& v2){
    float t,u,v;
    return intersect_triangle(ray,v0,v1,v2,t,u,v);
}

// intersect sphere
inline bool intersect_sphere(const ray3f& ray, float radius, float& t) {
    auto a = lengthSqr(ray.d);
    auto b = 2*dot(ray.d,ray.e);
    auto c = lengthSqr(ray.e) - radius*radius;
    auto d = b*b-4*a*c;
    if(d < 0) return false;
    float tm = (-b-sqrt(d)) / (2*a);
    float tM = (-b+sqrt(d)) / (2*a);
    bool hitm = (tm >= ray.tmin and tm <= ray.tmax);
    bool hitM = (tM >= ray.tmin and tM <= ray.tmax);
    if(!hitm and !hitM) return false;
    t = hitm ? tm : tM;
    return true;
}

// intersect sphere without returning values
inline bool intersect_sphere(const ray3f& ray, float radius) {
    float t; return intersect_sphere(ray, radius, t);
}

// intersect quad
inline bool intersect_quad(const ray3f& ray, float radius, float& t, vec3f& p) {
    if(ray.d.z == 0) return false;
    t = - ray.e.z / ray.d.z;
    p = ray.eval(t);
    if(radius < p.x or -radius > p.x or radius < p.y or -radius > p.y) return false;
    if (t < ray.tmin or t > ray.tmax) return false;
    return true;
}

// intersect cylinder
inline bool intersect_cylinder(const ray3f& ray, float radius, float height, float& t, vec3f& norm){
    t = ray.tmax;
    
    //intersect with cylinder
    auto a = ray.d.x*ray.d.x+ray.d.y*ray.d.y;
    auto b = 2*ray.e.x*ray.d.x+2*ray.e.y*ray.d.y;
    auto c = ray.e.x*ray.e.x+ray.e.y*ray.e.y-radius*radius;
    float d = b*b-4*a*c;
    if(d>=0){
        auto t1 = (-b-sqrt(d))/(2*a);
        auto z1 = ray.e.z+t1*ray.d.z;
        if (t1>ray.tmin && t1<ray.tmax && z1<=height && z1>=-height){
            if (t1<t){
                t=t1;
                norm = normalize(ray.eval(t1)-vec3f(0,0,z1));
            }
        }
        else{
            auto t2 = (-b+sqrt(d))/(2*a);
            auto z2 = ray.e.z+t2*ray.d.z;
            if(t2>ray.tmin && t2<ray.tmax && z2<=height && z2>=-height){
                if(t2<t){
                    t = t2;
                    norm = normalize(ray.eval(t2)-vec3f(0,0,z2));
                }
            }
        }
    }

    //intersect with top cap
    vec3f top_o = vec3f(0,0,height);
    auto top_t =dot((top_o-ray.e),vec3f(0,0,1))/dot(ray.d,vec3f(0,0,1));
    if (top_t>ray.tmin && top_t<ray.tmax){
        vec3f sur_point = ray.eval(top_t);
        auto tmp_radius = length(sur_point-top_o);
        if (tmp_radius<=radius && fabs(sur_point.z-height) < 0.000001){
            if(top_t<t){
                t =top_t;
                norm =vec3f(0,0,1.0);
            }
        }
    }

    //intersect with bottom cap
    vec3f bot_o = vec3f(0,0,height);
    auto bot_t =dot((bot_o-ray.e),vec3f(0,0,-1))/dot(ray.d,vec3f(0,0,-1));
    if (bot_t>ray.tmin && bot_t<ray.tmax){
        vec3f sur_point = ray.eval(bot_t);
        auto tmp_radius = length(sur_point-bot_o);
        if (tmp_radius<=radius && fabs(sur_point.z+height) < 0.000001){
            if(bot_t<t){
                t =bot_t;
                norm =-vec3f(0,0,1.0);
            }
        }
    }

    if( t<ray.tmax) return true;
    return false;
}

inline bool intersect_cylinder(const ray3f& ray, float radius, float height){
    float t; vec3f norm; return intersect_cylinder(ray,radius,height,t,norm);
}

// intersect triangle without returning bounds
inline bool intersect_quad(const ray3f& ray, float radius) {
    float t; vec3f p; return intersect_quad(ray, radius, t, p);
}

// intersect an accelerator
template<typename intersect_func>
intersection3f intersect(BVHAccelerator* bvh, int nodeid, const ray3f& ray,
                         const intersect_func& intersect_elem) {
    // grab node
    auto& node = bvh->nodes[nodeid];
    // intersect bbox
    if(not intersect_bbox(ray, node.bbox)) return intersection3f();
    // recursively intersect nodes
    intersection3f intersection;
    // copy the ray to allow for shortening it
    auto sray = ray;
    if(node.leaf) {
        for(int idx = node.start; idx < node.end; idx ++) {
            auto i = bvh->prims[idx];
            intersection3f sintersection = intersect_elem(i,sray);
            if(not sintersection.hit) continue;
            if(sintersection.ray_t > intersection.ray_t and intersection.hit) continue;
            intersection = sintersection;
            sray.tmax = intersection.ray_t;
        }
    } else {
        for(auto n : { node.n0, node.n1 }) {
            intersection3f sintersection = intersect(bvh,n,sray,intersect_elem);
            if(not sintersection.hit) continue;
            if(sintersection.ray_t > intersection.ray_t and intersection.hit) continue;
            intersection = sintersection;
            sray.tmax = intersection.ray_t;
        }
    }
    return intersection;
}

// intersect an accelerator
template<typename intersect_func>
bool intersect_shadow(BVHAccelerator* bvh, int nodeid, const ray3f& ray,
                      const intersect_func& intersect_elem_shadow) {
    // grab node
    auto& node = bvh->nodes[nodeid];
    // intersect bbox
    if(not intersect_bbox(ray, node.bbox)) return false;
    // recursively intersect nodes
    if(node.leaf) {
        // for(auto idx : range(node.start,node.end)) {
        for(int idx = node.start; idx < node.end; idx++) {
            auto i = bvh->prims[idx];
            if(intersect_elem_shadow(i,ray)) return true;
        }
    } else {
        if(intersect_shadow(bvh,node.n0,ray,intersect_elem_shadow)) return true;
        if(intersect_shadow(bvh,node.n1,ray,intersect_elem_shadow)) return true;
    }
    return false;
}

// build accelerator
BVHAccelerator* make_accelerator(vector<range3f>& bboxes) {
    vector<pair<range3f,int>> boxed_prims(bboxes.size());
    for(auto i : range(bboxes.size())) boxed_prims[i] = pair<range3f,int>(rscale(bboxes[i],1+BVHAccelerator_epsilon),i);
    auto bvh = new BVHAccelerator();
    bvh->nodes.push_back(BVHNode());
    make_accelerator_node(0, boxed_prims, bvh->nodes, 0, bboxes.size());
    bvh->prims.reserve(bboxes.size());
    for(auto i : range(boxed_prims.size())) bvh->prims[i] = boxed_prims[i].second;
    return bvh;
}

// intersects the scene and return the first intrerseciton
intersection3f intersect(Scene* scene, ray3f ray) {
    // create a default intersection record to be returned
    auto intersection = intersection3f();
    // foreach surface
    for(auto surface : scene->surfaces) {
        // if it is a quad
        if(surface->isquad) {
            // compute ray intersection (and ray parameter), continue if not hit
            auto tray = transform_ray_inverse(surface->frame,ray);
            
            // intersect quad
            auto t = 0.0f; auto p = zero3f;
            auto hit = intersect_quad(tray, surface->radius, t, p);
            
            // skip if not hit
            if(not hit) continue;
            
            // check if this is the closest intersection, continue if not
            if(t > intersection.ray_t and intersection.hit) continue;
            
            // if hit, set intersection record values
            intersection.hit = true;
            intersection.ray_t = t;
            intersection.pos = transform_point(surface->frame,p);
            intersection.norm = transform_normal(surface->frame,z3f);
            intersection.texcoord = {0.5f*p.x/surface->radius+0.5f,0.5f*p.y/surface->radius+0.5f};
            intersection.mat = surface->mat;
        }
        else if(surface->iscylinder){
            auto tray = transform_ray_inverse(surface->frame,ray);
            auto t = 0.0f; auto norm=zero3f;
            auto hit = intersect_cylinder(tray,surface->radius, surface->height,t,norm);
            if(not hit) continue;
            if (t > intersection.ray_t and intersection.hit) continue;
            auto p = tray.eval(t);
            intersection.hit = true;
            intersection.ray_t = t;
            intersection.pos = transform_point(surface->frame,p);
            intersection.norm = transform_normal(surface->frame, norm);
            intersection.mat = surface->mat;
        }
        else {
            // compute ray intersection (and ray parameter), continue if not hit
            auto tray = transform_ray_inverse(surface->frame,ray);
            
            // intersect sphere
            auto t = 0.0f;
            auto hit = intersect_sphere(tray, surface->radius, t);
            
            // skip if not hit
            if(not hit) continue;
            
            // check if this is the closest intersection, continue if not
            if(t > intersection.ray_t and intersection.hit) continue;
            
            // compute local point and normal
            auto p = tray.eval(t);
            auto n = normalize(p);
            
            // if hit, set intersection record values
            intersection.hit = true;
            intersection.ray_t = t;
            intersection.pos = transform_point(surface->frame,p);
            intersection.norm = transform_normal(surface->frame,n);
            intersection.texcoord = {(pif+(float)atan2(n.y, n.x))/(2*pif),(float)acos(n.z)/pif};
            intersection.mat = surface->mat;
        }
    }
    // foreach mesh
    for(Mesh* mesh : scene->meshes) {
        // quads are not supported: check for error
        error_if_not(mesh->quad.empty(), "quad intersection is not supported");
        // tranform the ray
        auto tray = transform_ray_inverse(mesh->frame, ray);
        // save auto mesh intersection
        auto sintersection = intersection3f();
        // if it is accelerated
        if(mesh->bvh) {
            sintersection = intersect(mesh->bvh, 0, tray,
                                      [mesh](int tid, ray3f tray){
                                          put_your_code_here("Intersection test");
                                          // grab triangle
                                          auto triangle = mesh->triangle[tid];
                                          // grab vertices
                                          auto v0 = mesh->pos[triangle.x];
                                          auto v1 = mesh->pos[triangle.y];
                                          auto v2 = mesh->pos[triangle.z];
                                          // intersect triangle
                                          auto t = 0.0f, u=0.0f, v=0.0f;
                                          auto hit = intersect_triangle(tray,v0,v1,v2,t,u,v);
                                          // skip if not hit
                                          if(!hit) return intersection3f();
                                          // if hit, set up intersection, trasforming hit data to world space
                                          auto sintersection = intersection3f();
                                          sintersection.hit = true;
                                          sintersection.ray_t = t;
                                          sintersection.pos = tray.eval(t);
                                          sintersection.norm = normalize(cross((v1-v0),(v2-v1)));
                                          // if no texture, set texcoord to zero2f
                                          if(mesh->texcoord.empty()){
                                              sintersection.texcoord = zero2f;
                                          }
                                          // else, compute texcoord
                                          else{
                                              sintersection.texcoord = u*mesh->texcoord[triangle.x]+v*mesh->texcoord[triangle.y]+(1-u-v)*mesh->texcoord[triangle.z];
                                          }
                                          // set material
                                          sintersection.mat = mesh->mat;
                                          return sintersection;
                                      });
        } else {
            // clear intersection
            sintersection = intersection3f();
            //put_your_code_here("Loop over each triangle and do intersection test");
            // foreach triangle
            for (auto tri: mesh->triangle){
                // grab vertices
                auto v0 = mesh->pos[tri.x];
                auto v1 = mesh->pos[tri.y];
                auto v2 = mesh->pos[tri.z];
                // intersect triangle
                auto t = 0.0f, u=0.0f, v=0.0f;
                auto hit = intersect_triangle(tray,v0,v1,v2,t,u,v);
                // skip if not hit
                if (!hit) continue;
                // check if closer than the found hit
                if (t>sintersection.ray_t and sintersection.hit) continue;
                // if hit, set up intersection, trasforming hit data to world space
                sintersection.hit = true;
                sintersection.ray_t = t;
                sintersection.pos = tray.eval(t);
                sintersection.norm = normalize(cross((v1-v0),(v2-v1)));
                // if no texture, set texcoord to zero2f
                if(mesh->texcoord.empty()){
                    sintersection.texcoord = zero2f;
                }
                // else, compute texcoord
                else{
                    sintersection.texcoord = u*mesh->texcoord[tri.x]+v*mesh->texcoord[tri.y]+(1-u-v)*mesh->texcoord[tri.z];
                }
                // set material
                sintersection.mat = mesh->mat;
            }
        }
        // if did not hit the mesh, skip
        if(not sintersection.hit) continue;
        // check not first intersection, skip
        if(sintersection.ray_t > intersection.ray_t and intersection.hit) continue;
        // set interserction
        intersection = sintersection;
        // transform by mesh frame
        intersection.pos = transform_point(mesh->frame,sintersection.pos);
        intersection.norm = transform_normal(mesh->frame,sintersection.norm);
        // set material
        intersection.mat = sintersection.mat;
    }
    
    // record closest intersection
    return intersection;
}

// intersects the scene and return for any intersection
bool intersect_shadow(Scene* scene, ray3f ray) {
    // foreach surface
    for(auto surface : scene->surfaces) {
        // if it is a quad
        if(surface->isquad) {
            // compute ray intersection (and ray parameter), continue if not hit
            auto tray = transform_ray_inverse(surface->frame,ray);
            
            // intersect quad
            if(intersect_quad(tray, surface->radius)) return true;
        }
        else if(surface->iscylinder){
            auto tray = transform_ray_inverse(surface->frame,ray);
            if(intersect_cylinder(tray, surface->radius, surface->height)) return true;
        }
        else {
            // compute ray intersection (and ray parameter), continue if not hit
            auto tray = transform_ray_inverse(surface->frame,ray);
            
            // intersect sphere
            if(intersect_sphere(tray, surface->radius)) return true;
        }
    }
    // foreach mesh
    for(auto mesh : scene->meshes) {
        // quads are not supported: check for error
        error_if_not(mesh->quad.empty(), "quad intersection is not supported");
        // tranform the ray
        auto tray = transform_ray_inverse(mesh->frame, ray);
        // if it is accelerated
        if(mesh->bvh) {
            if(intersect_shadow(mesh->bvh, 0, tray,
                                [mesh](int tid, ray3f tray){
                                    // grab triangle
                                    auto triangle = mesh->triangle[tid];
                                    
                                    // grab vertices
                                    auto v0 = mesh->pos[triangle.x];
                                    auto v1 = mesh->pos[triangle.y];
                                    auto v2 = mesh->pos[triangle.z];
                                    
                                    // return if intersected
                                    return intersect_triangle(tray, v0, v1, v2);})) return true;
        } else {
            // foreach triangle
            for(auto triangle : mesh->triangle) {
                // grab vertices
                auto v0 = mesh->pos[triangle.x];
                auto v1 = mesh->pos[triangle.y];
                auto v2 = mesh->pos[triangle.z];
                
                // intersect triangle
                if(intersect_triangle(tray, v0, v1, v2)) return true;
            }
        }
    }
    
    // no intersection found
    return false;
}

// prepare scene acceleration and triangulate meshes
void accelerate(Scene* scene) {
    // foreach mesh, init bvh acceleration structure to nullptr
    for(auto mesh : scene->meshes) mesh->bvh = nullptr;
    
    // if scene should be accelerated using bvh
    if(scene->accelerate_bvh) {
        // foreach mesh
        for (auto mesh : scene->meshes) {
            
            // triangulate quads (convert all quads into two tris)
            for(auto f : mesh->quad) {
                mesh->triangle.push_back({f.x,f.y,f.z});
                mesh->triangle.push_back({f.x,f.z,f.w});
            }
            // clear out quads vector
            mesh->quad.clear();
            
            // make acceleration structure
            // check whether to accelerate
            if (mesh->triangle.size()+mesh->quad.size() > BVHAccelerator_min_prims) {
                // grab all bbox
                auto bboxes = vector<range3f>(mesh->triangle.size());
                for(auto i : range(mesh->triangle.size())) {
                    auto f = mesh->triangle[i];
                    bboxes[i] = make_range3f({mesh->pos[f.x],mesh->pos[f.y],mesh->pos[f.z]});
                }
                // make accelerator
                mesh->bvh = make_accelerator(bboxes);
            }
        }
    }
}





#endif
