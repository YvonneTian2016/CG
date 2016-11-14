#include "intersect.h"
#include "montecarlo.h"
#include "scene.h"
#include "customfilereader.h"

#include <iostream>
#include <thread>
#include <cmath>
using std::thread;

#define MAX_DEPTH     1
#define MAX_LIGHTING_SAMPLES 1

// modify the following line to disable/enable parallel execution of the raytracer
bool parallel_raytrace = true;

image3f ray_trace(Scene* scene, bool multithread);
void ray_trace(Scene* scene, image3f* image, int offset_row, int skip_row, bool verbose);


// compute spherical coordinates given a direction
inline float compute_sphere_theta(const vec3f& v)
{
    float acos(v.y);
}

inline float compute_sphere_phi(const vec3f& v)
{
    float p = atan2f(v.z, v.x);
    return p < 0.0f ? p + 2 * M_PI : p;
}

inline vec3f compute_reflection_vector(const vec3f& normal, const vec3f& vector)
{
    return 2 * dot(normal, vector) * normal - vector;
}

// lookup texture value
vec3f lookup_scaled_texture(vec3f value, image3f* texture, vec2f uv, bool tile = false) {
    if(not texture) return value;
    
    // for now, simply clamp texture coords
    auto u = fmodf(uv.x, 1.0f);
    auto v = fmodf(uv.y, 1.0f);
    return value * texture->at(u*(texture->width()-1), v*(texture->height()-1));
}

// compute the brdf
vec3f eval_brdf(vec3f kd, vec3f ks, float n, vec3f v, vec3f l, vec3f norm, bool microfacet) {
    if (not microfacet) {
        auto h = normalize(v+l);
        return kd/pif + ks*(n+8)/(8*pif) * pow(max(0.0f,dot(norm,h)),n);
    } else {
        put_your_code_here("Implement microfacet brdf");
        return one3f; // <- placeholder
    }
}

// evaluate the environment map
vec3f eval_env(vec3f ke, image3f* ke_txt, vec3f dir) {
    
    float u = atan2f(dir.x, dir.z) / (2 * M_PI);
    float v = 1 - acosf(dir.y) / M_PI;
    
    return lookup_scaled_texture(ke, ke_txt, vec2f(u, v));
}

// compute the color corresponing to a ray by raytracing
vec3f raytrace_ray(Scene* scene, const ray3f& ray, Rng& rng, int depth) {
    
    // return black if greater than max depth.
    if(depth >= MAX_DEPTH)
        return zero3f;
    
    else
    {
        // get scene intersection
        auto intersection = intersect(scene,ray);
        
        // if not hit, return background (looking up the texture by converting the ray direction to latlong around y)
        if(not intersection.hit) {
            return eval_env(scene->background, scene->background_txt, ray.d);
        }
        
        // setup variables for shorter code
        auto pos = intersection.pos;
        auto norm = intersection.norm;
        auto v = -ray.d;
        
        // compute material values by looking up textures
        auto ke = lookup_scaled_texture(intersection.mat->ke, intersection.mat->ke_txt, intersection.texcoord);
        auto kd = lookup_scaled_texture(intersection.mat->kd, intersection.mat->kd_txt, intersection.texcoord);
        auto ks = lookup_scaled_texture(intersection.mat->ks, intersection.mat->ks_txt, intersection.texcoord);
        auto n = intersection.mat->n;
        auto mf = intersection.mat->microfacet;
        
        // accumulate color starting with ambient
        auto c = scene->ambient * kd;
        
        // add emission if on the first bounce
        if(depth == 0 and dot(v,norm) > 0) c += ke;
        
        if(scene->background_txt)
        {
            
            // Environment mapping
            vec3f sampled_dir = normalize(sample_direction_hemispherical_cosine(rng.next_vec2f()));
            float sampled_pdf = sample_direction_hemispherical_cosine_pdf(sampled_dir);
            
            auto cl = eval_env(vec3f(1.0f, 1.0f, 1.0f), scene->background_txt, sampled_dir);
            auto brdfcos = max(dot(norm,sampled_dir),0.0f) * eval_brdf(kd, ks, n, v, sampled_dir, norm, mf);
            
            vec3f shade = zero3f;
            if(sampled_pdf != 0.0f)
                shade = cl * brdfcos / sampled_pdf;
            
            if(scene->path_shadows && shade != zero3f && sampled_pdf != 0.0f) {
                // perform a shadow check and accumulate
                ray3f shadow_ray(pos, sampled_dir, ray3f_epsilon, INFINITY);
                if(not intersect_shadow(scene, shadow_ray))
                   c += shade;
            } else {
                // else just accumulate
                c += shade;
            }
        
        }
        else
        {
            // foreach point light
            if(scene->lights.size() != 0)
            {
                for(auto light : scene->lights) {
                    // compute light response
                    auto cl = light->intensity / (lengthSqr(light->frame.o - pos));
                    // compute light direction
                    auto l = normalize(light->frame.o - pos);
                    // compute the material response (brdf*cos)
                    auto brdfcos = max(dot(norm,l),0.0f) * eval_brdf(kd, ks, n, v, l, norm, mf);
                    // multiply brdf and light
                    auto shade = cl * brdfcos;
                    // check for shadows and accumulate if needed
                    if(shade == zero3f) continue;
                    // if shadows are enabled
                    if(scene->path_shadows) {
                        // perform a shadow check and accumulate
                        if(not intersect_shadow(scene,ray3f::make_segment(pos,light->frame.o))) c += shade;
                    } else {
                        // else just accumulate
                        c += shade;
                    }
                }
            }
            else
            {
                // scene might have emissive objects.
                // sample towards them.
                // compute the number of emissive surfaces
                // all the lights are either spheres or quads only.
                std::vector<int> light_list;
                auto num_lights = 0;
                auto index = 0;
                for(auto& surf : scene->surfaces)
                {
                    if(surf->mat->ke != zero3f)
                    {
                        // this is an emissive material
                        light_list.push_back(index);
                        num_lights++;
                    }
                }
                
                if(num_lights != 0)
                {
                    vec3f lighting = zero3f;
                    for(int samples = 0; samples < MAX_LIGHTING_SAMPLES; samples++)
                    {
                        int random_light = rng.next_int(vec2i(0, num_lights - 1));
                        auto& surf = scene->surfaces[light_list[random_light]];
                        
                        // choose a random point on the light.
                        vec2f rand_pos = rng.next_vec2f();
                        
                        vec3f sampled_location = sample_point(surf, rand_pos);
                        //vec3f sampled_location = surf->frame.o;
                        
                        // compute incoming radiance
                        auto cl = surf->mat->ke / (lengthSqr(sampled_location - pos));
                        // compute light direction
                        auto l = normalize(sampled_location - pos);
                        float dotp = dot(-l, surf->frame.z);
                        if(dotp < 0.0f) cl = zero3f;
                        
                        if(cl != zero3f)
                        {
                            float side = surf->radius * 2.0f;
                            float area = side * side;
                            // compute the material response (brdf*cos)
                            auto brdfcos = max(dot(norm,l),0.0f) * eval_brdf(kd, ks, n, v, l, norm, mf) * dotp * area;
                            // multiply brdf and light                            
                            if(scene->path_shadows) {
                                if(not intersect_shadow(scene,ray3f::make_segment(pos,sampled_location))) lighting += cl * brdfcos;
                            }
                            else{
                                lighting += cl * brdfcos;
                            }
                        }
                    }
                    lighting /= float(MAX_LIGHTING_SAMPLES);
                    c += lighting;
                }
            }
        }
        
        if((kd != zero3f || ks != zero3f) && depth < MAX_DEPTH)
        {
            // todo: sample the brdf for indirect illumination
            // if kd and ks are not zero3f and haven't reach max_depth
            // pick direction and pdf
            // compute the material response (brdf*cos)
            // accumulate recersively scaled by brdf*cos/pdf

            vec3f sampled_dir = normalize(sample_direction_hemispherical_cosine(rng.next_vec2f()));
            float sampled_pdf = sample_direction_hemispherical_cosine_pdf(sampled_dir);
            
            auto brdfcos = max(dot(norm,sampled_dir),0.0f) * eval_brdf(kd, ks, n, v, sampled_dir, norm, mf) / sampled_pdf;
            vec3f bounce = brdfcos * raytrace_ray(scene, ray3f(pos, sampled_dir, ray3f_epsilon, ray3f_rayinf), rng, ++depth);
            
            c += bounce;
        }
        
        // if the material has reflections
        if(not (intersection.mat->kr == zero3f)) {
            // create the reflection ray
            // accumulate the reflected light (recursive call) scaled by the material reflection
            
            vec3f reflection_direction = normalize(compute_reflection_vector(intersection.norm, v));
            ray3f refl_ray(intersection.pos + reflection_direction * ray3f_epsilon, reflection_direction, ray3f_epsilon, ray3f_rayinf);
            vec3f refl_component = raytrace_ray(scene, refl_ray, rng, ++depth);
            vec3f gloss_color = intersection.mat->kr * refl_component;
            c += gloss_color;
        }
        
        // return the accumulated color
        return c;
    }
}

// raytrace an image
void ray_trace(Scene* scene, image3f* image, int offset_row, int skip_row, bool verbose) {
    
    Rng rng;
    rng.seed(offset_row * 2 + 1001 * offset_row);               // randomize the seed.
    
    // if no anti-aliasing
    if (scene->image_samples == 1) {
        // foreach image row (go over image height)
        for( int y = offset_row; y < scene->image_height; y+= skip_row )
        {
            if(verbose) message("\r  rendering %03d/%03d        ", y, scene->image_height);
            // foreach pixel in the row (go over image width)
            for( int x = 0; x < scene->image_width; x++ )
            {
                // compute ray-camera parameters (u,v) for the pixel
                float u = (x + 0.5f) / image->width();
                float v = (y + 0.5f) / image->height();
                const vec3f Q_uv( ( u-0.5f ) * scene->camera->width, ( v-0.5f ) * scene->camera->height, - scene->camera->dist);
                // compute camera ray
                ray3f view_ray = transform_ray( scene->camera->frame, ray3f( zero3f, normalize(Q_uv) ) );
                view_ray.uv = vec2f(u, v);
                
                // set pixel to the color raytraced with the ray
                vec3f color = raytrace_ray(scene, view_ray, rng, 0);
                image->at(x, y) = color;
            }
        }
    }
    // else
    else{
        // foreach image row (go over image height)
        for( int y = offset_row; y < scene->image_height; y+= skip_row )
        {
            if(verbose) message("\r  rendering %03d/%03d        ", y, scene->image_height);
            // foreach pixel in the row (go over image width)
            for (int x = 0; x < scene->image_width; x++) {
                // init accumulated color
                vec3f color = zero3f;
                // foreach sample in y
                for (int j = 0; j < scene->image_samples; j++) {
                    // foreach sample in x
                    for( int i = 0; i < scene->image_samples; i++ )
                    {
                        // compute ray-camera parameters (u,v) for the pixel and the sample
                        auto u = (x + (i + 0.5f)/scene->image_samples) / image->width();
                        auto v = (y + (j + 0.5f)/scene->image_samples) / image->height();
                        // compute camera ray
                        vec3f Q_uv( ( u-0.5f ) * scene->camera->width, ( v-0.5f ) * scene->camera->height, - scene->camera->dist);
                        ray3f view_ray = transform_ray( scene->camera->frame, ray3f( zero3f, normalize(Q_uv) ) );
                        view_ray.uv = vec2f(u, v);
                        // set pixel to the color raytraced with the ray
                        color += raytrace_ray(scene, view_ray, rng, 0);
                    }
                }
                // scale by the number of samples
                color /= std::pow(scene->image_samples,2);
                image->at(x, y) = color;
            }
        }
    }
}

void test()
{
    //const ray3f ray( zero3f, vec3f(1,0,0) );
    //std::cout << ray_sphere_intersection(ray, vec3f(5,0,0), 2) << '\n';
}

// runs the raytrace over all tests and saves the corresponding images
int main(int argc, char** argv) {
    
    test();
    CustomFileReader* my_reader = nullptr;
    
    auto args = parse_cmdline(argc, argv,
                              { "01_raytrace", "raytrace a scene",
                                  {  {"resolution",     "r", "image resolution", typeid(int),    true,  jsonvalue()}  },
                                  {  {"scene_filename", "",  "scene filename",   typeid(string), false, jsonvalue("scene.json")},
                                      {"image_filename", "",  "image filename",   typeid(string), true,  jsonvalue("")}  }
                              });
    
    // generate/load scene either by creating a test scene or loading from json file
    string scene_filename = args.object_element("scene_filename").as_string();
    Scene *scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else if (scene_filename.substr(scene_filename.length() - 4, scene_filename.length()) == "json") {
        scene = load_json_scene(scene_filename);
    } else if (scene_filename.substr(scene_filename.length() - 3, scene_filename.length()) == "xml") {
        scene = new Scene();
        my_reader = new CustomFileReader(scene_filename, *scene);
    }
    error_if_not(scene, "scene is nullptr");
    
    auto image_filename = (args.object_element("image_filename").as_string() != "") ?
    args.object_element("image_filename").as_string() :
    scene_filename.substr(0,scene_filename.size()-5)+".png";
    
    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }
    
    message("accelerating...\n");
    accelerate(scene);
   
    message("rendering %s...\n", scene_filename.c_str());
    auto image = ray_trace(scene, parallel_raytrace);
    
    message("\nwriting to png...\n");
    write_png(image_filename, image, true);
    
    delete scene;
    message("done\n");
}


// pathtrace an image with multithreading if necessary
image3f ray_trace(Scene* scene, bool multithread) {
    // allocate an image of the proper size
    auto image = image3f(scene->image_width, scene->image_height);
    
    // if multitreaded
    if(multithread) {
        // get pointers
        auto image_ptr = &image;
        // allocate threads and pathtrace in blocks
        auto threads = vector<thread>();
        auto nthreads = thread::hardware_concurrency();
        for(auto tid : range(nthreads)) threads.push_back(thread([=](){
            return ray_trace(scene,image_ptr,tid,nthreads,tid==0);}));
        for(auto& thread : threads) thread.join();
    } else {
        // pathtrace all rows
        ray_trace(scene, &image, 0, 1, true);
    }
    
    // done
    return image;
}
