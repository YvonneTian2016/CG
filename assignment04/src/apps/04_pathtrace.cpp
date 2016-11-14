#include "scene.h"
#include "intersect.h"
#include "montecarlo.h"
#include "animation.h"
#include <iostream>
#include <thread>
using namespace std;
// modify the following line to disable/enable parallel execution of the pathtracer
bool parallel_pathtrace = true;

float _min = 10000;
float _max = 0;
image3f pathtrace(Scene* scene, bool multithread);
void pathtrace(Scene* scene, image3f* image, RngImage* rngs, int offset_row, int skip_row, bool verbose);



// lookusp texture value
vec3f lookup_scaled_texture(vec3f value, image3f* texture, vec2f uv, bool tile = false) {
    if(not texture) return value;

    int i = int(uv.x * texture->width());
    float s = uv.x * texture->width() - i;
    int i1 = i + 1;
    int j = int(uv.y * texture->height());
    float t = uv.y * texture->height() - j;
    int j1 = j + 1;

    if (not tile) {
        i = clamp(i, 0, texture->width() - 1);
        i1 = clamp(i1, 0, texture->width() - 1);
        j = clamp(j, 0, texture->height() - 1);
        j1 = clamp(j1, 0, texture->height() - 1);
    }
    else {
        i = i % texture->width();
        if (i < 0)
            i += texture->width();
        i1 = i1 % texture->width();
        if (i1 < 0) {
            i1 += texture->width();
        }
        j = j % texture->height();
        if (j < 0)
            j += texture->height();
        j1 = j1 % texture->height();
        if (j1 < 0)
            j1 += texture->height();
    }

    return value * (texture->at(i, j) * (1 - s) * (1 - t) + texture->at(i, j1) * (1 - s) * t + texture->at(i1, j) * s * (1 - t) + texture->at(i1, j1) * s * t);
}
/*
vec3f lookup_scaled_texture(vec3f value, image3f* texture, vec2f uv, bool tile = false) {
    if(not texture) return value;

    // for now, simply clamp texture coords
    auto u = clamp(uv.x, 0.0f, 1.0f);
    auto v = clamp(uv.y, 0.0f, 1.0f);
    return value * texture->at(u*(texture->width()-1), v*(texture->height()-1));
}*/

// compute the brdf
vec3f eval_brdf(vec3f kd, vec3f ks, float n, vec3f v, vec3f l, vec3f norm, bool microfacet) {
    if (not microfacet) {
        auto h = normalize(v+l);
        return kd/pif + ks*(n+8)/(8*pif) * pow(max(0.0f,dot(norm,h)),n);
    } else {
        vec3f h = normalize(v+l);
        float d = (n + 2) / (2 * pif) * pow(max(0.0f, dot(norm, h)), n);
        vec3f f = ks + (one3f - ks) * pow((1 - dot(h,l)), 5);
        float g = min(1.0, 2 * dot(h, norm) * dot(v, norm) / dot(v, h));
        g = min(g, 2 * dot(h, norm) * dot(l, norm) / dot(l, h));
        vec3f p = d * g * f / (4 * dot(l, norm) * dot(v, norm));
        return p; // <- placeholder
    }
}

// evaluate the environment map
vec3f eval_env(vec3f ke, image3f* ke_txt, vec3f dir) {
    if (ke_txt == nullptr) return ke;
    auto u = atan2(dir.x, dir.z) / (2 * pif);
    auto v = 1 - acos(dir.y) / pif;
    return lookup_scaled_texture(ke, ke_txt, vec2f(u, v), true);
}
image3f level_0 = read_png("level_0.png", true);
image3f level_1 = read_png("level_1.png", true);
image3f level_2 = read_png("level_2.png", true);
// compute the color corresponing to a ray by pathtrace
vec3f pathtrace_ray(Scene* scene, ray3f ray, Rng* rng, int depth) {
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

    vec3f kd;
    if (scene->mipmapping) {
        float distance = dist(intersection.pos, ray.e);
        if (distance > _max) _max = distance;
        if (distance < _min) _min = distance;
        //cout << distance << endl;
        float scale;
        if (distance < 1.8) {
            kd += lookup_scaled_texture(intersection.mat->kd, &level_0, intersection.texcoord);
        }
        else if (distance < 2.6) {
            scale = (2.6 - distance) / 0.8;
            kd += scale * lookup_scaled_texture(intersection.mat->kd, &level_0, intersection.texcoord) +
                    (1 - scale) * lookup_scaled_texture(intersection.mat->kd, &level_1, intersection.texcoord);
        }
        else if (distance < 3.4) {
            kd += lookup_scaled_texture(intersection.mat->kd, &level_1, intersection.texcoord);
        }
        else if (distance < 4.2) {
            scale = (4.2 - distance) / 0.8;
            kd += scale * lookup_scaled_texture(intersection.mat->kd, &level_1, intersection.texcoord) +
                    (1 - scale) * lookup_scaled_texture(intersection.mat->kd, &level_2, intersection.texcoord);
        }
        else if (distance >= 4.2) {
            kd += lookup_scaled_texture(intersection.mat->kd, &level_2, intersection.texcoord);
        }
    }
    else {
        kd = lookup_scaled_texture(intersection.mat->kd, intersection.mat->kd_txt, intersection.texcoord);
    }
    // compute material values by looking up textures
    auto ke = lookup_scaled_texture(intersection.mat->ke, intersection.mat->ke_txt, intersection.texcoord);
    auto ks = lookup_scaled_texture(intersection.mat->ks, intersection.mat->ks_txt, intersection.texcoord);
    auto n = intersection.mat->n;
    auto mf = intersection.mat->microfacet;

    // accumulate color starting with ambient
    auto c = scene->ambient * kd;

    // add emission if on the first bounce
    if(depth == 0 and dot(v,norm) > 0) c += ke;

    // foreach point light
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
    // foreach surface
    for (Surface* surface : scene->surfaces) {
        // skip if no emission from surface
        if (surface->mat->ke == zero3f)
            continue;
        // todo: pick a point on the surface, grabbing normal, area, and texcoord
        vec3f lightPosition;
        vec3f normal = zero3f;
        float area;
        vec2f r = rng->next_vec2f();
        vec3f newPoint = zero3f;
        // check if quad
        if (surface->isquad) {
            // generate a 2d random number
            newPoint.x = (r.x - 0.5) * 2 * surface->radius;
            newPoint.y = (r.y - 0.5) * 2 * surface->radius;
            // compute light position, normal, area
            lightPosition = transform_point_from_local(surface->frame, newPoint);
            area = 4 * surface->radius * surface->radius;
            normal = transform_normal_from_local(surface->frame, vec3f(0, 0, 1));
            // set tex coords as random value got before
            intersection.texcoord.x = r.x;
            intersection.texcoord.y = r.y;
        }
        else {
            // generate a 2d random number
            newPoint.x = r.x;
            newPoint.y = r.y;
            // compute light position, normal, area
            lightPosition = transform_point_from_local(surface->frame, surface->radius * sample_direction_spherical_uniform(r));
            area = 4 * pif * surface->radius * surface->radius;
            normal = transform_normal_from_local(surface->frame, sample_direction_spherical_uniform(r)); //CORRECT?
            // set tex coords as random value got before
            intersection.texcoord.x = r.x;
            intersection.texcoord.y = r.y;
        }
        // get light emission from material and texture
        auto emission = lookup_scaled_texture(surface->mat->ke, surface->mat->ke_txt, r);
        //vec3f emission = surface.mat->ke_txt;
        // compute light direction
        vec3f direction = normalize(lightPosition - pos);
        // compute light response (ke * area * cos_of_light / dist^2)
        auto response = emission * area * max(-1 * dot(direction, normal), 0.0f) / distSqr(pos, lightPosition);
        // compute the material response (brdf*cos)
        auto brdfcos = max(dot(norm, direction),0.0f) * eval_brdf(kd, ks, n, v, direction, norm, mf);
        // multiply brdf and light
        auto shade = response * brdfcos;
        // check for shadows and accumulate if needed
        if(shade == zero3f) continue;
        // if shadows are enabled
        if(scene->path_shadows) {
            // perform a shadow check and accumulate
            if(not intersect_shadow(scene,ray3f::make_segment(pos, lightPosition))) c += shade;
        } else {
            // else just accumulate
            c += shade;
        }
    }



    // todo: sample the brdf for environment illumination if the environment is there
    //if scene->background is not zero3f
    if (scene->background != zero3f) {
        // pick direction and pdf
        auto res = sample_brdf(kd, ks, n, v, norm, rng->next_vec2f(), rng->next_float());
        auto Lenv = eval_env(scene->background, scene->background_txt, res.first) / res.second;
        // compute the material response (brdf*cos)
        auto brdfcos = max(dot(norm, res.first),0.0f) * eval_brdf(kd, ks, n, v, res.first, norm, mf);
        // todo: accumulate response scaled by brdf*cos/pdf
        auto response = brdfcos * Lenv;
        // if material response not zero3f
        if (response != zero3f) {
            // if shadows are enabled
            if (scene->path_shadows) {
                // perform a shadow check and accumulate
                if(not intersect_shadow(scene,ray3f(pos, res.first))) c += response;
             }
            else
                c += response;
        }
    }

    // todo: sample the brdf for indirect illumination
    // if kd and ks are not zero3f and haven't reach max_depth
        // pick direction and pdf
        // compute the material response (brdf*cos)
        // accumulate recersively scaled by brdf*cos/pdf
    if(scene->russianRoulette == true){
           auto res = sample_brdf(kd, ks, n, v, norm, rng->next_vec2f(), rng->next_float());
           // compute the material response (brdf*cos)
           auto brdfcos = (max(dot(norm, res.first),0.0f)) * eval_brdf(kd, ks, n, v, res.first, norm, mf);
           // accumulate recersively scaled by brdf*cos/pdf
           if(res.second > 0.1){
               c += pathtrace_ray(scene, ray3f(pos, res.first), rng, depth + 1) * (brdfcos / res.second)  / (1 - res.second);
           }
    }
    else {
        if(depth < scene->path_max_depth && (kd != zero3f || ks != zero3f)){
            auto res = sample_brdf(kd, ks, n, v, norm, rng->next_vec2f(), rng->next_float());
            // compute the material response (brdf*cos)
            auto brdfcos= (max(dot(norm, res.first),0.0f)) * eval_brdf(kd, ks, n, v, res.first, norm, mf);
            // accumulate recersively scaled by brdf*cos/pdf
            c += pathtrace_ray(scene, ray3f(pos, res.first), rng, depth + 1) * (brdfcos / res.second);
        }
    }
    // if the material has reflections
        if(intersection.mat->kr != zero3f) {
            // if has blurry reflection
            if(scene->blurryReflection ){
                int num = 10;
                vec3f sum = zero3f;
                for(int k = 0; k < num; k++){
                    // create the reflection ray using random direction
                    vec3f reflection = reflect(ray.d,intersection.norm);
                    reflection = (1 - 0.2 * rng->next_float())*reflection;
                    auto rr = ray3f(intersection.pos, reflection);
                    // accumulate the reflected light (recursive call) scaled by the material reflection
                    sum += intersection.mat->kr * pathtrace_ray(scene,rr,rng,depth+1);
                }
                c += sum/num;
            }else{
                // create the reflection ray
                auto rr = ray3f(intersection.pos,reflect(ray.d,intersection.norm));
                // accumulate the reflected light (recursive call) scaled by the material reflection
                c += intersection.mat->kr * pathtrace_ray(scene,rr,rng,depth+1);
            }
        }

    // return the accumulated color
    return c;
}


// runs the raytrace over all tests and saves the corresponding images
int main(int argc, char** argv) {
    auto args = parse_cmdline(argc, argv,
        { "04_pathtrace", "raytrace a scene",
            {  {"resolution",     "r", "image resolution", typeid(int),    true,  jsonvalue() } },
            {  {"scene_filename", "",  "scene filename",   typeid(string), false, jsonvalue("scene.json") },
               {"image_filename", "",  "image filename",   typeid(string), true,  jsonvalue("") } }
        });

    auto scene_filename = args.object_element("scene_filename").as_string();
    Scene* scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else {
        scene = load_json_scene(scene_filename);
    }
    error_if_not(scene, "scene is nullptr");

    auto image_filename = (args.object_element("image_filename").as_string() != "") ?
        args.object_element("image_filename").as_string() :
        scene_filename.substr(0,scene_filename.size()-5)+".png";

    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }


    // NOTE: acceleration structure does not support animations
    message("reseting animation...\n");
    animate_reset(scene);

    message("accelerating...\n");
    accelerate(scene);

    message("rendering %s...\n", scene_filename.c_str());
    auto image = pathtrace(scene, parallel_pathtrace);

    message("saving %s...\n", image_filename.c_str());
    write_png(image_filename, image, true);

    delete scene;
    message("done\n");
}


/////////////////////////////////////////////////////////////////////
// Rendering Code


// pathtrace an image
void pathtrace(Scene* scene, image3f* image, RngImage* rngs, int offset_row, int skip_row, bool verbose) {
    if(verbose) message("\n  rendering started        ");
    // foreach pixel
    for(auto j = offset_row; j < scene->image_height; j += skip_row ) {
        if(verbose) message("\r  rendering %03d/%03d        ", j, scene->image_height);
        for(auto i = 0; i < scene->image_width; i ++) {
            // init accumulated color
            image->at(i,j) = zero3f;
            // grab proper random number generator
            auto rng = &rngs->at(i, j);
            // foreach sample
            for(auto jj : range(scene->image_samples)) {
                for(auto ii : range(scene->image_samples)) {
                    // compute ray-camera parameters (u,v) for the pixel and the sample
                    auto u = (i + (ii + rng->next_float())/scene->image_samples) /
                        scene->image_width;
                    auto v = (j + (jj + rng->next_float())/scene->image_samples) /
                        scene->image_height;
                    // compute camera ray
                    auto ray = transform_ray(scene->camera->frame,
                        ray3f(zero3f,normalize(vec3f((u-0.5f)*scene->camera->width,
                                                     (v-0.5f)*scene->camera->height,-1))));
                    // set pixel to the color raytraced with the ray
                    image->at(i,j) += pathtrace_ray(scene,ray,rng,0);
                }
            }
            // scale by the number of samples
            image->at(i,j) /= (scene->image_samples*scene->image_samples);
        }
    }
    if(verbose) message("\r  rendering done        \n");

}

// pathtrace an image with multithreading if necessary
image3f pathtrace(Scene* scene, bool multithread) {
    // allocate an image of the proper size
    auto image = image3f(scene->image_width, scene->image_height);

    // create a random number generator for each pixel
    auto rngs = RngImage(scene->image_width, scene->image_height);

    // if multitreaded
    if(multithread) {
        // get pointers
        auto image_ptr = &image;
        auto rngs_ptr = &rngs;
        // allocate threads and pathtrace in blocks
        auto threads = vector<thread>();
        auto nthreads = thread::hardware_concurrency();
        for(auto tid : range(nthreads)) threads.push_back(thread([=](){
            return pathtrace(scene,image_ptr,rngs_ptr,tid,nthreads,tid==0);}));
        for(auto& thread : threads) thread.join();
    } else {
        // pathtrace all rows
        pathtrace(scene, &image, &rngs, 0, 1, true);
    }

    // done
    return image;
}


