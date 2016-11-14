#include "scene.h"

// intersection record
struct intersection3f {
    bool        hit;        // whether it hits something
    float       ray_t;      // ray parameter for the hit
    vec3f       pos;        // hit position
    vec3f       norm;       // hit normal
    Material*   mat;        // hit material
    
    // constructor (defaults to no intersection)
    intersection3f() : hit(false) { }
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
    
    // Create a ray from a segment
    static ray3f make_segment(const vec3f& a, const vec3f& b) { return ray3f(a,normalize(b-a),ray3f_epsilon,dist(a,b)-2*ray3f_epsilon); }
};

// transform a ray by a frame
inline ray3f transform_ray(const frame3f& f, const ray3f& v) {
    return ray3f(transform_point(f,v.e), transform_vector(f,v.d), v.tmin, v.tmax);
}
// transform a ray by a frame inverse
inline ray3f transform_ray_inverse(const frame3f& f, const ray3f& v) {
    return ray3f(transform_point_inverse(f,v.e),transform_vector_inverse(f,v.d),v.tmin,v.tmax);
}


// intersects the scene and return the first intrerseciton
intersection3f intersect(Scene* scene, ray3f ray) {
    // create a default intersection record to be returned
    auto intersection = intersection3f();
   // put_your_code_here("insert your intersect() code here");

    float t,t_cl=ray.tmax,dt,t1,t2,a,b,c,distance_x,distance_y;


    vec3f p;

    int i=scene->surfaces.size()-1;

    Surface * s;

    while(i>=0)   // foreach surface
    {
         s = scene->surfaces[i];
         i--;

         if(s->isquad == true)      // if it is a quad
         {
            t=dot(s->frame.o-ray.e,s->frame.z)/dot(ray.d,s->frame.z);      // compute ray intersection (and ray parameter), continue if not hit

            p=ray.e+t*ray.d;
            distance_x=fabs(p.x-s->frame.o.x);//the length limit of quad
            distance_y=fabs(p.y-s->frame.o.y);

            if(distance_x>s->radius||distance_y>s->radius)
                continue;
            else
            {
                if(t>ray.tmin&&t<ray.tmax)         // check if computed param is within ray.tmin and ray.tmax
                {
                    if(t<=t_cl)                    // check if this is the closest intersection, continue if not
                       t_cl=t;
                    else
                       continue;
                 }
                 else
                    continue;

                intersection.hit=true;      // if hit, set intersection record values
                intersection.ray_t=t_cl;
                intersection.pos=ray.eval(t_cl);
                intersection.norm=normalize(s->frame.z);
                intersection.mat=s->mat;
            }
          }

         else if(s->iscylinder == true)    // if it is a cylinder
         {
             // message("**********Cylinder**************\n");
              a=pow(ray.d.z,2)+pow(ray.d.x,2);     // compute ray intersection (and ray parameter), continue if not hit
              b=2*(ray.e.z*ray.d.z+ray.e.x*ray.d.x);
              c=pow(ray.e.z,2)+pow(ray.e.x,2)-pow(s->radius,2);

              dt=pow(b,2)-4*a*c;


              if(dt<0)
                  continue;
              else if(dt>=0)
              {
                  t = (-b - sqrt(dt)) / (2*a);      // just grab only the first hit
              }

              p=ray.e+t*ray.d; //ray construction
              distance_y=p.y;

              if(distance_y>s->height/2)            //the height limit of cylinder
                   continue;
              else
              {
                   if(t>ray.tmin&&t<ray.tmax)    // check if computed param is within ray.tmin and ray.tmax
                  {
                          if(t<=t_cl)            // check if this is the closest intersection, continue if not
                         {
                              t_cl=t;
                         }
                         else
                             continue;
                   }
                  else
                         continue;



              intersection.hit=true;             // record closest intersection
              intersection.ray_t=t_cl;
              intersection.pos=ray.eval(t_cl);
              intersection.norm=normalize(intersection.pos-s->frame.o-vec3f(0,intersection.pos.y,0));
              intersection.mat=s->mat;

             }
         }

        else if(s->isquad == false)     // if it is a sphere
       {

             a = dot(ray.d, ray.d);       // compute ray intersection (and ray parameter), continue if not hit
             b = 2 * dot(ray.d, ray.e - s->frame.o);
             c = dot(ray.e - s->frame.o, ray.e - s->frame.o) - s->radius * s->radius;
             dt = pow(b,2) - 4 * a * c;
             if(dt<0)
                 continue;
             else if(dt>=0)
             {
                 t = (-b - sqrt(dt)) / (2*a);   // just grab only the first hit
             }

             if(t>ray.tmin&&t<ray.tmax)       // check if computed param is within ray.tmin and ray.tmax
            {
                 if(t<=t_cl)                  // check if this is the closest intersection, continue if not
                 {
                     t_cl=t;
                 }
                  else
                     continue;
            }
             else
                     continue;

             intersection.hit=true;     // record closest intersection
             intersection.ray_t=t_cl;
             intersection.pos=ray.eval(t_cl);
             intersection.norm=normalize(intersection.pos-s->frame.o);
             intersection.mat=s->mat;

        }

    }


    return intersection;
}

// compute the color corresponing to a ray by raytracing
vec3f raytrace_ray(Scene* scene, ray3f ray) {
  //  put_your_code_here("insert your raytrace_ray() code here");
    
    intersection3f intersection=intersect(scene,ray);  // get scene intersection
    intersection3f intersection_lig = intersection3f();
    ray3f ray_lig = ray3f(),ray_kr = ray3f();

    vec3f n,l,h,v,L=zero3f,intensity,d,d_kr;
    int i=scene->lights.size()-1;
    Light * lig;


    if(intersection.hit==false)   // if not hit, return background
        return scene->background;

     L=L+scene->ambient*intersection.mat->kd;   // accumulate color starting with ambient

    while(i>=0)        // foreach light
{

    lig=scene->lights[i];
    i--;

    n=normalize(intersection.norm);
    l=normalize(lig->frame.o-intersection.pos);
    v=normalize(-ray.d);
    h=normalize(v+l);

    intensity=(lig->intensity)/dot(lig->frame.o-intersection.pos, lig->frame.o-intersection.pos);// compute light response

    d=lig->frame.o-intersection.pos;  // compute light direction
    ray_lig=ray3f(intersection.pos,d);

    intersection_lig = intersect(scene,ray_lig);


    if(intersection_lig.hit==false)    // check for shadows and accumulate if needed
        L=L+intersection.mat->kd*intensity*max(0.0,dot(n,l))+intersection.mat->ks*intensity*pow(max(0.0,dot(n,h)),intersection.mat->n);// compute the material response (brdf*cos)
}
    if(intersection.mat->kr!=zero3f)                          // if the material has reflections
    {
        d_kr=normalize(2*dot(n,v)*n-v);
        ray_kr=ray3f(intersection.pos,d_kr);                  // create the reflection ray
        L=L+intersection.mat->kr*raytrace_ray(scene,ray_kr);  // accumulate the reflected light (recursive call) scaled by the material reflection
    }
    return L; // return the accumulated color (for now zero)
}

// raytrace an image
image3f raytrace(Scene* scene) {
    // allocate an image of the proper size

    // put_your_code_here("insert your raytrace() code here");

    auto image = image3f(scene->image_width, scene->image_height);
    
    float l,r,t,b,u,v;
    int i,j,ii,jj;
    vec3f D,color;
    ray3f R;


    l=-0.5;
    r=0.5;
    t=0.5;
    b=-0.5;


    if(scene->image_samples==1)                 // if no anti-aliasing
     {  for(i=0;i<scene->image_width;i++)       // foreach pixel in the row (go over image width)
           for(j=0;j<scene->image_height;j++)   // foreach image row (go over image height)
             {
                u=l+(r-l)*(i+0.5)/scene->image_width;      // compute ray-camera parameters (u,v) for the pixel
                v=b+(t-b)*(j+0.5)/scene->image_height;

                D=-scene->camera->frame.z+u*scene->camera->frame.x+v*scene->camera->frame.y;
                R=ray3f(scene->camera->frame.o,D);        // compute camera ray

                image.at(i,j)=raytrace_ray(scene,R);      // set pixel to the color raytraced with the ray

              }

    }

   else                                  // if anti-aliasing
     {
        {  for(i=0;i<scene->image_width;i++)             // foreach pixel in the row (go over image width)
              for(j=0;j<scene->image_height;j++)         // foreach image row (go over image height)
                {
                  color=zero3f;                          // init accumulated color
                    for(ii=0;ii<scene->image_samples;ii++)                    // foreach sample in x
                         for(jj=0;jj<scene->image_samples;jj++)              // foreach sample in y
                         {
                                u=l+(r-l)*(i+(ii+0.5)/scene->image_samples)/scene->image_width;   // compute ray-camera parameters (u,v) for the pixel and the sample
                                v=b+(t-b)*(j+(jj+0.5)/scene->image_samples)/scene->image_height;

                                D=-scene->camera->frame.z+u*scene->camera->frame.x+v*scene->camera->frame.y;
                                R=ray3f(scene->camera->frame.o,D);    // compute camera ray

                                color=color+raytrace_ray(scene,R);    // set pixel to the color raytraced with the ray

                          }

                    image.at(i,j)=color/pow(scene->image_samples,2);      // scale by the number of samples

                }
         }

    }
    return image;
}

// runs the raytrace over all tests and saves the corresponding images
int main(int argc, char** argv) {
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
    
    message("rendering %s...\n", scene_filename.c_str());
    auto image = raytrace(scene);
    
    message("writing to png...\n");
    write_png(image_filename, image, true);
    
    delete scene;
    message("done\n");
}
