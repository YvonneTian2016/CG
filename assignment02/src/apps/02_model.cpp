#include "common.h"
#include "scene.h"
#include "image.h"
#include "gls.h"

string scene_filename;  // scene filename
string image_filename;  // image filename
Scene* scene;           // scene arrays

void uiloop();          // UI loop


// map used to uniquify edges
struct EdgeMap {
    map<pair<int,int>,int>  _edge_map;  // internal map
    vector<vec2i>           _edge_list; // internal list to generate unique ids
    
    // create an edge map for a collection of triangles and quads
    EdgeMap(vector<vec3i> triangle, vector<vec4i> quad) {
        for(auto f : triangle) { _add_edge(f.x,f.y); _add_edge(f.y,f.z); _add_edge(f.z,f.x); }
        for(auto f : quad) { _add_edge(f.x,f.y); _add_edge(f.y,f.z); _add_edge(f.z,f.w); _add_edge(f.w,f.x); }
    }
    
    // internal function to add an edge
    void _add_edge(int i, int j) {
        if(_edge_map.find(make_pair(i,j)) == _edge_map.end()) {
            _edge_map[make_pair(i,j)] = _edge_list.size();
            _edge_map[make_pair(j,i)] = _edge_list.size();
            _edge_list.push_back(vec2i(i,j));
        }
    }
    
    // edge list
    const vector<vec2i>& edges() const { return _edge_list; }
    
    // get an edge from two vertices
    int edge_index(vec2i e) const {
        error_if_not(not (_edge_map.find(make_pair(e.x,e.y)) == _edge_map.end()), "non existing edge");
        return _edge_map.find(make_pair(e.x, e.y))->second;
    }
};


// make normals for each face - duplicates all vertex data
void facet_normals(Mesh* mesh) {
    // allocates new arrays
    auto pos = vector<vec3f>();
    auto norm = vector<vec3f>();
    auto texcoord = vector<vec2f>();
    auto triangle = vector<vec3i>();
    auto quad = vector<vec4i>();
    
    // foreach triangle
    for(auto f : mesh->triangle) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // add triangle
        triangle.push_back({nv,nv+1,nv+2});
        // add vertex data
        for(auto i : range(3)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    
    // foreach quad
    for(auto f : mesh->quad) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face normal
        auto fn = normalize(normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x])) +
                            normalize(cross(mesh->pos[f.z]-mesh->pos[f.x], mesh->pos[f.w]-mesh->pos[f.x])));
        // add quad
        quad.push_back({nv,nv+1,nv+2,nv+3});
        // add vertex data
        for(auto i : range(4)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    
    // set back mesh data
    mesh->pos = pos;
    mesh->norm = norm;
    mesh->texcoord = texcoord;
    mesh->triangle = triangle;
    mesh->quad = quad;
}

// smooth out normal - does not duplicate data
void smooth_normals(Mesh* mesh) {
    // set normals array to the same length as pos and init all elements to zero
    mesh->norm.assign(mesh->pos.size(), zero3f);
    
   // put_your_code_here("Implement normal smoothing");
    
    // foreach triangle
    for(auto f : mesh->triangle)
    {
        // compute face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // accumulate face normal to the vertex normals of each face index
        for (auto i : range(2))  mesh->norm[f[i]] += fn;
    }

    // foreach quad
    for(auto f : mesh->quad)
    {
        // compute face normal
        auto fn = normalize(normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x])) +
                            normalize(cross(mesh->pos[f.z]-mesh->pos[f.x], mesh->pos[f.w]-mesh->pos[f.x])));
        // accumulate face normal to the vertex normals of each face index
        for (auto i : range(3)) mesh->norm[f[i]] += fn;
    }

    // normalize all vertex normals
    for (auto& f : mesh->norm) f += normalize(f);
}

// smooth out tangents
void smooth_tangents(Mesh* polyline) {
    // set tangent array
    polyline->norm = vector<vec3f>(polyline->pos.size(),zero3f);
    // foreach line
    for(auto l : polyline->line) {
        // compute line tangent
        auto lt = normalize(polyline->pos[l.y]-polyline->pos[l.x]);
        // accumulate segment tangent to vertex tangent on each vertex
        for (auto i : range(2)) polyline->norm[l[i]] += lt;
    }
    // normalize all vertex tangents
    for (auto& t : polyline->norm) t = normalize(t);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
// subdivide using uniform sampling
void subdivide_bezier_uniform(Mesh *bezier) {
    auto pos = vector<vec3f>();
    auto line = vector<vec2i>();
    
    // determine number of steps
    int steps = 1 << bezier->subdivision_bezier_level;

   // put_your_code_here("Implement uniform Bezier spline subdivision");
    vec3f v0,v1,v2,v3,v_new;
    float t,b0,b1,b2,b3;
    int line_pos;

    // foreach spline segment
    for(int i = 0; i < bezier->spline.size(); i++)
    {
       // get control points of segment
       v0 = bezier->pos[bezier->spline[i].x];
       v1 = bezier->pos[bezier->spline[i].y];
       v2 = bezier->pos[bezier->spline[i].z];
       v3 = bezier->pos[bezier->spline[i].w];

       // note the starting index of new points
       line_pos = pos.size();

       // foreach step
       for(int j = 0; j <= steps; j++)
       {
           t = (float)j*(1.0f/(float)steps);    // compute t for current segment

           b0 = bernstein(t,0,3);   // compute blending weights
           b1 = bernstein(t,1,3);
           b2 = bernstein(t,2,3);
           b3 = bernstein(t,3,3);

           v_new = b0*v0 + b1*v1 + b2*v2 +b3*v3;    // compute new point position
           pos.push_back(v_new);      // add new point to pos vector
       }

       // foreach step
           // create line segment

      for(int k = 0; k < steps; k++)
      {
            line.push_back(vec2i(line_pos+k,line_pos+k+1));
      }
   }
    // copy vertex positions
    bezier->pos = pos;
    // copy line segments
    bezier->line = line;
    
    // clear bezier array from lines
    bezier->spline.clear();
    bezier->subdivision_bezier_level = 0;
    
    // run smoothing to get proper tangents
    smooth_tangents(bezier);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
// subdivide using de casteljau algorithm
void subdivide_bezier_decasteljau(Mesh *bezier) {
    auto pos = bezier->pos;
    auto splines = bezier->spline;
    
    //put_your_code_here("Implement de Casteljau algorithm")
    // *note*: this psuedocode is for an iterative implementation of the algorithm without adaptive subd
    // foreach level
     int level = bezier->subdivision_bezier_level;
     vec3f v0,v1,v2,v3,vm0,vm1,vm2,vmm0,vmm1,vmmm;
     for(int i = 0; i < level; i++)
     {
         auto new_pos = vector<vec3f>();  // make new arrays of positions and bezier segments
         auto new_splines = vector<vec4i>();

         // copy all the vertices into the new array (this waste space but it is easier for now)
         // foreach bezier segment
         for(int j = 0; j < splines.size(); j++)
         {
             // apply subdivision algorithm
             int size = new_pos.size();
             v0 = pos[splines[j].x];
             v1 = pos[splines[j].y];
             v2 = pos[splines[j].z];
             v3 = pos[splines[j].w];
             // prepare indices for two new segments
             vm0 = (v0+v1)/2.0f;
             vm1 = (v1+v2)/2.0f;
             vm2 = (v2+v3)/2.0f;

             vmm0 = (vm0+vm1)/2.0f;
             vmm1 = (vm1+vm2)/2.0f;

             vmmm = (vmm0+vmm1)/2.0f;
             // add mid point
             // add points for first segment and fix segment indices
             // add points for second segment and fix segment indices
             // add indices for both segments into new segments array
             new_pos.push_back(v0);
             new_pos.push_back(vm0);
             new_pos.push_back(vmm0);
             new_pos.push_back(vmmm);
             new_pos.push_back(vmm1);
             new_pos.push_back(vm2);
             new_pos.push_back(v3);

             new_splines.push_back(vec4i(size,size+1,size+2,size+3));
             new_splines.push_back(vec4i(size+3,size+4,size+5,size+6));
         }
         // set new arrays pos, segments into the working lineset
         pos=new_pos;
         splines=new_splines;

     }
    // copy vertex positions
    bezier->pos = pos;
    
    // copy bezier segments into line segments
    bezier->line.clear();
    for(auto spline : splines) bezier->line.push_back({spline.x,spline.w});
    
    // clear bezier array from lines
    bezier->spline.clear();
    bezier->subdivision_bezier_level = 0;
    
    // run smoothing to get proper tangents
    smooth_tangents(bezier);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
void subdivide_bezier(Mesh* bezier) {
    // skip is needed
    if(not bezier->subdivision_bezier_level) return;
    
    if(bezier->subdivision_bezier_uniform) subdivide_bezier_uniform(bezier);
    else subdivide_bezier_decasteljau(bezier);
}



// apply Catmull-Clark mesh subdivision
// does not subdivide texcoord
void subdivide_catmullclark(Mesh* subdiv) {
    // skip is needed
    if(not subdiv->subdivision_catmullclark_level) return;
    
    // allocate a working Mesh copied from the subdiv
    auto mesh = new Mesh(*subdiv);
    
    vec3f v0,v1,v2,v3,v_m;
    vec2i edges;
    vec3i triangles;
    vec4i quads;
    // foreach level
    for(auto l : range(subdiv->subdivision_catmullclark_level)) {
        // make empty pos and quad arrays

        auto pos = vector<vec3f>();
        auto quad = vector<vec4i>();
        
        // create edge_map from current mesh
        auto edge_map = EdgeMap(mesh->triangle,mesh->quad);

        //put_your_code_here("Implement Catmull-Clark Subdivision");
        
        // linear subdivision - create vertices --------------------------------------
        // copy all vertices from the current mesh


        for (int i = 0; i < mesh->pos.size(); i++)
        {
            pos.push_back(mesh->pos[i]);
        }
        // add vertices in the middle of each edge (use EdgeMap)
        for (int i = 0; i < edge_map.edges().size(); i++)
        {
            edges = edge_map.edges()[i];
            v0 = mesh->pos[edges.x];
            v1 = mesh->pos[edges.y];
            v_m = (v0+v1)/2.0f;
            pos.push_back(v_m);
        }
         // add vertices in the middle of each triangle
        for(int i = 0; i < mesh->triangle.size(); i++)
        {
            triangles = mesh->triangle[i];
            v0 = mesh->pos[triangles.x];
            v1 = mesh->pos[triangles.y];
            v2 = mesh->pos[triangles.z];
            v_m = (v0+v1+v2)/3.0f;
            pos.push_back(v_m);
        }
        // add vertices in the middle of each quad
        for(int i = 0; i< mesh->quad.size(); i++)
        {
            quads = mesh->quad[i];
            v0 = mesh->pos[quads.x];
            v1 = mesh->pos[quads.y];
            v2 = mesh->pos[quads.z];
            v3 = mesh->pos[quads.w];
            v_m = (v0+v1+v2+v3)/4.0f;
            pos.push_back(v_m);
        }


        // subdivision pass ----------------------------------------------------------
        int edge_off,trian_off,quad_off;
        int vt0,vt1,vt2,vq0,vq1,vq2,vq3,ve0,ve1,ve2,ve3,vf;
        vec2i edge0,edge1,edge2,edge3;

        // compute an offset for the edge vertices
        edge_off = mesh->pos.size();
        // compute an offset for the triangle vertices
        trian_off = edge_off + edge_map.edges().size();
        // compute an offset for the quad vertices
        quad_off = trian_off + mesh->triangle.size();

        // foreach triangle
            // add three quads to the new quad array
        for(int i = 0; i < mesh->triangle.size(); i++)
        {
             vt0 = mesh->triangle[i].x;
             vt1 = mesh->triangle[i].y;
             vt2 = mesh->triangle[i].z;
             edge0 = vec2i(vt0,vt1);
             edge1 = vec2i(vt1,vt2);
             edge2 = vec2i(vt2,vt0);
             ve0 = edge_off+edge_map.edge_index(edge0);
             ve1 = edge_off+edge_map.edge_index(edge1);
             ve2 = edge_off+edge_map.edge_index(edge2);
             vf = trian_off + i;

             quad.push_back(vec4i(vt0,ve0,vf,ve2)); //顺序可能有变
             quad.push_back(vec4i(ve0,vt1,ve1,vf));
             quad.push_back(vec4i(vf,ve1,vt2,ve2));
        }

        // foreach quad
            // add four quads to the new quad array
        for(int i = 0; i < mesh->quad.size(); i++)
        {
             vq0 = mesh->quad[i].x;
             vq1 = mesh->quad[i].y;
             vq2 = mesh->quad[i].z;
             vq3 = mesh->quad[i].w;

             edge0 = vec2i(vq0,vq1);
             edge1 = vec2i(vq1,vq2);
             edge2 = vec2i(vq2,vq3);
             edge3 = vec2i(vq3,vq0);

             ve0 = edge_off+edge_map.edge_index(edge0);
             ve1 = edge_off+edge_map.edge_index(edge1);
             ve2 = edge_off+edge_map.edge_index(edge2);
             ve3 = edge_off+edge_map.edge_index(edge3);

             vf = quad_off + i;

             quad.push_back(vec4i(vq0,ve0,vf,ve3)); //顺序可能有变
             quad.push_back(vec4i(ve0,vq1,ve1,vf));
             quad.push_back(vec4i(vf,ve1,vq2,ve2));
             quad.push_back(vec4i(ve3,vf,ve2,vq3));
        }

        // averaging pass ------------------------------------------------------------
        vec3f q_center;
        // create arrays to compute pos averages (avg_pos, avg_count)
        // arrays have the same length as the new pos array, and are init to zero
        auto avg_pos = vector<vec3f>(pos.size(),zero3f);
        auto avg_count = vector<int>(pos.size(),0);

        // for each new quad
        for(int i = 0; i < quad.size(); i++)
        {
            // compute quad center using the new pos array
           q_center = (pos[quad[i].x]+ pos[quad[i].y]+ pos[quad[i].z]+ pos[quad[i].w])/4.0f;
           // foreach vertex index in the quad
               // accumulate face center to the avg_pos and add 1 to avg_count
           avg_pos[quad[i].x]+=q_center;
           avg_pos[quad[i].y]+=q_center;
           avg_pos[quad[i].z]+=q_center;
           avg_pos[quad[i].w]+=q_center;

           avg_count[quad[i].x]++;
           avg_count[quad[i].y]++;
           avg_count[quad[i].z]++;
           avg_count[quad[i].w]++;
        }

       // normalize avg_pos with its count avg_count

        for(int i = 0; i < avg_pos.size(); i++)
        {
            avg_pos[i]/=avg_count[i];
        }
        
        // correction pass -----------------------------------------------------------
        // foreach pos, compute correction p = p + (avg_p - p) * (4/avg_count)
        
        for(int i = 0; i < pos.size(); i++)
        {
            pos[i]=pos[i]+(avg_pos[i]-pos[i])*(4.0f/avg_count[i]);
        }


        // set new arrays pos, quad back into the working mesh; clear triangle array
        mesh->pos = pos;
        mesh->triangle = vector<vec3i>();
        mesh->quad = quad;

    }
    
    // clear subdivision
    mesh->subdivision_catmullclark_level = 0;
    
    // according to smooth, either smooth_normals or facet_normals
    if(subdiv->subdivision_catmullclark_smooth) smooth_normals(mesh);
    else facet_normals(mesh);
    
    // copy back
    *subdiv = *mesh;
    
    // clear
    delete mesh;
}



void subdivide_surface(Surface* surface) {
    // create mesh struct
    auto mesh    = new Mesh{};
    // copy frame
    mesh->frame  = surface->frame;
    // copy material
    mesh->mat    = surface->mat;
    
    // get surface radius
    auto radius  = surface->radius;
    
    // vertexidx is used to look up index of vertex corresponding to (i,j)
    map<pair<int,int>,int> vertexidx;
    

    if(surface->isquad) {
        // compute how much to subdivide
        auto ci = 1 << surface->subdivision_level;
        auto cj = 1 << surface->subdivision_level;
        
        // compute corners of quad
        auto p00 = vec3f(-1,-1,0) * radius;
        auto p01 = vec3f(-1, 1,0) * radius;
        auto p10 = vec3f( 1,-1,0) * radius;
        auto p11 = vec3f( 1, 1,0) * radius;
        
        // foreach column
        for(auto i : range(ci+1)) {
            // foreach row
            for(auto j : range(cj+1)) {
                // compute u,v corresponding to column and row
                auto u = i / (float)ci;
                auto v = j / (float)cj;
                
                // compute new point location
                auto p = p00*u*v + p01*u*(1-v) + p10*(1-u)*v + p11*(1-u)*(1-v);
                
                // insert point into pos vector, remembering its index
                vertexidx[make_pair(i,j)] = mesh->pos.size();
                mesh->pos.push_back(p);
                mesh->norm.push_back(z3f);
            }
        }
        


        //edited by Yuan Tian for Displacement_Mapping

         float u,v,color_bright;
         vec3f color;
         float h = 0.2;
         if(surface->Displacement_Mapping)
       {
             image3f png = read_png("images.png",false);
            for(auto i : range(ci))
           {
                for(auto j : range(cj))
              {
                   u = floor(((float)i/ci)*png.width());
                   v = floor(((float)j/cj)*png.height());
                   color = png.at(u,v);
                   color_bright = (color.x+color.y+color.z)/3.0f;

                   mesh->pos[vertexidx[make_pair(i,j)]]+=h*color_bright*mesh->norm[vertexidx[make_pair(i,j)]];
              }
          }
      }


        // foreach column
        for(auto i : range(ci)) {
            // foreach row
            for(auto j : range(cj)) {
                // find indices of neigboring vertices
                int idx0 = vertexidx[make_pair(i+0,j+0)];
                int idx1 = vertexidx[make_pair(i+1,j+0)];
                int idx2 = vertexidx[make_pair(i+1,j+1)];
                int idx3 = vertexidx[make_pair(i+0,j+1)];
                
                // create quad
                mesh->quad.push_back({idx0, idx1, idx2, idx3});
            }
        }
    } else {
      // put_your_code_here("Implement sphere subdivision");
      float theta = 0, fai=0;
      vec3f new_v;
      // compute how much to subdivide
      int steps_lat = 1 << (surface->subdivision_level+1);
      int steps_long = 1 << (surface->subdivision_level+2);


      mesh->pos.push_back(vec3f(0,0,radius));    //top
      mesh->pos.push_back(vec3f(0,0,-radius));   //bottom



      // foreach column
          // foreach row
      for(int i = 1; i < steps_lat; i++)
      {
         for(int j = 0; j < steps_long; j++)
         {
               theta = (float)i/steps_lat*PI;                    // compute phi,theta for column and row
               fai = (float)j/steps_long*2*PI;

               new_v.x=radius*cos(fai)*sin(theta);    // compute new point location
               new_v.y=radius*sin(fai)*sin(theta);
               new_v.z=radius*cos(theta);

               // insert point into pos vector, remembering its index
               vertexidx[make_pair(i,j)] = mesh->pos.size();
               mesh->pos.push_back(new_v);
               mesh->norm.push_back(new_v-mesh->frame.o);
         }
      }

        // foreach column
            // foreach row
           for(int i = 1; i < steps_lat-1; i++)
               for(int j = 0; j < steps_long; j++)
               {
                  // find indices of neighboring vertices
                   int idx0 = vertexidx[make_pair(i+0,j+0)];
                   int idx1 = vertexidx[make_pair(i+1,j+0)];
                   int idx2 = vertexidx[make_pair(i+1,(j+1) % steps_long)];
                   int idx3 = vertexidx[make_pair(i+0,(j+1) % steps_long)];

                   // create quad
                      mesh->quad.push_back({idx0, idx1, idx2, idx3});

                }

        int idx_top = 0, idx_bot = 1;
        for (int i = 0; i < steps_long; i++)
        {
            // find indices of neighboring vertices
            int idx0 = vertexidx[make_pair(1,i)];
            int idx1 = vertexidx[make_pair(1,(i+1) % steps_long)];

            //create triangle (face touching pole)
            mesh->triangle.push_back({idx_top,idx0,idx1});
        }

        for (int i = 0; i < steps_long; i++)
        {
            // find indices of neighboring vertices
            int idx0 = vertexidx[make_pair(steps_lat-1,i)];
            int idx1 = vertexidx[make_pair(steps_lat-1,(i+1) % steps_long)];

            //create triangle (face touching pole)
            mesh->triangle.push_back({idx0,idx_bot,idx1});
        }

    }

    // according to smooth, either smooth_normals or facet_normals
    if(surface->subdivision_smooth) smooth_normals(mesh);
    else facet_normals(mesh);
    
    // update _display_mesh of surface
    surface->_display_mesh = mesh;



}


void subdivide(Scene* scene) {
    for(auto mesh : scene->meshes) {
        if(mesh->subdivision_catmullclark_level) subdivide_catmullclark(mesh);
        if(mesh->subdivision_bezier_level) subdivide_bezier(mesh);
    }
    for(auto surface : scene->surfaces) {
        subdivide_surface(surface);
    }

}






// main function
int main(int argc, char** argv) {
    auto args = parse_cmdline(argc, argv,
        { "02_model", "view scene",
            {  {"resolution",     "r", "image resolution", typeid(int),    true,  jsonvalue() }  },
            {  {"scene_filename", "",  "scene filename",   typeid(string), false, jsonvalue("scene.json")},
               {"image_filename", "",  "image filename",   typeid(string), true,  jsonvalue("")}  }
        });
    
    // generate/load scene either by creating a test scene or loading from json file
    scene_filename = args.object_element("scene_filename").as_string();
    scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else {
        scene = load_json_scene(scene_filename);
    }
    error_if_not(scene, "scene is nullptr");
    
    image_filename = (args.object_element("image_filename").as_string() != "") ?
        args.object_element("image_filename").as_string() :
        scene_filename.substr(0,scene_filename.size()-5)+".png";
    
    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }
    
    subdivide(scene);
    
    uiloop();
}




/////////////////////////////////////////////////////////////////////
// UI and Rendering Code: OpenGL, GLFW, GLSL


int gl_program_id         = 0;  // OpenGL program handle
int gl_vertex_shader_id   = 0;  // OpenGL vertex shader handle
int gl_fragment_shader_id = 0;  // OpenGL fragment shader handle
map<image3f*,int> gl_texture_id;// OpenGL texture handles

bool save      = false;         // whether to start the save loop
bool wireframe = false;         // display as wireframe

void init_shaders();            // initialize the shaders
void init_textures();           // initialize the textures
void shade();                   // render the scene with OpenGL
void _shade_mesh(Mesh* mesh);
void character_callback(GLFWwindow* window, unsigned int key);  // ...
                                // glfw callback for character input
void _bind_texture(string name_map, string name_on, image3f* txt, int pos); // ...
                                // utility to bind texture parameters for shaders
                                // uses texture name, texture_on name, texture pointer and texture unit position

// glfw callback for character input
void character_callback(GLFWwindow* window, unsigned int key) {
    if(key == 's') save = true;
    if(key == 'w') wireframe = not wireframe;
}

// uiloop
void uiloop() {
    auto ok_glfw = glfwInit();
    error_if_not(ok_glfw, "glfw init error");
    
    glfwWindowHint(GLFW_SAMPLES, scene->image_samples);
    
    auto window = glfwCreateWindow(scene->image_width,
                                   scene->image_height,
                                   "graphics13 | model", NULL, NULL);
    error_if_not(window, "glfw window error");
    
    glfwMakeContextCurrent(window);
    
    glfwSetCharCallback(window, character_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    
    auto ok_glew = glewInit();
    error_if_not(GLEW_OK == ok_glew, "glew init error");
    
    init_shaders();
    init_textures();
    
    auto mouse_last_x = -1.0;
    auto mouse_last_y = -1.0;
    
    while(not glfwWindowShouldClose(window)) {
        glfwGetFramebufferSize(window, &scene->image_width, &scene->image_height);
        scene->camera->width = (scene->camera->height * scene->image_width) / scene->image_height;
        
        shade();

        if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            if (mouse_last_x < 0 or mouse_last_y < 0) { mouse_last_x = x; mouse_last_y = y; }
            auto delta_x = x - mouse_last_x, delta_y = y - mouse_last_y;
            
            set_view_turntable(scene->camera, delta_x*0.01, -delta_y*0.01, 0, 0, 0);
            
            mouse_last_x = x;
            mouse_last_y = y;
        } else { mouse_last_x = -1; mouse_last_y = -1; }
        
        if(save) {
            auto image = image3f(scene->image_width,scene->image_height);
            glReadPixels(0, 0, scene->image_width, scene->image_height, GL_RGB, GL_FLOAT, &image.at(0,0));
            write_png(image_filename, image, true);
            save = false;
        }
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwDestroyWindow(window);
    
    glfwTerminate();
}

// initialize the shaders
void init_shaders() {
    // load shader code from files
    auto vertex_shader_code    = load_text_file("model_vertex.glsl");
    auto fragment_shader_code  = load_text_file("model_fragment.glsl");
    auto vertex_shader_codes   = (char *)vertex_shader_code.c_str();
    auto fragment_shader_codes = (char *)fragment_shader_code.c_str();

    // create shaders
    gl_vertex_shader_id   = glCreateShader(GL_VERTEX_SHADER);
    gl_fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    
    // load shaders code onto the GPU
    glShaderSource(gl_vertex_shader_id,1,(const char**)&vertex_shader_codes,nullptr);
    glShaderSource(gl_fragment_shader_id,1,(const char**)&fragment_shader_codes,nullptr);
    
    // compile shaders
    glCompileShader(gl_vertex_shader_id);
    glCompileShader(gl_fragment_shader_id);
    
    // check if shaders are valid
    error_if_glerror();
    error_if_shader_not_valid(gl_vertex_shader_id);
    error_if_shader_not_valid(gl_fragment_shader_id);
    
    // create program
    gl_program_id = glCreateProgram();
    
    // attach shaders
    glAttachShader(gl_program_id,gl_vertex_shader_id);
    glAttachShader(gl_program_id,gl_fragment_shader_id);
    
    // bind vertex attributes locations
    glBindAttribLocation(gl_program_id, 0, "vertex_pos");
    glBindAttribLocation(gl_program_id, 1, "vertex_norm");
    glBindAttribLocation(gl_program_id, 2, "vertex_texcoord");

    // link program
    glLinkProgram(gl_program_id);
    
    // check if program is valid
    error_if_glerror();
    error_if_program_not_valid(gl_program_id);
}

// initialize the textures
void init_textures() {
    // grab textures from scene
    auto textures = get_textures(scene);
    // foreach texture
    for(auto texture : textures) {
        // if already in the gl_texture_id map, skip
        if(gl_texture_id.find(texture) != gl_texture_id.end()) continue;
        // gen texture id
        unsigned int id = 0;
        glGenTextures(1, &id);
        // set id to the gl_texture_id map for later use
        gl_texture_id[texture] = id;
        // bind texture
        glBindTexture(GL_TEXTURE_2D, id);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
        // load texture data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     texture->width(), texture->height(),
                     0, GL_RGB, GL_FLOAT, texture->data());
    }
}


// utility to bind texture parameters for shaders
// uses texture name, texture_on name, texture pointer and texture unit position
void _bind_texture(string name_map, string name_on, image3f* txt, int pos) {
    // if txt is not null
    if(txt) {
        // set texture on boolean parameter to true
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_TRUE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // bind texture object to it from gl_texture_id map
        glBindTexture(GL_TEXTURE_2D, gl_texture_id[txt]);
        // set texture parameter to the position pos
        glUniform1i(glGetUniformLocation(gl_program_id, name_map.c_str()), pos);
    } else {
        // set texture on boolean parameter to false
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_FALSE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // set zero as the texture id
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

// render the scene with OpenGL
void shade() {
    // enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    // disable culling face
    glDisable(GL_CULL_FACE);
    // let the shader control the points
    glEnable(GL_POINT_SPRITE);
    
    // set up the viewport from the scene image size
    glViewport(0, 0, scene->image_width, scene->image_height);
    
    // clear the screen (both color and depth) - set cleared color to background
    glClearColor(scene->background.x, scene->background.y, scene->background.z, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // enable program
    glUseProgram(gl_program_id);
    
    // bind camera's position, inverse of frame and projection
    // use frame_to_matrix_inverse and frustum_matrix
    glUniform3fv(glGetUniformLocation(gl_program_id,"camera_pos"),
                 1, &scene->camera->frame.o.x);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_frame_inverse"),
                       1, true, &frame_to_matrix_inverse(scene->camera->frame)[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_projection"),
                       1, true, &frustum_matrix(-scene->camera->dist*scene->camera->width/2, scene->camera->dist*scene->camera->width/2,
                                                -scene->camera->dist*scene->camera->height/2, scene->camera->dist*scene->camera->height/2,
                                                scene->camera->dist,10000)[0][0]);
    
    // bind ambient and number of lights
    glUniform3fv(glGetUniformLocation(gl_program_id,"ambient"),1,&scene->ambient.x);
    glUniform1i(glGetUniformLocation(gl_program_id,"lights_num"),scene->lights.size());
    
    // foreach light
    auto count = 0;
    for(auto light : scene->lights) {
        // bind light position and internsity (create param name with tostring)
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_pos[%d]",count).c_str()),
                     1, &light->frame.o.x);
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_intensity[%d]",count).c_str()),
                     1, &light->intensity.x);
        count++;
    }
    
    // foreach mesh
    for(auto mesh : scene->meshes) {
        _shade_mesh(mesh);
    }
    
    for(auto surf : scene->surfaces) {
        _shade_mesh(surf->_display_mesh);
    }
}

void _shade_mesh(Mesh* mesh) {
    // bind material kd, ks, n
    ERROR_IF_NOT(mesh, "mesh is null");
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_kd"),1,&mesh->mat->kd.x);
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_ks"),1,&mesh->mat->ks.x);
    glUniform1f(glGetUniformLocation(gl_program_id,"material_n"),mesh->mat->n);
    
    // bind texture params (txt_on, sampler)
    _bind_texture("material_kd_txt",   "material_kd_txt_on",   mesh->mat->kd_txt,   0);
    _bind_texture("material_ks_txt",   "material_ks_txt_on",   mesh->mat->ks_txt,   1);
    _bind_texture("material_norm_txt", "material_norm_txt_on", mesh->mat->norm_txt, 2);
    
    // bind mesh frame - use frame_to_matrix
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"mesh_frame"),1,true,&frame_to_matrix(mesh->frame)[0][0]);

    // enable vertex attributes arrays and set up pointers to the mesh data
    auto vertex_pos_location = glGetAttribLocation(gl_program_id, "vertex_pos");
    auto vertex_norm_location = glGetAttribLocation(gl_program_id, "vertex_norm");
    auto vertex_texcoord_location = glGetAttribLocation(gl_program_id, "vertex_texcoord");
    glEnableVertexAttribArray(vertex_pos_location);
    glVertexAttribPointer(vertex_pos_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->pos[0].x);
    glEnableVertexAttribArray(vertex_norm_location);
    glVertexAttribPointer(vertex_norm_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->norm[0].x);
    if(not mesh->texcoord.empty()) {
        glEnableVertexAttribArray(vertex_texcoord_location);
        glVertexAttribPointer(vertex_texcoord_location, 2, GL_FLOAT, GL_FALSE, 0, &mesh->texcoord[0].x);
    }
    else glVertexAttrib2f(vertex_texcoord_location, 0, 0);
    
    // draw triangles and quads
    if(not wireframe) {
        if(mesh->triangle.size()) glDrawElements(GL_TRIANGLES, mesh->triangle.size()*3, GL_UNSIGNED_INT, &mesh->triangle[0].x);
        if(mesh->quad.size()) glDrawElements(GL_QUADS, mesh->quad.size()*4, GL_UNSIGNED_INT, &mesh->quad[0].x);
    } else {
        auto edges = EdgeMap(mesh->triangle, mesh->quad).edges();
        glDrawElements(GL_LINES, edges.size()*2, GL_UNSIGNED_INT, &edges[0].x);
    }
    
    // draw line sets
    if(not mesh->line.empty()) glDrawElements(GL_LINES, mesh->line.size()*2, GL_UNSIGNED_INT, mesh->line.data());
    for(auto segment : mesh->spline) glDrawElements(GL_LINE_STRIP, 4, GL_UNSIGNED_INT, &segment);
    
    // disable vertex attribute arrays
    glDisableVertexAttribArray(vertex_pos_location);
    glDisableVertexAttribArray(vertex_norm_location);
    if(not mesh->texcoord.empty()) glDisableVertexAttribArray(vertex_texcoord_location);
}

