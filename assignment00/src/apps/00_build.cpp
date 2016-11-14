#include "common.h"
#include "scene.h"
#include "gls.h"

// returns the largest component
float max_component(const vec3f& a) {
    // return the largest component of a (a.x, a.y, a.z)
   // put_your_code_here("return the largest component of a (a.x, a.y, a.z)");
    
    float max;
    max=std:: max(a.x,a.y);
   
    return max=std:: max(max,a.z);
}

// returns the sum of the three given vectors
vec3f sum_three(const vec3f &a, const vec3f &b, const vec3f &c) {
    vec3f s = zero3f;
    // accumulate the sum of a + b + c and store in s
   // put_your_code_here("accumulate the sum of a + b + c and store in s");
  
    s=a+b+c;
    return s;
}

// returns the sum of the vectors in the given list
vec3f sum_many(const vector<vec3f> &vs) {
    vec3f s = zero3f;
    // accumulate the sum of vectors in vs and store in s
   // put_your_code_here("accumulate the sum of vectors in vs and store in s");
    
    for(vec3f vec : vs)
    {
        s+=vec;
    }
    
    return s;
}



// uiloop
void uiloop() {
    int w = 200, h = 200;
    // init glfw
    auto glfw_ok = glfwInit();
    error_if_not(glfw_ok, "glfw init error");
    
    // create window
    auto window = glfwCreateWindow(w, h, "graphics13 | build", NULL, NULL);
    error_if_not(window, "glfw window error");
    glfwMakeContextCurrent(window);
    
    // init glew
    auto glew_ok = glewInit();
    error_if_not(GLEW_OK == glew_ok, "glew init error");
    
    // run a few cycles
    for(int i = 0; i < 10; i++) {
        glfwGetFramebufferSize(window, &w, &h);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    // kill window and terminate glfw
    glfwDestroyWindow(window);
    glfwTerminate();
}

inline string tostring(const vec3f &v) { char buf[1024]; sprintf(buf, "vec3f(%0.2f,%0.2f,%0.2f)", v.x, v.y, v.z); return string(buf); }

// runs the raytrace over all tests and saves the corresponding images
int main(int argc, char** argv) {
    
    // test that glfw and glew are linked properly
    uiloop();
    message("GLFW and GLEW seem to work\n\n");
    
    // test max_component function
    vec3f v(1,2,-3);
    float max_val = max_component(v);
    message("Result of max_component: %f\n", max_val);
    
    // test sum of three vectors
    vec3f va(1,0,0);
    vec3f vb(0,4,0);
    vec3f vc(0,0,2);
    vec3f vabc = sum_three(va, vb, vc);
    message("Result of sum_three: %s\n", tostring(vabc).c_str());
    
    // test sum of vectors
    vector<vec3f> vs = {
        {3.14,1.5,2.7},
        {2.71,8.2,8.2},
        {1.61,8.0,3.4},
        {1.41,4.2,1.4},
    };
    vec3f vsum = sum_many(vs);
    message("Result of sum_many: %s\n", tostring(vsum).c_str());
    
    message("\nThis message indicates a successful build!\n\n");
    
    return 0;
}

