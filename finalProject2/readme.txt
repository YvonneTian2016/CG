******Run the code: Input:each json file   
                    Output:png pictures in the same folder
**************************************************************************

******Function Description
1.intersect
  Find the intersection between view point and the object. If there is a hit of the object,then record the intersect point, including its position,the ray and the value of t,etc. 

2.raytrace_ray
  If there is no intersection of object with this ray,just return background color to this point. If there is a intersection, we need to compute the color of the intersection point. The color is depended on the original object color, the ambient light and the material of object, etc. 

3.raytrace
  We need to compute every ray from the view point to the scence, and use the function intersect to get the hit point of object and raytrace_ray to get the color of the hit point.Then we could get all the hit point and draw it to the output image.
************************************************************************

******Cylinder
To add a cylinder shape, I made a json file, 07_Cylinder.json. So to demonstrate my code with a scene containing a Blinn-Phone cylinder, you just change the input json file to 07_Cylinder.json file, and then you will get the 07_Cylinder.png output of the cylinder. 
I also add some elements in the scene.h to make the function work, like add the element height and iscylinder in the struct Surface,etc. 


