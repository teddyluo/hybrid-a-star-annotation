# Dubins-Curve-For-MATLAB
# About
A MATLAB version of Dubins' Curve based on [Andrew Walker's](https://github.com/AndrewWalker/Dubins-Curves#shkel01) work

# Intro
A Dubin's curve is a nearly kinemetically feasible path solution for car-like platform. The method explicitly find the trajectory composed of 3 segment: two curves and one staight line, or three curves. The curves are part of the circle. There are only 6 kind of the composition that proved to be minimun in length, and thease 6 types are called Dubin's curve. In the program,  the shourtest path is found and taken as the desired path.   
Despite the kindly source code that Andrew Walker has provided, I'm still tired of searching the method to implement the C++ code into MATLAB using MEX and other compiler. Thus I decided to write my own MATLAB .m file.
# Examples
After switching to corresponding folder that contains both files within the MATLAB, you may use the following command to easily generate a plot that contains a Dubins' curve defined by your input:
```Matlab
pointA = [ 1, 2,   0*pi/180 ];     
pointB = [ 9, 5, 120*pi/180 ];    
TurnRadius = 5;   
PathStep = -1;   
dubins_curve(pointA,pointB, TurnRadius, PathStep);
``` 
By default, the program will plot the path with marked start and end point. The execution time of calculating Dubins path and plotting is also shown in the MATLAB console. Plot is drawn dot by dot and can take much longer then Dubins calculation depends on the setting. To make implementation as a function simpler, you may suppressed the output (both console and plot) by:
```quiet = true;
dubins_curve(pointA,pointB, TurnRadius, PathStep, quiet);
``` 
# References
* Dubins-Curves by Andrew Walker https://github.com/AndrewWalker/Dubins-Curves#shkel01   
* Dubins, L.E. (July 1957). "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents". American Journal of Mathematics 79 (3): 497–516   
* Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins set". Robotics and Autonomous Systems 34 (2001) 179–202   
