## dependencies

`C++`:
```bash
apt install libeigen3-dev libglpk-dev libtinyxml2-dev
```
`python3` (for displaying the tests results):
* numpy
* matplotlib
* cvxopt
* scipy

## build

To build this, the repository has to be cloned in the src folder of a catkin workspace then use:
```bash
mkdir build
cd build
cmake ..
make
```

## running

Run the code in the build folder using:
```bash
./build/main [OPTIONS]
```
The options are:
* --mode : choose the experiment mode for now the modes are:
  * 1 : (default) generate one robot and compute its stability polyhedron;
  * 2 : compute the balance region for all robot in a folder given by `--robot`
* --robot : allows to give the path to the desired robot xml files. Some exemples are available in /robots. (default 'rospack find stabiliplus'/robots/robot_8.xml);
* --fric_sides : allows to choose the number of sides for the friction cone approximation (default 16).
* --solver : allows to choose the desired solver among GLPK (default), LP_SOLVE and GUROBI.
* --robust : (TRUE | FALSE) if true the robust stability volume will be computed (default) if false the static stability area is computed.


Once the main program has been launched, the results can be displayed using:
```bash
python3 postprocess.py
```
The default output should look like: ![default output](https://github.com/JulienRo7/stabiliplus/blob/master/default_output.png)

## using the projection library

The cmake code also generate a C++ library for projecting a high dimension convex to dimension 2 or 3. You can install it from the build folder using:
```bash
sudo make install
```
Then you should be able to import in you project using cmake. 

The main class to project a convex are `staticStabilityPolytope` and `robustStabilityPolytope` for 2D and 3D respectively. The `problemDescriptor` class is used to give the equalities and inequalities that describe the higher dimension convex. The `contactSet` class is an example on how to use it. This project was first written for an application in balance of humanoid robots, and a paper has been published about it (TODO: add the link of the paper here).  
