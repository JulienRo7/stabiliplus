## dependencies

```bash
apt install libeigen3-dev libglpk-dev libtinyxml2-dev
```

## build

To build this, the repository has to be cloned in the src folder of a catkin workspace then use:
```bash
catkin_make
```

## running

Run the code in the build folder using:
```bash
rosrun stabiliplus stabiliplus [OPTIONS]
```
The options are:
* --mode : choose the experiment mode for now the modes are:
  * 1 : (default) generate one robot and compute its stability polyhedron;
  * 2 : NOT IMPLEMENTED YET! generate a timming "benchmark";
  * 3 : Compute a sequence of stability polyhedron for the postprocess to generate an animation;
* --robot : allows to give the path to the desired robot xml files. Some exemples are available in /robots. (default 'rospack find stabiliplus'/robots/robot_8.xml);
* --fric_sides : allows to choose the number of sides for the friction cone approximation (default 16).


Once the main program has been launched, the results can be displayed using:
```bash
roscd stabiliplus
python3 postprocess.py
```
Required libraries:
* numpy;
* matplotlib.
