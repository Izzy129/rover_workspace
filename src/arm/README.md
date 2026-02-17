nav team put ur packages here

do not copy install, log, or build folders here!
only source files (like package.xml)


once copied, go back to root directory, and resolve dependencies with rosdep

```
rosdep install --from-paths src --ignore-src -r -y
```

then build with 
```
colcon build
```