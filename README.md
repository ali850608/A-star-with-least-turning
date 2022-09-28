# A star with least turning
A modified a star algorithm for topological map to generate a shortest path considering turning weight. 
It calculates the angle between the current vertex' parent, current vertex, and the next vertex.
If it is a turn, add an extra g value to that node.
By doing so, we can get a shortest path considering turning weight. 
## Usage
The code requires the external library [boost](https://www.boost.org/).
If you are using Ubantu, you can install it simply by
```shell script
sudo apt install libboost-all-dev
``` 
If neither of the above method works, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.


After you installed boost and downloaded the source code, go into the directory of the source code and compile it with CMake:
```shell script
cmake .
make
```

Then, you are able to run the code:
```
./ASTAR_least_turning -m twenty_robots -s l06 -g r01
```

- m: the map file in dot format
- s: the start label 
- g: the goal label

You can define your owned turning weight in "A-star-with-least-turning/src/graph_utils.cpp" isTurning founction

The idea is that if the robot is doing turning, then it will add extra g_value to the node. 
Hence, when the a star calculates the shortest path, the turning will be considered. 

## License
PBS is released under USC – Research License. See license.md for further details.

