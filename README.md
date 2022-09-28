# A star with least turning
A modified a star algorithm for topological map to generate a shortest path considering turning counts. 

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


## License
PBS is released under USC – Research License. See license.md for further details.

