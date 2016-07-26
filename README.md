# rsband_local_planner
The rsband_local_planner combines an **elastic band planner**, a **reeds shepp planner** and  
a **fuzzy logic based path tracking controller**, to achieve reactive local planning for car like and
4WS robots.

#### External Dependencies
- [fuzzylite](https://github.com/fuzzylite/fuzzylite):  fuzzy logic control library,
required for the path tracking controller

<br/>

##### How to install fuzzylite
```
$ git clone git@github.com:fuzzylite/fuzzylite.git
$ cd fuzzylite/fuzzylite
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```
