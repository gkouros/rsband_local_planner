# rsband_local_planner
The rsband_local_planner combines an **elastic band planner**, a **reeds shepp planner** and a  
**fuzzy logic based path tracking controller**, to achieve reactive local planning for Car-Like  
robots with Ackermann or 4-Wheel-Steering.

#### Important Dependencies
- [eband_local_planner](https://github.com/utexas-bwi/eband_local_planner): Elastic Band Algorithm implementation used to dynamically deform the global path
- [OMPL](https://github.com/ompl/ompl): Motion planning library, that contains a Reeds-Shepp State Space used in the Reeds-Shepp Path Planner
- [fuzzylite](https://github.com/fuzzylite/fuzzylite):  Fuzzy logic control library, used in the fuzzy path tracking controller

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

#### [Documentation](https://gkouros.github.io/rsband_local_planner/doc/html/index.html)

