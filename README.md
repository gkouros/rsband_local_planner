# rsband_local_planner
The rsband_local_planner combines an **elastic band planner**, a **reeds shepp planner** and a
**fuzzy logic based path tracking controller**, to achieve reactive local planning for Car-Like
robots with Ackermann or 4-Wheel-Steering.

#### Important Dependencies
- [eband_local_planner](https://github.com/utexas-bwi/eband_local_planner): Elastic Band Algorithm implementation used to dynamically deform the global path
- [OMPL](https://github.com/ompl/ompl): Motion planning library, that contains a Reeds-Shepp State Space used in the Reeds-Shepp Path Planner
- [fuzzylite](https://github.com/fuzzylite/fuzzylite):  Fuzzy logic control library, used in the fuzzy path tracking controller


#### How to install fuzzylite
```
$ git clone git@github.com:fuzzylite/fuzzylite.git
$ cd fuzzylite/fuzzylite
$ git checkout fuzzylite-6.x
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

#### [Documentation](https://gkouros.github.io/rsband_local_planner/doc/html/index.html)

#### References
- M. Khatib et al. “Dynamic path modification for car-like
nonholonomic mobile robots”. In: Robotics and Automation, 1997.
Proceedings., 1997 IEEE International Conference on. Vol. 4. Apr. 1997,
2920–2925 vol.4. DOI: 10.1109/ROBOT.1997.606730
