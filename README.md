<h1 align="center">
  <br>
  <a href="https://github.com/despargy/maestro_mujoco"><img src="maestro_mujoco.drawio.png" alt="Maestro Mujoco" width="600"></a>
  <br>
  Body Posture and Movement in Quadruped Robots for  <a href="https://mujoco.org/" target="_blank">Mujoco</a>
  <br>
</h1>

<h3 align="center"> Trajectory tracking and locomotion control in quadrupeds.</h3> <h4 align="left"> An analytical solution combined with weighted functions for possible adaptation (upcoming) in slippery terrains .</h4> 

<p align="center">
  <a href="#key-features">Key Features</a> •
  <a href="#releated-packages">Releated Packages</a> •
  <a href="#how-to-use">How To Use</a> •
  <a href="#download">Download</a> •
  <a href="#contact">Contact</a> •
  <a href="#license">License</a>
</p>

## Key Features

Implementing a trajectory tracking controller featuring a potential (upcoming) weighted control effort distribution and time scaling, specifically designed for slippery terrains.

Proposing a novel analytical solution for locomotion, utilizing a weighted control effort distribution for the swinging leg.

## Releated Packages

The current project's package version, developed for Gazebo and/or the Unitree GO1 robot, is available at <a href="https://github.com/despargy/maestro/tree/master" target="_blank">Maestro ROS</a>
based on previous work for slip detection 
<a href="https://github.com/MichaelMarav/ProbabilisticContactEstimation" target="_blank">Probabilistic Contact Estimation</a>.

## How To Use
#### Specify the local workspace path within the code lines.: 
   * line 6, src/Data.cpp
   * line 21, src/trajMujoco.cpp
   * line 21, src/walkMujoco.cpp

All .
   ```sh
   cd maestro_mujoco/src/cmake 
   mkdir build
   cd build
   cmake ..
   make
   ```

Trajecotry tracking execution.
 ```sh
   cd maestro_mujoco/src/cmake/build
   ./traj  # Run trajectory
   cd maestro_mujoco/src/creat_plots
   python3 plot_traj.py # Vizualize results
   ```

Locomotion execution.
 ```sh
   cd maestro_mujoco/src/cmake/build
   ./walk # Run locomotion
   cd maestro_mujoco/src/creat_plots
   python3 plot_loc.py # Vizualize results
   ```

  
## Download

   ```sh
   git clone https://github.com/despargy/maestro_mujoco.git
   ```

## Contact
   Despina-Ekaterini Argiropoulos - despinar@ics.forth.gr         

[![LinkedIn][linkedin-shield]][linkedin-url] 


[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]:https://www.linkedin.com/in/despar/


## License

MIT License