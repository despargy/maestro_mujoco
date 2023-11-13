# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O3  -I/home/despinar/mujoco_ws/maestro_mujoco/src/include -I/usr/include/eigen3 -std=c++17 -pthread -Wl,-no-as-needed
# MUJOCO = -I/home/despinar/.mujoco/mujoco-2.3.3/include -L/home/despinar/.mujoco/mujoco-2.3.3/lib
MAESTRO_CPP = main/FSM.cpp main/Robot.cpp main/Leg.cpp main/Controller.cpp main/Data.cpp main/TopLevelControl.cpp main/Math.cpp main/Trajectory.cpp main/Wrapper.cpp

.PHONY: all
all:
	g++ $(COMMON)   main/mainMujoco.cpp -lm -lmujoco -w -lglfw -lnlopt -lorocos-kdl -o exe/main_mujoco $(MAESTRO_CPP)

.PHONY: clean
clean:
	rm -f exe/main_handler

