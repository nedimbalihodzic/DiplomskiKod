# DiplomskiKod

# Requirements 
- MuJoCo binaries - it can be installed from this link https://github.com/deepmind/mujoco
- GLFW library - it can be installed from this link https://www.glfw.org/docs/latest/index.html (version 3.3)
- x64 Native Tools Command Prompt for VS 2022

# How to include GLFW library in MuJoCo 
- Paste glfw3dll.lib inside MuJoCo lib folder
- Paste GLFW with cpp header files inside MuJoCo include folder

# How to create project 
- Create folder named myproject inside MuJoCo folder
- Create folder named quadrotor inside myproject folder
- Paste files from GitHub inside quadrotor project

# Simulation with Windows 
- Navigate to quadrotor folder through x64 Native Tools Command Prompt for VS 2022
- type: run_win

# Simulation with Linux or macOS 
- comment lines related to WINDOWS in makefile, and uncomment LINUX or MAC
- navigate to quadrotor folder and type run_unix


