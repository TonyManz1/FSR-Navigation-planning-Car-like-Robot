# FSR-Navigation-planning-Car-like-Robot
This repository contains the final technical project of the Field and Service Robotics course held by Prof. Fabio Ruggiero at Federico II, Naples.

## Usage

### Running a simulation

#### For Navigation Planning Only (without Control)
1. Open MATLAB and navigate to the project directory
2. Choose a planning approach (RRT*, A*, Dijkstra)
3. run the main.m file

#### For Navigation Planning with IO feedback linearization controller 
1. Open the Simulink model
2. In Simulink, go to:
- Model Properties > Callbacks > InitFcn
- Uncomment the lines related to the algorithm you want to use:
* For RRT*: Uncomment the RRT* section
* For Dijkstra: Uncomment the Dijkstra section
* For A*: Uncomment the A* section
3. Run the simulation
   
   
   
