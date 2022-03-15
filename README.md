# EEP520 Final C++ Programming project-- 

## Overall Goal: Implementation of Prim Maze Generator Algorithm and Simulate a Drone Finding Path Out with Deep First Path Search.

The main goal of this project is to realize the Prim maze generate algorithm and deep first search algorithm using C++ enviro environment. Simulating a drone insidde this maze trying to find a wayout using DFS.

### Maze Generator
Randomized Prim's algorithm
This algorithm is a randomized version of Prim's algorithm.

1. Start with a grid full of walls.
2. Pick a cell, mark it as part of the maze. Add the walls of the cell to the wall list.
3. While there are walls in the list:
    - Pick a random wall from the list. If only one of the cells that the wall divides is visited, then:
        - Make the wall a passage and mark the unvisited cell as part of the maze.
        - Add the neighboring walls of the cell to the wall list.
    - Remove the wall from the list.

### Deep First Path Search
A non-recursive implementation of DFS with worst-case space complexity O(|E|), with the possibility of duplicate vertices on the stack:

procedure DFS_iterative(G, v) is

    let S be a stack
    S.push(v)
    while S is not empty do
        v = S.pop()
        if v is not labeled as discovered then
            label v as discovered
            for all edges from v to w in G.adjacentEdges(v) do 
                S.push(w)

more information about algorithms see wikipedia here. 

Maze generator: https://en.wikipedia.org/wiki/Maze_generation_algorithm 

Deep First Search: https://en.wikipedia.org/wiki/Depth-first_search

## Getting Started
To start Maze Generation and Auto Path Search, excute the following command on your terminal:

```bash
docker run -p80:80 -p8765:8765 -v $PWD:/source -it klavins/enviro:v1.6 bash
esm start
```

The program is based on `enviro` environment, see here for more information about `enviro`: https://github.com/klavinslab/enviro

Then clone the repo into your directory and make:

```bash
cd $dir/520_project
esm start
make
enviro
```

Then navigate to `http://localhost` you should see a rectangular walled area with a blue drone sitting at the up-left corner and an exit at the bottom-right corner.

<img src='https://github.com/jiayi-wang98/520-project/blob/main/maze_gen.jpg' width=70%>

You can click on the `Generate Maze` button at the up-right corner to generate a random maze by Prim Maze Generation Algorithm.

Then you can click on the `Begin Finding the way to the target` button at the up-right corner to start the auto path searching. Since the map is quite big, it should take 5-20 minites to find the exit depending on different maps.

<img src='https://github.com/jiayi-wang98/520-project/blob/main/found.jpg' width=70%>

Tips and Problems:

The initial thinking of `Clear Maze` button is trying to clear the maze and re-generate one after the completion of the search. But it always has `std::bad_function_call` error whatever I try to fix it. So in order to try another maze, just `Ctrl+C` to terminate the current process and use `enviro` to create a new one.

## Key Challages
The key challages are how to realize the Prim maze generate alforithm and deep first path search algorithm in c++ enviro.

### Maze generator
How to represent the wall and generate it randomly?

Solution: 
1. Create a static agent called *wall*, respond to `button_click`.
2. Use 29*29 matrix `wall_location` to represent the map location, 1 means wall, 0 means road.
3. Implement Prim's algorithm on the matrix to generate the wall map.
4. According to the wall map, place wall agent at the corresponding position.

### Deep First Search
A. How to get the wall pass information?

Solution: 

Use range sensor. Suppose a map unit=20, place the sensor at the center of the drone. If the range sensor value < 10, then it means its wall in front of the drone. This is implemented in src/wall.h file.

B. How to control the move of the drone according to the DFS results?

Solution: 

I tried to let the drone move one unit once. It will go into a state machine to control the overall behaviour.

```c++
class droneSM : public StateMachine, public AgentInterface {

    public:
    //droneController() : Process(), AgentInterface() {}
    droneSM() : StateMachine() {
    set_initial(initial);
    add_transition("start", initial, standby);
    add_transition("exit", standby,initial);
    add_transition("begin", standby, moving_forward);
    add_transition("move forward", standby, moving_forward);
    add_transition("blockage", moving_forward, standby);
    add_transition("target reached", moving_forward, standby);
    add_transition("turn right", standby, rotating_right);
    //add_transition("turn right", moving_forward, rotating_right);
    add_transition("turn around", standby, turn_around);
    add_transition("turn left", standby, rotating_left);
    add_transition("move forward", rotating_right, moving_forward);
    add_transition("move forward", rotating_left, moving_forward);
    add_transition("move forward", turn_around, moving_forward);
    }

    MovingForward moving_forward;
    RotatingRight rotating_right;
    RotatingLeft rotating_left;
    Standby standby;
    TurnAround turn_around;
    Initial initial;
};
```

3. How to trace back if the drone comes to an end of a path?

Solution: Maintain a path stack `std::vector<std::pair<int,int>> path_stack` to record the path the drone has passed, and trace back one by one step to the last cross-section that has not been fully explored. This is implemented in src/drone.h file.


## File Breakdown

```
520-project
   |--src    //source file containing drone.h and wall.h
   |   |--drone.h //drone behaviour with DFS
   |   |--drone.cc //nothing
   |   |--wall.h  //wall generate with Prim's algorithm
   |   |--wall.cc //nothing
   |   |--README.md
   |--defs    //physical design
   |   |--drone.json //drone physical define
   |   |--wall.json  //wall physical define
   |   |--README.md
   |--lib     //lib
   |   |--...
   |--config.json   //contains the initial agent definition and map definition
   |--Makefile
   |--README.md
```   
   









