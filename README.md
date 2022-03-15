# EEP520 Final C++ Programming project-- 

## Overall Goal: Implementation of Deep first Path Search and Prim Maze Generator Algorithm

The main goal of this project is to realize the Prim maze generate algorithm and deep first search algorithm using C++ enviro environment.

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
A non-recursive implementation of DFS with worst-case space complexity {\displaystyle O(|E|)}O(|E|), with the possibility of duplicate vertices on the stack:[6]

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


Then clone the repo into your directory and make:

```bash
cd $dir/520_project
make
enviro
```

Then navigate to `http://localhost` you should see a rectangular walled area with a blue dot sitting at the up-left corner and an exit at the bottom-right corner.

<img src='https://github.com/jiayi-wang98/520-project/blob/main/maze_gen.jpg' width=70%>

You can click on the `Generate Maze` button at the up-right corner to generate a random maze by Prim Maze Generation Algorithm.

Then you can click on the `Begin Finding the way to the target` button at the up-right corner to start the auto path searching. Since the map is quite big, it should take 5-20 minites to find the exit depending on different maps.

<img src='https://github.com/jiayi-wang98/520-project/blob/main/final.jpg' width=70%>

## Key Challages
The key challages are how to realize the Prim maze generate alforithm and deep first path search algorithm in c++ enviro.







