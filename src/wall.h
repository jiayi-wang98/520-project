#ifndef __WALL_AGENT__H
#define __WALL_AGENT__H 

#include "enviro.h"
#include <vector>
#include <stdlib.h>
#include <algorithm>
using namespace enviro;

class wallController : public Process, public AgentInterface {

    public:
    wallController() : Process(), AgentInterface(){}

    void init() {
        watch("button_click", [this](Event e){
            if (e.value()["value"]=="generate") {
                
                //delete existing maze first
                while(id_pool.size()>0){
                    std::cout<<"clear "<<id_pool.size()<<" walls\n";
                    remove_agent(id_pool.back());
                    id_pool.pop_back();
                }
                
                std::cout << "Wall remaining = "<<id_pool.size()<<"\n"; 
                //generate new maze
                std::cout << "Generating new random map\n"; 
                j["fill"]="gray";
                j["stroke"]="none";
                srand (time(0));
                prim_maze_gen();
                for(int x=0;x<29;x++){
                    for(int y=0;y<29;y++){
                        if(wall_location[x][y]==1) {
                            Agent& wall=add_agent("wall", index_to_map(x), index_to_map(y), 0,j);
                            id_pool.push_back(wall.get_id());
                        }
                    }
                }
                std::cout<<"generate "<<id_pool.size()<<" walls\n";
            }
        });
    
    }
    void start() {}
    void update() {}
    void stop() {}
    json j;

    private:
    //translate the index to map coordinate
    int index_to_map(int index){
        return (-280+20*index);
    }

    //judge if the place is reachable or have been reached during random route
    bool goal_check(int index_x,int index_y){
        if (index_x<0 or index_y<0 or index_x>28 or index_y>28) return false;
        else return true;
    }

    //generate random maze map
    void prim_maze_gen(){
        std::vector<std::pair<int,int>> collection; //restore the points to be explored
        std::pair<int,int> coordinate;
        
        // initialize wall matrix
        for(int i=0;i<29;i++){
            for(int j=0;j<29;j++)
                {
                wall_location[i][j]=1;
                not_explored[i][j]=true;
            }
        }
        
        // begin generate ramdom route
        //start_point
        int x=0;
        int y=0;
        not_explored[x][y]=false;
        wall_location[x][y]=0;

        //push the first two points
        coordinate.first=x+2;
        coordinate.second=y;
        collection.push_back(coordinate);
    
        coordinate.first=x;
        coordinate.second=y+2;
        collection.push_back(coordinate);


        //begin prim finding loop
        //find until the collection is empty
        int iteration=0;
        while (collection.size()!=0){ 
            //std::cout<<"size of collection="<<collection.size()<<std::endl;
            //std::cout<<"iteration:"<<iteration<<"\n";
            iteration++;
            //randomly choose one point from the collection
            int rand_index=rand()%collection.size();
            x=collection[rand_index].first;
            y=collection[rand_index].second;

            //remove this point from the collection
            collection.erase (collection.begin()+rand_index);

            //mark as explored
            not_explored[x][y]=false;
            wall_location[x][y]=0;
            
            //record connectable neighbour points
            int num_point_has_explored=0;
            std::vector<std::pair<int,int>> to_connect;

            if(goal_check(x+2,y)) {
                coordinate.first=x+2;
                coordinate.second=y;
                if(not_explored[x+2][y]==false){ //which means the point has been connected
                    //std::cout<<"("<<x+2<<","<<y<<") has been connected\n";
                    num_point_has_explored+=1;
                    to_connect.push_back(coordinate); //ready to connect this point
                }
                else if(find(collection.begin(),collection.end(),coordinate)==collection.end()){ //have not explored and not in collection, add it to collection
                    collection.push_back(coordinate);
                    //std::cout<<"add_to_collection ("<<x+2<<","<<y<<")\n";
                }
            }
            if(goal_check(x-2,y)) {
                coordinate.first=x-2;
                coordinate.second=y;
                if(not_explored[x-2][y]==false){ //which means the point has been connected
                    num_point_has_explored+=1;
                    to_connect.push_back(coordinate); //ready to connect this point
                }
                else if(find(collection.begin(),collection.end(),coordinate)==collection.end()){ //have not explored and not in collection, add it to collection
                    collection.push_back(coordinate);
                    //std::cout<<"add_to_collection ("<<x-2<<","<<y<<")\n";
                }
            }
            if(goal_check(x,y+2)) {
                coordinate.first=x;
                coordinate.second=y+2;
                if(not_explored[x][y+2]==false){ //which means the point has been connected
                    num_point_has_explored+=1;
                    to_connect.push_back(coordinate); //ready to connect this point
                }
                else if(find(collection.begin(),collection.end(),coordinate)==collection.end()){ //have not explored and not in collection, add it to collection
                    collection.push_back(coordinate);
                    //std::cout<<"add_to_collection ("<<x<<","<<y+2<<")\n";
                }
            }
            if(goal_check(x,y-2)) {
                coordinate.first=x;
                coordinate.second=y-2;
                if(not_explored[x][y-2]==false){ //which means the point has been connected
                    num_point_has_explored+=1;
                    to_connect.push_back(coordinate); //ready to connect this point
                }
                else if(find(collection.begin(),collection.end(),coordinate)==collection.end()){ //have not explored and not in collection, add it to collection
                    collection.push_back(coordinate);
                    //std::cout<<"add_to_collection ("<<x<<","<<y-2<<")\n";
                }
            }
            //end finding neighbour points
            //begin connection
            int rand_connect=rand()%to_connect.size();
            wall_location[(x+to_connect[rand_connect].first)/2][(y+to_connect[rand_connect].second)/2]=0;// break the wall between these two points
            //std::cout<<"find "<<to_connect.size()<<" points to connect\n";
            //std::cout<<"connect ("<<(x+to_connect[rand_connect].first)/2<<","<<(y+to_connect[rand_connect].second)/2<<")\n";
        }
    }


    bool not_explored[29][29];//map matrix record the coordinate that has been visited
    std::vector<int> id_pool;//record wall id for deleting
    int wall_location[29][29];
    
};

class wall : public Agent {
    public:
    wall(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    wallController c;
};

DECLARE_INTERFACE(wall)

#endif