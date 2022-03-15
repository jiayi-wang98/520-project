#ifndef __DRONE_AGENT__H
#define __DRONE_AGENT__H 

#include "enviro.h"
#include <vector>
#include <stdlib.h>
#include <algorithm>
using namespace enviro;
#include <json/json.h>

using nlohmann::json;

class Initial: public State, public AgentInterface {
    void entry(const Event& e) {
        std::cout<<"wait for beginning!\n";
    }
    void during() {
        track_velocity(0,0); 
    }
    void exit(const Event& e) {}
};


//maintain memory and decide where to go next according to dfs
class Standby : public State, public AgentInterface {
    public:
    void entry(const Event& e) {
        std::cout<<"enter standby!\n";
        if(initial){
            std::cout<<"Initialize memory!\n";
            for(int i=0;i<29;i++){
                for(int j=0;j<29;j++)
                    has_visited[i][j]=false;
            }
            has_visited[0][0]=true;
            coordinate.first=0;
            coordinate.second=0;
            path_stack.push_back(coordinate);
            x_i=0;
            y_i=0;

            //initial point stack
            if(sensor_value(1)>25) {
                has_visited[0][1]=true;
                coordinate.first=0;
                coordinate.second=1;
                point_stack.push_back(coordinate);
                point_angle_stack.push_back(1);   
            }

            //initial point stack
            if(sensor_value(0)>25) {
                has_visited[1][0]=true;
                coordinate.first=1;
                coordinate.second=0;
                point_stack.push_back(coordinate);
                point_angle_stack.push_back(0);
                path_stack.push_back(coordinate);
            }
            initial=false;
            emit(Event("begin"));
        }
        else {
            direction=angle_to_direction(angle()); 
            x_i=map_to_index(x());
            y_i=map_to_index(y());
            if(x_i==28 && y_i==28) {
                std::cout<<"=================Found the exit!!!!!!!!!===================\n";
                emit(Event("exit"));
            }
            coordinate.first=x_i;
            coordinate.second=y_i;
            auto in_point=find(point_stack.begin(),point_stack.end(),coordinate);
            if(in_point!=point_stack.end()){
                point_stack.erase(in_point);
                point_angle_stack.erase(in_point-point_stack.begin()+point_angle_stack.begin());
            }
            x_f=x_i;
            x_l=x_i;
            x_r=x_i;
            y_f=y_i;
            y_l=y_i;
            y_r=y_i;
            std::cout<<"update info! x_i= "<<x_i<<" y_i="<<y_i<<"\n";
            std::pair<int,int> coordinate;
            
            switch(direction){
            case 0:{
                x_f+=1; //forward
                y_r+=1; //right
                y_l-=1; //left
                break;
            }
            case 1:{
                y_f+=1;
                x_r-=1;
                x_l+=1;
                break;
            }
            case 2:{
                x_f-=1;
                y_r-=1;
                y_l+=1;
                break;
            }
            case 3:{
                y_f-=1;
                x_r+=1;
                x_l-=1;
                break;
            }
            default:break;
            }
            //TODO decide where to go
            //check front
            //std::cout<<"check around! "<<sensor_value(0)<<"  "<<sensor_value(1)<<"  "<<sensor_value(2)<<"  ";
            //std::cout<<"check around! "<<check_point(x_f,y_f)<<"  "<<check_point(x_r,y_r)<<"  "<<check_point(x_l,y_l)<<"  ";
            //check neighbour points to add to point_stack
            int action=3;
            //check left block
            if(sensor_value(2)>25 && check_point(x_l,y_l)) {
                coordinate.first=x_l;
                coordinate.second=y_l;
                has_visited[x_l][y_l]=true;
                auto in_point=find(point_stack.begin(),point_stack.end(),coordinate);
                if(in_point==point_stack.end()){
                    point_stack.push_back(coordinate);
                    point_angle_stack.push_back((direction+3)%4);
                }
                //emit(Event("turn left"));
                action=2;
            }

            //check right block
            if(sensor_value(1)>25 && check_point(x_r,y_r)) {
                coordinate.first=x_r;
                coordinate.second=y_r;
                has_visited[x_r][y_r]=true;
                auto in_point=find(point_stack.begin(),point_stack.end(),coordinate);
                if(in_point==point_stack.end()){
                    point_stack.push_back(coordinate);
                    point_angle_stack.push_back((direction+1)%4);
                }
                //emit(Event("turn right"));
                action=1;
            }

            //check forward block
            if(sensor_value(0)>25 && check_point(x_f,y_f)) {
                coordinate.first=x_f;
                coordinate.second=y_f;
                has_visited[x_f][y_f]=true;
                auto in_point=find(point_stack.begin(),point_stack.end(),coordinate);
                if(in_point==point_stack.end()){
                    point_stack.push_back(coordinate);
                    point_angle_stack.push_back(direction);
                }
                //emit(Event("move forward"));
                action=0;
            }
            
            std::cout<<"point stack size = "<<point_stack.size()<<"\n";
            std::cout<<"path stack size = "<<path_stack.size()<<"\n";

            //else emit(Event("turn around"));
            switch (action){
                case 0:{
                    //move forward
                    //record path
                    coordinate.first=x_f;
                    coordinate.second=y_f;
                    auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                    if(in_path==path_stack.end())
                        path_stack.push_back(coordinate);
                    emit(Event("move forward"));
                    break;
                }
                case 1:{
                    //move right
                    //record path
                    coordinate.first=x_r;
                    coordinate.second=y_r;
                    auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                    if(in_path==path_stack.end())
                        path_stack.push_back(coordinate);
                    emit(Event("turn right"));
                    break;
                }
                case 2:{
                    //move left
                    //record path
                    coordinate.first=x_l;
                    coordinate.second=y_l;
                    auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                    if(in_path==path_stack.end())
                        path_stack.push_back(coordinate);
                    emit(Event("turn left"));
                    break;
                }
                default:{
                    //move back to the last point in stack, teleport first, then try to navigate
                    if(move_back==false){
                        //record the move back target
                        xt_back=point_stack.back().first;
                        yt_back=point_stack.back().second;
                        dir_back=point_angle_stack.back();
                        std::cout<<"begin back trace to ( "<<xt_back<<" , "<<yt_back<<" )!\n";
                        point_angle_stack.pop_back();
                        point_stack.pop_back();
                        move_back=true;
                        path_stack.pop_back();
                        emit(Event("turn around")); //turn around and move back 1 unit
                    }

                    //moving back on going
                    else
                    {
                        //check if the target is reachable

                        //1. the target is near the current block
                        if((x_i==xt_back && ((y_i==(yt_back+1))||(y_i==(yt_back-1))))||(y_i==yt_back && ((x_i==(xt_back+1))||(x_i==(xt_back-1))))){
                            std::cout<<"found the next branch!\n";
                            //forward
                            if(dir_back==direction){
                                //move forward
                                has_visited[x_f][y_f]=true;
                                //record path
                                coordinate.first=x_f;
                                coordinate.second=y_f;
                                auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                                if(in_path==path_stack.end())
                                    path_stack.push_back(coordinate);
                                move_back=false;
                                emit(Event("move forward"));
                            }
                            else if (dir_back==((direction+1)%4)){
                                //turn right
                                has_visited[x_r][y_r]=true;
                                //record path
                                coordinate.first=x_r;
                                coordinate.second=y_r;
                                auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                                if(in_path==path_stack.end())
                                    path_stack.push_back(coordinate);
                                move_back=false;
                                emit(Event("turn right"));
                            }
                            else if (((dir_back+1)%4)==direction){
                                //turn left
                                has_visited[x_l][y_l]=true;
                                //record path
                                coordinate.first=x_l;
                                coordinate.second=y_l;
                                auto in_path=find(path_stack.begin(),path_stack.end(),coordinate);
                                if(in_path==path_stack.end())
                                    path_stack.push_back(coordinate);
                                move_back=false;
                                emit(Event("turn left"));
                            }
                            //should not be here
                            else std::cout<<"error found in detect trace point!\n";
                        }

                        //still move back
                        else {
                            //pop this path
                            path_stack.pop_back();
                            //get the next target point
                            int x_n,y_n;
                            x_n=path_stack.back().first;
                            y_n=path_stack.back().second;
                            std::cout<<"still trace back! to ( "<<x_n<<" , "<<y_n<<" )!\n";
                            //check the direction and move towards it

                            //1 up and down
                            if(x_n==x_i){
                                //down
                                if(y_n==(y_i+1)){
                                    //check my angle
                                    if(direction==1){
                                        emit(Event("move forward"));
                                    }
                                    else if (direction==0){
                                        emit(Event("turn right"));
                                    }
                                    else if (direction==2){
                                        emit(Event("turn left"));
                                    }
                                    else std::cout<<"error found in trace back down!\n";
                                }
                                //up
                                else if(y_n==(y_i-1)){
                                    if(direction==3){
                                        emit(Event("move forward"));
                                    }
                                    else if (direction==2){
                                        emit(Event("turn right"));
                                    }
                                    else if (direction==0){
                                        emit(Event("turn left"));
                                    }
                                    else std::cout<<"error found in trace back up!\n";
                                }
                            }
                            //left or right
                            else if(y_n==y_i){
                                //right
                                if(x_n==(x_i+1)){
                                    //check my angle
                                    if(direction==0){
                                        emit(Event("move forward"));
                                    }
                                    else if (direction==3){
                                        emit(Event("turn right"));
                                    }
                                    else if (direction==1){
                                        emit(Event("turn left"));
                                    }
                                    else std::cout<<"error found in trace back right!\n";
                                }
                                //left
                                else if(x_n==(x_i-1)){
                                    if(direction==2){
                                        emit(Event("move forward"));
                                    }
                                    else if (direction==1){
                                        emit(Event("turn right"));
                                    }
                                    else if (direction==3){
                                        emit(Event("turn left"));
                                    }
                                    else std::cout<<"error found in trace back left!\n";
                                }
                            }
                        }
                        
                    }
                    break;
                }
            }
        }
    }
    void during() {
        track_velocity(0,0);
    }
    void exit(const Event& e) {}

    bool check_point(int x,int y){
        if (x<0 or y<0 or x>28 or y>28) return false; //outside map
        else if(has_visited[x][y]==true) return false;
        else return true;
    }

    private:
    //return the absolute direction of the drone. (0 to right. 1 to down. 2 to left. 3 to up)
    int angle_to_direction(double angle){
        int abs_angle=(int)(100*(6.28+angle))%628;
        if((abs_angle>=0 && abs_angle<78)||(abs_angle>549 && abs_angle<=627)) return 0;
        else if (abs_angle>=78 && abs_angle<235) return 1;
        else if (abs_angle>=235 && abs_angle<392) return 2;
        else if (abs_angle>=392 && abs_angle<549) return 3;
        else return -1;
    }

    //return the matrix index of the current block
    int map_to_index(double x){
        int x_index=((int)(x+290)/20);
        return x_index;
    }

    //return the map coordinate of the current block
    double index_to_map(int x){
        double x_co=(double)x*20.0-280;
        return x_co;
    }
    std::pair<int,int> coordinate;
    std::vector<std::pair<int,int>> path_stack;
    std::vector<std::pair<int,int>> point_stack;
    std::vector<int> point_angle_stack;
    bool has_visited[29][29]={false};
    bool initial=true;
    bool move_back=false;

    //record current index and angle
    int direction; 
    int x_i;
    int y_i;

    //index of neighbour point
    int x_f,x_l,x_r;
    int y_f,y_l,y_r;

    //record target point when moving back
    int xt_back,yt_back,dir_back;
};

//move 1 unit ahead
class MovingForward : public State, public AgentInterface {
    public:
    void entry(const Event& e) {
        std::cout<<"enter move forward!\n";
        //get direction and current location and target
        direction=angle_to_direction(angle()); 
        x_i=map_to_index(x());
        y_i=map_to_index(y());
        switch(direction){
            case 0:{
                x_i+=1;
                break;
            }
            case 1:{
                y_i+=1;
                break;
            }
            case 2:{
                x_i-=1;
                break;
            }
            case 3:{
                y_i-=1;
                break;
            }
            default:break;
        }
        std::cout<<"target ("<<x_i<<" , "<<y_i<<" )\n";
    }
    void during() {
        // Go forward one unit
        //if blocked, stop
        if ( sensor_value(0) < 12 ) {
            std::cout<<"emit blockage"<<"\n";
            emit(Event("blockage"));
        }
        //else move
        else{
        //move
            //std::cout<<"move to ("<<index_to_map(x_i)<<" , "<<index_to_map(y_i)<<" )\n";
            move_toward(index_to_map(x_i),index_to_map(y_i),15,5);

        //check if the destination has been reached
            double diff_x=x()-index_to_map(x_i);
            double diff_y=y()-index_to_map(y_i);
            if(diff_x<=4 && diff_x>=-4 && diff_y<=4 && diff_y>=-4){
                std::cout<<"target reached!\n";
                emit(Event("target reached"));
            }
        }
    }
    void exit(const Event& e) {
        x_i=map_to_index(x());
        y_i=map_to_index(y());
        //std::cout<<"exit move forward with ("<<x()<<" , "<<y()<<" )!\n";
        std::cout<<"exit move forward with ("<<map_to_index(x())<<" , "<<map_to_index(y())<<" )!\n";
    }

    //return the absolute direction of the drone. (0 to right. 1 to down. 2 to left. 3 to up)
    int angle_to_direction(double angle){
        int abs_angle=(int)(100*(6.28+angle))%628;
        if((abs_angle>=0 && abs_angle<78)||(abs_angle>549 && abs_angle<=627)) return 0;
        else if (abs_angle>=78 && abs_angle<235) return 1;
        else if (abs_angle>=235 && abs_angle<392) return 2;
        else if (abs_angle>=392 && abs_angle<549) return 3;
        else return -1;
    }

    //return the matrix index of the current block
    int map_to_index(double x){
        int x_index=((int)(x+290)/20);
        return x_index;
    }

    //return the map coordinate of the current block
    double index_to_map(int x){
        double x_co=(double)x*20.0-280;
        return x_co;
    }

    int direction; 
    int x_i;
    int y_i;
};

class RotatingRight : public State, public AgentInterface {
    public:
    void entry(const Event& e) {
        std::cout<<"enter turn right!\n";
        pre_angle=angle();
    }
    void during() {
        track_velocity(0,5); // Rotate
        if ((angle()-pre_angle)>1.4){ //if has rotate 90 degree
            //std::cout<<"angle "<<angle()<<" pre "<< pre_angle<<"\n";
            //if (sensor_value(0) > 25 ) { //if has path, go straight forward
            //std::cout<<"sensor > 25 emit move forward"<<"\n";
            emit(Event("move forward"));
            //}
            //else pre_angle=angle(); //else rotate 90 degree again
        }

    }
    void exit(const Event& e) {}
    double pre_angle;
};

class RotatingLeft : public State, public AgentInterface {
    public:
    void entry(const Event& e) {
        std::cout<<"enter turn left!\n";
        pre_angle=angle();
    }
    void during() {
        track_velocity(0,-5); // Rotate
        if ((pre_angle-angle())>1.4){ //if has rotate 90 degree (should be 157 in precise, leave 4 margin here)
            //std::cout<<"angle "<<angle()<<" pre "<< pre_angle<<"\n";
            //if (sensor_value(0) > 25 ) { //if has path, go straight forward
            //std::cout<<"sensor > 25 emit ping"<<"\n";
            emit(Event("move forward"));
            //}
            //else pre_angle=angle(); //else rotate 90 degree again
        }

    }
    void exit(const Event& e) {}
    double pre_angle;
};

class TurnAround : public State, public AgentInterface {
    public:
    void entry(const Event& e) {
        std::cout<<"enter turn around!\n";
        pre_angle=angle();
    }
    void during() {
        track_velocity(0,5); // Rotate
        //std::cout<<"angle "<<angle()<<" pre "<< pre_angle<<"\n";
        if ((angle()-pre_angle)>3){ //if has rotate 90 degree
            //std::cout<<"angle "<<angle()<<" pre "<< pre_angle<<"\n";
            //if (sensor_value(0) > 25 ) { //if has path, go straight forward
            //std::cout<<"sensor > 25 emit move forward"<<"\n";
            emit(Event("move forward"));
            //}
            //else pre_angle=angle(); //else rotate 90 degree again
        }

    }
    void exit(const Event& e) {}
    double pre_angle;
};


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

class droneInit : public Process, public AgentInterface {

    public:
    droneInit() : Process(), AgentInterface() {}

    void init() {
        watch("button_click", [this](Event e){
            if (e.value()["value"]=="find") { 
                std::cout<<"emit begin"<<"\n";
                emit(Event("start"));
            }
        });
    }
    
    void start() {}
    void update() {}
    void stop() {}
};

class drone : public Agent {
    public:
    drone(json spec, World& world) : Agent(spec, world) {
        add_process(c);
        add_process(d);
    }
    private:
    droneSM c;
    droneInit d;
};

DECLARE_INTERFACE(drone)

#endif