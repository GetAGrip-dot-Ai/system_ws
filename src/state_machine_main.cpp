/*
CMU MRSD Program: Course 16-681
Team Name: GetAGrip.AI
Team Members: Alec Trela, Jiyoon Park, Sridevi Kaza, Solomon Fenton, & Shri Ishwarya S V
Rev0: Mar. 24, 2023
Code Description: FSM main that controls Peter
*/


#include "state_machine.cpp"

int main(int argc, char **argv){
    // start system node
    ros::init(argc, argv, "peter_node");

    PeterStateMachine Peter;

    Peter.run();

    
}