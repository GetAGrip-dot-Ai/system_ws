#include "state_machine.cpp"

int main(int argc, char **argv){
    // start system node
    ros::init(argc, argv, "system_node");

    PeterStateMachine Peter;

    Peter.run();

    
}