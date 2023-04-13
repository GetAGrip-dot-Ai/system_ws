#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include "system_ws/harvest.h"
#include "std_msgs/Int16.h"

enum State{
    MOVE_2_RESET_POSE, // state 0
    APPROACH_PLANT_POSITIONS, // state 1
    DETECT_PEPPERS, // state 2
    CREATE_PEPPER_OBSTACLES, // state 3
    CHOOSE_PEPPER, // state 4
    REMOVE_PEP_OBS_N_MOVE, // state 5
    DETECT_PEDUNCLES, // state 6
    MOVE_2_PREGRASP, // state 7
    OPEN_END_EFFECTOR, // state 8
    MOVE_2_POI, // state 9
    EXTRACT_PEPPER, // state 10
    MOVE_2_BASKET, // state 11
    OPEN_GRIPPER_CLOSE_EE, // state 12
    ALL_POSITIONS_APPROACHED, // state 13
    MANUAL_INTERVENTION // state 14
};


class PeterStateMachine {

    public:

        PeterStateMachine();
        // ~PeterStateMachine();

        void run();

    private:

        // node to host the clients, pubs, subs
        ros::NodeHandle PeterNode;
        ros::ServiceClient manipulation_client;
        ros::ServiceClient perception_client;
        ros::Publisher ee_client_pub;
        ros::Subscriber ee_client_sub;
        void endEffectorCallback(const std_msgs::Int16& rsp);

        // create a timer that checks the state 
        ros::Timer state_checker;
        // how long the timer will check
        double checker_time = 0.5;

        // the service type that holds service communications
        system_ws::harvest harvest_srv;
        // the pub/sub object to work over the pseudoservice 
        std_msgs::Int16 harvest_req;
        int ee_response;
        

        // current state object to track our states
        State current_state;
        void stateCheckerCallback(const ros::TimerEvent& event);
        // next state to check transitions
        State next_state;

        // returns the state as a string
        std::string stateToString(State state);

        // returns the staet as an int
        int stateToInt(State state){return static_cast<int>(state);}

        // Prints the transition
        void printTransition(State current_state, State next_state, bool failure, bool attempt);
        void printTransition(State current_state, State next_state, bool failure);
        void printTransition(State current_state, State next_state);
};
