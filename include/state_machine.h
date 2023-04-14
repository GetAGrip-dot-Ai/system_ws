#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include "system_ws/harvest.h"
#include "std_msgs/Int16.h"

enum State{
    MOVE_2_RESET_POSE, // state 0
    APPROACH_PLANT_POSITIONS, // state 1
    MULTIFRAME, // state 2
    CREATE_OBS_MOVE_2_PREGRASP, // state 3
    OPEN_END_EFFECTOR, // state 4
    MOVE_2_POI, // state 5
    EXTRACT_PEPPER, // state 6
    MOVE_2_BASKET_REMOVE_OBS, // state 7
    OPEN_GRIPPER_CLOSE_EE, // state 8
    MANUAL_INTERVENTION, // state 9
    VISUAL_SERVOING, // state 10
    END_STATE // state 11
   };


class PeterStateMachine {

    public:

        PeterStateMachine();
        // ~PeterStateMachine();

        void run();

    private:

        // node to host the clients, pubs, subs
        ros::NodeHandle PeterNode;
        ros::Publisher state_pub;
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
        // state machine msg type
        std_msgs::Int16 system_state;

        // returns the state as a string
        std::string stateToString(State state);

        // returns the staet as an int
        int stateToInt(State state){return static_cast<int>(state);}

        // Prints the transition
        void  printTransitionSuccess(State current_state, State next_state);
        void  printTransitionFailure(State current_state, State next_state);
        void printTransition(State current_state, State next_state);
};
