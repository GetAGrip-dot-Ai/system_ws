#include "state_machine.h"

PeterStateMachine::PeterStateMachine(){
    // create all the clients, pubs, subs
    this->manipulation_client = this->PeterNode.serviceClient<system_ws::harvest>("/manipulation/harvest");
    this->perception_client = this->PeterNode.serviceClient<system_ws::harvest>("/perception/harvest");
    this->ee_client_pub = this->PeterNode.advertise<std_msgs::Int8>("/end_effector/harvest_req", 1);
    this->ee_client_sub = PeterNode.subscribe("/end_effector/harvest_rsp", 1, &PeterStateMachine::endEffectorCallback, this);

    //create the timer that will check the state
    this->state_checker = this->PeterNode.createTimer(ros::Duration(this->checker_time), &PeterStateMachine::stateCheckerCallback, this);

    // after setting up the objects peter is online. update and print init state
    ROS_INFO("Peter is online...");

    // init the curr and next states to be the same at the beginning
    this->current_state = State::MANUAL_INTERVENTION;
    this->next_state = State::MOVE_2_RESET_POSE;

    ROS_INFO("Peter beginning to approach plant...");
}

// ~PeterStateMachine::PeterStateMachine(){}

void PeterStateMachine::stateCheckerCallback(const ros::TimerEvent& event){

    // if the next state has been updated
    if(this->current_state != this->next_state){

        // print to the roslog the transition being executed
        printTransition(this->current_state, this->next_state, false, true);

        switch(this->next_state){

            case State::MOVE_2_RESET_POSE:{

                this->current_state = State::MOVE_2_RESET_POSE;

                harvest_srv.request.req_id = stateToInt(State::MOVE_2_RESET_POSE);
                ROS_INFO("Reset Pose Req Id: ");
                ROS_INFO_STREAM(harvest_srv.request.req_id);

                if(manipulation_client.call(harvest_srv)){

                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_RESET_POSE).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to approach plant positions");
                        this->next_state = State::APPROACH_PLANT_POSITIONS;
                    }
                    else{
                        ROS_INFO("Reset pose failed, transition to manual intervention");
                        this->next_state = State::MANUAL_INTERVENTION;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                
                break;
            }

            case State::APPROACH_PLANT_POSITIONS:{

                this->current_state = State::APPROACH_PLANT_POSITIONS;

                harvest_srv.request.req_id = stateToInt(State::APPROACH_PLANT_POSITIONS);
                ROS_INFO("Approach plant positions req_id: ");
                ROS_INFO_STREAM(harvest_srv.request.req_id);

                if(manipulation_client.call(harvest_srv)){

                    ROS_INFO("Received Response From: %s", stateToString(State::APPROACH_PLANT_POSITIONS).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to detect peppers");
                        this->next_state = State::DETECT_PEPPERS;
                    }
                    else{
                        ROS_INFO("Transition to all positions approached");
                        this->next_state = State::ALL_POSITIONS_APPROACHED;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                
                break;
            }
            
            case State::DETECT_PEPPERS:{
                if(perception_client.call(harvest_srv)){

                    ROS_INFO("Received Response From: %s", stateToString(State::DETECT_PEPPERS).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                    else{
                        ROS_INFO("Transition to approach plant positions");
                        this->next_state = State::APPROACH_PLANT_POSITIONS;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }
                
            case State::CREATE_PEPPER_OBSTACLES:{
                if(manipulation_client.call(harvest_srv)){

                    ROS_INFO("Received Response From: %s", stateToString(State::CREATE_PEPPER_OBSTACLES).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CHOOSE_PEPPER;
                    }
                    else{
                        ROS_INFO("Transition to detect peppers");
                        this->next_state = State::DETECT_PEPPERS;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::CHOOSE_PEPPER:{
                if(perception_client.call(harvest_srv)){

                    ROS_INFO("Received Response From: %s", stateToString(State::CHOOSE_PEPPER).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::REMOVE_PEP_OBS_N_MOVE;
                    }
                    else{
                        // int state = 1;
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::REMOVE_PEP_OBS_N_MOVE:{
                if(manipulation_client.call(harvest_srv)){
                    ROS_INFO("Received Response From: %s", stateToString(State::REMOVE_PEP_OBS_N_MOVE).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to detect peduncles");
                        this->next_state = State::DETECT_PEDUNCLES;
                    }
                    else{
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                }
                else{
                    ROS_INFO("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::DETECT_PEDUNCLES:{
                if(perception_client.call(harvest_srv)){
                    ROS_INFO("Received Response From: %s", stateToString(State::DETECT_PEDUNCLES).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to move to pregrasp POI");
                        this->next_state = State::MOVE_2_PREGRASP;
                    }
                    else{
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::MOVE_2_PREGRASP:{
                if(manipulation_client.call(harvest_srv)){
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_PREGRASP).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to open end effector");
                        this->next_state = State::OPEN_END_EFFECTOR;
                    }
                    else{
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::OPEN_END_EFFECTOR:{
                this->current_state = State::OPEN_END_EFFECTOR;

                // havent received a response yet
                this->ee_response = -1;
                
                this->harvest_req.data = stateToInt(State::OPEN_END_EFFECTOR);
                do{
                    ee_client_pub.publish(this->harvest_req);
                }
                while(this->ee_response== -1);

                if(this->ee_response == 1){
                    ROS_INFO_STREAM(this->ee_response);
                    ROS_INFO("Transition to move 2 poi");
                    this->next_state = State::MOVE_2_POI;
                }
                else{
                    ROS_ERROR("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
                }     
                // else{
                //     ROS_INFO("No reponse received.");
                //     this->next_state = State::MANUAL_INTERVENTION;
                //     printTransition(this->current_state, this->next_state, true);
                // }
                
                break;
            }

            case State::MOVE_2_POI:{
                if(manipulation_client.call(harvest_srv)){
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_POI).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to extract pepper");
                        this->next_state = State::EXTRACT_PEPPER;
                    }
                    else{
                        ROS_INFO("Transition to create pepper obstacles");
                        this->next_state = State::CREATE_PEPPER_OBSTACLES;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::EXTRACT_PEPPER:{
                this->current_state = State::EXTRACT_PEPPER;

                // havent received a response yet
                this->ee_response = -1;
                
                this->harvest_req.data = stateToInt(State::EXTRACT_PEPPER);
                do{
                    ee_client_pub.publish(this->harvest_req);
                }
                while(this->ee_response== -1);

                if(this->ee_response == 1){
                    ROS_INFO_STREAM(this->ee_response);
                    ROS_INFO("Transition to move to basket");
                    this->next_state = State::MOVE_2_BASKET;
                }
                else{
                    ROS_ERROR("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
                }     
                
                break;
            }

            case State::MOVE_2_BASKET:{
                if(manipulation_client.call(harvest_srv)){
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_BASKET).c_str());
                    int success = harvest_srv.response.reply;
                    ROS_INFO_STREAM(success);
                    if(success==1){
                        ROS_INFO("Transition to open gripper and close end-effector");
                        this->next_state = State::OPEN_GRIPPER_CLOSE_EE;
                    }
                    else{
                        ROS_INFO("Transition to open gripper and close end-effector");
                        this->next_state = State::OPEN_GRIPPER_CLOSE_EE;
                    }
                }
                else{
                    ROS_ERROR("No reponse received.");
                    this->next_state = State::MANUAL_INTERVENTION;
                    printTransition(this->current_state, this->next_state, true);
                }
                break;
            }

            case State::OPEN_GRIPPER_CLOSE_EE:{
                this->current_state = State::OPEN_GRIPPER_CLOSE_EE;

                // havent received a response yet
                this->ee_response = -1;
                
                this->harvest_req.data = stateToInt(State::OPEN_GRIPPER_CLOSE_EE);
                do{
                    ee_client_pub.publish(this->harvest_req);
                }
                while(this->ee_response== -1);

                if(this->ee_response == 1){
                    ROS_INFO_STREAM(this->ee_response);
                    ROS_INFO("Transition to move 2 poi");
                    this->next_state = State::MOVE_2_POI;
                }
                else{
                    ROS_INFO("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
                }
                break;
            }

            case State::ALL_POSITIONS_APPROACHED:{
                ROS_INFO("All positions have been approached, terminating state machine");
                
                std::string command = "rosnode kill system_node";
                system(command.c_str());

                break;
            }

            case State::MANUAL_INTERVENTION:{
               ROS_ERROR("MANUAL INTERVENTION REQUIRED: KILLING NODE");

                // kill the node by printing to the terminal
               std::string command = "rosnode kill system_node";
               system(command.c_str());

               break;
            }
                

            default:
                break;

        }

    } 

}

void PeterStateMachine::printTransition(State current_state, State next_state, bool failure = false, bool attempt = false){

    // if the error is true, print error occured
    if(attempt){
        ROS_INFO("Attempted maneuver from %s to %s", stateToString(current_state).c_str(), stateToString(next_state).c_str());
    }
}

void PeterStateMachine::printTransition(State current_state, State next_state, bool failure = false){

    // if the error is true, print error occured
    if(failure){
        ROS_INFO("ERROR: please service Peter!");
    }

    // print the starting state and the next state
    ROS_INFO("Started in: %s", stateToString(current_state).c_str());
    ROS_INFO("Transitioning to: %s", stateToString(next_state).c_str());

}


void PeterStateMachine::printTransition(State current_state, State next_state){

    // print the starting state and the next state
    ROS_INFO("Started in: %s", stateToString(current_state).c_str());
    ROS_INFO("Transitioning to: %s", stateToString(next_state).c_str());

}



void PeterStateMachine::endEffectorCallback(const std_msgs::Int8& rsp){

    this->ee_response = rsp.data;   
}

void PeterStateMachine::run(){

    // Keep the node spinning 
    while(ros::ok()){
        ros::spinOnce();
    }

}

std::string PeterStateMachine::stateToString(State state){

    switch(state){
        case State::MOVE_2_RESET_POSE:
            return "Move to Reset Pose";
        case State::APPROACH_PLANT_POSITIONS:
            return "Approach Plant Positions";
        case State::DETECT_PEPPERS:
            return "Detect Peppers";
        case State::CREATE_PEPPER_OBSTACLES:
            return "Create pepper obstacles";
        case State::CHOOSE_PEPPER:
            return "Choose pepper";
        case State::REMOVE_PEP_OBS_N_MOVE:
            return "Remove the pepper as an obstacle and move to close position";
        case State::DETECT_PEDUNCLES:
            return "Detect peduncles";
        case State::MOVE_2_PREGRASP:
            return "Move to the pregrasp pose";
        case State::OPEN_END_EFFECTOR:
            return "Open the End-Effector";
        case State::MOVE_2_POI:
            return "Move to the POI";
        case State::EXTRACT_PEPPER:
            return "Extract pepper";
        case State::MOVE_2_BASKET:
            return "Move to the basket";
        case State::OPEN_GRIPPER_CLOSE_EE:
            return "Open the gripper, then close the end-effector";
        case State::ALL_POSITIONS_APPROACHED:
            return "All positions approached, peppers harvested";
        case State::MANUAL_INTERVENTION:
            return "Manual Intervention";
        default:
            return "FAILED";
    }

}



// int main(int argc, char **argv){
//     // start system node
//     ros::init(argc, argv, "system_node");
//     ros::NodeHandle n;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

    // ros::ServiceClient manipulation_client = n.serviceClient<system_ws::harvest>("/manipulation/harvest");
    // ros::ServiceClient perception_client = n.serviceClient<system_ws::harvest>("/perception/harvest");
//     // change the one for the end effector buffer if bad shit happens
    // ros::Publisher ee_client_pub = n.advertise<std_msgs::Int8>("/end_effector/harvest_req", 1)
//     // harvest_rsp topic updates the state from the ee side
    // ros::Subscriber ee_client_sub = n.subscribe("/end_effector/harvest_rsp", 1, endEffectorCallback);

//     // used to perform service calls & pseudo service calls over ros
//     system_ws::harvest harvest_srv;
//     std_msgs::Int8 harvest_req;

//     // tells the subsystem what action to perform
//     switch(state){
//         case 1:
//             

//         case 2:
//             harvest_srv.request.req_id = state;
//             perception_client.call(srv);
//             int success = srv.response.reply
//             if(success){
//                 state = 3;
//             }
//             else{
//                 state = 1;
//             }
//             break;

//         case 3:
//             break;
//         case 4:
//         case 5:
//         case 6:
//         case 7:
//         case 8:
//             // state one is to open both gripper & cutter
//             harvest_req.data = 1;
//             ee_client_pub.publish(&harvest_req)
//         case 9:
//         case 10:
//         case 11:
//         case 12:
//         case 13:

//     }
    

    


//     // if (){
//     //     // ROS_INFO("Response: %ld", (long int)srv.response.reply);
//     //     ROS_INFO("called it");
//     // }
//     // else{
//     //     ROS_ERROR("Failed to call service");
//     //     return 1;
//     // }

//     return 0;

// }