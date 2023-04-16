#include "../include/state_machine.h"

PeterStateMachine::PeterStateMachine(){
    // create all the clients, pubs, subs
    this->manipulation_client = this->PeterNode.serviceClient<system_ws::harvest>("/manipulation/harvest");
    this->perception_client = this->PeterNode.serviceClient<system_ws::harvest>("/perception/harvest");
    this->ee_client_pub = this->PeterNode.advertise<std_msgs::Int16>("/end_effector/harvest_req", 1);
    this->ee_client_sub = this->PeterNode.subscribe("/end_effector/harvest_rsp", 1, &PeterStateMachine::endEffectorCallback, this);
    this->state_pub = this->PeterNode.advertise<std_msgs::Int16>("/system/state", 1);

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
        printTransition(this->current_state, this->next_state);

        switch(this->next_state){

            case State::MOVE_2_RESET_POSE:{
                
                harvest_srv.request.req_id = stateToInt(State::MOVE_2_RESET_POSE);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_RESET_POSE).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, approach the plant positions
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::MOVE_2_RESET_POSE;
                        this->next_state = State::APPROACH_PLANT_POSITIONS;
                    }
                    else{
                        // if it was bad, you need manual intervention
                        printTransitionFailure(this->current_state, this->next_state);
                        this->current_state = State::MOVE_2_RESET_POSE;
                        this->next_state = State::MANUAL_INTERVENTION;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }

            case State::APPROACH_PLANT_POSITIONS:{

                harvest_srv.request.req_id = stateToInt(State::APPROACH_PLANT_POSITIONS);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::APPROACH_PLANT_POSITIONS).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, need to multiframe
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::APPROACH_PLANT_POSITIONS;
                        this->next_state = State::MULTIFRAME;
                    }
                    else{
                        // if it failed, no peppers were detected, and you are done!
                        ROS_INFO("All positioned approached! Resetting Peter.");
                        this->current_state = State::MOVE_2_RESET_POSE;
                        this->next_state = State::END_STATE;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }
            
            case State::MULTIFRAME:{

                harvest_srv.request.req_id = stateToInt(State::MULTIFRAME);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::MULTIFRAME).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, need to create peppers obs & move to pregrasp
                        ROS_INFO("Pepper detected!");
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::MULTIFRAME;
                        this->next_state = State::CREATE_OBS_MOVE_2_PREGRASP;
                    }
                    else{
                        // if it failed, no peppers were detected, need to approach another pos 
                        ROS_INFO("No peppers detected! Approaching another position...");
                        this->current_state = State::MULTIFRAME;
                        this->next_state = State::APPROACH_PLANT_POSITIONS;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }
                
            case State::CREATE_OBS_MOVE_2_PREGRASP:{

                harvest_srv.request.req_id = stateToInt(State::CREATE_OBS_MOVE_2_PREGRASP);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::CREATE_OBS_MOVE_2_PREGRASP).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, need to open the end effector
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::CREATE_OBS_MOVE_2_PREGRASP;
                        this->next_state = State::OPEN_END_EFFECTOR;
                    }
                    else{
                        // if it failed, need to go to approach positions again
                        printTransitionFailure(this->current_state, this->next_state);
                        this->current_state = State::MULTIFRAME;
                        this->next_state = State::APPROACH_PLANT_POSITIONS;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }

            case State::OPEN_END_EFFECTOR:{
               
                // havent received a response yet
                // this->ee_response = -1;
                this->ee_response = 1;
                this->harvest_req.data = stateToInt(State::OPEN_END_EFFECTOR);

                this->ee_client_pub.publish(this->harvest_req);
                ros::Duration(0.25).sleep();
                ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                                
                // process the response
                if(this->ee_response){
                    // if you opened ee, need to visual servo
                    printTransitionSuccess(this->current_state, this->next_state);
                    this->current_state = State::OPEN_END_EFFECTOR;
                    this->next_state = State::VISUAL_SERVOING; // 
                }
                else{
                    // if you didn't get an answer you need manual intervention
                    printTransitionFailure(this->current_state, this->next_state);
                    ROS_ERROR("Motors not working...attempt to factory reset motors");
                    this->current_state = State::OPEN_END_EFFECTOR;
                    this->next_state = State::FACTORY_RESET_MOTORS;
                }
                                
                break;
            }

            case State::VISUAL_SERVOING:{

                harvest_srv.request.req_id = stateToInt(State::VISUAL_SERVOING);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::VISUAL_SERVOING).c_str());
                    int success = harvest_srv.response.reply;

                    //! CHANGE THIS -> THIS IS TO SKIP VISUAL SERVOING AND GO TO POI
                    success = 0;
                    
                    if(success){
                        // if the response was good, you can extract the pepper
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::VISUAL_SERVOING;
                        this->next_state = State::EXTRACT_PEPPER;
                    }
                    else{
                        // if it failed, just go to the poi anyways
                        printTransitionFailure(this->current_state, this->next_state);
                        ROS_INFO("Attempting to move to POI without Visual Servoing...");
                        this->current_state = State::VISUAL_SERVOING;
                        this->next_state = State::MOVE_2_POI;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }

            case State::MOVE_2_POI:{
                
                harvest_srv.request.req_id = stateToInt(State::MOVE_2_POI);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_POI).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, extract the pepper
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::MOVE_2_POI;
                        this->next_state = State::EXTRACT_PEPPER;
                    }
                    else{
                        // if it failed, you never moved to the poi and need to move
                        // back to creating pep obs & moving to pregrasp
                        printTransitionFailure(this->current_state, this->next_state);
                        this->current_state = State::VISUAL_SERVOING;
                        this->next_state = State::CREATE_OBS_MOVE_2_PREGRASP;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;

            }

            case State::EXTRACT_PEPPER:{
                
                // havent received a response yet
                // this->ee_response = -1;
                this->ee_response = 1;
                this->harvest_req.data = stateToInt(State::EXTRACT_PEPPER);
              
                this->ee_client_pub.publish(this->harvest_req);
                ros::Duration(0.25).sleep();
                ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                                
                // process the response
                if(this->ee_response){
                    // if you opened ee, need to visual servo
                    printTransitionSuccess(this->current_state, this->next_state);
                    this->current_state = State::EXTRACT_PEPPER;
                    this->next_state = State::MOVE_2_BASKET_REMOVE_OBS;

                    // Need to add a delay so the cutter can finish cutting
                    // moving to basket
                    ros::Duration(3).sleep();
                }
                else{
                    // if you didn't get an answer you need manual intervention
                    printTransitionFailure(this->current_state, this->next_state);
                    ROS_ERROR("Motors not working...attempt to factory reset motors");
                    this->current_state = State::EXTRACT_PEPPER;
                    this->next_state = State::FACTORY_RESET_MOTORS;
                }
                                
                break;
            }

            case State::MOVE_2_BASKET_REMOVE_OBS:{
                
                harvest_srv.request.req_id = stateToInt(State::MOVE_2_BASKET_REMOVE_OBS);
                
                if(manipulation_client.call(harvest_srv)){

                    // you got a response
                    ROS_INFO("Received Response From: %s", stateToString(State::MOVE_2_BASKET_REMOVE_OBS).c_str());
                    int success = harvest_srv.response.reply;
                    
                    if(success){
                        // if the response was good, need to drop the pepper
                        printTransitionSuccess(this->current_state, this->next_state);
                        this->current_state = State::MOVE_2_BASKET_REMOVE_OBS;
                        this->next_state = State::OPEN_GRIPPER_CLOSE_EE;
                    }
                    else{
                        // if it failed, you still need to drop the pepper
                        printTransitionFailure(this->current_state, this->next_state);
                        this->current_state = State::EXTRACT_PEPPER;
                        this->next_state = State::OPEN_GRIPPER_CLOSE_EE;
                    }
                }
                else{
                    // you didnt get a response, so you should try to do this again: 
                    // dont change the current and next states
                    ROS_ERROR("No reponse received.");
                    printTransitionFailure(this->current_state, this->next_state);
                }
                
                break;
            }

            case State::OPEN_GRIPPER_CLOSE_EE:{
                // havent received a response yet
                // this->ee_response = -1;
                this->ee_response = 1;
                this->harvest_req.data = stateToInt(State::OPEN_GRIPPER_CLOSE_EE);

                this->ee_client_pub.publish(this->harvest_req);
                ros::Duration(0.25).sleep();
                ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                                
                // process the response
                if(this->ee_response){
                    // if you let go of the pepper, you need to go back to approaching positions
                    printTransitionSuccess(this->current_state, this->next_state);
                    this->current_state = State::OPEN_GRIPPER_CLOSE_EE;
                    this->next_state = State::APPROACH_PLANT_POSITIONS;
                }
                else{
                    // if you didn't get an answer you need manual intervention
                    printTransitionFailure(this->current_state, this->next_state);
                    ROS_ERROR("Motors not working...attempt to factory reset motors");
                    this->current_state = State::OPEN_GRIPPER_CLOSE_EE;
                    this->next_state = State::FACTORY_RESET_MOTORS;
                }
                                
                break;
            }

            case State::SYSTEM_RESET_CHECK_MOTORS:{
                
                // kill the nodes and relaunch them
                resetArmNCamera();

                // see if the motors are working by closing them when they should be closed
                int motors_working = checkMotors();

                if(!motors_working){
                    ROS_ERROR("Motors not working during reset...attemping to reset...");
                    this->next_state = State::FACTORY_RESET_MOTORS;
                }     
            }

            case State::FACTORY_RESET_MOTORS:{

                if(!factory_reset_motors()){
                    ROS_ERROR("Factory reset failed...need manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
                    this->current_state == State::FACTORY_RESET_MOTORS;
                }
                else{
                    // if we came from resetting the motors we need to go back to approaching the plant positions
                    if(this->current_state == State::SYSTEM_RESET_CHECK_MOTORS){
                        ROS_INFO("The motors have been reset at system reset, approaching plant positions");
                        this->next_state == State::APPROACH_PLANT_POSITIONS;
                    }
                    else{
                        // if not we need to go back to the last state
                        ROS_INFO("The motors have been reset, returning to last state");
                        this->next_state == this->current_state;
                        this->current_state == State::FACTORY_RESET_MOTORS;
                    }

                }

            }         

            case State::MANUAL_INTERVENTION:{
               ROS_ERROR("MANUAL INTERVENTION REQUIRED: KILLING NODE");

                // kill the node by printing to the terminal
               std::string command = "rosnode kill system_node";
               system(command.c_str());

               break;
            }

            case State::END_STATE:{
                ROS_INFO("Peter has successfully picked all peppers!, Powering down...");

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

void PeterStateMachine::printTransitionSuccess(State current_state, State next_state){

    // print the starting state and the next state
    ROS_INFO("Successful transition from %s to %s", stateToString(current_state).c_str(), stateToString(next_state).c_str());

}


void PeterStateMachine::printTransitionFailure(State current_state, State next_state){

    // print the starting state and the next state
ROS_ERROR("Failed transition from %s to %s", stateToString(current_state).c_str(), stateToString(next_state).c_str());
}


void PeterStateMachine::printTransition(State current_state, State next_state){

// print the starting state and the next state
ROS_INFO("Started in: %s", stateToString(current_state).c_str());
ROS_INFO("Transitioning to: %s", stateToString(next_state).c_str());

}


void PeterStateMachine::endEffectorCallback(const std_msgs::Int16& rsp){

this->ee_response = rsp.data;  

}

void PeterStateMachine::resetArmNCamera(){

    int num_nodes = 6;
    const string nodes[num_nodes] = {
        'camera_node',
        '/my_gen3/joint_state_publisher',
        '/my_gen3/move_group',
        '/my_gen3/my_gen3_driver',
        '/my_gen3/robot_state_publisher',
        '/my_gen3/rviz/'
    };
    
    // kill all the nodes in the list
    for(int i = 0; i<num_nodes; i++){
        ROS_INFO_STREAM("Killing " + nodes[i] + "node...");
        std::string command = "rosnode kill" + nodes[i];
        system(command.c_str());
    }

    // launch it again
    ROS_INFO("Launching: pepper_ws kinova_real.launch ");
    std::string command = "roslaunch pepper_ws kinova_real.launch";
    system(command.c_str());

}

int PeterStateMachine::checkMotors(){

    // no response from the end_effector yet
    this->ee_response = -1;

    // confirms if both motors are operational- assume yes
    int operational = 1;

    // try to make the gripper close
    int close_grip_req = 24; // 24 closes the gripper
    int close_cut_req = 25; // 25 closes the cutter

    this->ee_client_pub.publish(close_grip_req);
    ros::Duration(0.25).sleep();
    ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                    
    // process the response
    if(!this->ee_response){
        // if the response is not 1 (success) we need to return not operational
        operational = 0;
    }

    // change the ee_response again for the new message
    this->ee_response = -1;

    this->ee_client_pub.publish(close_cut_req);
    ros::Duration(0.25).sleep();
    ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                    
    // process the response
    if(!this->ee_response){
        // if the response is not 1 (success) we need to return not operational
        operational = 0;
    }

    return operational;
}

int PeterStateMachine::factory_reset_motors(){

    // assume the reset is successful
    int success = 1;

    // change the ee_response again for the new message
    this->ee_response = -1;

    // factory reset is command is 20 
    int factory_reset_req = 20;

    this->ee_client_pub.publish(factory_reset_req);
    ros::Duration(0.25).sleep();
    ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                    
    // process the response
    if(!this->ee_response){
        // if the response is not 1 (success) we need to return not a failed factory reset
        success = 0;
    }

    return success;

}


void PeterStateMachine::run(){

    // Keep the node spinning 
    while(ros::ok()){
        ros::spinOnce();
        this->system_state.data = stateToInt(this->current_state);
        this->state_pub.publish(this->system_state);
    }

}


std::string PeterStateMachine::stateToString(State state){

    switch(state){
        case State::MOVE_2_RESET_POSE:
            return "Move to Reset Pose";
        case State::APPROACH_PLANT_POSITIONS:
            return "Approach Plant Positions";
        case State::MULTIFRAME:
            return "Use Multiframe Pepper Detection";
        case State::CREATE_OBS_MOVE_2_PREGRASP:
            return "Create Pepper Obstacles & Move to Pre-Grasp Pose";
        case State::OPEN_END_EFFECTOR:
            return "Open the End-Effector Gripper & Cutter";
        case State::MOVE_2_POI:
            return "Move to the POI";
        case State::EXTRACT_PEPPER:
            return "Extract Pepper";
        case State::MOVE_2_BASKET_REMOVE_OBS:
            return "Move to the basket & Remove Pepper Obstacles";
        case State::OPEN_GRIPPER_CLOSE_EE:
            return "Open the Gripper, then Close the End-Effector";
        case State::MANUAL_INTERVENTION:
            return "Manual Intervention";
        case State::VISUAL_SERVOING:
            return "Visual Servoing";
        case State::SYSTEM_RESET_CHECK_MOTORS:
            return "Resetting the Arm & Checking Motor Operation";
        case State::FACTORY_RESET_MOTORS:
            return "Factory Reset Motors";
        default:
            return "FAILED";
    }

}