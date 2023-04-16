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

                // do{
                //     this->ee_client_pub.publish(this->harvest_req);
                //     ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                //     ros::Duration(0.1).sleep();
                // }
                // while(this->ee_response == -1);

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
                    ROS_ERROR("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
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

                // do{
                //     this->ee_client_pub.publish(this->harvest_req);
                //     ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                //     ros::Duration(0.1).sleep();
                // }
                // while(this->ee_response == -1);

                
                this->ee_client_pub.publish(this->harvest_req);
                ros::Duration(0.25).sleep();
                ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                                
                // process the response
                if(this->ee_response){
                    // if you opened ee, need to visual servo
                    printTransitionSuccess(this->current_state, this->next_state);
                    this->current_state = State::EXTRACT_PEPPER;
                    this->next_state = State::MOVE_2_BASKET_REMOVE_OBS;
                }
                else{
                    // if you didn't get an answer you need manual intervention
                    printTransitionFailure(this->current_state, this->next_state);
                    ROS_ERROR("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
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

                // do{
                //     this->ee_client_pub.publish(this->harvest_req);
                //     ros::spinOnce(); //! MAY BE A TERRIBLE IDEA
                //     ros::Duration(0.1).sleep();
                // }
                // while(this->ee_response == -1);

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
                    ROS_ERROR("Transition to manual intervention");
                    this->next_state = State::MANUAL_INTERVENTION;
                }
                                
                break;
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
        default:
            return "FAILED";
    }

}