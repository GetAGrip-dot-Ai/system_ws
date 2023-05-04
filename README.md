# Peter FSM

[flowchart](https://github.com/artrela/system_ws/blob/master/Resources/fsm-flowchart.png)

*All movements of the finite-state machine and the particular error handling is described below*

| **State**                    | **Description**                                                                                         | **Upon Success**                                   | **Upon Failure/Completion**               |
|------------------------------|---------------------------------------------------------------------------------------------------------|----------------------------------------------------|-------------------------------------------|
| Reset Pose                   | Approach Kinematically Favorable Start Position                                                         | Ready to approach the plant                        | Must require manual intervention          |
| Approach Plant Positions     | A set of multiple points to view the plant to ensure all peppers have been harvested                    | Prepare to capture multiframe pictures             |  Must require manual intervention         |
| Multiframe Manipulation      | Short moves to provide perception subsystem with a means to filter out false positives/negatives        | Frames have been captures and need to be processed | All frames have been processed            |
| Multiframe Perception        | Multiple object detections to increase detection robustness                                             | Ready to receive next multiframe image             | Reset to next multiframe move             |
| Find POI                     | Once peppers have been identified, the pepper of interest's POI is located                              |  -                                                 |  -                                        |
| Move to Pregrasp POI Pose    | The manipulator then moves close to the POI                                                             | The arm has moved near the POI                     | Must require manual intervention          |
| Open End-Effector            | Prepares to harvest pepper                                                                              | Ready to grasp the peduncle                        | Attempt to factory reset the motors       |
| Visual Servoing              | Fine adjustment to alleviate accumulated error in positioning and recognition                           | Moves to POI with adjusted information             | Move to POI with not adjusted information |
| Move to POI                  | Cartesian move along depth axis to insert End-Effector within grasp of the peduncle                     | Ready to extract the pepper                        | Needs to go back to approach position     |
| Extract Pepper               | Actuates End-Effector to Grip and Cut Peppers                                                           | Pepper has been removed                            | Attempt to factory reset the motors       |
| Move to Basket Drop          | After a short Cartesian move away from the plant, the End-Effector is placed over the basket            | Ready to drop pepper in the basket                 | Open the end-effector and drop the pepper |
| Open then Close End-Effector | The pepper is then released                                                                             | Pepper has been dropped successfully               | Attempt to factory reset the motors       |
| Factory Reset Motors         | If the motors stall out, they require factory reset                                                     | Motors are ready to receive another command        | Manual intervention will be required      |
| Manual Intervention          | If the system reaches a scenario it cannot handle autonomously, it must prepare for manual intervention |  -                                                 |  -                                        |
