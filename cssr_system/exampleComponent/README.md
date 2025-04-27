<div align="center">
  <h1>Example Component</h1>
</div>


<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `exampleComponent` ROS --detailed overview--

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [DX.Y Example Component](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).


 
# Run the Example Component Node
## Physical Robot 
### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_rob_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```
       
3. **Update Configuration File:**
   
   Navigate to the configuration file located at `~/workspace/pepper_rob_ws/src/cssr4africa/exampleComponent/config/exampleComponentConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `keyA` | Target platform | `valueA_1` or `valueA_2` |
   | `keyB` | Test iconic gestures | `valueB_1`, `valueB_2` |

   - To execute the gestures on the physical platform, change the first line of `exampleComponentConfiguration.ini` file in the config folder to “`platform robot`”. 
   - Any other important description

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/DX.Y_Example_Component.pdf" style="color: #66b3ff;">DX.Y Example Component</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>

4. **Run the `exampleComponent` from `cssr_system`  package:**
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the `exampleComponent` node.
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system exampleComponent
        ```
    
      N.B: Running the `exampleComponent` node requires the `/sample/service` service and the `/sample/topic` topic to be available, which can be hosted by running the `sampleComponent` and `sampleComponent_2` node in the `cssr_system` package or running the `exampleComponentStub` in the `unit_tests` package before running the `exampleComponent`  node in step 5 above:
         - (Option 1A): Run the `sampleComponent` node of the `cssr_system` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun cssr_system sampleComponent
      ```
         - (Option 1B): Run the `sampleComponent_2` node of the `cssr_system` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun cssr_system sampleComponent_2
      ```
         - (Option 2): Run the `exampleComponentStub` of the `unit_tests` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun unit_tests exampleComponentStub
      ```

## Simulator Robot

### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in the simulated environment. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

2. **Clone and build the project (if not already cloned):**
   - Move to the source directory of the workspace:
      ```bash
      cd $HOME/workspace/pepper_sim_ws/src
      ```
   - Clone the `CSSR4Africa` software from the GitHub repository:
      ```bash
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files:
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```

3. **Update Configuration File:**
   
   Navigate to the configuration file located at `~/workspace/pepper_sim_ws/src/cssr4africa/exampleComponent/config/exampleComponentConfiguration.ini`and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `keyA` | Target platform | `valueA_1` or `valueA_2` |
   | `keyB` | Test iconic gestures | `valueB_1`, `valueB_2` |

   - To execute the gestures on the simulator platform, change the first line of the file to “`platform simulator`”.
   - Any other important description


    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/DX.Y_Example_Component.pdf" style="color: #66b3ff;">DX.Y Example Component</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>


4. **Run the `exampleComponent` from `cssr_system` package:**:
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the simulator robot:
        ```bash
          roslaunch cssr_system cssrSystemLaunchSimulator.launch
        ```
    - Open a new terminal to launch the `exampleComponent` node.
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system exampleComponent
        ```
    
      N.B: Running the `exampleComponent` node requires the `/sample/service` service and the `/sample/topic` topic to be available, which can be hosted by running the `sampleComponent` and `sampleComponent_2` node in the `cssr_system` package or running the `exampleComponentStub` in the `unit_tests` package before running the `exampleComponent`  node in step 5 above:
         - (Option 1A): Run the `sampleComponent` node of the `cssr_system` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun cssr_system sampleComponent
      ```
         - (Option 1B): Run the `sampleComponent_2` node of the `cssr_system` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun cssr_system sampleComponent_2
      ```
         - (Option 2): Run the `exampleComponentStub` of the `unit_tests` package (in a new terminal): 
      ```sh
      source ~/workspace/pepper_rob_ws/devel/setup.bash && rosrun unit_tests exampleComponentStub
      ```

## Executing an Action
Upon launching the node, the hosted service (`/exampleComponent/example_service`) is available and ready to be invoked. This can be verified by running the following command in a new terminal:

```sh
rostopic list | grep /exampleComponent
```

The command below invokes the service to execute an action (run in a new terminal) with the request parameters defined below:

```sh
rosservice call /exampleComponent/example_service -- argument_a argument_b
```
### Service Request Parameters
#### 1. Argument A (argument_a)
- `argument_a_1`: Description of argument_a_1
- `argument_a_2`: Description of argument_a_2
- `argument_a_3`: Description of argument_a_3
- `argument_a_4`: Description of argument_a_4

#### 1. Argument B (argument_B)
- `argument_b_1`: Description of argument_b_1
- `argument_b_2`: Description of argument_b_2
- `argument_b_3`: Description of argument_b_3
- `argument_b_4`: Description of argument_b_4

### Sample Invocations
- Give samples of how it can be invoked

## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the exampleComponent node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_DX.Y.pdf" style="color: #66b3ff;">DX.Y Example Component</a>.These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:email@andrew.cmu.edu">email@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2024-12-10
