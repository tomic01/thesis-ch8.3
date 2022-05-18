Behavior Manager

Here we describe the current version of the monarch behaviors package, reformulated after the IW of July 2014. The current version allows the concurrent execution of multiple behaviors as long as they use a non-conflicting set of resources. The reformulated behavior manager treats all behaviors in a generic way, which makes the inclusion, reformulation or removal of behaviors from our system much simpler than before. Adding a new behavior requires adding a single line of code instead of around 50 lines in the previous version. To allow for this more generic approach behaviors have to respect a certain structure for their goal, feedback and result messages. More details about this below. If you want to design a behavior that was not used during the IW please contact José Nuno Pereira (jose.pereira@epfl.ch) to discuss the appropriate changes.

Overview

The monarch behaviors package groups four different aspects of the execution of behaviors by the MOnarCH robots.

1. Fake Planner: allows us to input directly what behaviors we would like to test in the robots without being constrained by what is dictated by the Constraint-Satisfaction Planner (CSP). Outputs BehaviorSelection messages which specify: behavior id, which robots to execute the behavior, which resources to be used and an adequate time window for the execution. (One node, running on mcentral)

2. Global Behavior Manager (GBM): deals with the connection between the CSP (or the substitute fake planner) and the robots by receiving requests for behavior execution (as BehaviorSelection messages) from the CSP and sending them to the appropriate robots via the Situational Awareness Module (SAM). (One node, running on mcentral)

3. Behavior Executors: each behavior to be executed by the robots is implemented in a different node as an Actionlib server. For each, an .action file must be defined with the goal the behavior needs to be executed and with the feedback and result it should provide during and upon conclusion of execution. To allow for the generic approach described above, the three fields of the .action file (goal, result, feedback) must all be specified using the message monarch_msgs/KeyValuePairArray. Upon reception of a goal the behavior executors will make the robot perform the desired operation. Several of the behaviors currently implemented use ROS SMACH (library for finite state machines and beyond) combined with a ActionServerWrapper that allows us to specify behaviors as a finite state machine that can be used directly as an Actionlib server. This approach to the implementation of behaviors is not mandatory but might be useful for designers. Check the currently implemented behaviors to assess if it might be a good option for the behavior you need to design. (One node per behavior and per robot, running on mbots)

4. Local Behavior Manager (LBM): contains one Actionlib client for each of the behavior executors implemented as Actionlib servers. The LBM receives behavior execution commands from the GBM via SAM and, via these Actionlib clients, sends goals to the appropriate behavior executors, leading to the execution of the behavior requested by the CSP. In the current version, the LBM allows the execution of multiple behaviors as long as they use a non-conflicting set of resources. Resources to be used by each behavior are specified by the CSP in the BehaviorSelection messages. This means that upon reception of a new behavior execution request the LBM preempts only the currently executing behaviors which are using resources needed by the newly requested behavior. If a behavior terminates its execution and no other behavior command is available the LBM send a goal to an "idle" behavior executor so that there is always one behavior being executed. (One node per robot, running on mbots)

Configuration

Several of the nodes in the monarch behaviors package require changing some parameters before execution to take into account the environment the robots are operating on.

- In SAM: the following slots must be available for each robot in operation: MBotXXExecuteBehavior, MBotXXCurrentExecutingBehavior, MBotXXCurrentBehaviorFeedback, MBotXXCurrentBehaviorResult, where XX is the number of the robot (e.g., MBot02CurrentBehaviorResult). Check the SAM configuration file used during IW at code/trunk/catkin_ws/src/monarch_situational_awareness/config/slot_config_iw.yaml.

- In global_behavior_manager.py: the variable robot_id_list (line 16) must indicate what robots are operational. For instance, if you are using mbots 2, 4 and 12, then set robot_id_list = ['02','04','12'].

- In local_behavior_manager.py: the variable resource_list (line 30) must indicate all the resources available to the robots. This variable is used to specify resources for the idle behavior, meaning that idle uses all the resources and must be preempted as soon as any other behavior is requested. We are using functionality names to describe resources, for instance resource_list = ('RNAV','RLED','RVocSnd'). Check deliverable D6.1 or the Behaviors Categories, Templates and Examples working document for more functionality names. Unless you are ading a new functionality there is no need to change this variable.

- In docking.py: the position of the dock must be correctly set. Use variables DOCK_X, DOCK_Y, DOCK_T.

- In goto.py: the mapping between the coordinates you want your robot to travel to and the name of the location must be defined in the execute() function of the StateStart class. Check lines 53:77.

- In patrolling.py: the waypoints for the patrolling route must be set. Check lines 66:93.

Use

The GBM is launched within mcentral.launch (in code/trunk/launch). Behavior executors and the LBM are launched within mbot.launch (same folder). If you want to test directly some behavior and not use the planner run the fake_SAP_output node of the monarch_behaviors package on mcentral and follow the on-screen instructions.