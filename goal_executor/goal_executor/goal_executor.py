import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, PoseStamped, PoseArray
from custom_message.msg import *
import tf2_ros
import threading
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
import time
from datetime import datetime
from goal_executor.operation_state_machine import *
import tf_transformations
from std_msgs.msg import String, Int32

class TF2Echo(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trans = None

    # We get the transform from map to base_link to get current position from robot
    def doLookUpTransform(self, source_frame, target_frame):
        try:
            self.trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn("Waiting for TF")

    # For debugging
    def printTransform(self, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info("Translation: [x={}, y={}, z={}]".format(translation.x, translation.y, translation.z))
        self.get_logger().info("Rotation: [x={}, y={}, z={}, w={}]".format(rotation.x, rotation.y, rotation.z, rotation.w))
    
    # Return None if no TF from map to base_link received
    def getTF(self):
        if(self.trans != None):
            return self.trans

class GoalExecutor(Node):
    def __init__(self):
        super().__init__('GoalExecutor')
        
        self.declare_parameter("use_amcl", False)
        self.use_amcl_param_value = self.get_parameter("use_amcl").value
        self.get_logger().info(f'Using {self.use_amcl_param_value} for localization.')
        self.get_logger().info(f'Type: {type(self.use_amcl_param_value)}')
        self.current_mode = None

        self.create_subscription(
            Int32,
            'move_to_goal',
            self.moveCallback,
            1
        )

        # self.create_subscription(
        #     PatrolConfig,
        #     'patrol_config_update',
        #     self.patrolConfigUpdateCb,
        #     1
        # )

        self.create_subscription(
            String,
            'autonomous_mode_switch',
            self.autonomousModeSwitchCb,
            1
        )

        self.create_subscription(
            String,
            'mode',
            self.CurrentModeCb,
            1
        )

        self.create_subscription(
            Bool,
            'update_goal_list',
            self.UpdateGoalListCb,
            1
        )

        self.isParticleCloudReceived = False

        self.subscription = self.create_subscription(
            PoseArray,
            '/particlecloud',
            self.particlecloudCb,
            10
        )

        self.navigator = None
        self.monitoring_thread = None
        self.stop_monitoring = False
        self.currentTranslation = Vector3Stamped()
        self.currentQuaternion = QuaternionStamped()
        self.goalListFile = '/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/goal_list.json'

        self.goalState = GoalState()
        self.goalState.current_goal_id = 0
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # This sets how many messages to keep
        )

        self.patrolConfig = self.loadPatrolConfig()
        # load patrol configuration
        
        self.patrolRosMsg = self.patrolDictToRosMsg(self.patrolConfig)

        self.goalStatePublisher = self.create_publisher(GoalState, '/goal_state', self.qos_profile)
        self.patrolConfigPub = self.create_publisher(PatrolConfig, '/patrol_config', self.qos_profile)
        # self.routeListPub = self.create_publisher(RouteList, '/route_list', self.qos_profile)

        # Publish transform from baselink(robot center) to map frame
        self.tf_baselink_to_map_pub = self.create_publisher(RobotPose, 'tf_baselink_map', self.qos_profile)
        # Publish transform from goals to map frame
        self.goalListPub = self.create_publisher(GoalList, 'tf_goals_map', self.qos_profile)

        if not self.patrolConfig:
            self.get_logger().warn(f"patrol config is not set.")
        else:
            self.patrolConfigPub.publish(self.patrolRosMsg)

        self.isPatrolThreadAlive = False
        self.timeRangeCheckFlag = True
        self.isPatrolStart = False
        # self.currentAutonomousMode = AutonomousMode.GOAL_NAVIGATE
        self.state_machine = OperationStateMachine()

        self.signalAutonomousModeThread = threading.Thread(target=self.signalAutonomousModeMonitor, args=())
        self.signalAutonomousModeThread.daemon = True  # Ensures the thread exits when the main program exits
        self.condition = threading.Condition()
        self.signalThreadCondition = threading.Condition()

        self.publishGoalNaviStateOnce = False

        self.goalList = self.loadGoalList()
        self.goalListMsg = GoalList()
        self.goalListMsg = self.goalListToRosMsg(self.goalList)
        if not self.goalList:
            self.get_logger().warn(f"Goal list is empty")
        else:
            self.goalListPub.publish(self.goalListMsg)

    def initNavigator(self):
        self.navigator = BasicNavigator()
        self.get_logger().info("initNavigator called")
        if self.use_amcl_param_value == True:
            self.get_logger().info("Detected AMCL for localization")
            self.navigator.waitUntilNav2Active(localizer='amcl')
        else:
            self.get_logger().info("Detected N_localization for localization")
            self.navigator.waitUntilNav2Active(localizer='neo_localization')
            
        while self.isParticleCloudReceived:
            self.get_logger().info("waiting for localization active ...")
            time.sleep(1)
        self.get_logger().info("initNavigator successfully")

    def CurrentModeCb(self, msg: String):
        if self.current_mode != msg.data:
            self.current_mode = msg.data

    def particlecloudCb(self, msg):
        """ Callback function when /particlecloud receives data """
        if not self.isParticleCloudReceived and len(msg.poses) > 0:
            self.isParticleCloudReceived = True
            self.get_logger().info("Particle cloud data received. Initializing Navigator...")
        
    def loadGoalList(self):
        """Load goals from the JSON file."""
        try:
            with open(self.goalListFile, 'r') as f:
                data = json.load(f)
                return data.get("goals", [])
        except FileNotFoundError:
            self.get_logger().warn("Goals JSON not found. Starting with an empty list.")
            return []
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON file. Starting with an empty list.")
            return []
    
    def saveGoalList(self):
        """Save the updated goals list back to the JSON file."""
        try:
            with open(self.goalListFile, 'w') as f:
                json.dump({"goals": self.goalList}, f, indent=4)
            self.get_logger().info("Goals successfully saved to JSON file.")
        except IOError as e:
            self.get_logger().error(f"Failed to save goals: {e}")
            
    # def cancelGoalCallBack(self, msg):
    #     if msg.data == True:
    #         self.navigator.cancelTask()

    # def moveCallback(self, msg):
    #     goalId = msg.goal_id
    #     # Ignore request from app/dashboard if in shuttle or patrol mode
    #     if self.currentAutonomousMode != AutonomousMode.GOAL_NAVIGATE:
    #         self.get_logger().info(f"Request fails, robot is in mode: {self.currentAutonomousMode}")
    #         return
    #     #self.navigator.cancelTask()
    #     #Request robot move to goal
    #     self.moveToGoal(goalId)
    #     self.goalState.next_goal_id = goalId

    def UpdateGoalListCb(self, msg: Bool):
        self.goalList = self.loadGoalList()
 
    def moveCallback(self, msg: Int32):
        # Ignore request from app/dashboard if in shuttle or patrol mode
        # if self.currentAutonomousMode != AutonomousMode.GOAL_NAVIGATE:
        if self.current_mode != 'NORMAL':
            self.get_logger().info(f"Request fails, robot is in mode: {self.current_mode}")
            return
        if msg.data == -1:
            self.get_logger().info(f"Goal state: {msg.data}")
            return
        goalId = msg.data
        self.goalState.current_goal_id = goalId
        #self.navigator.cancelTask()
        
        if self.monitoring_thread is not None and self.monitoring_thread.is_alive():
            self.stop_monitoring = True 
            self.monitoring_thread.join()  

        #Request robot move to goal
        self.moveToGoal(goalId)

    def updateGoalPose(self, transform):
        self.currentTranslation.vector.x = transform.transform.translation.x
        self.currentTranslation.vector.y = transform.transform.translation.y
        self.currentQuaternion.quaternion.x  = transform.transform.rotation.x
        self.currentQuaternion.quaternion.y  = transform.transform.rotation.y
        self.currentQuaternion.quaternion.z  = transform.transform.rotation.z
        self.currentQuaternion.quaternion.w  = transform.transform.rotation.w

    def getGoalById(self, goal_id):
        #print(self.goalList[goal_id])        
        """Retrieve a single goal by its goal_id."""
        goal = next((g for g in self.goalList if g["goal_id"] == goal_id), None)
        if not goal:
            self.get_logger().error(f"Goal with goal_id: {goal_id} not found in JSON.")
        return goal

    def moveToGoal(self, goalId):
        goal = self.getGoalById(goalId)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Map goal data to PoseStamped fields
        goal_pose.pose.position.x = goal['x']
        goal_pose.pose.position.y = goal['y']
        goal_pose.pose.position.z = 0.0  # Assuming 2D navigation, z is set to 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = goal['z']
        goal_pose.pose.orientation.w = goal['w']

        self.navigateToPose(goal_pose, goalId)

    def monitorTask(self, goal_id=None, route_id=None):
        # This loop will run in a separate thread
        if goal_id != None:
            self.stop_monitoring = False

            while not self.navigator.isTaskComplete():
                if self.stop_monitoring == True:
                    self.get_logger().info(f"MonitorTask for goal {goal_id} stopped.")
                    return
                
                feedback = self.navigator.getFeedback()
                self.goalState.estimated_time_remaining = (
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                )
                self.goalState.goal_state = "PROCESSING"
                self.goalStatePublisher.publish(self.goalState)
                time.sleep(0.2)

            # if self.goalState.next_goal_id != goal_id:
            #     self.get_logger().info(f"Goal {goal_id} was replaced by {self.goalState.next_goal_id}, skipping update.")
            #     return
            
            # Handle task completion or cancellation
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                # self.goalState.current_goal_id = goal_id
                self.goalState.goal_state = "SUCCEEDED"
                self.goalState.estimated_time_remaining = 0.0
            elif result == TaskResult.FAILED:
                # self.goalState.current_goal_id = -1
                self.goalState.goal_state = "FAILED"
                self.goalState.estimated_time_remaining = 0.0
            elif result == TaskResult.CANCELED:
                # self.goalState.current_goal_id = -1
                self.goalState.goal_state = "CANCELED"
                self.goalState.estimated_time_remaining = 0.0
            else:
                self.get_logger().error('Goal has an invalid return status!')

            self.goalStatePublisher.publish(self.goalState)
        
        if route_id != None:
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters.")

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Navigation succeeded!")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Navigation canceled.")
            elif result == TaskResult.FAILED:
                self.get_logger().error("Navigation failed.")
            else:
                self.get_logger().warn("Unknown result.")

    def navigateToPose(self, goalPose, goal_id):
        self.navigator.clearGlobalCostmap()
        self.navigator.goToPose(goalPose)
        self.get_logger().info(f"Sending goal id: {goal_id}")
        # This block the whole callback from ros which make calcelTask() could not be called
        # TODO: Move this into a thread
        self.monitoring_thread = threading.Thread(
            target=self.monitorTask, kwargs={"goal_id": goal_id}
        )
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def navigateToPoseBlock(self, goalPose, goal_id):
        self.navigator.clearGlobalCostmap()
        self.navigator.goToPose(goalPose)
        self.get_logger().info(f"Sending goal id: {goal_id}")
        # This block the whole callback from ros
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.goalState.estimated_time_remaining = (Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            self.goalState.goal_state = "PROCESSING"
            self.goalStatePublisher.publish(self.goalState)
            time.sleep(0.2)
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.goalState.current_goal_id = goal_id
            self.goalState.goal_state = "SUCCEEDED"
            self.goalState.estimated_time_remaining = 0.0
        elif result == TaskResult.FAILED:
            self.goalState.current_goal_id = -1
            self.goalState.goal_state = "FAILED"
            self.goalState.estimated_time_remaining = 0.0
        elif result == TaskResult.CANCELED:
            self.goalState.current_goal_id = -1
            self.goalState.goal_state = "CANCELED"
            self.goalState.estimated_time_remaining = 0.0
        else:
            print('Goal has an invalid return status!')
        self.goalStatePublisher.publish(self.goalState)

    def patrolConfigUpdateCb(self, msg):
        self.get_logger().info("Received patrol config update")
        # Convert the ROS message to a Python OrderedDict
        msg_dict = message_to_ordereddict(msg)
        # Convert the OrderedDict to JSON
        msg_json = json.dumps(msg_dict)
        # Remove slash in json
        json_without_slash = json.loads(msg_json)
        with open('/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/patrol_config.json', 'w') as json_file:
            json.dump(json_without_slash, json_file, indent=4)
        # load patrol configuration
        self.patrolConfig = self.loadPatrolConfig()
        self.patrolRosMsg = self.patrolDictToRosMsg(self.patrolConfig)
        self.patrolConfigPub.publish(self.patrolRosMsg)
        # make sure we check the time range when a config is update
        self.timeRangeCheckFlag = True

    def autonomousModeSwitchCb(self, msg: String):
        if (self.state_machine.get_state_string() != msg.data):
            self.get_logger().info(f"Received external request for mode: {msg}")
            self.state_machine.update_state(msg.data)
            #self.timeRangeCheckFlag = True
            self.navigator.cancelTask()
            with self.signalThreadCondition:
                self.signalThreadCondition.notify()
            # Wait for signal thread update current autonomousMode 
            # then wakeup current shutte/patrol thread to exit gracefully
            time.sleep(1)
            with self.condition:
                self.condition.notify()
        else:
            self.get_logger().info(f"Received same external request for mode: {msg}")
    
    def loadPatrolConfig(self):
        config = {}
        try:
            with open('/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/patrol_config.json', 'r') as json_file:
                config = json.load(json_file)
            return config
        except FileNotFoundError:
            self.get_logger().warn(f"File not found: {'/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/patrol_config.json.json'}")
            return {}
        except json.JSONDecodeError:
            self.get_logger().warn(f"Error decoding JSON file, file is empty or malformed: {'/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/patrol_config.json'}")
            return {}

    def updateExtendedOperationMode(self, mode):
        self.extendedOperationMode = mode
        return

    def patrolDictToRosMsg(self, patrolDict):
        if not patrolDict:
            return None
        # Create the PatrolConfig message
        patrolMsg = PatrolConfig()

        # Assigning goal_patrol
        patrolMsg.goal_patrol = patrolDict["goal_patrol"]

        # Assigning shift (converting each shift dictionary to a Shift message)
        patrolMsg.shift = []
        for shift_data in patrolDict["shift"]:
            shift_msg = Shift()
            shift_msg.start_time = shift_data["start_time"]
            shift_msg.end_time = shift_data["end_time"]
            patrolMsg.shift.append(shift_msg)

        # Assigning start_date
        patrolMsg.start_date = Date()
        patrolMsg.start_date.year = patrolDict["start_date"]["year"]
        patrolMsg.start_date.month = patrolDict["start_date"]["month"]
        patrolMsg.start_date.day = patrolDict["start_date"]["day"]

        # Assigning end_date
        patrolMsg.end_date = Date()
        patrolMsg.end_date.year = patrolDict["end_date"]["year"]
        patrolMsg.end_date.month = patrolDict["end_date"]["month"]
        patrolMsg.end_date.day = patrolDict["end_date"]["day"]

        # Assigning sequence_type and auto_run
        patrolMsg.sequence_type = patrolDict["sequence_type"]
        patrolMsg.auto_run = patrolDict["auto_run"]

        return patrolMsg

    def isInTimeRange(self, rosMsg):
        """
        Check if the current time is within the start_date to end_date range
        and within any shift start_time to end_time ranges.
        """
        # Get the current date and time
        current_datetime = datetime.now()
        if rosMsg is None:
            return False
        try:
            # 1. Check if the current date is within the start_date and end_date
            start_date = datetime(rosMsg.start_date.year, rosMsg.start_date.month, rosMsg.start_date.day)
            end_date = datetime(rosMsg.end_date.year, rosMsg.end_date.month, rosMsg.end_date.day)
        except ValueError as e:
            self.get_logger().info(f"Checked time failed with error: {e}")
            # Handle the error, e.g., by skipping or using a default date
            return False  # or some other logic
        
        # If current date is not in the date range, return False
        if not (start_date <= current_datetime <= end_date):
            return False
        
        # 2. Check if the current time is within any of the shift start_time and end_time ranges
        current_time = current_datetime.hour + current_datetime.minute / 100  # Convert current time to float (e.g., 13:30 -> 13.30)
        for shift in rosMsg.shift:
            if shift.start_time <= current_time <= shift.end_time:
                self.get_logger().info(f"Shift: {shift} satisfied")
                return True  # Current time is within a shift
        
        # If no shift contains the current time, return False
        return False
    
    def isShiftStartCheck(self, rosMsg):
        """
        """
        # Get the current date and time
        current_datetime = datetime.now()
        if rosMsg is None:
            return False

        try:
            # 1. Check if the current date is within the start_date and end_date
            start_date = datetime(rosMsg.start_date.year, rosMsg.start_date.month, rosMsg.start_date.day)
            end_date = datetime(rosMsg.end_date.year, rosMsg.end_date.month, rosMsg.end_date.day)
        except ValueError as e:
            self.get_logger().info(f"Checked time failed with error: {e}")
            # Handle the error, e.g., by skipping or using a default date
            return False  # or some other logic

        # If current date is not in the date range, return False
        if not (start_date <= current_datetime <= end_date):
            return False
        
        # 2. Check if the current time is within any of the shift start_time and end_time ranges
        current_time = current_datetime.hour + current_datetime.minute / 100  # Convert current time to float (e.g., 13:30 -> 13.30)
        for shift in rosMsg.shift:
            if shift.start_time == current_time:
                # self.get_logger().info(f"Shift: {shift} signal starts")
                return True  # Shift start time kicks
        
        # If no shift contains the current time, return False
        return False

    def signalAutonomousModeMonitor(self):
        while True:
            # Check if shuttle/patrol shift satisfy
            if (self.timeRangeCheckFlag):
                self.isPatrolStart = self.isInTimeRange(self.patrolRosMsg)
 #               self.isShuttleStart = self.isInTimeRange(self.shuttleRosMsg)
                self.timeRangeCheckFlag = False
            else:
                self.isPatrolStart = self.isShiftStartCheck(self.patrolRosMsg)
                # self.isShuttleStart = self.isShiftStartCheck(self.shuttleRosMsg)
            if self.patrolRosMsg:
                if ((self.current_mode == 'PATROL') and (self.isPatrolStart) and (self.isPatrolThreadAlive == False)):
                    # Time shift satisfied and auto_run config is set. create a thread to run in patrol mode
                    patrolThread = threading.Thread(target=self.patrolThread, args=())
                    patrolThread.daemon = True
                    # self.currentAutonomousMode = AutonomousMode.PATROL
                    self.state_machine.update_state('PATROL')
                    # Priority for shuttle thread is always higher than patrol
                    if self.isPatrolThreadAlive == False:
                        patrolThread.start()
            else:
                self.get_logger().warn("No patrol config is set")

            # Check if received mode change from autonomousModeSwitchCb
            # if (self.currentAutonomousMode != self.state_machine.get_state()):
            #     # Pub mode and set state
            #     self.currentAutonomousMode = self.state_machine.get_state()
            #     self.get_logger().info(f"mode changes to: {self.currentAutonomousMode}")
            if (self.current_mode == 'PATROL') and (self.patrolRosMsg):
                patrolThread = threading.Thread(target=self.patrolThread, args=())
                patrolThread.daemon = True
                self.current_mode = 'PATROL'
                self.state_machine.update_state('PATROL')
                if self.isPatrolThreadAlive == False:
                    patrolThread.start()
                    self.get_logger().info(f"Create patrol thread by request: {self.isPatrolThreadAlive}")
                # if (self.currentAutonomousMode == AutonomousMode.SHUTTLE) and (self.shuttleRosMsg):
                #     shuttleThread = threading.Thread(target=self.shuttleThread, args=())
                #     shuttleThread.daemon = True
                #     self.currentAutonomousMode = AutonomousMode.SHUTTLE
                #     self.state_machine.update_state('SHUTTLE')
                #     #print("lol")
                #     if self.isShuttleThreadAlive == False:
                #         shuttleThread.start()
                #         self.get_logger().info(f"Create shuttle thread by request: {self.isShuttleThreadAlive}")
            # if(self.currentAutonomousMode == AutonomousMode.GOAL_NAVIGATE) and self.publishGoalNaviStateOnce == False:
            #     #self.get_logger().info(f"Current mode is: {self.currentAutonomousMode}")
            #     self.autonomousModeFeedbackMsg.current_mode = 'GOAL_NAVIGATE'
            #     self.autonomousModeFeedbackMsg.state = ''
            #     self.autonomousModeFeedbackPub.publish(self.autonomousModeFeedbackMsg)
            #     self.publishGoalNaviStateOnce = True
            #time.sleep(1)
            with self.signalThreadCondition:
                self.signalThreadCondition.wait(timeout=1)

    def palindrome_loop(self, sequence):
        while True:
            for i in sequence:
                yield i
            for i in sequence[-2:0:-1]:
                yield i

    def patrolThread(self):
        self.publishGoalNaviStateOnce = False
        self.isPatrolThreadAlive = True
        self.get_logger().info(f"Patrol thread started")
        patrolGoalArr = self.patrolConfig['goal_patrol']
        # For PALINDROME
        palindrome_gen = self.palindrome_loop(patrolGoalArr)
        cancelPrevGoal = False
        # For REPEATED_HEAD
        patrolGoalArrIndex = 0
        goal_counter = 0
        shift_started = False
        publishStartOnce = False
        publishWaitOnce = False
        # while self.currentAutonomousMode == AutonomousMode.PATROL:
        while self.current_mode == 'PATROL':
            if (cancelPrevGoal == False):
                self.navigator.cancelTask()
                cancelPrevGoal = True
            if (self.isInTimeRange(self.patrolRosMsg)):
                if (publishStartOnce == False):
                    # self.autonomousModeFeedbackMsg.current_mode = 'PATROL'
                    # self.autonomousModeFeedbackMsg.state = 'PATROL_STARTED'
                    # self.autonomousModeFeedbackPub.publish(self.autonomousModeFeedbackMsg)
                    publishStartOnce = True
                shift_started = True
                if (self.patrolRosMsg.sequence_type == 'PALINDROME'):
                    goal_id = next(palindrome_gen)
                    goal = goal = self.getGoalById(goal_id)
                else:
                    #print(patrolGoalArr[patrolGoalArrIndex])
                    goal = self.getGoalById(patrolGoalArr[patrolGoalArrIndex])
                    patrolGoalArrIndex += 1
                    if patrolGoalArrIndex >= len(patrolGoalArr):
                        patrolGoalArrIndex = 0
                self.goalState.next_goal_id = goal['goal_id']
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()

                # Map goal data to PoseStamped fields
                goal_pose.pose.position.x = goal['x']
                goal_pose.pose.position.y = goal['y']
                goal_pose.pose.position.z = 0.0  # Assuming 2D navigation, z is set to 0.0

                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = goal['z']
                goal_pose.pose.orientation.w = goal['w']
                self.navigateToPoseBlock(goal_pose, self.goalState.next_goal_id)
                goal_counter += 1
                self.get_logger().info(f"Patrol goal number: {goal_counter} done")
            else:
                #if (self.isShuttleThreadAlive == False) and shift_started:
                if shift_started:
                    self.state_machine.update_state('GOAL_NAVIGATE')
                # else:
                if (publishWaitOnce == False):
                    self.get_logger().info(f"Patrol thread waiting ...")
                    # self.autonomousModeFeedbackMsg.current_mode = 'PATROL'
                    # self.autonomousModeFeedbackMsg.state = 'PATROL_WAITING'
                    # self.autonomousModeFeedbackPub.publish(self.autonomousModeFeedbackMsg)
                    publishWaitOnce = True
                time.sleep(1)
        self.isPatrolThreadAlive = False
        # self.autonomousModeFeedbackMsg.current_mode = 'PATROL'
        # self.autonomousModeFeedbackMsg.state = 'PATROL_FINISHED'
        # self.autonomousModeFeedbackPub.publish(self.autonomousModeFeedbackMsg)
        self.get_logger().info("Exit patrol thread")

    # def loadRouteList(self):
    #     try:
    #         with open(self.routeListFile, 'r') as f:
    #             data = json.load(f)
    #             if "routes" in data:
    #                 return data
    #             else:
    #                 self.get_logger().warn("JSON file exists but is missing 'routes' key. Initializing fresh structure.")
    #                 return {"routes": []}
    #     except FileNotFoundError:
    #         self.get_logger().warn("Routes JSON not found. Starting with an empty structure.")
    #         return {"routes": []}
    #     except json.JSONDecodeError:
    #         self.get_logger().warn("Invalid JSON file. Starting with an empty structure.")
    #         return {"routes": []}
        
    # def saveRoute(self):
    #     with open(self.routeListFile, 'w') as f:
    #         json.dump(self.routeList, f, indent=4)
    #     self.get_logger().info("Routes JSON saved.")

    # def buildRouteCb(self, msg):
    #     route_id = msg.route_id
    #     pose_id = msg.pose_id
    #     if self.currentTranslation and self.currentQuaternion:
    #         x = self.currentTranslation.vector.x
    #         y = self.currentTranslation.vector.y
    #         z = self.currentQuaternion.quaternion.z
    #         w = self.currentQuaternion.quaternion.w
    #         new_pose = {
    #             'x': x,
    #             'y': y,
    #             'z': z,
    #             'w': w
    #         }
    #     # Find or create the route
    #     route = next((r for r in self.routeList["routes"] if r["route_id"] == route_id), None)
    #     if not route:
    #         route = {"route_id": route_id, "poses": []}
    #         self.routeList["routes"].append(route)

    #     # Find or create the pose
    #     pose_id = int(pose_id)
    #     pose = next((p for p in route["poses"] if p["pose_id"] == pose_id), None)
    #     if not pose:
    #         pose = {"pose_id": pose_id}
    #         route["poses"].append(pose)

    #     pose.update(new_pose)
    #     routeListMsg = RouteList()
    #     routeListMsg = self.routeListToRosMsg(self.routeList)
    #     self.routeListPub.publish(routeListMsg)
    #     self.saveRoute()
    
    # def followRouteCb(self, msg):
    #     routeId = msg.route_id
    #     self.followRoute(routeId)

    # def getPosesFromRouteList(self, route_id):
    #     """Retrieve poses from the specified route_id."""
    #     route = next((r for r in self.routeList["routes"] if r["route_id"] == route_id), None)
    #     if not route:
    #         self.get_logger().warn(f"Route ID {route_id} not found in JSON.")
    #         return []

    #     poses = []
    #     for pose in route["poses"]:
    #         pose_stamped = PoseStamped()
    #         pose_stamped.header.frame_id = "map"  # Adjust based on your robot's frame
    #         pose_stamped.pose.position.x = pose["x"]
    #         pose_stamped.pose.position.y = pose["y"]
    #         pose_stamped.pose.orientation.z = pose["z"]
    #         pose_stamped.pose.orientation.w = pose["w"]
    #         poses.append(pose_stamped)
    #     return poses
    
    # def followRoute(self, route_id):
    #     """Navigate through all poses in the specified route."""
    #     poses = self.getPosesFromRouteList(route_id)
    #     if not poses:
    #         self.get_logger().error(f"No poses found for route_id: {route_id}")
    #         return

    #     self.get_logger().info(f"Navigating through {len(poses)} poses for route_id: {route_id}")
    #     self.navigator.clearGlobalCostmap()
    #     self.navigator.goThroughPoses(poses)
    #     self.monitoring_thread = threading.Thread(
    #         target=self.monitorTask, kwargs={"route_id": route_id}
    #     )
    #     self.monitoring_thread.daemon = True
    #     self.monitoring_thread.start()
    #     # while not self.navigator.isTaskComplete():
    #     #     feedback = self.navigator.getFeedback()
    #     #     if feedback:
    #     #         self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters.")

    #     # result = self.navigator.getResult()
    #     # if result == TaskResult.SUCCEEDED:
    #     #     self.get_logger().info("Navigation succeeded!")
    #     # elif result == TaskResult.CANCELED:
    #     #     self.get_logger().warn("Navigation canceled.")
    #     # elif result == TaskResult.FAILED:
    #     #     self.get_logger().error("Navigation failed.")
    #     # else:
    #     #     self.get_logger().warn("Unknown result.")

    # def routeListToRosMsg(self, routeList):
    #     if not routeList:
    #         return None
    #     # Create the PatrolConfig message
    #     routeListMsg = RouteList()

    #     for route in routeList['routes']:
    #         routeMsg = Route()
    #         routeMsg.route_id = route["route_id"]

    #         # Convert each pose in the route to a GoalPose message
    #         for pose in route["poses"]:
    #             purePose = PurePose()
    #             purePose.pose_id = pose["pose_id"]
    #             purePose.x = pose["x"]
    #             purePose.y = pose["y"]
    #             purePose.z = pose["z"]
    #             purePose.w = pose["w"]

    #             # Add the GoalPose to the Route
    #             routeMsg.poses.append(purePose)

    #         # Add the Route to the RouteList
    #         routeListMsg.routes.append(routeMsg)

    #     return routeListMsg

    def publishRobotPosition(self):
        # rotation = transform.transform.rotation
        quaternion = (
            self.currentQuaternion.quaternion.x,
            self.currentQuaternion.quaternion.y,
            self.currentQuaternion.quaternion.z,
            self.currentQuaternion.quaternion.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        robot_pose_msg = RobotPose()
        robot_pose_msg.vector.x = self.currentTranslation.vector.x
        robot_pose_msg.vector.y = self.currentTranslation.vector.y
        robot_pose_msg.vector.z = self.currentTranslation.vector.z
        robot_pose_msg.radian = yaw
        self.tf_baselink_to_map_pub.publish(robot_pose_msg)

    def goalListToRosMsg(self, goalList):
        if not goalList:
            return None
        # Create the PatrolConfig message
        goalListMsg = GoalList()

        for goal in goalList:
            goal_pose = GoalPose()
            goal_pose.goal_id = goal["goal_id"]
            goal_pose.x = goal["x"]
            goal_pose.y = goal["y"]
            goal_pose.z = goal["z"]
            goal_pose.w = goal["w"]
            goalListMsg.goals.append(goal_pose)

        return goalListMsg


def main():
    rclpy.init()
    rclpy.create_node('goal_executor')
    goalExecutor = GoalExecutor()
    goalExecutor.initNavigator()
    tf2Echo = TF2Echo('goalExecutor_tf2_echo')
    goalExecutor.signalAutonomousModeThread.start()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tf2Echo)
    executor.add_node(goalExecutor)
    # daemon is set to True. No need to join()
    et = threading.Thread(target=executor.spin, daemon=True)
    et.start()
    # Looping at 10HZ rate
    rate = tf2Echo.create_rate(10)
    # Loop indefinitely
    while rclpy.ok():
        tf2Echo.doLookUpTransform('base_link', 'map')
        transform = tf2Echo.getTF()
        if transform != None:
            goalExecutor.updateGoalPose(transform)
            goalExecutor.publishRobotPosition()
            #goalExecutor.dequeueAndMove()
        rate.sleep()

    tf2Echo.destroy_node()
    goalExecutor.destroy_node()

if __name__ == '__main__':
    main()