#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from sensor_msgs.msg import LaserScan
from utils import nearest_point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# TODO CHECK: include needed ROS msg type headers and libraries


@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = = [steering, acceleration]
    TK: int = 8  # finite time horizon length kinematic

    # ---------------------------------------------------
    # TODO: you may need to tune the following matrices
    Rk: list = field(
        default_factory=lambda: np.diag([0.1, 50.0]) #0.01 100
    )  # input cost matrix, penalty for inputs - [accel, steering]
    Rdk: list = field(
        default_factory=lambda: np.diag([0.1, 50.0])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering]
    Qk: list = field(
        default_factory=lambda: np.diag([38.0, 53.5, 43.0, 3.0]) #13.5, 13.5, 13.0, 5.5
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, v, yaw]
    Qfk: list = field(
        default_factory=lambda: np.diag([38.0, 53.5, 43.0, 3.0])
    )  # final state error matrix, penalty  for the final state constraints: [x, y, v, yaw]
    # ----------------------------------------------f---

    N_IND_SEARCH: int = 20  # Search index number
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.35  # dist step [m] kinematic 0.03
    LENGTH: float = 0.58  # Length of the vehicle [m]
    WIDTH: float = 0.31  # Width of the vehicle [m]
    WB: float = 0.33  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad(180.0)  # maximum steering speed [rad/s]
    MAX_SPEED: float = 3.0  # maximum speed [m/s] 6
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 3.0  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    
class MPC(Node):
    """ 
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('mpc_node')
        # TODO: create ROS subscribers and publishers
        #       use the MPC as a tracker (similar to pure pursuit)
        # TODO: get waypoints here


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ogrid_pub = self.create_publisher(OccupancyGrid, 'Ogrid', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        
        self.occupancyGrid = None
        self.gridSize = 50
        self.resolution = 0.05
        self.lastTime = 0

        self.poseSub = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 10)

        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.path_pub = self.create_publisher(
            MarkerArray,
            "/selected_path",
            1
        )

        filename = '/home/tianhao/sim_ws/src/f1tenth_lab7/mpc/waypoint1atrium4M.csv' #default track
        filenameO = '/home/tianhao/sim_ws/src/f1tenth_lab7/mpc/waypoint1atrium4MOver.csv' #overtake
        with open (filename, 'r') as f:
            lines = f.readlines()
            self.wp = []

            for line in lines:
                count = 0
                tempx = ''
                tempy = ''
                tempyaw = ''
                tempv = ''
                for i in range (0, len(line) -1):
                    if count == 0 and line[i] != ',':
                        tempx += line[i]
                    elif count == 1 and line[i] != ',':
                        tempy += line[i]
                    elif count == 2 and line[i] != ',':
                        tempyaw += line[i]
                    elif count == 3 and line[i] != ',':
                        tempv += line[i]
                    else:
                        count += 1
                    
                if tempx != '' and tempy != '':
                    self.wp.append([float(tempx), float(tempy), float(tempyaw), float(tempv)])

        with open (filenameO, 'r') as f:
            lines = f.readlines()
            self.wpO = []

            for line in lines:
                count = 0
                tempx = ''
                tempy = ''
                tempyaw = ''
                tempv = ''
                for i in range (0, len(line) -1):
                    if count == 0 and line[i] != ',':
                        tempx += line[i]
                    elif count == 1 and line[i] != ',':
                        tempy += line[i]
                    elif count == 2 and line[i] != ',':
                        tempyaw += line[i]
                    elif count == 3 and line[i] != ',':
                        tempv += line[i]
                    else:
                        count += 1
                    
                if tempx != '' and tempy != '':
                    self.wpO.append([float(tempx), float(tempy), float(tempyaw), float(tempv)])

        
        
        # print(self.wp)
        self.wp = np.array(self.wp)
        self.wpO = np.array(self.wpO)

        # self.waypoints = None

        self.config = mpc_config()
        self.odelta = None
        self.oa = None
        self.init_flag = 0

        # initialize MPC problem
        self.mpc_prob_init()

    def scan_callback(self, scan_msg):
        interval = scan_msg.header.stamp.nanosec - self.lastTime
        if interval > 1e7 or interval < 0:
            self.lastTime = scan_msg.header.stamp.nanosec
            self.occupancyGrid = np.full((self.gridSize, self.gridSize), 1)

            # x<-----
            #       |
            #       | car
            #       |
            #       y
        
            for i in range(180, 901): 
                for j in range(0, int(scan_msg.ranges[i]/ self.resolution)):
                    x = j * math.sin(scan_msg.angle_increment * (i - 180))
                    y = self.gridSize/2 - j * math.cos(scan_msg.angle_increment * (i - 180))
                    if x >= 0 and x <= self.gridSize - 1 and y >= 0 and y <= self.gridSize - 1:
                        self.occupancyGrid[int(x)][int(y)] = 0

            msg = OccupancyGrid()
            msg.header.frame_id = 'ego_racecar/base_link'
            msg.info.width = self.gridSize
            msg.info.height = self.gridSize
            msg.info.resolution = self.resolution
            msg.info.origin.position.x =  0.0
            msg.info.origin.position.y = -1.25

            msg.data = [0] * (msg.info.width * msg.info.height)

            for i2 in range (0, self.gridSize):
                for j2 in range (0, self.gridSize):
                    # print(int(self.occupancyGrid[i2][j2]))
                    msg.data[i2 * self.gridSize + j2] = int(self.occupancyGrid[j2][i2] * 100)
            
            self.ogrid_pub.publish(msg)
    


    def pose_callback(self, pose_msg:Odometry):

        

        # TODO: extract pose from ROS msg
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                                pose_msg.pose.pose.orientation.y,
                                pose_msg.pose.pose.orientation.z,
                                pose_msg.pose.pose.orientation.w])
        euler = euler_from_quaternion(quaternion)
        euler_in_2pi = euler[2]
        if euler_in_2pi < 0:
            euler_in_2pi += 2 * np.pi
        # elif euler_in_2pi > 2 * np.pi:
        #     euler_in_2pi -= 2 * np.pi
        # print("euler ", euler[2])
        # print("euler_in_2pi", euler_in_2pi)
        vehicle_state = State()
        vehicle_state.x = pose_msg.pose.pose.position.x
        vehicle_state.y = pose_msg.pose.pose.position.y
        # print("speeeeeeeed is ", pose_msg.twist.twist.linear.x)
        vehicle_state.v = pose_msg.twist.twist.linear.x
        # vehicle_state.v = 2.0
        vehicle_state.yaw = euler_in_2pi

        _, _, _, ind = nearest_point(np.array([vehicle_state.x, vehicle_state.y]), np.array([self.wp[:, 0], self.wp[:, 1]]).T)
        print("idx", ind)

        _, _, _, indO = nearest_point(np.array([vehicle_state.x, vehicle_state.y]), np.array([self.wpO[:, 0], self.wpO[:, 1]]).T)

        flag1 = True
        flag2 = True

        
        for i in range(-1, 1): #1,2
            if ind + i >= len(self.wp):
                idxtemp = ind + i - len(self.wp)
            else:
                idxtemp = ind + i
            if indO + i >= len(self.wp):
                idxtempO = indO + i - len(self.wpO)
            else:
                idxtempO = indO + i    

            print(idxtemp)
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x = float(self.wp[idxtemp][0])
            point.point.y = float(self.wp[idxtemp][1])
            point.point.z = 0.0

            pointO = PointStamped()
            pointO.header.frame_id = "map"
            pointO.point.x = float(self.wpO[idxtempO][0])
            pointO.point.y = float(self.wpO[idxtempO][1])
            pointO.point.z = 0.0

            try:
                t = self.tf_buffer.lookup_transform("ego_racecar/base_link", "map", rclpy.time.Time())
                pose_transformed = tf2_geometry_msgs.do_transform_point(point, t)
                pose_transformedO = tf2_geometry_msgs.do_transform_point(pointO, t)
                # self.get_logger().info(f"Transformed pose: {pose_transformed}")
                # transfer waypoint to car frame, find the point in freespace to be the goal
                # print("pose x y", pose_transformed.point.x, pose_transformed.point.y)
                tx = int(pose_transformed.point.x/self.resolution)
                ty = int(pose_transformed.point.y/self.resolution)
                # print("tx, ty", tx, int(self.gridSize/2 + ty))
                if ty > -self.gridSize/2 - 1 and ty < self.gridSize/2 - 1 and tx > 0 and tx < self.gridSize - 1:
                    
                    if self.occupancyGrid[int(tx)][int(self.gridSize/2 + ty)] == 1:
                        # print("ffff")
                        flag1 = False

                txO = int(pose_transformedO.point.x/self.resolution)
                tyO = int(pose_transformedO.point.y/self.resolution)
                if tyO > -self.gridSize/2 and tyO < self.gridSize/2 and txO > 0 and txO < self.gridSize:
                    if self.occupancyGrid[int(txO)][int(self.gridSize/2 + tyO)] == 1:
                        flag2 = False
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform')
        speedFlag = True
                
        if flag1:
            print("f1")
            ref_x = self.wp[:, 0]
            ref_y = self.wp[:, 1]
            ref_yaw = self.wp[:, 2]
            ref_v = self.wp[:, 3]
        elif flag2:
            print("f2")
            ref_x = self.wpO[:, 0]
            ref_y = self.wpO[:, 1]
            ref_yaw = self.wpO[:, 2]
            ref_v = self.wpO[:, 3]
        else:
            print("f3")
            ref_x = self.wp[:, 0]
            ref_y = self.wp[:, 1]
            ref_yaw = self.wp[:, 2]
            ref_v = self.wp[:, 3]
            speedFlag = False





        # TODO: Calculate the next reference trajectory for the next T steps
        #       with current vehicle pose.
        #       ref_x, ref_y, ref_yaw, ref_v are columns of self.waypoints
        ref_path = self.calc_ref_trajectory(vehicle_state, ref_x, ref_y, ref_yaw, ref_v)
        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        # TODO: solve the MPC control problem
        (
            self.oa,
            self.odelta,
            ox,
            oy,
            oyaw,
            ov,
            state_predict,
        ) = self.linear_mpc_control(ref_path, x0, self.oa, self.odelta)

        # TODO: publish drive message.
        steer_output = self.odelta[0]
        speed_output = max(vehicle_state.v + self.oa[0] * self.config.DTK, 1.0)
        # print("steer_output = ", steer_output, "speed_output = ", speed_output)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer_output
        if speedFlag:
            drive_msg.drive.speed = speed_output
        else:
            drive_msg.drive.speed = 0.5
        self.ackermann_pub.publish(drive_msg)

    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
        Will be solved every iteration for control.
        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
        More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html
        """
        # Initialize and create vectors for the optimization problem
        # Vehicle State Vector
        self.xk = cvxpy.Variable(
            (self.config.NXK, self.config.TK + 1)
        )
        # Control Input vector
        self.uk = cvxpy.Variable(
            (self.config.NU, self.config.TK)
        )
        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        # Initializes block diagonal form of R = [R, R, ..., R] (NU*T, NU*T)
        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (NU*(T-1), NU*(T-1))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.TK)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere

        # TODO: Objective part 1: Influence of the control inputs: Inputs u multiplied by the penalty R
        # print(R_block)
        # print(self.uk.shape)
        # print(cvxpy.vec(self.uk).value)
        objective += cvxpy.quad_form(cvxpy.vec(self.uk), R_block)
        # print(objective)
        # TODO: Objective part 2: Deviation of the vehicle from the reference trajectory weighted by Q, including final Timestep T weighted by Qf
        objective += cvxpy.quad_form(cvxpy.vec(self.xk - self.ref_traj_k), Q_block)
        # TODO: Objective part 3: Difference from one control input to the next control input weighted by Rd
        objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)
        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            # print(path_predict[2, t], path_predict[3, t])
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            # print(A)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)


        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape

        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))

        # Setting sparse matrix data
        self.Annz_k.value = A_block.data
 

        # Now we use this sparse version instead of the old A_ block matrix
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")
        # print(self.Ak_)

        # Same as A
        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")
        self.Bnnz_k.value = B_block.data

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem
        #       This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_
        constraints += [cvxpy.vec(self.xk[:, 1:]) == self.Ak_ @ cvxpy.vec(self.xk[:, :-1]) + self.Bk_ @ cvxpy.vec(self.uk) + (self.Ck_)]
        
        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle
        #       cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.DTK
        constraints += [cvxpy.abs(cvxpy.diff(self.uk[1, :]))/self.config.DTK<=self.config.MAX_DSTEER]
        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs
        #       and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER
        constraints += [self.xk[:,0] == self.x0k, self.xk[2,:] >= self.config.MIN_SPEED, self.xk[2,:] <= self.config.MAX_SPEED, 
                        self.uk[0,:] <= self.config.MAX_ACCEL, self.uk[0,:] >= -self.config.MAX_ACCEL, 
                        self.uk[1,:] <= self.config.MAX_STEER, self.uk[1,:] >= -self.config.MAX_STEER]
        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Heading
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # print(np.array([cx, cy]))
        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state.v) * self.config.DTK
        # print("t is ", travel)
        dind = travel / self.config.dlk
        # print(dind)
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)

        ind_list[ind_list >= ncourse] -= ncourse
        # print(ind_list)
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(
            cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state.yaw < -4.5] = np.abs(
            cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi)
        )
        ref_traj[3, :] = cyaw[ind_list]
        # print("ref  ", ref_traj[3, :])
        self.visualize_path(ref_traj)
        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict
    
    def visualize_path(self, path):
        # Create a MarkerArray
        marker_array = MarkerArray()

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]

            # LINE STRIPS
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            point1 = Point()
            point1.x=path[0, i]
            point1.y=path[1, i]
            point2 = Point()
            point2.x = path[0, i+1]
            point2.y = path[1, i+1]
            line_marker.points.append(point1)
            line_marker.points.append(point2)
            line_marker.scale.x = 0.08  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.lifetime.sec = 1
            marker_array.markers.append(line_marker)
        self.path_pub.publish(marker_array)

    def update_state(self, state, a, delta):

        # input check
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = (
            state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        )
        state.v = state.v + a * self.config.DTK

        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
        Linear System: Xdot = Ax +Bu + C
        State vector: x=[x, y, v, yaw]
        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        # print("path_predict = ", path_predict)
        # self.visualize_path(path_predict)

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict

def main(args=None):
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()