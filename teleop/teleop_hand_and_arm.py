import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
import numpy as np

from top_tier.hardware.robot import RobotFSMState, ControlMode
from top_tier.general.constants import WALKING_SPEED
from top_tier.general.motion_data import MOTION_DATA_DICT
from top_tier.general.xr_controllers import XRControllers, XRControllerButton, XRControllerFloat, XRControllerMatrix
from top_tier.hardware.joint import JointType
from top_tier.hardware.robot import Robot, RobotType

logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.top_tier.general.keyboard_listener import KeyboardListener
from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_JointArmIndex, R1_A5_JointIndex, R1_A5_JointArmIndex
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, R1_A5_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.ipc import IPC_Server
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }


def convert_to_joint_map(poses: np.ndarray, kffs: np.ndarray, robot_type: RobotType) -> dict[JointType, tuple[float, float]]:
    """
    Convert some joint-specific data to a dict which maps what joint it represents to its value

    Assumes the data only applies to the two arms, and is in the same order as used in xr_teleoperate

    Args:
        poses: the joint poses to convert, in a vector provided by unitree
        kffs: the joint feedforward values to convert, in a vector provided by unitree
        robot_type: what type of robot the data comes from

    Returns:
        the same data, keyed by JointType instead of an arbitrary index
    """
    joint_order = G1_29_JointArmIndex if robot_type == RobotType.G1 else R1_A5_JointIndex
    res = {}
    for unitree_joint, pos, kff in zip(joint_order, poses, kffs):
        joint = JointType.from_unitree(unitree_joint)
        assert joint.is_upper_body, f"{joint} (from {type(unitree_joint)}, {unitree_joint.name}) is not upper body"
        res[JointType.from_unitree(unitree_joint)] = (pos, kff)
    return res


def get_current_dual_arm_q(robot: Robot) -> np.ndarray:
    """
    Get the current position of the joints of the arms, in Unitree's format

    Args:
        robot: the robot that this code is controlling

    Returns:
        the positions of the arm joints (L then R) as a vector, with the order defined by the relevant Unitree enum
    """
    unitree_joint_type = G1_29_JointArmIndex if robot.robot_type == RobotType.G1 else R1_A5_JointArmIndex
    joint_order = [JointType.from_unitree(j) for j in unitree_joint_type]
    current_q = robot.get_current_joint_positions()
    return np.array([current_q[j] for j in joint_order])


def get_current_dual_arm_dq(robot: Robot) -> np.ndarray:
    """
    Get the current velocity of the joints of the arms, in Unitree's format

    Args:
        robot: the robot that this code is controlling

    Returns:
        the velocities of the arm joints (L then R) as a vector, with the order defined by the relevant Unitree enum
    """
    unitree_joint_type = G1_29_JointArmIndex if robot.robot_type == RobotType.G1 else R1_A5_JointArmIndex
    joint_order = [JointType.from_unitree(j) for j in unitree_joint_type]
    current_dq = robot.get_current_joint_velocities()
    return np.array([current_dq[j] for j in joint_order])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'R1_A5'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex1_internal', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    # network parameters
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    # record mode and task info
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    if args.ee == "dex1_internal" and args.motion:
        parser.error("--ee dex1_internal does not currently support --motion.")

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     arm_reference_mode="head_yaw"
                                     )

        xr_motion_data_ready = Value('b', False, lock=True)        # [input] whether XR hand/controller motion data has arrived

        if args.ee == "dex1_internal":
            if args.arm != "G1_29":
                raise ValueError("dex1_internal is only supported with --arm G1_29.")
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            robot_type = RobotType.G1
        elif args.arm == "R1_A5":
            arm_ik = R1_A5_ArmIK()
            robot_type = RobotType.R1
        else:
            raise ValueError(f"Unknown robot type: {args.arm}")

        # end-effector
        if args.ee in ("dex3", "inspire_ftp", "inspire_dfx") and args.input_mode == "controller":
            raise ValueError(f"{args.ee} does not support controller input mode.")
        elif args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "hand":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_hand
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_hand(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                                dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "controller":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_ctrl
            left_gripper_trigger_in = Value('d', 10.0, lock=True)  # [input]
            left_gripper_squeeze_in = Value('d', 0.0, lock=True)   # [input]
            right_gripper_trigger_in = Value('d', 10.0, lock=True) # [input]
            right_gripper_squeeze_in = Value('d', 0.0, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_ctrl(left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                                                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        else:
            pass
        
        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # extra setup for top tier
        controller_data = XRControllers(tv_wrapper)
        robot = Robot(robot_type, True)
        robot.initialize()
        time.sleep(0.5)
        robot.set_max_velocity(50.)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        logger_mp.info("🔵  Recording is DISABLED")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")

        robot.set_control_mode(ControlMode.Mixed)
        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        def print_state():
            print(robot.get_current_joint_positions())
        KeyboardListener.add_listener("l", print_state)

        # main loop. robot start to follow VR user's motion
        loop_time = 1. / args.frequency
        while not STOP:
            start_time = time.time()
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

            # get xr's tele data
            controller_data.update()
            with xr_motion_data_ready.get_lock():
                xr_motion_data_ready.value = controller_data.get_button(XRControllerButton.IsValid)
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # command robot to enter damping mode. soft emergency stop function
                if controller_data.get_button(XRControllerButton.LeftJoystick) and controller_data.get_button(XRControllerButton.RightJoystick):
                    robot.set_fsm_state(RobotFSMState.Damping)
                    START = False
                    STOP = True
                robot.loco_client.Move(controller_data.get_float(XRControllerFloat.LeftJoystickY) * WALKING_SPEED,
                                      -controller_data.get_float(XRControllerFloat.LeftJoystickX) * WALKING_SPEED,
                                      -controller_data.get_float(XRControllerFloat.RightJoystickX)* WALKING_SPEED)

            # get current robot state data.
            current_lr_arm_q = get_current_dual_arm_q(robot)
            current_lr_arm_dq = get_current_dual_arm_dq(robot)

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff = arm_ik.solve_ik(
                controller_data.get_matrix(XRControllerMatrix.LeftHandPose),
                controller_data.get_matrix(XRControllerMatrix.RightHandPose),
                current_lr_arm_q,
                current_lr_arm_dq
            )
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")

            # start an arm motion on button press
            for button, motion in MOTION_DATA_DICT.items():
                if controller_data.was_button_just_pressed(button):
                    robot.move_to_waypoints(motion, max_vel=3., block=False)
            # do teleop
            # any joints being controlled by a motion will not move here
            pose = convert_to_joint_map(sol_q, sol_tauff, robot.robot_type)
            robot.set_target_position(pose)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, loop_time - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process run time: {time_elapsed}, sleeping for {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        try:
            robot.shutdown(False)
        except Exception as e:
            logger_mp.error(f"Failed to shutdown robot: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")

        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
