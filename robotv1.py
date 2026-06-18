import math
import os
import time

import pybullet as p
import pybullet_data


# ============================================================
# 1. Diana7 URDF 文件路径
# ============================================================

ROBOT_PATH = (
    r"D:\robot_demo\diana7_demo_starter_v1"
    r"\diana7_demo_starter_v1"
    r"\assets\diana7\urdf\diana7_pybullet.urdf"
)


# ============================================================
# 2. 场景参数
#
# 坐标约定：
# X：左右方向
# Y：前后方向，+Y 为工作台前方
# Z：竖直向上
# ============================================================

# ---------------- 工作台 ----------------

# 台面尺寸：X长度、Y宽度、Z厚度，单位 m
TABLE_TOP_SIZE = [1.30, 0.85, 0.06]

# 台面中心位置
TABLE_TOP_POSITION = [0.0, 0.38, 0.46]

# 桌腿尺寸
TABLE_LEG_SIZE = [0.06, 0.06, 0.46]

# 四条桌腿的位置参数
TABLE_LEG_X = 0.58
TABLE_LEG_Y_FRONT = 0.72
TABLE_LEG_Y_REAR = 0.04
# ---------------- 鼠标中键作用 ----------------

# 鼠标中键拖动视角相关参数
middle_mouse_down = False
last_mouse_x = 0
last_mouse_y = 0

# PyBullet 里鼠标中键通常是 2
# 如果你发现中键没反应，可以把它改成 1 试试
MIDDLE_BUTTON_INDEX = 1

# ---------------- 中央机器人本体 ----------------

# 底部基座
PEDESTAL_SIZE = [0.68, 0.56, 0.22]
PEDESTAL_POSITION = [0.0, -0.42, 0.11]

# 中央主体
BODY_SIZE = [0.48, 0.42, 0.78]
BODY_POSITION = [0.0, -0.42, 0.61]

# 肩部横梁
SHOULDER_SIZE = [1.10, 0.32, 0.26]
SHOULDER_POSITION = [0.0, -0.42, 1.05]


# ---------------- 左右机械臂 ----------------

# 两台机械臂安装在肩部横梁左右两侧
LEFT_BASE_POSITION = [-0.68, -0.42, 1.05]
RIGHT_BASE_POSITION = [0.68, -0.42, 1.05]

# Diana7 原本是竖直安装。
# 绕 Y 轴旋转 90° 后，变成左右侧装，类似人的两条手臂。
LEFT_BASE_EULER = [
    0.0,
    -math.pi / 2.0,
    0.0,
]

RIGHT_BASE_EULER = [
    0.0,
    math.pi / 2.0,
    0.0,
]


# ---------------- 初始关节角 ----------------

# 左臂初始关节角，单位 rad
LEFT_INITIAL_JOINT_ANGLES = [
    0.0,
    -0.35,
    0.0,
    1.15,
    0.0,
    0.35,
    0.0,
]

# 右臂初始关节角，单位 rad
RIGHT_INITIAL_JOINT_ANGLES = [
    0.0,
    0.35,
    0.0,
    1.15,
    0.0,
    -0.35,
    0.0,
]


# ---------------- 相机参数 ----------------

# 正面观察，工作台在前，机器人本体在后
CAMERA_DISTANCE = 3.20
CAMERA_YAW = 180.0
CAMERA_PITCH = -12.0
CAMERA_TARGET = [0.0, 0.05, 0.72]


# ============================================================
# 3. 基础几何体创建函数
# ============================================================

def create_box(
    size,
    position,
    color,
    collision_enabled=True,
):
    """
    创建一个固定长方体。

    参数
    ----------
    size:
        [X长度, Y宽度, Z高度]，单位 m。
    position:
        几何体中心位置。
    color:
        RGBA颜色。
    collision_enabled:
        是否创建碰撞体。
    """

    half_extents = [
        value / 2.0
        for value in size
    ]

    if collision_enabled:
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=half_extents,
        )
    else:
        collision_shape = -1

    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color,
    )

    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )

    return body_id


# ============================================================
# 4. 创建工作台
# ============================================================

def create_table():
    """创建工作台台面和四条桌腿。"""

    table_ids = []

    # 工作台台面
    table_top_id = create_box(
        size=TABLE_TOP_SIZE,
        position=TABLE_TOP_POSITION,
        color=[0.78, 0.78, 0.82, 1.0],
    )

    table_ids.append(table_top_id)

    # 桌腿中心高度
    leg_z = TABLE_LEG_SIZE[2] / 2.0

    leg_positions = [
        [
            -TABLE_LEG_X,
            TABLE_LEG_Y_FRONT,
            leg_z,
        ],
        [
            TABLE_LEG_X,
            TABLE_LEG_Y_FRONT,
            leg_z,
        ],
        [
            -TABLE_LEG_X,
            TABLE_LEG_Y_REAR,
            leg_z,
        ],
        [
            TABLE_LEG_X,
            TABLE_LEG_Y_REAR,
            leg_z,
        ],
    ]

    for leg_position in leg_positions:
        leg_id = create_box(
            size=TABLE_LEG_SIZE,
            position=leg_position,
            color=[0.28, 0.30, 0.34, 1.0],
        )

        table_ids.append(leg_id)

    return table_ids


# ============================================================
# 5. 创建中央机器人本体
# ============================================================

def create_robot_body():
    """
    创建：
    1. 底部基座；
    2. 中央主体；
    3. 肩部横梁。
    """

    pedestal_id = create_box(
        size=PEDESTAL_SIZE,
        position=PEDESTAL_POSITION,
        color=[0.25, 0.27, 0.31, 1.0],
    )

    body_id = create_box(
        size=BODY_SIZE,
        position=BODY_POSITION,
        color=[0.48, 0.50, 0.56, 1.0],
    )

    shoulder_id = create_box(
        size=SHOULDER_SIZE,
        position=SHOULDER_POSITION,
        color=[0.36, 0.38, 0.44, 1.0],
    )

    return (
        pedestal_id,
        body_id,
        shoulder_id,
    )


# ============================================================
# 6. 加载机器人
# ============================================================

def load_robot(position, euler):
    """加载一台固定基座的 Diana7。"""

    robot_orientation = p.getQuaternionFromEuler(
        euler
    )

    robot_id = p.loadURDF(
        fileName=ROBOT_PATH,
        basePosition=position,
        baseOrientation=robot_orientation,
        useFixedBase=True,
    )

    return robot_id


def set_robot_color(robot_id, rgba_color):
    """为机器人基座和全部连杆设置统一颜色。"""

    # 机器人基座
    p.changeVisualShape(
        objectUniqueId=robot_id,
        linkIndex=-1,
        rgbaColor=rgba_color,
    )

    # 各活动连杆
    for joint_index in range(
        p.getNumJoints(robot_id)
    ):
        p.changeVisualShape(
            objectUniqueId=robot_id,
            linkIndex=joint_index,
            rgbaColor=rgba_color,
        )


# ============================================================
# 7. 读取可动关节
# ============================================================

def get_movable_joints(robot_id):
    """读取旋转关节和移动关节。"""

    movable_joints = []

    number_of_joints = p.getNumJoints(
        robot_id
    )

    for joint_index in range(
        number_of_joints
    ):
        joint_info = p.getJointInfo(
            robot_id,
            joint_index,
        )

        joint_name = joint_info[1].decode(
            "utf-8"
        )

        joint_type = joint_info[2]

        lower_limit = joint_info[8]
        upper_limit = joint_info[9]

        link_name = joint_info[12].decode(
            "utf-8"
        )

        # 跳过固定关节
        if joint_type not in (
            p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC,
        ):
            continue

        # 如果URDF没有有效的限位，则临时使用[-pi, pi]
        if lower_limit >= upper_limit:
            lower_limit = -math.pi
            upper_limit = math.pi

        movable_joints.append(
            {
                "index": joint_index,
                "name": joint_name,
                "link_name": link_name,
                "lower": lower_limit,
                "upper": upper_limit,
            }
        )

    return movable_joints


# ============================================================
# 8. 创建关节滑块
# ============================================================

def create_joint_sliders(
    robot_id,
    prefix,
    initial_joint_angles,
):
    """
    为一台机器人创建关节滑块。

    prefix:
        L 表示左臂；
        R 表示右臂。
    """

    joints = get_movable_joints(
        robot_id
    )

    sliders = []

    if len(joints) != 7:
        print(
            f"警告：{prefix}臂识别到 "
            f"{len(joints)} 个可动关节，"
            "Diana7正常应为7个。"
        )

    for order, joint in enumerate(joints):

        if order < len(initial_joint_angles):
            initial_value = (
                initial_joint_angles[order]
            )
        else:
            initial_value = 0.0

        # 将初始关节角限制在URDF上下限内
        initial_value = max(
            joint["lower"],
            min(
                joint["upper"],
                initial_value,
            ),
        )

        slider_id = p.addUserDebugParameter(
            paramName=(
                f"{prefix}_{joint['name']}"
            ),
            rangeMin=joint["lower"],
            rangeMax=joint["upper"],
            startValue=initial_value,
        )

        # 设置机器人初始关节状态
        p.resetJointState(
            bodyUniqueId=robot_id,
            jointIndex=joint["index"],
            targetValue=initial_value,
        )

        sliders.append(
            {
                "joint_index": joint["index"],
                "slider_id": slider_id,
                "joint_name": joint["name"],
                "link_name": joint["link_name"],
            }
        )

    return sliders


# ============================================================
# 9. 根据滑块控制机器人
# ============================================================

def update_robot_from_sliders(
    robot_id,
    sliders,
):
    """读取滑块值，并更新机器人关节。"""

    for slider in sliders:

        target_angle = (
            p.readUserDebugParameter(
                slider["slider_id"]
            )
        )

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=slider["joint_index"],
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=400.0,
            positionGain=0.30,
            velocityGain=1.00,
        )
# ============================================================
# 9. 根据鼠标中键控制视角
# ============================================================


def handle_middle_mouse_pan():
    global middle_mouse_down, last_mouse_x, last_mouse_y

    mouse_events = p.getMouseEvents()

    for event in mouse_events:
        event_type = event[0]
        mouse_x = event[1]
        mouse_y = event[2]
        button_index = event[3]
        button_state = event[4]

        if event_type == 2 and button_index == MIDDLE_BUTTON_INDEX:
            if button_state & p.KEY_IS_DOWN:
                middle_mouse_down = True
                last_mouse_x = mouse_x
                last_mouse_y = mouse_y
            else:
                middle_mouse_down = False

        if event_type == 1 and middle_mouse_down:
            dx = mouse_x - last_mouse_x
            dy = mouse_y - last_mouse_y

            last_mouse_x = mouse_x
            last_mouse_y = mouse_y

            cam = p.getDebugVisualizerCamera()

            camera_distance = cam[10]
            camera_yaw = cam[8]
            camera_pitch = cam[9]
            camera_target = list(cam[11])

            yaw = math.radians(camera_yaw)
            pitch = math.radians(camera_pitch)

            # 相机看向的方向
            forward_dir = [
                math.cos(pitch) * math.sin(yaw),
                math.cos(pitch) * math.cos(yaw),
                math.sin(pitch),
            ]

            # 相机屏幕的右方向
            right_dir = [
                math.cos(yaw),
                -math.sin(yaw),
                0.0,
            ]

            # 相机屏幕的上方向 = 右方向 × 前方向
            up_dir = [
                right_dir[1] * forward_dir[2] - right_dir[2] * forward_dir[1],
                right_dir[2] * forward_dir[0] - right_dir[0] * forward_dir[2],
                right_dir[0] * forward_dir[1] - right_dir[1] * forward_dir[0],
            ]

            pan_speed = camera_distance * 0.0015

            camera_target[0] -= dx * pan_speed * right_dir[0]
            camera_target[1] -= dx * pan_speed * right_dir[1]
            camera_target[2] -= dx * pan_speed * right_dir[2]

            camera_target[0] += dy * pan_speed * up_dir[0]
            camera_target[1] += dy * pan_speed * up_dir[1]
            camera_target[2] += dy * pan_speed * up_dir[2]

            p.resetDebugVisualizerCamera(
                cameraDistance=camera_distance,
                cameraYaw=camera_yaw,
                cameraPitch=camera_pitch,
                cameraTargetPosition=camera_target,
            )


# ============================================================
# 10. 主程序
# ============================================================

def main():

    # 检查URDF路径
    if not os.path.exists(ROBOT_PATH):
        raise FileNotFoundError(
            "找不到机器人URDF文件：\n"
            f"{ROBOT_PATH}"
        )

    # 连接PyBullet图形界面
    client_id = p.connect(p.GUI)
    p.configureDebugVisualizer(
    p.COV_ENABLE_MOUSE_PICKING,
    0,
)

    if client_id < 0:
        raise RuntimeError(
            "PyBullet GUI连接失败。"
        )

    try:
        # 设置PyBullet数据路径
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        # 设置重力
        p.setGravity(
            0.0,
            0.0,
            -9.81,
        )

        # 使用手动步进仿真
        p.setRealTimeSimulation(0)

        p.setPhysicsEngineParameter(
            fixedTimeStep=1.0 / 240.0,
            numSolverIterations=50,
        )

        # 关闭左侧三个相机预览窗口
        p.configureDebugVisualizer(
            p.COV_ENABLE_RGB_BUFFER_PREVIEW,
            0,
        )

        p.configureDebugVisualizer(
            p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
            0,
        )

        p.configureDebugVisualizer(
            p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
            0,
        )

        # 加载地面
        plane_id = p.loadURDF(
            "plane.urdf"
        )

        # 创建工作台
        table_ids = create_table()

        # 创建中央本体
        (
            pedestal_id,
            body_id,
            shoulder_id,
        ) = create_robot_body()

        # 加载左机械臂
        left_robot = load_robot(
            position=LEFT_BASE_POSITION,
            euler=LEFT_BASE_EULER,
        )

        # 加载右机械臂
        right_robot = load_robot(
            position=RIGHT_BASE_POSITION,
            euler=RIGHT_BASE_EULER,
        )

        # 左臂红色
        set_robot_color(
            robot_id=left_robot,
            rgba_color=[
                0.78,
                0.22,
                0.22,
                1.0,
            ],
        )

        # 右臂蓝色
        set_robot_color(
            robot_id=right_robot,
            rgba_color=[
                0.22,
                0.36,
                0.82,
                1.0,
            ],
        )

        # 添加左臂文字
        p.addUserDebugText(
            text="LEFT ARM",
            textPosition=[
                -0.72,
                -0.42,
                1.68,
            ],
            textColorRGB=[
                1.0,
                0.0,
                0.0,
            ],
            textSize=1.2,
        )

        # 添加右臂文字
        p.addUserDebugText(
            text="RIGHT ARM",
            textPosition=[
                0.72,
                -0.42,
                1.68,
            ],
            textColorRGB=[
                0.0,
                0.0,
                1.0,
            ],
            textSize=1.2,
        )

        # 创建左臂7个滑块
        left_sliders = create_joint_sliders(
            robot_id=left_robot,
            prefix="L",
            initial_joint_angles=(
                LEFT_INITIAL_JOINT_ANGLES
            ),
        )

        # 创建右臂7个滑块
        right_sliders = create_joint_sliders(
            robot_id=right_robot,
            prefix="R",
            initial_joint_angles=(
                RIGHT_INITIAL_JOINT_ANGLES
            ),
        )

        print("=" * 60)
        print("人形双臂场景加载成功")
        print(f"地面ID：{plane_id}")
        print(
            f"工作台部件数量："
            f"{len(table_ids)}"
        )
        print(f"底部基座ID：{pedestal_id}")
        print(f"中央本体ID：{body_id}")
        print(f"肩部横梁ID：{shoulder_id}")
        print(f"左臂ID：{left_robot}")
        print(f"右臂ID：{right_robot}")
        print(
            "左臂可动关节数量："
            f"{len(left_sliders)}"
        )
        print(
            "右臂可动关节数量："
            f"{len(right_sliders)}"
        )
        print(
            "右侧Params面板应出现"
            "14个关节滑块。"
        )
        print(
            "退出方式：Bullet窗口按Q或ESC，"
            "或者终端按Ctrl+C。"
        )
        print("=" * 60)

        # 正面相机
        p.resetDebugVisualizerCamera(
            cameraDistance=CAMERA_DISTANCE,
            cameraYaw=CAMERA_YAW,
            cameraPitch=CAMERA_PITCH,
            cameraTargetPosition=CAMERA_TARGET,
        )

        # 主循环
        while p.isConnected():
            handle_middle_mouse_pan()

            keyboard_events = (
                p.getKeyboardEvents()
            )

            q_pressed = (
                ord("q") in keyboard_events
                and keyboard_events[ord("q")]
                & p.KEY_WAS_TRIGGERED
            )

            esc_pressed = (
                27 in keyboard_events
                and keyboard_events[27]
                & p.KEY_WAS_TRIGGERED
            )

            if q_pressed or esc_pressed:
                print(
                    "收到退出按键，程序结束。"
                )
                break

            # 更新左臂
            update_robot_from_sliders(
                robot_id=left_robot,
                sliders=left_sliders,
            )

            # 更新右臂
            update_robot_from_sliders(
                robot_id=right_robot,
                sliders=right_sliders,
            )

            # 仿真步进
            p.stepSimulation()

            time.sleep(
                1.0 / 240.0
            )

    except KeyboardInterrupt:
        print(
            "收到Ctrl+C，程序结束。"
        )

    except p.error as exc:
        print(
            "PyBullet窗口已关闭或连接中断："
            f"{exc}"
        )

    finally:
        if p.isConnected():
            p.disconnect()

        print(
            "PyBullet已断开连接。"
        )


if __name__ == "__main__":
    main() 
