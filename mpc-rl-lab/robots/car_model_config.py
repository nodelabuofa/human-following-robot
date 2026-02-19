from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

ACKERMANN_CAR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path="/path/to/your/ackermann_car.usd"),
    actuators={
        "drive_wheels": ImplicitActuatorCfg(
            joint_names_expr=["rear_left_wheel", "rear_right_wheel"],
            stiffness=0.0, damping=500.0
        ),
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["front_left_steer", "front_right_steer"],
            stiffness=800.0, damping=40.0
        ),
    },
)
