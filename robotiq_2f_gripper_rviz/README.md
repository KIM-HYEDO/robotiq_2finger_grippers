# robotiq_2f_gripper_rviz

RViz panel with the controls of [`example/03_gui`](../example/03_gui): speed and force sliders, **OPEN**,
**CLOSE**, **STOP** and a status line. It sends `CommandRobotiqGripper` goals to the action server of
`robotiq_2f_action_server.py`, so it works with a real gripper and with `sim:=true`.

RViz panels are C++ Qt plugins (RViz has no Python panel API), which is why this package is C++ while the
examples are Python.

## Use

```bash
cd ~/catkin_ws && catkin build robotiq_2f_gripper_rviz && source devel/setup.bash
roslaunch robotiq_2f_gripper_examples sim_quickstart.launch    # action server + RViz
```

In RViz: **Panels > Add New Panel > robotiq_2f_gripper_rviz > GripperPanel**. Save the RViz config to keep it.

| Control | Meaning |
|---------|---------|
| Action | Name of the action server, `command_robotiq_action` by default (e.g. `/left/command_robotiq_action` for two grippers) |
| Speed | 0.013 to 0.1 m/s |
| Force | 0 to 100 % |
| OPEN / CLOSE | Full stroke at the chosen speed and force. Buttons are enabled once the server is connected |
| STOP | Stops the current motion (real gripper only, the simulation ignores `stop`) |
| Status | Server state, motion or object detection, fault code, and the finger width in mm |

The width comes from the action feedback while a goal runs and from `finger_joint` in `/joint_states`
otherwise.

## RViz config keys

Saved with the panel in the `.rviz` file: `action_name`, `speed` [m/s], `force` [%], `stroke` [m]
(0.085 for the 2F-85, 0.140 for the 2F-140) and `joint_name`.
