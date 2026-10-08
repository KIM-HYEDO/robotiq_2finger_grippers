# Tutorial 4: Real gripper

Control a real Robotiq 2F-85 / 2F-140 gripper through the USB RS-485 adapter (Modbus RTU).
The commands are the same as in the simulation tutorials, only the server talks to hardware.

## Safety checklist

- The gripper **activates when the server starts**, and the fingers may move. Keep them clear of
  objects and fingers (yours included) the first time.
- Start with low `force` and `speed`. Increase them only after you see how the gripper behaves.
- Make sure the gripper is mounted or held securely before closing it on an object.

## 1. Set up the serial port (once)

```bash
sudo adduser $USER dialout        # log out and in afterwards
dmesg | grep tty                  # look for "... converter now attached to ttyUSB0"
ls -l /dev/ttyUSB*                # the port must exist and belong to group dialout
```

The default baud rate is 115200 and must match the gripper configuration.

## 2. Start the server

```bash
roslaunch robotiq_2f_gripper_examples real_gripper.launch comport:=/dev/ttyUSB0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `comport` | `/dev/ttyUSB0` | Serial port of the gripper |
| `baud` | `115200` | Must match the gripper configuration |
| `stroke` | `0.085` | `0.085` for the 2F-85, `0.140` for the 2F-140 |
| `rviz` | `true` | Show the gripper model in RViz (set `false` on a machine without a display) |

The server is ready when the terminal prints `Robotiq server started`. If it prints
`Gripper on port ... seems not to respond`, see Troubleshooting.

## 3. Check the connection (does not move the gripper)

```bash
rosrun robotiq_2f_gripper_examples check_connection.py
```

It prints the current finger distance. With a 2F-140, add `_stroke:=0.140`.

## 4. Move the gripper

Any script from [Tutorial 2](../02_python_client) and the GUI from [Tutorial 3](../03_gui) work with the
real gripper. Keep the server from step 2 running. For the GUI, start only the node
instead of `gripper_gui.launch` (the launch file starts its own server):

```bash
rosrun robotiq_2f_gripper_examples robotiq_2f_gripper_gui.py
```

## 5. Grasp an object

```bash
rosrun robotiq_2f_gripper_examples grasp_object.py _force:=20
```

The script opens the gripper, waits 3 seconds for you to place an object, closes slowly with low
force, and prints the object width when `obj_detected` is true. Then it releases.

## Troubleshooting

| Symptom | Check |
|---------|-------|
| `Permission denied: '/dev/ttyUSB0'` | You are not in the `dialout` group, or you have not logged in again |
| `Gripper on port ... seems not to respond` | Wrong `comport` or `baud`, loose cable, gripper not powered (24 V) |
| `/dev/ttyUSB0` missing | Replug the adapter and run `dmesg \| grep tty`. The port number can change |
| Another program uses the port | Close other serial tools, then restart the server |
| Fingers do not reach the target | Normal when an object is in the way. Check `obj_detected` in the result |
