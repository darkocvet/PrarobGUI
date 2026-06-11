# PRAROB Complete Presentation Runbook

Follow these exact steps to successfully launch the AI Drawing Robot from scratch in your lab environment using WSL.

---

## 1. Physical Hardware & WSL USB Passthrough

Because you are running Linux inside Windows (WSL), USB devices must be explicitly handed over to WSL.

1. Plug in the USB Web Camera.
2. Plug in the U2D2 / Dynamixel adapter for the robot arm.
3. Open a **Windows PowerShell (Run as Administrator)** and list your USBs:
   ```powershell
   usbipd wsl list
   ```
4. Attach the Camera and the Robot controller:
   ```powershell
   usbipd wsl attach --busid <CAMERA_BUSID>
   usbipd wsl attach --busid <ROBOT_BUSID>
   ```

---

## 2. Launch the Camera Feed

Open **WSL Terminal 1** and start the USB camera. 
*Note: We force `mjpeg2rgb` to prevent WSL from crashing due to USB bandwidth issues, and we load your calibration YAML so YOLO gets perfect un-warped images.*

```bash
source activate.sh
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/camera/rgb/image_raw \
  -p pixel_format:=mjpeg2rgb \
  -p camera_info_url:=file:///mnt/c/Users/Darko/OneDrive/Documents/prarob_gui_2/resources/calibration.yaml
```

---

## 3. Launch the YOLO Vision AI

Open **WSL Terminal 2** and start the YOLO detection network. 
*Note: Tracking is disabled to prevent missing library crashes.*

```bash
source activate.sh
run_yolo
```

---


## 4. Launch the LLM Brain (ROSA)

Open **WSL Terminal 4** and start the autonomous agent that translates English into robot movements.

```bash
source activate.sh
ros2 run prarob_interact nl_agent
```

---

## 5. Launch the Control GUI

Open **WSL Terminal 5** and start the Python GUI.

```bash
source activate.sh
python3 gui.py
```

---

## 6. Execution

1. Go to the **Autonomous Mode** tab in your GUI.
2. Type your command: `"Connect the traffic light to the ball"` (or any other COCO dataset objects you have placed on the table).
3. The LLM will parse the text, ask YOLO for the coordinates, calculate the Inverse Kinematics, and send the trajectory to the robot arm!

---

### ⚠️ Final Important Note on Calibration (Homography)
While `calibration.yaml` fixes the camera's lens distortion, the agent still needs to know exactly where the camera is physically mounted relative to the robot's base coordinate system to calculate exact millimeter distances. 

Currently, `nl_agent_node.py` uses a temporary math fallback to convert pixels to millimeters. For the physical robot to perfectly trace the objects, you and your teammates must replace that fallback in `image_to_base()` with a **4-Point Homography Matrix** that maps the 2D camera view to the physical 3D workspace.
