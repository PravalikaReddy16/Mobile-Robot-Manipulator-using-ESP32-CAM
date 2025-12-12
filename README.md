# Mobile Robot Manipulator using ESP32-CAM 



Overview
- Project: Mobile differential-drive robot with a 4‑DOF servo manipulator using an ESP32-CAM for video and visual localization.
- Main components:
  - Firmware: main (Arduino sketch) — runs the camera, MJPEG stream, WebSocket teleop server, motor and servo drivers, and serves a web UI.
  - Map builder: map_builder (Python) — captures keyframes and ORB descriptors from the camera stream and saves a simple visual map.
  - Localizer: locailser (Python) — matches live frames to the saved map to determine the best-matching location (visual relocalization).
  - Teleoperation client: teleop (Python) — optional keyboard-based teleop that connects over WebSocket and sends drive/servo commands.

This README documents the design, dependencies, installation and run procedures, hardware wiring and BOM notes, algorithmic details, calibration and tuning, troubleshooting, and many suggested future developments you can pursue.

Repository file notes (file names used in this repo)
- main — Arduino sketch (use main.ino when opening in Arduino IDE or main.cpp for PlatformIO).
- map_builder — Python script (map_builder.py).
- locailser — Python script (locailser.py). NOTE: the filename retains the project's current spelling.
- teleop — Python script (teleop.py).
- requirements.txt — Python package list.
- BOM.md — Bill of Materials and wiring notes (if present).

High-level architecture
- ESP32-CAM (AI-Thinker recommended) runs:
  - Camera capture (JPEG frames).
  - A lightweight MJPEG stream endpoint (recommended: port 81, path /stream) for low-latency video consumption by both the embedded UI and Python clients.
  - An AsyncWebServer on port 80 that serves the web UI and a WebSocket endpoint at /ws for real-time control messages.
  - Motor control logic (LEDC PWM + IN pins to an H-bridge).
  - Servo control for a 4‑DOF manipulator (ESP32Servo or similar).
- Python tools use OpenCV (ORB) to capture keyframes and to localize live frames against saved descriptors:
  - map_builder: captures frames from MJPEG stream and saves images + ORB keypoints/descriptors (as .pkl or similar).
  - locailser: reads the map and compares live-frame ORB descriptors to map descriptors using a matcher (BFMatcher/Hamming for ORB), then displays best match and matches overlay.
- Control messages:
  - JSON over WebSocket, examples:
    - Drive: {"type":"drive","left":<int -255..255>,"right":<int -255..255>}
    - Servo: {"type":"servo","id":<0..3>,"angle":<0..180>}
  - The embedded UI sends these messages; the ESP32 applies motor/servo commands and may broadcast state to connected clients.

Hardware & BOM (summary)
- ESP32-CAM (AI-Thinker)
- FTDI USB-to-Serial adapter (5V) for flashing
- 2 × DC motors and wheels (differential drive)
- Motor driver (e.g., L298N, TB6612FNG, or more modern current-capable driver)
- 4 × servos for manipulator (choose torque-rated servos for heavier arms)
- 5V power supply able to source peak servo/motor current (common ground required)
- Optional: encoders, IMU (MPU6050/9250), motor current sensors, voltage monitor
- Wires, connectors, chassis, servo mounts, gripper, etc.

Suggested pin mapping (match these to main before flashing)
- Camera: AI-Thinker default pins (configured in main).
- Motors:
  - LEFT_PWM: GPIO 12 (LEDC channel 0)
  - LEFT_IN1: GPIO 14
  - LEFT_IN2: GPIO 27
  - RIGHT_PWM: GPIO 13 (LEDC channel 1)
  - RIGHT_IN1: GPIO 25
  - RIGHT_IN2: GPIO 26
- Servos (example): SERVO0=GPIO2, SERVO1=GPIO15, SERVO2=GPIO4, SERVO3=GPIO16
- Adjust pins if your hardware layout differs. Document any changes.

Software dependencies
- Arduino firmware (main):
  - ESP32 board support installed in Arduino IDE (or PlatformIO).
  - Libraries:
    - ESP32Servo (or equivalent)
    - AsyncTCP
    - ESPAsyncWebServer
    - ArduinoJson
  - Compiler: Arduino IDE (with ESP32 support) or PlatformIO.
- Python (map_builder, locailser, teleop):
  - requirements.txt (suggested):
    - opencv-python >= 4.5
    - numpy
    - websocket-client
    - imutils
  - Install: pip install -r requirements.txt

Quick start — flash and test
1. Edit main:
   - Set your Wi‑Fi SSID and password in the top constants.
   - Verify the pin mappings match your wiring.
2. Install required Arduino libraries and ESP32 board support.
3. Connect FTDI to ESP32-CAM (use 5V Vcc). If using AI-Thinker, ensure IO0 is set correctly when flashing.
4. Compile and upload main.ino to the ESP32-CAM.
5. Open Serial Monitor at 115200 to see output and IP address.
6. Open a browser to http://<ESP_IP>/ — you should see the embedded web UI; the video element will point to /stream (port 81).
7. Test drive controls (start at low speed) and servo sliders. Use safe, unloaded servo mounting when you first test.

Using the Python tools
- Capture a map:
  - python map_builder.py --stream http://<ESP_IP>:81/stream --out mapdir --interval 2.5
  - Press 'c' to capture keyframes manually; the script can also auto-capture at the specified interval.
  - Keyframes saved: image files (kf_0000.jpg, ...) and descriptor metadata files (kf_0000.pkl).
- Localize:
  - python locailser.py --stream http://<ESP_IP>:81/stream --map mapdir --min_matches 12
  - The script performs ORB detection on live frames and finds the best matching keyframe from the map, displaying matches in a window and printing the best match and match count.
- Teleop client (optional):
  - python teleop.py --host <ESP_IP> --port 80
  - Keyboard controls: w/s/a/d/x for movement; 0-3 to select a servo; r/f to change selected servo angle; q to quit.

Algorithmic details — visual localization
- Feature detector/descriptor: ORB (fast, binary descriptors, uses Hamming distance).
- Matcher: Brute-Force matcher with Hamming norm and optional crossCheck.
- Matching heuristic:
  - Sort matches by distance; apply a threshold (e.g., distance < 60) or ratio test.
  - Choose the keyframe with the highest number of “good” matches.
  - Optionally compute homography and check inliers to further verify the match.
- Limitations:
  - ORB provides robust matching for many scenes but is not a complete SLAM system.
  - Environmental changes (lighting, dynamic objects) reduce matching reliability.
  - If you need metric pose (x,y,theta), you must add extra sensors (odometry/IMU) or use markers (AprilTags) or a calibrated multi-view pipeline with known geometry.

Data formats
- Keyframes: JPEG images + pickled metadata for keypoints (x,y) and descriptor arrays.
- Map folder structure: simple flat directory with matching pairs (kf_XXXX.jpg + kf_XXXX.pkl).
- You can convert the descriptors into a more compact/portable format (e.g., numpy .npz) if desired.

Calibration & tuning
- Camera:
  - Frame size: FRAMESIZE_VGA or lower for higher framerate / lower latency.
  - jpeg_quality: lower number = higher quality (increase quality if you need better matches).
- ORB:
  - nfeatures: increase if the scene is rich; decrease if CPU-bound.
  - Match thresholds: tune distance threshold and min_matches in locailser.
- Motors:
  - Tune PWM frequency and resolution (LEDC_FREQ, LEDC_RESOLUTION).
  - Adjust motor duty mapping so that PWM values map to a safe, repeatable speed.
- Servos:
  - Set safe angle limits before mechanical assembly (constrain ranges to prevent binding).
  - Calibrate servo zero positions and function of each joint.

Safety guidelines
- Never power motors and servos from the ESP32 5V pin for anything beyond low-power experiments. Use a dedicated supply sized for stall currents.
- Keep hands and loose wiring clear of actuators during testing.
- Test servo sweeps at low speed and with no attached payload initially.

Troubleshooting
- No camera stream:
  - Check camera pin configuration matches your module (AI‑Thinker vs other variants).
  - Inspect power supply and ensure stable 5V during camera operation.
  - On failure to initialize camera, Serial Monitor will print an error code.
- WebSocket disconnects:
  - Check WiFi latency and signal strength.
  - If multiple clients connect, ensure the ESP has sufficient memory to manage them.
- Poor or inconsistent localization:
  - Collect richer keyframes (cover the environment more densely).
  - Improve lighting consistency, or use artificial lighting.
  - Increase ORB nfeatures, or switch to more descriptive features (SIFT/SURF or deep descriptors) if legal and feasible.
- Motor driver issues:
  - Verify IN pin logic for forward/reverse with small test scripts.
  - Confirm PWM pins are using LEDC channels and that frequency/resolution are appropriate for your driver.

Extensive list of possible developments / enhancements
(These are potential next steps you can choose to implement; each bullet can be elaborated into tasks and issues.)

1. Improved mapping & SLAM
   - Integrate ORB-SLAM2/3 (requires more compute or an offboard computer).
   - Use visual-inertial SLAM (add IMU) for more robust odometry.
   - Add loop closure and pose graph optimization.

2. Absolute localization
   - Add AprilTags or ArUco markers in the environment for absolute pose estimation (PnP).
   - Use fiducials to derive metric transforms and calibrate map scale.

3. Sensor fusion & odometry
   - Add wheel encoders and fuse visual localization + encoders + IMU using an Extended Kalman Filter (EKF).
   - Implement dead-reckoning with occasional visual corrections.

4. ROS integration
   - Provide ROS node wrappers for camera stream, teleop, map serving, and localization.
   - Expose topics: /camera/image_raw, /cmd_vel, /joint_states, /pose_estimate.

5. Manipulator control & kinematics
   - Implement forward and inverse kinematics for the 4‑DOF arm (compute joint angles for target end-effector poses).
   - Add trajectory generation, trapezoidal velocity profiling, and joint limits handling.
   - Add force/torque or tactile sensors and closed-loop grasping.

6. UI improvements
   - Extract UI files (index.html, app.js, styles.css) to SPIFFS or LittleFS so they can be edited independently.
   - Add video overlays (draw localization bounding boxes, keypoints) and a map view.
   - Add presets for manipulator poses and macro recording/Playback.

7. Security & reliability
   - Add authentication to the web UI and WebSocket (simple token or HTTP basic auth).
   - Use TLS (wss/https) if deploying on untrusted networks (requires additional overhead).
   - Add watchdogs to restart the ESP on fatal errors.

8. Performance & scaling
   - Offload heavy processing (ORB or deep descriptors) to an onboard SBC (Raspberry Pi, Jetson Nano) or to a remote PC.
   - Implement compressed binary descriptor storage for faster map loading.
   - Add quality-of-service (QoS) for command delivery (acknowledgements).

9. Vision enhancements
   - Replace ORB with deep-learning-based place recognition (NetVLAD, DELF) for robust long-term localization.
   - Add semantic segmentation to avoid dynamic objects and focus features on static structures.

10. Mechanical & power improvements
    - Design a rigid manipulator mount and proper cable routing.
    - Use a dedicated power distribution board with voltage regulation and current sensing.
    - Add a power switch, battery state monitoring, and safe shutdown behavior.

11. Testing & CI
    - Add unit tests for the Python components (mock stream input).
    - Add integration tests with a simulated camera stream (use prerecorded streams).
    - Add GitHub Actions for linting and running Python tests.

12. Documentation & tutorials
    - Expand this README into a multi-page docs site with wiring diagrams, photos, and step‑by‑step videos.
    - Provide example maps and recorded sessions for quick testing.

Deployment checklist (pre-flight)
- Verify servo and motor power sources are robust and common ground is established with ESP32.
- Verify safe servo ranges and zero positions.
- Make sure the environment is adequately lit for visual feature detection.
- Start with low PWM values and test simple forward/stop/backward cycles.

Advanced notes (open problems & considerations)
- Visual relocalization is scene-dependent: moving furniture or strong lighting changes degrade matches.
- For metric pose estimation you'll need known scene geometry, marker sizes, or stereo / depth sensors.
- Real-time closed-loop navigation requires reliable pose updates and a motion planner; this project focuses on teleoperation + visual relocalization, not full autonomous navigation.

Example commands summary
- Flash (Arduino IDE / PlatformIO): open main.ino, set Wi-Fi, compile & upload.
- Build map:
  - pip install -r requirements.txt
  - python map_builder.py --stream http://192.168.1.50:81/stream --out mapdir --interval 2.5
- Run localizer:
  - python locailser.py --stream http://192.168.1.50:81/stream --map mapdir --min_matches 12
- Teleop from terminal:
  - python teleop.py --host 192.168.1.50 --port 80

Troubleshooting quick tips
- If video is jittery or not loading: reduce camera resolution or increase JPEG compression (lower quality number).
- If Python can't open stream: ensure the stream URL includes port 81 (http://<ip>:81/stream) and that no firewall is blocking.
- If web UI shows "WebSocket disconnected": check that main is running and the /ws endpoint is active; check Serial logs for memory errors.

Licensing and credits
- Core code described here is provided without warranty. Use/adapt as you like.
- Third-party libraries:
  - ESPAsyncWebServer, AsyncTCP, ESP32Servo, ArduinoJson — check their respective licenses for compliance.
  - OpenCV (BSD) — check OpenCV license details.


