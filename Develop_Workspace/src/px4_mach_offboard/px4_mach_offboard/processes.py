
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/Repos/PX4-Autopilot && \
    PX4_HOME_LAT=37.546718              \
    PX4_HOME_LON=127.119070             \
    PX4_HOME_ALT=0                      \
    make px4_sitl gazebo-classic_standard_vtol",

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
