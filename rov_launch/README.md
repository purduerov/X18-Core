# Automated Startup 

### Purpose
On Pi startup the script will automatically run `build.sh` and `run.sh` from the scripts directory in X17-Core repo on the pi to launch the ROS2 nodes. We put it in the system dir in systemd directory. The scripts in this directory run when the Pi starts. We include a sleep time to let some of the needed processes to start and then run the 2 scripts on boot up. We send the output to a file in the X17-Core directory. The following process can be stopped using `scripts/kill-switch.sh`. 

### File Path to Script
`/etc/systemd/system/myscript.service`

### Script

```
[Unit]
Description=My Startup Script
After=network.target syslog.target remote-fs.target

[Service]
Type=simple
ExecStartPre=/bin/sleep 15
ExecStart=/bin/bash -c "v4l2-ctl --list-devices | tee /home/pi/ros2_ws/src/X17-Core/log/testing_automation.txt > /dev/null && /home/p>Restart=on-failure
User=pi
WorkingDirectory=/home/pi/ros2_ws/src/X17-Core

[Install]
WantedBy=multi-user.target
```

> **Warning:** The code might be different on the Pi due to updates