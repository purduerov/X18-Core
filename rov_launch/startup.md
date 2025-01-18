# Automated Startup 

### Purpose
On pi startup the script will automatically run build.sh and run.sh to launch ros2 nodes.

### File Path to Script
/etc/systemd/system/myscript.service

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