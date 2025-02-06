# Camera Streams

Our camera streams are designed to be as simple as possible to get up and running.

### How it works

There are two main nodes for this package:

1. `IpSubscriber` - This node is responsible for subscribing to the IP address of the the surface station, and when a valid IP address is received, it will start the camera nodes with the provided IP address.
2. `Camera` - This nodes is responsible for creating a ffmpeg stream from the camera, and publishing it to the surface station.

### IpSubscriber

This node first creates a subscription to the `surface_ip` topic and waits for a message to be published. When a message is published, it will start the camera nodes with the provided IP address.

After receiving the IP address, the node will run the command: `v4l2-ctl --list-devices`. This will list all the camera devices and their respective paths. The output will usually look something like this, with an entry for each connected video device:

```bash
exploreHD USB Camera: exploreHD (usb-0000:01:00.0-1.1.4):
    /dev/video4
    /dev/video5
    /dev/video6
    /dev/video7
    /dev/media4
```

The various devices listed under the camera name are the different video devices that can be used to stream from the camera. They each correspond to a different format or resolution that the camera can stream in. The node currently selects the 3rd device listed for each camera, but this can be changed by modifying the `camera_device_number` parameter in the `get_ip.py` file.

The node will select the specified number of cameras. If there are more cameras than specified, the node will select the first specified number of cameras. If there are fewer cameras than specified, the node will select the available cameras.


The camera nodes will be launched with the following command:

```bash
ros2 run videos videos_launch.py --ros-args -p ip:={ip} -p device:={camera} -p camera_number:={i} -r __node:=camera{i}
```

Where `{ip}` is the IP address of the surface station, `{camera}` is the path to the camera device in the URL, and `{i}` is the camera number.

### Camera

This node is responsible for creating a ffmpeg stream from the camera, and publishing it to the surface station. The ffmpeg command is as follows:

```bash
ffmpeg -f v4l2 -fflags nobuffer -i dev_name -vcodec copy -g 10 -f rtsp rtsp_url
```

Where `dev_name` is the path to the camera device, and `rtsp_url` is the URL to publish the stream to.

To learn more about ffmpeg and its options, visit the [ffmpeg documentation](https://ffmpeg.org/ffmpeg.html).
