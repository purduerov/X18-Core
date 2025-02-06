# Camera Streams

Our camera streams are designed to be as simple as possible to get up and running.

### How it works

There are two main nodes for this package:

1. `IpSubscriber` - This node is responsible for subscribing to the IP address of the the surface station, and when a valid IP address is received, it will start the camera nodes with the provided IP address.
2. `Camera` - This nodes is responsible for creating a ffmpeg stream from the camera, and publishing it to the surface station.