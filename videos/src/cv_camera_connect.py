#!/home/pi/depthai-env/bin/python

import depthai as dai
import threading
import signal
import subprocess
import time

PROFILE = dai.VideoEncoderProperties.Profile.H264_MAIN
quitEvent = threading.Event()

signal.signal(signal.SIGTERM, lambda *_args: quitEvent.set())
signal.signal(signal.SIGINT, lambda *_args: quitEvent.set())

# Your MediaMTX RTSP URL
ip = "192.168.1.51"
rtsp_url = f"rtsp://{ip}:8554/camera_1"  # replace with your MediaMTX server

# Create DepthAI pipeline

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
camRgb.setFps(30)

video = pipeline.create(dai.node.VideoEncoder)
video.setDefaultProfilePreset(
    30, dai.VideoEncoderProperties.Profile.H264_MAIN
)

camRgb.video.link(video.input)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("h264")
video.bitstream.link(xout.input)

device = dai.Device(pipeline)

encoded_xout = device.getOutputQueue(
    name="h264",
    maxSize=30,
    blocking=True
)


# Start FFmpeg process
ffmpeg_cmd = [
    "ffmpeg",
    "-y", ## What
    "-f", "h264",
    "-i", "pipe:0",      # input from stdin
    "-codec:v", "copy",      # copy H264 directly
    "-g", "10",
    "-f", "rtsp",
    rtsp_url
]
ffmpeg = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

print("Streaming to MediaMTX server. Press Ctrl+C to stop.")
while not quitEvent.is_set():
    packet = encoded_xout.get() # get H264 packet
    # assert isinstance(packet, dai.ImgFrame)
    if packet is not None:
        ffmpeg.stdin.write(packet.getData())

ffmpeg.stdin.close()
ffmpeg.wait()
device.close()
