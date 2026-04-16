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
ip = 'localhost'#"192.168.1.51"
rtsp_url = f"rtsp://{ip}:8554/stereo_camera"  # replace with your MediaMTX server

# Create DepthAI pipeline

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xout_depth = pipeline.create(dai.node.XLinkOut)

xout_depth.setStreamName("depth")

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# Shadow & Noise Mitigation (DepthAI 2.31.1 Syntax)
stereo.initialConfig.PostProcessing().spatialFilter.enable = True
stereo.initialConfig.PostProcessing().spatialFilter.holeFillingRadius = 5
stereo.initialConfig.PostProcessing().spatialFilter.numIterations = 1

stereo.initialConfig.PostProcessing().temporalFilter.enable = True
stereo.initialConfig.PostProcessing().temporalFilter.persistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4

# 2. These methods usually work fine as direct setters on the config object
stereo.initialConfig.setConfidenceThreshold(235)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
# Stereo Settings
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True) # Crucial for accuracy on large objects
stereo.setExtendedDisparity(True)
# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xout_depth.input)


video = pipeline.create(dai.node.VideoEncoder)
video.setDefaultProfilePreset(
    30, dai.VideoEncoderProperties.Profile.H264_MAIN
)

stereo.video.link(video.input)

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
