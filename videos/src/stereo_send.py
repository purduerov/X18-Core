import cv2
import depthai as dai
import numpy as np
from pathlib import Path

DATA_FILE = 'img.pkl'

pipeline = dai.Pipeline()

# Nodes
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

# Globals for mouse
mouse_x, mouse_y = 320, 200 # Default to center

length = -1

def get_cm(y_pos, cy, fy, z):
    return (y_pos - cy) * z / fy

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

bottom = -1
top = -1

pic_num = 0

with dai.Device(pipeline) as device:
    device.setIrLaserDotProjectorIntensity(200)
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    intrinsics = device.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 400)
    fx, fy, cx, cy = intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2]
    # print(fx, fy, cx, cy)

    # cv2.namedWindow("RGB")
    # cv2.setMouseCallback("RGB", mouse_callback)
    # cv2.namedWindow("disp", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('disp', 640, 400)
    # cv2.setMouseCallback("disp", mouse_callback)
    # Use WINDOW_NORMAL to enable resizing and moving
    cv2.namedWindow("disp", cv2.WINDOW_NORMAL)

    # Set identical sizes
    cv2.resizeWindow("disp", 1280, 800)

    cv2.moveWindow("disp", 100, 100)

    # cv2.setMouseCallback("disp", mouse_callback)
    frame_depth = q_depth.get().getFrame()

    z = frame_depth[mouse_y, mouse_x]

    if z > 0:
        x_mm = (mouse_x - cx) * z / fx
        y_mm = (mouse_y - cy) * z / fy
        label = f"X: {int(x_mm/10)} Y: {int(y_mm/10)} Z: {int(z/10)} in cm"
    else:
        label = "Z: Invalid (Too close or low texture)"
        color = (0, 0, 255) # Red for invalid
    label += f' Length is {int(length)}'
    # UI Overlay
    disp_frame = cv2.normalize(frame_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disp_frame = cv2.applyColorMap(disp_frame, cv2.COLORMAP_JET)
    center_y, center_x = disp_frame.shape[:2]
    
    cv2.putText(disp_frame, label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    # cv2.circle(disp_frame, (mouse_x, mouse_y), 5, (255, 255, 255), -1)

    if bottom != -1 and top != -1:
      bottom_coord = (center_x // 2, bottom)
      top_coord =  (center_x // 2, top)

      cv2.circle(disp_frame, center=bottom_coord, radius = 1, color = (255, 255, 255), thickness = 2)
      cv2.circle(disp_frame, center=top_coord, radius = 1, color = (255, 255, 255), thickness = 2)
    key = cv2.waitKey(1)
    if key == ord('q'):
        print(f'Number of saved folders is {pic_num}')
       # break
    elif key == ord('e'):
      name = f'Iceberg{pic_num}'
      output_dir = Path(name)
      output_dir.mkdir(parents=True, exist_ok=True)
      np.save(output_dir / 'frame_depth.npy', frame_depth)
      np.save(output_dir / 'intrinsics.npy', intrinsics)
      print('Thank you')
      pic_num += 1
    cv2.imshow("disp", disp_frame)

for i in intrinsics:
    print(i)

for i in frame_depth:
    print(i)
