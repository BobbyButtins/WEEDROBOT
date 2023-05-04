import jetson.inference
import jetson.utils

# Set detection threshold (default = 0.5 (50%), can increase to detect less objects or decrease to detect more)
net = jetson.inference.detectNet(argv=['--model=/home/weedrobot/jetson-inference/python/training/detection/ssd/models/weeds/ssd-mobilenet.onnx', \
                                       '--labels=/home/weedrobot/jetson-inference/python/training/detection/ssd/models/weeds/labels.txt', \
                                       '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.5)
# Create camera object (can use "v4l2-ctl --list-devices" in terminal to find cameras)
camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
# Create display window
display = jetson.utils.glDisplay()

# Continuously loop display window
while display.IsOpen():
    img, width, height = camera.CaptureRGBA()
    detections = net.Detect(img, width, height)
    display.RenderOnce(img, width, height)
    display.SetTitle("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
