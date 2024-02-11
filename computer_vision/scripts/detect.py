import rospy
import cv2
import numpy as np
import os
from computer_vision.msg import BoundingBox, BoundingBoxes
from ultralytics import YOLO
from ultralytics.utils import ops
from ultralytics.utils.torch_utils import select_device
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from ultralytics.utils.plotting import Annotator
from ultralytics.data.augment import LetterBox
from ultralytics.engine.predictor import BasePredictor
from pprint import pprint

names = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET",
]


base_path = (
    "{HOME}/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/".format(
        **os.environ
    )
)

# Create a dictionary with the selected values using them as values and numbers as keys
selected_values = {}
silhouettes_values = {}
for i, name in enumerate(names):
    selected_values[name] = i

pprint(selected_values)


class Detector:
    def __init__(self, robot_name="ur"):
        pprint(f"[DETECTOR] Initializing detector")
        # super().__init__(robot_name=robot_name)
        self.robot_name = robot_name
        self.real_robot = False
        self.bridge = CvBridge()
        self.weights = base_path + "weight/best.pt"
        self.model = YOLO(self.weights)
        self.image_sub = rospy.Subscriber(
            "/ur5/zed_node/left_raw/image_raw_color", Image, self.callback
        )
        self.prediction_pub = rospy.Publisher(
            "/computer_vision/bounding_box", BoundingBoxes, queue_size=10
        )
        self.permission_sub = rospy.Subscriber(
            "/computer_vision/permission", Bool, self.permission_callback
        )
        self.cv_image = None
        self.zed_camera_image_shape = (540, 960)

        self.device = select_device("0")

        self.letterbox = LetterBox(
            new_shape=self.zed_camera_image_shape,
            stride=self.model.model.stride,
        )

        self.allowed_to_watch = False
        self.results = None
        self.conf_threshold = 0.7
        self.iou_threshold = 0.4
        self.show_image = True
        self.boxes = {}  # Dictionary with the bounding boxes
        self.objects = {}  # List with all the objects detected

        for i in range(len(names)):
            self.objects[i] = []

    def preprocess(self, image):
        # Resize image
        orig_img = image.copy()
        image = self.letterbox(image=image)
        return image, orig_img

    def show_bounding_boxes(self):
        zed_Annotator = Annotator(self.cv_image, line_width=1)
        for key, box in self.boxes.items():
            zed_Annotator.box_label(
                box=[box.xmin, box.ymin, box.xmax, box.ymax],
                label=f"Class: {box.Class} Conf: {box.probability} ",
                color=(0, 0, 255),
            )
        img = zed_Annotator.result()
        cv2.imshow("Result Streched", img)
        cv2.waitKey(3)

    def permission_callback(self, data):
        pprint(f"[DETECTOR] Permission callback Data: {data.data}")
        self.allowed_to_watch = data.data

    def publish_bounding_boxes(self):
        # Create the message
        msg = BoundingBoxes()
        for _key, box in self.boxes.items():
            msg.boxes.append(box)
        # for key, box in self.silhouette.items():
        #     msg.silhouttes.append(box)
        self.prediction_pub.publish(msg)
        # clean boxes
        self.boxes = {}

    def save_image(self, image):
        # rescale to 1280x720
        # image = cv2.resize(image, (1280, 720))
        cv2.imwrite(f"../data_generation/Background2.png", image)

    def callback(self, data):
        # self.allowed_to_watch = True
        if self.allowed_to_watch:
            print("[YOLO] Entered in Callback")
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.proc_image, self.cv_image = self.preprocess(self.cv_image)
            except CvBridgeError as e:
                print(e)

            # Detect objects
            preds = self.model(self.cv_image)

            for pred in preds:
                to_cpu = pred.cpu().numpy()
                for i, conf in enumerate(to_cpu.boxes.conf):
                    if (
                        conf > self.conf_threshold
                        and to_cpu.names[i] in selected_values
                    ):

                        self.objects[selected_values[to_cpu.names[i]]].append(
                            to_cpu.boxes[i]
                        )

            # Rescale bounding boxes
            for key, objs in self.objects.items():
                # Draw bounding boxes
                for obj in objs:
                    bbox = BoundingBox()
                    if self.real_robot:
                        new_coords = ops.scale_boxes(
                            self.cv_image.shape[:2], obj.xyxy, self.proc_image.shape[:2]
                        )[0]
                        bbox.xmin = new_coords[0].astype(int) + 5
                        bbox.ymin = new_coords[1].astype(int) + 5
                        bbox.xmax = new_coords[2].astype(int) - 5
                        bbox.ymax = new_coords[3].astype(int) - 5

                    else:
                        bbox.xmin = obj.xyxy[0][0].astype(int) + 5
                        bbox.ymin = obj.xyxy[0][1].astype(int) + 5
                        bbox.xmax = obj.xyxy[0][2].astype(int) - 5
                        bbox.ymax = obj.xyxy[0][3].astype(int) - 5

                    bbox.Class = to_cpu.names[obj.cls[0]]
                    bbox.class_n = int(obj.cls[0])
                    bbox.probability = obj.conf
                    self.boxes[key] = bbox

            self.show_bounding_boxes()
            self.publish_bounding_boxes()
            self.boxes = {}
            for key, value in self.objects.items():
                self.objects[key] = []
        else:
            print("[YOLO] Not allowed to watch")
            return


if __name__ == "__main__":
    rospy.init_node("detector", anonymous=True)
    detector = Detector("ur")
    rospy.spin()
