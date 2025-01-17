#!/usr/bin/python3
import cv2 as cv
import numpy as np
import onnxruntime as ort
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Classificator class
class modelPredict:
    def __init__(self, model:str, class_list:list, conf_thres:float, cuda:bool) -> None:
        self.__model = model
        self.__class_list = class_list
        self.__colors = np.random.uniform(0, 255, size=(len(self.__class_list), 3))
        self.__conf = conf_thres
        self.__buildModel(cuda)
        self.__y = None

    def __buildModel(self, is_cuda:bool) -> None:
        if is_cuda:
            print("Attempting to use CUDA")
            self.__session = ort.InferenceSession(self.__model, providers=['CUDAExecutionProvider'])
        # print("Intentando")
        else:
            print("Running on CPU")
            self.__session = ort.InferenceSession(self.__model, providers=['CPUExecutionProvider'])
        shape = self.__session.get_inputs()[0].shape
        self.__inputWidth, self.__inputHeight = shape[2:4]

    def __formatImg(self, img:np.ndarray) -> np.ndarray:
        image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        image = np.array(cv.resize(image, [self.__inputWidth, self.__inputHeight])) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return np.expand_dims(image, axis=0).astype(np.float32)

    def __detect(self, img:np.ndarray) -> np.ndarray:
        inputs = {self.__session.get_inputs()[0].name: img}
        preds = self.__session.run(None, inputs)
        return np.squeeze(preds[0])

    def __wrapDetection(self, modelOutput:np.ndarray, object:str) -> tuple:
        class_ids, boxes, scores = [], [], []

        x_factor = self.__imgWidth / self.__inputWidth
        y_factor = self.__imgHeight / self.__inputHeight

        rows = modelOutput.shape[0]
        for r in range(rows):
            row = modelOutput[r]

            if row[4] > self.__conf:
                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)
                max_score = classes_scores[class_id]

                if (max_score > self.__conf) and (self.__class_list[class_id] == object):
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()

                    left = (x - 0.5 * w) * x_factor
                    top = (y - 0.5 * h) * y_factor
                    width, height = w*x_factor, h*y_factor

                    class_ids.append(class_id)
                    scores.append(max_score)
                    boxes.append(np.array([left, top, width, height]))

        indices = cv.dnn.NMSBoxes(boxes, scores, self.__conf, 0.5)

        result_class_ids, result_boxes, result_scores = [], [], []
        for i in indices:
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])
            result_scores.append(scores[i])
        return result_class_ids, result_boxes, result_scores

    def _startDetection(self, img:np.ndarray, object:str, width:float) -> None:
        self.__imgHeight, self.__imgWidth = img.shape[:2]
        formatImg = self.__formatImg(img)
        outs = self.__detect(formatImg)
        class_ids, boxes, scores = self.__wrapDetection(outs, object)

        for (classid, box, score) in zip(class_ids, boxes, scores):
            x, y, w, h = box
            color = self.__colors[classid]
            cv.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
            cv.rectangle(img, (int(x), int(y) - 15), (int(x) + int(w), int(y) + 15), color, -1)
            cv.putText(img, f"{object}: {score:.2f}", (int(x), int(y) + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1, cv.LINE_AA)
            print("Encontre algo")

            PixToMeters = width / w
            self.__y = PixToMeters*(x + (w - self.__imgWidth)/2)
        cv.imshow('Classificator', img)
        cv.waitKey(1)

    def getY(self) -> float:
        return self.__y
    
    def image_callback(msg):
        try:

            # # Convert ROS image message to OpenCV image
            # print("entra")
            # # Obtén los datos de la imagen en forma de lista de bytes
            # np_arr = np.fromstring(msg.data, np.uint8)
            # print("despues DEL np_arr")
            # # Convierte la lista de bytes a un array de numpy
            # image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)
            # print("despues DEL IMAGE_NP")

            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)



            # bridge = CvBridge()
            # image_np = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # cv.imshow("imagen", image_np)

            # cv.imwrite("/home/carlos/nueva/src/camara_rospy/scripts/Entrenamiento.png", image_np)

            # Define the object you want to detect
            target_object = 'Beans_Angular_LeafSpot'  # Replace with the class name you're interested in

            # Define the width of the object in meters
            object_width = 0.15  # Replace with the actual width of the object in meters

            # Start detection
            model._startDetection(image_np, target_object, object_width)

            # Get the Y coordinate
            y_coordinate = model.getY()
            print(f"The Y coordinate of the detected object is: {y_coordinate}")

        except Exception as e:
            print(e)

def main():
    # Initialize ROS node
    rospy.init_node('object_detection_node')

    # Load model, class list, and image
    model_path = '/home/puzzlebot/catkin_ws/src/plaga/plantas.onnx'
    class_list = ['Beans_Angular_LeafSpot', 'Beans_Rust', 'Strawberry_Angular_LeafSpot', 'Strawberry_Anthracnose_Fruit_Rot', 'Strawberry_Blossom_Blight', 'Strawberry_Gray_Mold', 'Straberry_Leaf_Spot', 'Straberry_Powdery_Mildew_Fruit', 'Straberry_Powdery_Mildew_Leaf', 'Tomato_Blight', 'Tomato_Leaf_Mold','Tomato_Spider_Mites']  # Replace with your actual class names
    conf_thres = 0.3
    cuda = True  # Set to True if you want to use CUDA

    global model
    model = modelPredict(model_path, class_list, conf_thres, cuda)

    # Define the camera topic you want to subscribe to
    #camera_topic = "/usb_camera/image_raw"  # Replace with the actual topic name

    # Create a subscriber for the camera topic
    rospy.Subscriber("/img", CompressedImage, modelPredict.image_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
