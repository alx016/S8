#!/usr/bin/python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
import numpy as np

import sys
sys.path.append("/home/S8/agrobot/src/imgDetection/scripts")
from modelPredict import modelPredict

##COMANDO PARA CAMBIAR EL COLOR rostopic pub /object/class std_msgs/String "COLOR(Verde,Roja,Naranja)"


class objectClassificator:
    def __init__(self, model:str, classes:list, conf_threshold:float, cuda:bool) -> None:
        # Instance of the class modelPredict
        self.__model = modelPredict(model, classes, conf_threshold, cuda)

        # Initialize the variables
        self.__img = None
        self.__object = None # Desired Class name of the object
        self.__grabbed = False
        self.__objWidth, self.__objHeight = 0.05, 0.15 # Object dimensions (m)
        self.__carry = None
        self.__cont = 0
        self.__coordarm = {"x":0.01, "y":0.0, "z":self.__objHeight}
        self.__tol = 10
        self.__errorT = 0.02
        self.__lastY = 0.0
        self.__kp = 0.5
        self.__lastZ = 0.0
        self.__on = True

        # Compressed image message
        self.__compressedImg = CompressedImage()
        self.__compressedImg.format = "jpeg"

        # Initialize the subscribers and publishers
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.__imageCallback) # Get the image from the camera
        rospy.Subscriber("/object/class", String, self.__classCallback) # Get the class of the object
        rospy.Subscriber("/object/drop", Bool, self.__drCallback)
        rospy.Subscriber("/carryPos", Bool, self.__carryPosCallback)


        self.__back_pub = rospy.Publisher("/key", Bool, queue_size= 2) #Publish if the arm can go back to the init position 
        self.__coord_pub = rospy.Publisher("/object/coords", Point, queue_size = 1) # Publish the coordinates of the object (m)
        self.__grab_pub = rospy.Publisher("/object/grab", Bool, queue_size = 1) # Publish if the object is ready to be grabbed
        self.__detection_pub = rospy.Publisher("/usb_cam/model_prediction/compressed", CompressedImage, queue_size = 10) # Publish the image with the prediction
        print("entre al init")
        self.__back_pub.publish(True)
                    

    def __imageCallback(self, msg:CompressedImage) -> None:
        try:
            self.__img = msg.data
        except Exception as e:
            rospy.loginfo(e)

    # Callback function for the class name
    def __classCallback(self, msg:String) -> None:
        self.__object = msg.data

    def __drCallback(self, msg:Bool) -> None:
        if msg.data:
            self.__grab = False
        else:
            self.__grab
        self.__coordarm["y"] = 0.0

    # Callback funtion for the image
    def __carryPosCallback(self, msg:Bool) -> None:
        self.__carry = msg.data

    # Start the model classification
    def _startModel(self) -> None:
        if self.__img is not None and self.__on is not False:
            decod_img=self.__model._startDetection(self.__img, self.__object, self.__objWidth) # Detect on current frame}
            y, depth = self.__model.getY(), self.__model.getdepth()
            
            #print(self.__object)

            if y is not None:
                self.__sendCoord(y,depth)
            # Publish the compressed image
            self.__compressedImg.data = decod_img
            self.__detection_pub.publish(self.__compressedImg)
            
        


    def __sendCoord(self, y:float, depth:float) -> None:
        if(-0.001 < (self.__lastY- y) < 0.001 or not self.__tol):
            self.__tol += 1
        else:
            self.__tol -= 1 

        self.__lastY = y
        self.__lastZ = depth
        print(self.__cont)
        if(self.__tol >= 1 and self.__cont==0 and self.__lastZ < 0.32):
            print("entre")
            self.__cont = self.__cont + 1
            self.__back_pub.publish(False)



        if (self.__tol > 3):
           print("entre a la segunda")
           #if (self.__lastY  > 0.2):
           y_rec = np.tanh(y) * 1.6
           self.__coordarm["y"] = y_rec 
           self.__tol = 0
           #self.__coord_pub.publish(self.__coordarm["x"],self.__coordarm["y"],self.__coordarm["z"]-(0.10))
          # else:
           x =  (depth**2 - self.__coordarm["z"]**2)**0.5

           rospy.sleep(1.0)

           self.__coord_pub.publish(self.__coordarm["x"]+(x+0.0175), self.__coordarm["y"], self.__coordarm["z"]-(0.15))
           print(self.__carry)
           #if self.__carry is True:
               #self.__grab = True
               #self.__object = None
               #self.__on = False
               
               #self.__back_pub.publish(True)
               

    # Stop Condition
    def _stop(self) -> None:
        print("Stopping classificator node")



if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Classificator")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rate", default = 10))
    
    # Get the parameters
    #model = rospy.get_param("model/path", default = "./Model/bestV5-25e.onnx")
    #class_list = rospy.get_param("classes/list", default = ["Fanta", "Pepsi", "Seven"])
    model = rospy.get_param("model/path", default = "./plantas.onnx")
    class_list = rospy.get_param("classes/list", default = ['Beans_Angular_LeafSpot', 'Beans_Rust', 'Strawberry_Angular_LeafSpot', 'Strawberry_Anthracnose_Fruit_Rot', 'Strawberry_Blossom_Blight', 'Strawberry_Gray_Mold', 'Straberry_Leaf_Spot', 'Straberry_Powdery_Mildew_Fruit', 'Straberry_Powdery_Mildew_Leaf', 'Tomato_Blight', 'Tomato_Leaf_Mold','Tomato_Spider_Mites'])
    conf = rospy.get_param("confidence/value", default = 0.01)
    cuda = rospy.get_param("isCuda/value", default = False)
    print(conf)

    # Create the instance of the class
    classificator = objectClassificator(model, class_list, conf, cuda)

    # Shutdown hook
    rospy.on_shutdown(classificator._stop)

    # Run the node
    print("The Classificator is Running")
    while not rospy.is_shutdown():
        try:
            classificator._startModel()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
        
        rate.sleep()
