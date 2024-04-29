import cv2 as cv
import numpy as np
import onnxruntime as ort


class ModelPredict:
    def __init__(self, model:str, class_list:list, conf_thres:float, cuda:bool) -> None:
        # Initialize attributes
        self.__model = model
        self.__class_list = class_list
        self.__colors = np.random.uniform(0, 255, size=(len(self.__class_list), 3))
        self.__conf = conf_thres
        self.__build_model(cuda) # Build the model for inference

        # Y coordinate of the object
        self.__y = None

    # Define if OpenCV runs with CUDA or CPU (False = CPU, True = CUDA)
    def __build_model(self, is_cuda:bool) -> None:
        if is_cuda:
            print("Attempting to use CUDA")
            self.__session = ort.InferenceSession(self.__model, providers=['CUDAExecutionProvider'])
        else:
            print("Running on CPU")
            self.__session = ort.InferenceSession(self.__model, providers=['CPUExecutionProvider'])
        # Get the input image shape for the model (width, height)
        shape = self.__session.get_inputs()[0].shape
        self.__inputWidth, self.__inputHeight = shape[2:4]

    # Format image to be used by the model
    def __format_img(self, img:np.ndarray) -> np.ndarray:
        image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        image = np.array(cv.resize(image, (self.__inputWidth, self.__inputHeight))) / 255.0 # Resize (input shape) and normalize (0-1)
        image = np.transpose(image, (2, 0, 1)) # Transpose to (channels, width, height)
        return np.expand_dims(image, axis=0).astype(np.float32) # Add batch dimension to create tensor (b, c, w, h)

    # Detect objects in the image
    def __detect(self, img:np.ndarray) -> np.ndarray:
        inputs = {self.__session.get_inputs()[0].name: img} # Get the input name of the model
        preds = self.__session.run(None, inputs) # Perform inference
        return np.squeeze(preds[0]) # Remove batch dimension 

    # Wrap the detection processing
    def __wrap_detection(self, model_output:np.ndarray, object:str) -> tuple:
        # Initialize lists
        class_ids, boxes, scores = [], [], []

        # Iterate over the model output 
        rows = model_output.shape[0]
        for r in range(rows):
            row = model_output[r]
            
            # Check if the object confidence is greater than the threshold
            if row[4] > self.__conf:
                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)
                max_score = classes_scores[class_id]
                
                # Check if the score is greater than the threshold and if the detected object is the desired one
                if (max_score > self.__conf) and (self.__class_list[class_id] == object):
                    x, y, w, h = row[:4] # Get the bounding box coordinates

                    # Append the results to the lists
                    class_ids.append(class_id)
                    scores.append(max_score)
                    boxes.append(np.array([int(x), int(y), int(w), int(h)]))

        # Apply non-maximum suppression to suppress overlapping boxes
        indices = cv.dnn.NMSBoxes(boxes, scores, self.__conf, 0.5)

        # Get the final results
        result_class_ids, result_boxes, result_scores = [], [], []
        for i in indices:
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])
            result_scores.append(scores[i])
        return result_class_ids, result_boxes, result_scores

    # Start the detection process
    def start_detection(self) -> None:
        MA = 0
        MD = 0
        BA = 0
        BD = 0
        cap = cv.VideoCapture(0) # Open the camera
        while cap.isOpened():
            ret, frame = cap.read() # Capture frame-by-frame
            if ret:
                img = cv.flip(frame, 1)  # Flip the image vertically
                self.__img_height, self.__img_width = img.shape[:2]
                
                # Perform detection for each object in class_list
                for object in self.__class_list:
                    format_img = self.__format_img(img)
                    outs = self.__detect(format_img)
                    class_ids, boxes, scores = self.__wrap_detection(outs, object)
                    
                    # Draw bounding boxes and labels for detected objects
                    if class_ids:
                        index = np.argmax(scores)
                        x, y, w, h = boxes[index]
                        color = self.__colors[class_ids[index]]

                        # Adjust the bounding box coordinates for better visualization
                        # For example, you can center the box around the detected object
                        x1 = max(0, int(x - w / 2))
                        y1 = max(0, int(y - h / 2))
                        x2 = min(self.__img_width, int(x + w / 2))
                        y2 = min(self.__img_height, int(y + h / 30))

                        # Draw the adjusted bounding box and label
                        cv.rectangle(img, (x1, y1), (x2, y2), color, 2)
                        cv.putText(img, f"{object}: {scores[index]:.3f}", (x1, y1 - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv.LINE_AA)
                        print(f"{object} detectado")
                        if(object == 'MA'):
                            MA +=1
                        elif(object == 'MD'):
                            MD += 1
                        elif(object == 'BA'):
                            BA += 1
                        elif(object == 'BD'):
                            BD += 1
                    else:
                        print(f"No se detectó {object}")

                cv.imshow('Object Detection', img) # Display the resulting frame
                if cv.waitKey(1) & 0xFF == ord('q'): # Press 'q' to exit
                    print(MA,MD,BA,BD)
                    break
            else:
                break

        cap.release() # Release the capture
        cv.destroyAllWindows() # Close all OpenCV windows
        


if __name__ == "__main__":
    # Define the paths and parameters
    model_path = "/home/alex/Documents/S8/project1_fuegoAzteca/best5.onnx"
    class_list = ['BA', 'BD', 'MA', 'MD']
    confidence_threshold = 0.3
    use_cuda = False

    # Create an instance of ModelPredict
    model_predictor = ModelPredict(model_path, class_list, confidence_threshold, use_cuda)
    # Start object detection
    model_predictor.start_detection()
    
