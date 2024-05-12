
import cv2
import sys
import os


deviceID = 0
width = 960
height = 540

cap = cv2.VideoCapture(deviceID)
if not cap.isOpened():
    print("Error: Failed to open camera")
    sys.exit(0)


frameWidth  = width
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)
print("Set Frame Width: {}".format(frameWidth))

frameHeight  = height
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)
print("Set Frame Height: {}".format(frameHeight))

fps = 30
cap.set(cv2.CAP_PROP_FPS, fps)
print("Set Frame Rate: {}".format(fps))

print("Current camera properties:")
print("Device ID: {}".format(deviceID))

frameWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
print("Frame Width: {}".format(frameWidth))

frameHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("Frame Height: {}".format(frameHeight))

frameRate = cap.get(cv2.CAP_PROP_FPS)
print("Frame Rate: {}".format(frameRate))

folder = "snapshots/"
try:
    os.mkdir(folder)
except OSError as error:
    print(error)


img_count = 1

while cap.isOpened():

    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to receive frame")
        break

    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=0.5, tileGridSize=(8, 8))
    clahe_image = clahe.apply(gray_image)

    k = cv2.waitKey(1)

    if k == ord('q'):
        break
    elif  k == ord('s'):
        cv2.imwrite(folder + "img_" + str(img_count) + ".png", clahe_image)
        print("Image {} saved!".format(img_count))
        img_count += 1

    cv2.imshow("Frame capture", clahe_image)

cap.release()
cv2.destroyAllWindows()