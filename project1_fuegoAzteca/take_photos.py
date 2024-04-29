# This code allows the user to take a certain 
# amount of fotos and save them into a folder
# Output: fotos

import cv2
import os

init_n_fotos= 840
end_n_fotos = 980
delay = 50
cap = cv2.VideoCapture(2)

output_directory = "/home/alex/Documents/S7/project2/data/juntas2"
# Verifica si la carpeta de destino existe, si no, créala
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

for i in range(init_n_fotos, end_n_fotos):
    ret, frame = cap.read()
    cv2.imshow('img', frame)
    image_filename = os.path.join(output_directory, "juntas_" + str(i) + ".jpg")
    cv2.imwrite(image_filename, frame)
    cv2.waitKey(delay)
print("Done")
cap.release()
cv2.destroyAllWindows()
