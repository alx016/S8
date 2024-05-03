import cv2
import numpy as np
import onnxruntime as rt

# Cargar el modelo ONNX
model_path = "/home/al3x/S8/agrobot_ws/src/img_detection/scripts/plantas.onnx"
session = rt.InferenceSession(model_path)
input_name = session.get_inputs()[0].name

# Clases de objetos detectados
class_names = ['Beans_Angular_LeafSpot', 'Beans_Rust', 'Strawberry_Angular_LeafSpot', 'Strawberry_Anthracnose_Fruit_Rot', 
               'Strawberry_Blossom_Blight', 'Strawberry_Gray_Mold', 'Straberry_Leaf_Spot', 'Straberry_Powdery_Mildew_Fruit', 
               'Straberry_Powdery_Mildew_Leaf', 'Tomato_Blight', 'Tomato_Leaf_Mold', 'Tomato_Spider_Mites']

# Configurar la captura de video desde la cámara
cap = cv2.VideoCapture(0)

while True:
    # Capturar frame por frame
    ret, frame = cap.read()
    
    if not ret:
        break

    # Preprocesar la imagen para que coincida con el formato de entrada del modelo
    input_img = cv2.resize(frame, (640, 640))  # Redimensionar la imagen para que coincida con el tamaño de entrada del modelo
    input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)  # Convertir de BGR a RGB
    input_img = np.transpose(input_img, (2, 0, 1))  # Transponer a (canales, ancho, alto)
    input_img = np.expand_dims(input_img.astype(np.float32), axis=0)  # Agregar dimensión de lote y convertir a flotante
    input_img /= 255.0  # Normalizar los valores de píxel

    # Realizar inferencia con el modelo
    output = session.run(None, {input_name: input_img})[0]

    # Procesar las predicciones
    for detection in output:
        class_id = int(detection[0][0])
        confidence = float(detection[1][0])
        if confidence > 0.5:  # Filtro de confianza
            class_name = class_names[class_id]
            box = detection[2][0]
            x1, y1, x2, y2 = map(int, box * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]]))
            print(class_name)
            
            # Dibujar caja delimitadora y etiqueta de clase
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{class_name}: {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Mostrar el frame con las detecciones
    cv2.imshow('Object Detection', frame)



    # Presionar 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
