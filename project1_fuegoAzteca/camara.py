import cv2

# Abre el dispositivo de video /dev/video1
cap = cv2.VideoCapture(1)

# Verifica si el dispositivo de video se abrio correctamente
if not cap.isOpened():
    print("Error: No se pudo abrir el dispositivo de video /dev/video1")
else:
    # Lee y muestra el video frame a frame
    while True:
        ret, frame = cap.read()  # Lee un frame del dispositivo de video
        if not ret:
            print("Error: No se pudo recibir el frame")
            break

        cv2.imshow('Video desde /dev/video1', frame)  # Muestra el frame

        # Presiona 'q' para salir del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera el objeto VideoCapture y destruye todas las ventanas abiertas
    cap.release()
    cv2.destroyAllWindows()