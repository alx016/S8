import numpy as np
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import time 
from datetime import datetime

# Configura las credenciales de Firebase
cred = credentials.Certificate('/home/al3x/S8/database_tryout/firebase_key.json')   #Ubicación del archivo de credenciales
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://agrobot-6668a-default-rtdb.firebaseio.com/'
})#URL de la página

ref = db.reference('/')

#Tiempo de simulación
init = 1
max = 1000
step = 0.1

for i in np.arange(init, max, step):
    # Obtén la fecha y hora actual
    t = datetime.now()

    # Formatea la fecha y hora como una cadena para usarla como clave
    clave = t.strftime("%Y-%m-%d %H:%M:%S")

    #Obtener datos de temperatura y humedad simulada
    temperatura = np.random.randint(15, 35)
    humedad = np.random.randint(15, 35)

    #Formatear los datos
    datos = {'temperatura':temperatura, 
             'humedad' : humedad}

    # Enviar datos a la base de datos de Firebase
    ref.child('datos').child(clave).push(datos)

    #Visualizar datos localmente
    print(clave, " : ", datos)

    # Simulación de retraso en sensores
    time.sleep(1)
    
print("Datos enviados a Firebase.")
