import numpy as np
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import time 
from datetime import datetime

# Configura las credenciales de Firebase
cred = credentials.Certificate('/home/alex/S8/database_tryout/agrobot-6668a-firebase-adminsdk-tnt08-2ea5fec9e7.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://agrobot-6668a-default-rtdb.firebaseio.com/'
})

# Obtén una referencia a la base de datos en tiempo real
ref = db.reference('/')

init = 1
max = 1000
step = 0.1

for i in np.arange(init, max, step):
    # Obtén la fecha y hora actual
    t = datetime.now()

    # Formatea la fecha y hora como una cadena para usarla como clave
    clave = t.strftime("%Y-%m-%d %H:%M:%S")
    temperatura = np.random.randint(15, 35)
    humedad = np.random.randint(15, 35)
    datos = {'temperatura':temperatura, 
             'humedad' : humedad}

    # Enviar datos a la base de datos de Firebase
    ref.child('datos').child(clave).push(datos)

    print(clave, " : ", datos)

    time.sleep(5)

    # Puedes agregar más datos si es necesario
    
print("Datos enviados a Firebase.")
