import dht
import network
from machine import Pin, SPI, ADC
from time import sleep, sleep_us
from umqtt.simple import MQTTClient
from max6675 import MAX6675

# Configuración del WiFi
WIFI_SSID = "xxxx" #Nombre de la red Wifi
WIFI_PASSWORD = "xxxx" #Contraseña de la red Wifi

# Configuración del broker MQTT y los topics
MQTT_BROKER = "xxxx"  # Ejemplo: "192.168.1.100"
MQTT_CLIENT_ID = "curd" # Debe ser diferente al ID del broker
TEMPERATURE_TOPIC = "temperatura"
HUMIDITY_TOPIC = "humedad"
PH_TOPIC = "ph"

# Variables de control
horas_giro = 24
horas_fermentado = 216
humedad_ideal = 80
pH_ideal = 4.5
temperatura_ideal = 45
fermentado = 1

# Selección de pines
#sensor = dht.DHT22(Pin(14))
sensor = dht.DHT11(Pin(27))
pH_pin = 34 # Pin al que se asigna el sensor de pH
cs_pin = 12  # Pin CS conectado al pin 10 de ESP32
clock_pin = 13  # Pin SCK conectado al pin 13 de ESP32
data_pin = 14  # Pin SO conectado al pin 12 de ESP32
pin_motor = machine.Pin(12, machine.Pin.OUT)  # Pin del rele que avtiva al motor

# Configuración de los pines
spi = SPI(1, baudrate=1000000, sck=Pin(clock_pin), mosi=Pin(23), miso=Pin(data_pin)) #sck 
cs = Pin(cs_pin, Pin.OUT)

thermocouple = MAX6675(spi, cs)

ph_sensor = ADC(Pin(pH_pin))
ph_sensor.width(ADC.WIDTH_12BIT)
ph_sensor.atten(ADC.ATTN_11DB)

# Conectar al WiFi
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect(WIFI_SSID, WIFI_PASSWORD)
while not sta_if.isconnected():
    pass
print("Conectado al WiFi")

#Funciones para los sensores
def leer_temperatura():
    try:
        temp_c = thermocouple.read_temp_c()
        return temp_c
    except OSError as e:
        return "Failed to read Termopar sensor."

def leer_humedad():
    try:
        sensor.measure()
        hum = sensor.humidity()
        return hum
    except OSError as e:
        return "Failed to read DTH11 sensor."

def leer_ph():
    try:
        value = adc.read()
        voltage = value * (3.3 / 4095.0)
        ph = 3.3 * voltage
        return ph
    except OSError as e:
        return "Failed to read pH sensor."


# Función para encender el motor
def accionar_motor():
    # Coloca aquí el código para accionar el motor
    pin_motor.on()  # Enciende el motor
    time.sleep(100)   # Espera 5 segundos (ajusta según sea necesario)
    pin_motor.off()  # Apaga el motor

# Función para la configuración del tiempo (en segundos)
def tiempo_segundos(horas):
    return horas * 3600  # cantidad de segundos en una hora
    
# Función para escribir en un archivo CSV
def escribir_csv(humedad, temperatura, ph):
    try:
        # Abre el archivo en modo de escritura (w)
        with open("datos.csv", "a") as archivo:
            # Escribe la línea CSV con el timestamp, temperatura y humedad
            archivo.write("{},{},{},{}\n".format(time.time(), humedad, temperatura, ph))
    except Exception as e:
        print("Error al escribir en el archivo:", e)

# Función para publicar los datos en los topics MQTT
def publish_sensor_data(client):
    temperature = leer_temperatura()
    humidity = leer_humedad()
    ph = leer_ph()
    client.publish(TEMPERATURE_TOPIC, str(temperature))
    client.publish(HUMIDITY_TOPIC, str(humidity))
    client.publish(PH_TOPIC, str(humidity))
    print("Datos publicados - Temperatura: {}°C, Humedad: {}%, pH: {}".format(temperature, humidity,pH))
    escribir_csv(temperature, humidity, ph)

# Conectar al broker MQTT con autenticación
client = MQTTClient(client_id=MQTT_CLIENT_ID, server=MQTT_BROKER)
client.connect()

# Inicio del temporizador
ts_fermentado = tiempo_segundos(horas_fermentado)
ts_giro = tiempo_segundos(horas_giro)
ti_fermentado = time.time()
ti_giro

# Bucle principal
try:
    while fermentado == 1:
        # Verifica el tiempo asignado para el giro
        if time.time() - ti_giro >= ts_giro:
            accionar_motor()
            ti_giro = time.time()  # Reinicia el temporizador
        # Verifica la temperatura
        if leer_temperatura() > temperatura_ideal:
            accionar_motor()
        # Verifica el pH
        if leer_ph() > ph_ideal:
            accionar_motor()
        # Verifica el paso del tiempo
        if time.time() - ti_fermentado >= ts_fermentado:
            fermentado = 0  # Para el proceso
        publish_sensor_data(client)
        sleep(2)  # Publicar cada 60 segundos
        
    print("Fermentado finalizado")
except KeyboardInterrupt:
    print("Desconectando...")
    client.disconnect()
