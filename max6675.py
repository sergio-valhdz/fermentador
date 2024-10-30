import machine
from machine import Pin, SPI, ADC, SoftI2C
from time import sleep, time, localtime
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
import dht
from max6675 import MAX6675

# Configuración de la LCD
I2C_ADDR = 0x27
totalRows = 2
totalColumns = 16
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)
lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)

# Variables de control
horas_giro = 24
horas_fermentado = 216
humedad_ideal = 80
pH_ideal = 4.5
temperatura_ideal = 45
fermentado = 1

# Selección de pines
sensor = dht.DHT11(Pin(27))
pH_pin = 34
cs_pin = 12
clock_pin = 13
data_pin = 14
pin_motor = Pin(12, Pin.OUT)

# Configuración de los pines
spi = SPI(1, baudrate=1000000, sck=Pin(clock_pin), mosi=Pin(23), miso=Pin(data_pin))
cs = Pin(cs_pin, Pin.OUT)

thermocouple = MAX6675(spi, cs)

ph_sensor = ADC(Pin(pH_pin))
ph_sensor.width(ADC.WIDTH_12BIT)
ph_sensor.atten(ADC.ATTN_11DB)

# Funciones para los sensores
def leer_temperatura():
    try:
        temp_c = thermocouple.read_temp_c()
        return temp_c
    except OSError:
        return "Error en Temp"

def leer_humedad():
    try:
        sensor.measure()
        hum = sensor.humidity()
        return hum
    except OSError:
        return "Error en Hum"

def leer_ph():
    try:
        value = ph_sensor.read()
        voltage = value * (3.3 / 4095.0)
        ph = 3.3 * voltage  # Ajustar según sea necesario para obtener el pH real
        return ph
    except OSError:
        return "Error en pH"
    
def obtener_fecha_hora():
    fecha_hora = localtime()
    return "{:02}-{:02}-{:02} {:02}:{:02}:{:02}".format(
        fecha_hora[0], fecha_hora[1], fecha_hora[2],
        fecha_hora[3], fecha_hora[4], fecha_hora[5]
    )

# Función para encender el motor
def accionar_motor():
    pin_motor.on()
    sleep(5)
    pin_motor.off()

# Función para la configuración del tiempo (en segundos)
def tiempo_segundos(horas):
    return horas * 3600

# Función para escribir en un archivo CSV
def escribir_csv(humedad, temperatura, ph, archivo="datos.csv"):
    try:
        # Verifica si el archivo ya existe
        with open(archivo, "a") as f:
            if f.tell() == 0:  # Si el archivo está vacío, escribe el encabezado
                f.write("Fecha_Hora,Humedad,Temperatura,pH\n")
            # Escribe los datos en el archivo CSV
            f.write("{},{},{},{}\n".format(obtener_fecha_hora(), humedad, temperatura, ph))

    except Exception as e:
        print("Error al escribir en el archivo:", e)

# Mostrar datos en la pantalla LCD
def mostrar_en_lcd(humedad, temperatura, ph):
    lcd.clear()
    lcd.move_to(0, 0)
    
    # Mostrar temperatura
    if isinstance(temperatura, (int, float)):
        lcd.putstr("Temp: {:.1f}C".format(temperatura))
    else:
        lcd.putstr(temperatura)

    # Mostrar humedad
    lcd.move_to(0, 1)
    if isinstance(humedad, (int, float)):
        lcd.putstr("Humedad: {:.1f}%".format(humedad))
    else:
        lcd.putstr(humedad)

    # Mostrar pH
    lcd.move_to(9, 0)
    if isinstance(ph, (int, float)):
        lcd.putstr("pH: {:.2f}".format(ph))
    else:
        lcd.putstr(ph)

    # Mostrar fecha y hora
    lcd.move_to(0, 1)
    lcd.putstr(obtener_fecha_hora())

# Bucle principal
try:
    ti_fermentado = time()
    while fermentado == 1:
        temperatura = leer_temperatura()
        humedad = leer_humedad()
        ph = leer_ph()

        mostrar_en_lcd(humedad, temperatura, ph)
        escribir_csv(humedad, temperatura, ph)  # Guarda los datos en el archivo CSV
        
        # Acciones de control del motor según las lecturas
        if isinstance(temperatura, (int, float)) and temperatura > temperatura_ideal:
            accionar_motor()
        if isinstance(ph, (int, float)) and ph > pH_ideal:
            accionar_motor()

        # Verifica el paso del tiempo del proceso de fermentado
        if time() - ti_fermentado >= tiempo_segundos(horas_fermentado):
            fermentado = 0  # Termina el proceso de fermentación

        sleep(2)

    lcd.clear()
    lcd.putstr("Fermentado finalizado")
except KeyboardInterrupt:
    lcd.clear()
    lcd.putstr("Desconectando...")

