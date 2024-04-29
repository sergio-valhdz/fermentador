import machine
import time

class MAX6675:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs

    # Lee la temperatura en grados Celsius
    def read_temp_c(self):
        self.cs.off()
        time.sleep_us(50)
        data = self.spi.read(2)
        self.cs.on()

        temp = (data[0] << 8 | data[1]) >> 3
        return temp * 0.25
