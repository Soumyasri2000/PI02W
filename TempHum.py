from adafruit_ahtx0 import AHTx0
import board
import sys
import RPi.GPIO as  GPIO
import busio
import logging
import logfunc
import time
import math
from i2c_expanders.PCA9555 import PCA9555

Outside_DB,Outside_RH,Supply_DB,Supply_RH= None,None,None,None


#initialize shared I2Cbus and PCA9555 expander(Address 0x20)
i2c = board.I2C()
pca = PCA9555(i2c, address=0x20)
#setup P0-P2 as outputs initial state is OFF
pca.iodir &= ~0b111
pca.gpio  &= ~0b111
#I2C Expander PCA9555PW with three AM2108 sensors
def _read_channel(channel_index: int, warmup: float = 0.05):
    """
    powers on the sensor, read values, powers it off
    """
    mask = 1<< channel_index
    pca.gpio = (pca.gpio &~0b111) | mask
    time.sleep(warmup)
    sensor = AHTx0(i2c)
    temp = sensor.temperature
    hum = sensor.humiidty
    pca.gpio &= ~mask
    return temp, hum

def getOutdoor():
    return _read_channel(0)
def getSupply():
    return _read_channel(1)
def getRoomTemp():
    return _read_channel(2)


def Temperature():
    while True:
        try:
            Outside_DB, Outside_RH = getOutdoor()
            logging.info(f" Outdoor:{Outside_DB:.2f}°C, {Outside_RH:.2f}%")
            Supply_DB, Supply_RH = getSupply()
            logging.info(f" Outdoor:{Supply_DB:.2f}°C, {Supply_RH:.2f}%")
            Roomtemp_DB, Roomtemp_RH = getRoomTemp()
            logging.info(f" Outdoor:{Roomtemp_DB:.2f}°C, {Roomtemp_RH:.2f}%")
        except Exception as e:
            logging.info(f" Error Reading the  tempoerature sensors:",{e})
        time.sleep(10)

#wetbulb temperature and drybulb temperature
def calculate_Wet_bulb(dry_bulb_temperature: float, relative_humidity: float) -> float:
    t  = dry_bulb_temperature
    rh = relative_humidity
    wet_bulb_temp = ( 
          t* math.atan(0.151977 * math.sqrt(rh + 8.313659))
         +math.atan(t+rh)
         -math.atan(rh - 1.676331)
         +0.00391838 * (rh ** 1.5) * math.atan(0.023101 * rh)
         -4.686035
    )
    return wet_bulb_temp

#saturation vapour pressure
def saturation_vapour_pressure(temperature: float) -> float:
    """
    Compute saturation vapor pressure (hPa) using the Magnus formula,
    based on temperature (°C).
    """ 
    return 6.112 * math.exp(17.67 * temperature) / (temperature + 243.5)
 
