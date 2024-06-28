import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P1)

def exponential_moving_average(value, prev_ema, alpha=0.1):
    return alpha * value + (1 - alpha) * prev_ema

def adc_to_angle(adc_value):
    MAX_ADC = 32767
    GAIN_VOLTAGE = 4.096
    MAX_INPUT_VOLTAGE = 3.293 # measured max

    MAX_ADC_3V3 = (MAX_INPUT_VOLTAGE / GAIN_VOLTAGE) * MAX_ADC

    degree_per_ADC_step = 180 / MAX_ADC_3V3

    return adc_value * degree_per_ADC_step

prev_ema = 0
alpha = 0.5 # the smaller the value, the smaller of an affect newer values have
            # larger alpha, means we are more responsive/affected by newer value

while True:
    raw_value = chan.value
    prev_ema = exponential_moving_average(raw_value, prev_ema, alpha)
    angle = adc_to_angle(prev_ema)

    print(angle, "-"*10, chan.voltage)
    time.sleep(0.1)
