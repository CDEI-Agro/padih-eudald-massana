import time
import board
import busio
#import adafruit_ads1x15.ads1015 as ADS
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from numpy import interp

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan0 = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)

# Create differential input between channel 0 and 1
#chan = AnalogIn(ads, ADS.P0, ADS.P1)

print("{:>5}\t{:>5}".format('raw', 'v'))
while True:
    if chan0.value > 13420:
        val0 = interp(chan0.value,[13420,26240],[0,1])
    elif chan0.value < 12280:
        val0 = interp(chan0.value,[0,12280],[-1,0])
    else:
        val0 = 0

    if chan1.value > 13420:
        val1 = interp(chan1.value,[13420,26240],[0,1])
    elif chan1.value < 12280:
        val1 = interp(chan1.value,[0,12280],[-1,0])
    else:
        val1 = 0
    print("{:>5}\t{:>5.3f}\t{:>5}\t{:>5.3f}".format(chan0.value, val0, chan1.value, val1))
    time.sleep(0.5)
    