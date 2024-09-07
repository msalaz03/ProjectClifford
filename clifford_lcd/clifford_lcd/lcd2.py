import drivers
from time import sleep as sleep
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board

display = drivers.Lcd()
i2c = board.I2C()
#i2c = busio.I2C(board.SCL, board.SDA)

#ads = ADS.ADS1115(i2c)
#ads.gain = 1
#chan = AnalogIn(ads,P0)

def main(args=None):
    while True:
            
        #rawValue = chan.voltage
        #voltage = rawValue * (3.3 / 65536)
        voltage = 7.6
        voltage_max = 8.4
        voltage_min = 7.4
        voltage_recalc = (voltage * 550) / 330
        batt_percentage = (7.6 - voltage_min) / (voltage_max - voltage_min) * 100
        batt_percentage = int (max(0, min(100, batt_percentage)))
            
        print(batt_percentage)
        display.lcd_display_string(f"Battery: {batt_percentage}% MCC", 1) 
       	
if __name__ == "__main__":
    main()