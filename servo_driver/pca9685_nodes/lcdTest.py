import time
import RPi.GPIO as GPIO
#from adafruit_character_lcd.character_lcd import Character_LCD 
#from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C 

# LCD setup 
LCD_COLUMNS = 16 
LCD_ROWS = 2 

#LCD_CHR = True
#LCD_CMD = False

# GPIO pins for the LCD 
RS = 26 
E = 19
D4 = 25
D5 = 24
D6 = 22
D7 = 27

# Setup GPIO mode and pins 
GPIO.setmode(GPIO.BCM) 
GPIO.setup(RS, GPIO.OUT) 
GPIO.setup(E, GPIO.OUT) 
GPIO.setup(D4, GPIO.OUT) 
GPIO.setup(D5, GPIO.OUT) 
GPIO.setup(D6, GPIO.OUT) 
GPIO.setup(D7, GPIO.OUT)

 # Initialize the LCD 
lcd = Character_LCD( rs=RS,
 e=E, 
d4=D4,
 d5=D5, 
d6=D6, 
d7=D7,
 cols=LCD_COLUMNS,
 rows=LCD_ROWS ) 

# # Analog input setup 
# analog_pin = analogio.AnalogIn(board.A0) 

# def read_voltage():
#  # Read the raw value from the analog input 
# raw_value = analog_pin.value 
# # Convert the raw value to voltage 
# voltage = raw_value * (3.3 / 65536) 

# # Adjust based on your ADC resolution
#  return voltage 

# def calculate_battery_percentage(voltage): 
# voltage_max = 7.8 
# voltage_min = 6.4 
# voltage_recalc = (voltage * 550) / 330
#  batt_percentage = (voltage_recalc - voltage_min) / (voltage_max - voltage_min) * 100 batt_percentage = max(0, min(100, batt_percentage))

#  # Constrain to the range 0-100 
# return int(batt_percentage)

#  while True: 
# voltage = read_voltage() 
# print(f"Voltage: {voltage:.2f} V") # Print voltage to console 

# batt_percentage = calculate_battery_percentage(voltage) 
# lcd.clear()
# lcd.set_cursor(0, 0) 
# lcd.message(f"Battery:{batt_percentage:3d}%% ") 
# lcd.set_cursor(13, 0) 
lcd.message("MCC")
time.sleep(1)
