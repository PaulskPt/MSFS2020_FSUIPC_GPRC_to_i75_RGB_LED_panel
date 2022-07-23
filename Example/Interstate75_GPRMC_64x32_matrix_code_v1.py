#!/usr/bin/python3
"""
    FOR USE WITH MICROPYTHON 

    Project to display Microsoft Flightsimulator airplane track on an Adafruit 64x32 RGB LED Matrix Panel
    controlled by a Pimoroni Interstate 75 controller using MicroPython programming language.
    Date of start of this project: 2022-01-14 20h10 PT.
    Hardware used:
    - Interstate 75, USB-C powered controller for HUB75 panels (Product ID The Pi Hut: SKU: PIM584) £ 11.25;
    - RGB LED Matrix Panel - 64x32 3mm pitch (Product ID The Pi Hut: SKU: ADA2279) £ 32.58 each;
    - Adafruit MCP2221A, Gen Purpose USB to GPIO ADC I2C Stemma QT/Qwiic breakout (Product ID: 4471);
    - Adafruit ISO1540x Bidirectional I2C isolator (Product ID: 4903);
    - DC power supply (adjustable) 5V maximum 5 Ampères.
    
    Project goal acomplished on 2022-01-21. 
          
    Origin of this project is in my former project:
    a python script for the project 'GPRMC_via_serial'.
    Featuring reception, filtering and displaying of GPRMC GPS messages.
    using an Adafruit 64x32 3mm-pitch LED Matrix panel controlled by a Pimoroni Interstate 75 (RP2040).
    Before this version of the project, it was designed to be used with a RPi Zero2W. 
    The GPRC part of this script originally had been created to be used with a
    Raspberry Pi Pico dev board, mounted on a Seeed Grove ... for Pi Pico
    Attached to it via I2C: a 4x20 character LCD Hitachi with piggy-back I2C expander.
    See the README.md for details about this project.

    GPS datagram of type GPRMC example:
        b'GPRMC,122328.00,A,5131.8445,N,00014.8693,E,0.0,213.3,110122,0.5,E*5D\r'
        
    This version has a modified ck_uart()
    Now ck_uart tries to read a full line through uart.readline()
    If this gives an error "NoneType has no attribute 'readline()'
    ck_uart will try c = uart.read() and then if a character has been received.
    The data received is a bytearray. This bytearray is converted into a text buffer (rx_buffer).
    
    Next the received GPS datagram will be split into twelve data items, saved as a msg_lst list.
    In this project we only will use the Groundspeed data item and the 'track made good' data item.
    The groundspeed data is used to discern if the airplane is moving or not. When moving, the LED
    panel will display the track. When the airplane is stopped or parked, the LED panel will be OFF.
    
    Update 2022-07-23. Some mods in ck_uart and added gc module.
    In ck_uart() blocking received $GPGGA messages.
    NOTE: if you get errors or garbled / delayed reception of gps messages: Check the baudrate in FSUIPC GPSout
    and here in this script both are set for 4800 baud.

"""
# Imports for the Interstate 75
import hub75
from font_8x12_v4 import lw8x12, lh8x12, font8x12
from font_10x14 import lw10x14, lh10x14, font10x14 #etter_width, letter_height
from pimoroni import RGBLED, Button
from time import ticks_ms
#import time
import math
from machine import RTC, Pin, UART
from pimoroni_i2c import PimoroniI2C # builtin in Pimoroni's micropython
from breakout_rtc import BreakoutRTC # idem

#from microcontroller import pin
import sys
from time import sleep
from array import *
import gc
# ------------------------------------+
# Imports for Pimoroni DisplayHatMini |
# ------------------------------------+
import random
import time
import math

# -------------------------------+
# General debug flag             |
my_debug = False  # Debug flag   |
# -------------------------------+
# Other important flags          |
# -------------------------------+
use_button_a = True

gc.enable() # Enable autmatic garbage collection

#sound = ADC(1)
# --------------------------------------+
# Variables for Pimoroni Interstate75   |
# --------------------------------------+

#matrix = hub75.Hub75(WIDTH, HEIGHT)
#matrix.start()

# object creation in CircuitPython:
"""
matrix = rgbmatrix.RGBMatrix(
    width=64, height=32, bit_depth=6,
    rgb_pins=[board.R0, board.G0, board.B0, board.R1, board.G1, board.B1],
    addr_pins=[board.ROW_A, board.ROW_B, board.ROW_C, board.ROW_D],
    clock_pin=board.CLK, latch_pin=board.LAT, output_enable_pin=board.OE)
"""
dh_left = 10

WIDTH, HEIGHT = 64, 32
MAX_DIST = (WIDTH * WIDTH + HEIGHT * HEIGHT) * 0.5
DIGITS = [0b1110111,
          0b0100100,
          0b1011101,
          0b1101101,
          0b0101110,
          0b1101011,
          0b1111010,
          0b0100101,
          0b1111111,
          0b1101111]

rtc = None
rtc2 = None

s_ymd = s_wd = s_hms = None

dow_lst = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

hub = hub75.Hub75(WIDTH, HEIGHT, stb_invert=False)
hub.start()
hub.clear()
hub.flip()

font_small = 0   # 0 = font8x12, 1 = font10x14
font_large = 1

set_hsv = hub.set_hsv
set_rgb = hub.set_rgb


dh_row_lst = [0, 9, 18, 27]
dh_hdg_row_lst = [14,23]
ribbon = None

# +--------------------------------------------------+
# | Definitions for 320x240 Displayhat mini display  |
# +--------------------------------------------------+
dh_rowlen = WIDTH
dh_maxrows = 8
dh_name1 = "MSFS 2020"
dh_name2 = "GPRMC data RX"
dh_black = (0, 0, 0)
dh_white = (255, 255, 255)

# +----------------------+
# | Definition for I2C   |
# +----------------------+
#i2c_sda = Pin(3, Pin.OUT)  # i2c sda
#i2c_scl = Pin(4, Pin.IN, Pin.PULL_UP) # i2c scl
try:
    PINS_INTERSTATE_75 = {"sda": 20, "scl": 21}  # i2c Pins for Interstate 75
    i2c = PimoroniI2C(**PINS_INTERSTATE_75)
except Exception as e:
    print("Error while creating i2c object", e)
# end of imports for the Interstate 75
# +----------------------+
# | Definition for UART  |
# +----------------------+
# 
# via a TTL-to-USB converter connected to desktop PC Paul1 (COM10)
#
#UART_TX = pin.GPIO24
#UART_RX = pin.GPIO25
"""
uart = serial.Serial(
    port='/dev/serial0', # was: ttyS0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize= serial.EIGHTBITS,
    timeout = 1
)
"""
try:
    #uart = serial.Serial("COM26", baudrate=9600, timeout=10)
    tx_pin = Pin(20, Pin.OUT)  # i2c sda
    rx_pin = Pin(21, Pin.IN, Pin.PULL_UP) # i2c scl
    uart = UART(1, 4800, parity=None, stop=1, bits=8, rx=rx_pin, tx=tx_pin)

    # wait a minimum amount of time before trying to read the sensor
    sleep(0.25)
except ValueError:  # e.g.: "bad TX pin"
    uart = None
    pass  # for the sake of debugging the rest of this script, let go!

rx_err_cnt = 0  # log the number of misses
rx_unicode_err_cnt = 0 # idem for unicode errors
rx_serexcept_err_cnt = 0  # idem for serial.exception errors
# Global definitions
# +--------------------------------------------+
max_lp_cnt = 14  # <<<=========== LOOP COUNT   |
# +--------------------------------------------+
my_platform = sys.platform # for the Pi Pico the value is: 'RP2040'

if sys.version_info > (3,): # Checked 2101-06-02: sys.version_info results in: (3, 4, 0)
    long = int  # See the notation: '100L' below at line 503
# Note: sys.stdin  results in '<io.FileIO 0>'
#       sys.stdio  results in '<io.FileIO 1>'
#       sys.stderr results in '<io.FileIO 2>'

ID_s = "" # The datagram ID e.g. GPRMC
ide_name = "MSFS2020 GPS GPRMC data reception decoder sketch by Paulsk (mailto: ct7agr@live.com.pt). "

# BI_LED_R = 16 # RGB LED
# BI_LED_G = 17 # idem
# BI_LED_B = 18 # idem

led = RGBLED(hub75.LED_R, hub75.LED_G, hub75.LED_B)

button_a = Button(hub75.BUTTON_A)
button_boot = Button(hub75.BUTTON_USER)

led.set_rgb(0, 0, 0) # Switch led off

led_interval = 1000
biLdIsOn = False
lp_cnt = 0
lstop = 0
startup = -1
previousMillis = 0  # unsigned long
loop_time = 0
# Message serial nr
msg_nr = 0
GPRMC_cnt = 0
nr_msg_items = 0
gs_item = 7 # was: 6
gs_old = 0.0
ac_parked = True
t_parked_init = 0

# Buffers
rx_buffer_len = 120
rx_buffer = ""
msg_lst = [] # Create an empty message list. Will later be filled with the GPRMC message parts splitted


def setDT():
    global rtc, rtc2, dow_lst, time, i2c, BreakoutRTC
    if my_debug:
        print("setDT(): we passed here")
    if i2c:
        try:
            rtc2 = BreakoutRTC(i2c)
        except Exception as e:
            print("setDT(): error while trying to create rtc2 object: ", e)
            return
        time.sleep(2)
        if rtc2:
            if my_debug:
                print("setDT(): rtc2 object created")
            rtc2.enable_periodic_update_interrupt(True)
            if my_debug:
                print("rtc2.is_12_hour(): ", rtc2.is_12_hour())
            if rtc2.is_12_hour():
                rtc2.set_24_hour()
            if rtc2.read_periodic_update_interrupt_flag():
                rtc2.clear_periodic_update_interrupt_flag()
            if rtc2.update_time():
                yy = rtc2.get_year()
                print("year", yy)
                mo = rtc2.get_month()
                print("month", mo)
                dd = rtc2.get_date()
                print("date", dd)
                wd = rtc2.get_weekday()
                print("weekday", wd)
                hh = rtc2.get_hours()
                print("hours", hh)
                mm = rtc2.get_minutes()
                print("minutes", mm)
                ss = rtc2.get_seconds()
                print("seconds", ss)
            rtc2 = None # rtc.deinit()  # was: rtc2 = None # detach rtc2
            rtc = RTC()
            #year, month, day, wd, hour, minute, second, _ = rtc.datetime()
            
            #if year != 2022:  # only set if rtc was not set before
            try:
                if my_debug:
                    print("weekday value: {}".format(wd), end='\n')
                    print("trying to set built-in rtc to: {} {}-{}-{} {}:{}:{}".format(dow_lst[wd],yy,mo,dd,hh,mm,ss), end='\n')
                rtc.datetime((yy, mo, dd, wd, hh, mm, ss, 0))  # was: ((2022, 1, 13, 3, 15, 01, 0, 0))
                time.sleep(1)
                if my_debug:
                    print("built-in rtc set to: {} {}-{}-{} {}:{}:{}".format(dow_lst[wd],yy,mo,dd,hh,mm,ss), end='\n')
                
                year, month, day, wd, hour, minute, second, _ = rtc.datetime()
                ymd = "{:04}-{:02}-{:02}".format(year, month, day)
                hms = "{:02}:{:02}:{:02}".format(hour, minute, second)
                if my_debug:
                    print("built-in rtc values: {} {}, {}".format(dow_lst[wd], ymd, hms), end='\n')
            except OSError as e:
                print("unable to set built-in rtc")
        else:
            if my_debug:
                print("setDT(): type rtc2 = ", type(rtc2))
    else:
        if my_debug:
            print("setDT(): type i2c = ", type(i2c))
            
# Calcultate a time in milliseconds derived from rtc's hour, minute, second and sub_second values
# Set the global previousMillis to the calculated time value (mSec)
# return the calculated time value (mSec)
def rtc_time():
    global previousMillis
    if my_debug:
        print("rtc_time(): we passed here")
    """
    # get the datetime tuple
    # see: https://www.unitjuggler.com/convert-time-from-s-to-ms.html
    year, month, day, wd, hour, minute, second, sub_second = rtc.now()  # was: rtc.datetime()
    currMillis = previousMillis + (hour*3600000) + (minute*60000) + (second*1000) + sub_second
    print("rtc_time(): currMillis = ", currMillis)
    """
    currMillis = ticks_ms()
    previousMillis = currMillis
    return currMillis
  

class HdgRibbon():
    def __init__(self):
        global WIDTH, HEIGHT, use_button_a, hub, set_hsv, set_rgb, lw8x12, lh8x12, lw10x14, lh10x14
        if my_debug:
            print("HdgRibbon.__init__(): we passed here")
        self.clr = 155
        self.fs_heading = 0
        self.heading_old = 0.0
        self.min_zoom_lvl = 0
        self.max_zoom_lvl = 3
        self.zoom_lvl = 1
        self.old_zoom_lvl = self.zoom_lvl
        self.fs_hdg_step = None
        self.set_fs_hdg_step()  # set self.fs_hdg_step according to self.zoom_lvl
        self.x = 0  # Left  was:25
        self.y = HEIGHT / 2
        self.next_y = self.y
        self.width = WIDTH
        self.height = HEIGHT
        self.lw8x12 = lw8x12  # letter_width
        self.lh8x12 = lh8x12  # letter_height
        self.lw10x14 = lw10x14
        self.lh10x14 = lh10x14
        self.digit_width = 8   # for the Pimoroni HUB75 LED matrix panels
        self.digit_height = 15 # idem
        self.digit_spacing = 2
        self.ox = self.width // 2
        self.oy = self.height // 2
        self.hue = 0
        self.h_pts =   [  0,  12,  22,  32,  42,  52,  64]   # for the HUB75 LED matrix panels
        self.hdg_pts = [-30, -20, -10,   0,  10,  20,  30]
        self.hub_clear()
        self.flip()
        sleep(1)

    def shader_fg(self, x, y):
        global MAX_DIST, set_hsv
        """Shades the lit pixels of a digit"""
        h = ((x - self.ox) * (x - self.ox) + (y - self.oy) * (y - self.oy)) / MAX_DIST
        set_hsv(x, y, 1.0, 1.0, 1.0) # was set_hsv(x,y, h + self.hue, 1.0, 1.0)

    def shader_bg(self, x, y):
        global set_rgb
        """Shades the unlit pixels of a digit"""
        #set_rgb(x, y, 10, 10, 10)
        set_rgb(x, y, 0, 0, 0)


    def draw_number(self, x, y, number, fg=None, bg=None):
        global DIGITS, my_debug, set_rgb
        if my_debug:
            #print("draw_number(): we passed here")
            print("Number to print: {}, type(number): {}".format(number, type(number)), end='\n')
        """Draw a sequence of digits.

        Uses lines to draw digits like so:
         _      _  _       _      _   _   _
        | |  |  _| _| |_| |_  |_   | |_| |_|
        |_|  | |_  _|   |  _| |_|  | |_|   |

        Digits are bit-packed into DIGITS,
        each part corresponds to 1-bit:

            0b0000001 = Top
            0b0000010 = Top Left
            0b0000100 = Top Right
            0b0001000 = Middle
            0b0010000 = Bottom Left
            0b0100000 = Bottom Right
            0b1000000 = Bottom

        """
        v_line = int((self.digit_height - 3) / 2)
        h_line = self.digit_width - 2
        
        if my_debug:
            print("type(fg): {}, fg: {}".format(type(fg), fg), end='\n')
        if fg is None:
            def fg(x, y):
                set_rgb(x, y, 0, 0, 0) # was: set_rgb(x,y,255, 255, 255)  # color white

        if my_debug:
            print("type(bg): {}, bg: {}".format(type(bg), bg), end='\n')
        if bg is None:
            def bg(x, y):
                pass
            
        print("number: \'{}\'".format(number), end='\n')
        for _ in range(len(number)):
        #for digit in number:
            digit = number[_]
            if digit == " ":
                digit = "0"
                #x += (self.letter_w)  # + self.digit_spacing
                #continue

            if digit == ".":
                return x,y
                #fg(x, y + v_line + v_line + 2)
                #x += self.digit_spacing

            try:
                parts = DIGITS[ord(digit) - 48]
            except IndexError:
                print("draw_number():  IndexError occurred")
                x += self.digit_spacing
                continue

            shader = fg if parts & 1 else bg  # top
            for px in range(h_line):
                shader(x + px + 1, y)

            shader = fg if parts & 2 else bg  # top left
            for py in range(v_line):
                shader(x, y + py + 1)

            shader = fg if parts & 4 else bg  # top right
            for py in range(v_line):
                shader(x + h_line + 1, y + 1 + py)

            shader = fg if parts & 8 else bg  # middle
            for px in range(h_line):
                shader(x + px + 1, y + v_line + 1)

            shader = fg if parts & 16 else bg  # bottom left
            for py in range(v_line):
                shader(x, y + v_line + 2 + py)

            shader = fg if parts & 32 else bg  # bottom right
            for py in range(v_line):
                shader(x + h_line + 1, y + v_line + 2 + py)

            shader = fg if parts & 64 else bg  # bottom
            for px in range(h_line):
                shader(x + px + 1, y + v_line + v_line + 2)

            x += self.digit_width + self.digit_spacing

        return x, y

    def text(self, text, fonttyp):
        global previousMillis, font8x12, font10x14, hub
        if fonttyp == 0:
            lw = self.lw8x12
            lh = self.lh8x12
        elif fonttyp == 1:
            lw = self.lw10x14
            lh = self.lh10x14
            
        if my_debug:
            print("text(): we passed here")
        #fnt = ImageFont.load_default()
        t = (ticks_ms() - previousMillis) / 50.0
        previousMillis = ticks_ms()  # refresh previousMillis
        self.display_text(text, 0, 3, t)
    
    def display_text(self, text, fonttyp, y, t):
        global font8x12, font10x14, hub
        s_x_offset = 1
        if fonttyp == 0:
            lw = self.lw8x12
            lh = self.lh8x12
        elif fonttyp == 1:
            lw = self.lw10x14
            lh = self.lh10x14
        n_created = False
        n = 3 # color
        s_y = y
        if my_debug:
            print("scroll_text(): we passed here")
        text_length = len(text)
        x = s_x_offset # = int(t)
        letter = 0 # int((x / lw) % text_length)
        pixel = 0 # x % lw
        char = ord(text[letter]) # fetch the value of the first character
        s_x = 1  # Leave column 0 blank
        zero_cols = 0
        ltr_lst = ["l", "f", "r"]
        le = 0
        if my_debug:
            print("text_length: {}".format(text_length), end='\n')
        while True:
            if fonttyp == 0: # using the 'small font' ?
                le = len(font8x12[char-32])  # get the number of column bytes
                if le > 0:  # do we have column bytes?
                    if pixel < le:  # yes, can we continue ?
                        col = font8x12[char - 32][pixel] # yes, get the next column byte
                    else:
                        col = 0
            else:  # using the 'large font'
                le = lw  # for the large font we don't measure how many column bytes the character has
                col = font10x14[char - 32][pixel]

            h = s_x / self.width  # This determines the hue (in this moment not used)
            
            if n is None:
                n_created = True
                n = 1 # was: random.randint(1,10)
            if pixel == 0: # print some debugging info if debug flag is set
                if my_debug:
                    if letter < text_length:
                        print("nr of cols (le) = ", le)
                    print("char = \'{}\', letter = {}, pixel = {}, s_x = {}, s_y = {}, h = {}, col = {}, n_created = {}, n {}".format(text[letter], letter, pixel, s_x, s_y, h, col, n_created, n), end='\n')

            hub.set_color_masked(s_x, s_y, col, hub75.color(100,0,0))  # set the pixel to red color
            s_x += 1  # advance the horizontal index
            pixel += 1 # advance the pixel index

            if pixel >= le: # all letter columns done?
                s_x += 1 # character spacing
                pixel = 0 
                letter += 1  # advance to next letter
            else:
                if pixel > 6 and zero_cols > 0:
                    pixel = 0
                    letter += 1

            if s_x >= self.width:  # passed end of the current row?
                s_y = s_y + lh + 2   # yes, move to the next row
                s_x = s_x_offset
            if letter >= text_length: # all text done ?
                break  # yes, exit
            char = ord(text[letter])  # no, advance to the next letter
            
            if char == 32:  # char is a space ?
                if s_x > self.width//2: # yes, passed half of the panel width?
                    # wrap around to next line
                    s_y = s_y + lh + 2   # move to the next row
                    s_x = s_x_offset
                    pixel = 0
                    letter += 1  # skip the space char when we did a wrap
                    if letter >= text_length: # all text done ?
                        break  # yes, exit
                else:  # no, continue on same row
                    s_x += 3  # advance letter spacing
                    if s_x >= self.width:  # passed end of the current row?
                        s_y = s_y + lh + 2   # yes, move to the next row
                        s_x = s_x_offset
                    pixel = 0
                    letter += 1  # advance to next letter
                    if letter >= text_length:  # all text done ?
                        break  # yes, exit

                char = ord(text[letter]) # fetch value of the next character
        hub.flip()
        
    def millis(self):
        return int(round(time.time() * 1000))
            
    def set_heading(self, dr):
        if my_debug:
            print("set_heading(): we passed here")
        hs = self.get_fs_hdg_step()
        if dr == True:
            if self.fs_heading + hs > 360:
                self.fs_heading = (self.fs_heading - 360) + hs
            else:
                self.fs_heading += hs
        else:
            if self.fs_heading - hs < 0:
                self.fs_heading = 360 + (self.fs_heading - hs)
            else:
                self.fs_heading -= hs
                
    def set_heading_fm_sim(self, hdg):
        global my_debug
        if my_debug:
            print("set_heading_fm_sim(): heading set to: ", hdg)
        self.fs_heading = hdg
        
    def draw_heading(self):
        global hub, my_debug
        if my_debug:
            print("draw_heading(): we passed here")
        font_type = 0
        if self.fs_heading != self.heading_old:  # only show heading if it differs from the previous value
            self.hub_clear()
            self.heading_old = self.fs_heading
            t2 = "{:5.1f}".format(self.fs_heading)
            print("draw_heading(): t2: ",t2)
            #my_pos = (WIDTH//2 - font_width//2), (HEIGHT//2-75) - ((font_height // 2))
            my_xpos = 18 #(self.width//2 - len(t2)//2)
            my_ypos =  (self.height//2) - ((self.digit_height // 2))
            self.draw_number(my_xpos, my_ypos, t2, fg=self.shader_fg, bg=self.shader_bg)
            hub.flip()
        else:
            self.ck_btns()
        
    def ck_btns(self):
        global use_button_a, button_a, button_boot, led, hub, biLdIsOn
        if my_debug:
            print("ck_btns(): we passed here")

        if use_button_a:
            r = 255
            g = 0
            b = 0
            if button_a.read():
                led.set_rgb(r, g, b)   # set led to red
                biLdIsOn = True 
                if my_debug:
                    print("ck_btns(): button a pressed")
                self.hub_clear()
                hub.stop()
                raise SystemExit  # use button a as an emergency stop
                
            if button_boot.read():
                r = 0
                g = 0
                b = 255
                led.set_rgb(r, g, b)  # set led to blue
                biLdIsOn = True 
                if my_debug:
                    print("ck_btns(): button boot (user) pressed")
                return True

            if self.get_zoom_lvl() != self.old_zoom_lvl:
                self.old_zoom_lvl = self.get_zoom_lvl()
                self.set_fs_hdg_step()
                #self.hub_clear()
                self.draw_heading()
                #self.render(hub)
                hub.flip()
                return True
        return False
               
    def get_zoom_lvl(self):
        if my_debug:
            print("get_zoom_lvl(): we passed here")
        return self.zoom_lvl
        
    def inc_zoom_lvl(self):
        if my_debug:
            print("inc_zoom_lvl(): we passed here")
        self.zoom_lvl += 1
        if self.zoom_lvl > self.max_zoom_lvl:
            self.zoom_lvl = self.max_zoom_lvl
        if my_debug:
            print("dec_zoom_lvl(): zoom level has been set to: {}".format(self.zoom_lvl), end='\n')
        self.set_fs_hdg_step()
            
    def dec_zoom_lvl(self):
        if my_debug:
            print("dec_zoom_lvl(): we passed here")
        self.zoom_lvl -= 1
        if self.zoom_lvl < self.min_zoom_lvl:
            self.zoom_lvl = self.min_zoom_lvl
            if my_debug:
                print("dec_zoom_lvl(): zoom level has been set to: {}".format(self.zoom_lvl), end='\n')
        self.set_fs_hdg_step()
        
    def draw_zoom_lvl(self):
        if my_debug:
            print("draw_zoom_lvl(): we passed here")
        font_type = 1
        if self.fs_hdg_step == 1:
            dgs = "deg"
        else: 
            dgs = "degs"
        t2 = "Zoom: {0:d} {1:s}".format(self.fs_hdg_step, dgs)  # was: (self.zoom_lvl)
        #(font_width, font_height) = my_font_hdg.getsize(t2)
        #my_pos = (55 + WIDTH//2 - font_width//2), (HEIGHT-20) - ((font_height // 2))
        my_xpos = (55 + self.width//2 - self.digit_width//2)
        my_ypos = (self.height-20) - ((self.digit_height // 2))
        #self.draw_number(my_xpos, my_ypos, t2, fg=self.shader_fg, bg=self.shader_bg)  # draw on the led matrix
        print(t2)

    def set_fs_hdg_step(self):
        if my_debug:
            print("set_fs_hdg_step(): we passed here")
        if self.zoom_lvl == 0:
            self.fs_hdg_step = 10
        elif self.zoom_lvl == 1:
            self.fs_hdg_step = 5  # default
        elif self.zoom_lvl == 2:
            self.fs_hdg_step = 2
        elif self.zoom_lvl == 3:
            self.fs_hdg_step = 1
            
    def get_fs_hdg_step(self):
        if my_debug:
            print("get_fs_hdg_step(): we passed here")
        return self.fs_hdg_step
        
    def hub_clear(self):
        global hub  # hub75
        if my_debug:
            print("hub_clear(): we passed here")
        hub.clear()
        #background_color = hub.color(0,0,0)  # hub75.color(0, 0, 0)
        #background_color = hub.set_all_color(0)
        #hub.flip_and_clear(background_color)
        
    def flip(self):
        hub.flip()

    def draw_10degs(self):
        global my_font_std, set_rgb
        if my_debug:
            print("draw_10degs(): we passed here")
        #h_pts =   [  0,  54, 107, 160, 213, 266, 319]

        h_idx = -1
        #text_size = 18
        font_type = 2  # we use my_font_std
        v_pos = self.height // 2 + 50
        r = 0
        g = 255
        b = 255
        
        min = (self.fs_heading -30) # 320 / 60 = 6
        max = (self.fs_heading +30) 
        # Draw the baseline
        """
        dh_draw.rectangle((
                0,
                v_pos - 5,
                WIDTH - 1,
                v_pos + 5),
                (self.clr, self.clr, self.clr)) # was: 64, 64, 64))
        """

        for x in range(0, self.width):
            for y in range(v_pos-1, v_pos+1):
                set_rgb(x, y, r, g, b)
        hub.flip()

        for _ in range(0,self.width):   # show 60 degrees
            if _ in self.h_pts:
                for i in range(len(self.h_pts)):
                    if _ == self.h_pts[i]:
                        h_idx = i
                        break
                if h_idx >= 0:
                    #dh_draw.rectangle((_ -2, (v_pos) -20, _ +2, (v_pos)), (self.clr, self.clr, self.clr))
                    for x in range(_ -2, _ +2):
                        for y in range(v_pos-2, v_pos):
                            set_rgb(x, y, r, g, b)
                    hub.flip()
                    # Calculate compass division values

                    if self.fs_hdg_step == 10:
                        my_hdg_pts = self.hdg_pts[h_idx]
                    elif self.fs_hdg_step == 5:
                        my_hdg_pts = self.hdg_pts[h_idx] // 2
                    elif self.fs_hdg_step == 2:
                        my_hdg_pts = self.hdg_pts[h_idx] // 5
                    elif self.fs_hdg_step == 1:
                        my_hdg_pts = self.hdg_pts[h_idx] // 10

                    if self.fs_heading + my_hdg_pts < 0:
                        t = 360 + my_hdg_pts
                    elif self.fs_heading + my_hdg_pts >= 360:
                        t = (self.fs_heading + my_hdg_pts) - 360
                    else:
                        t = self.fs_heading + my_hdg_pts
                    # Create and display the compass division marker texts
                    t2 = "{0:03d}".format(round(int(t),1))
                    #(font_width, font_height) = my_font_std.getsize(t2)
                    h_pos = self.h_pts[h_idx]
                    #my_pos = (h_pos - font_width//2), (v_pos-35) - ((font_height // 2))
                    my_xpos = (self.digit_width//2-h_pos)
                    my_ypos =  ((self.digit_height // 2)-(v_pos-3))
                    my_color = (self.clr, self.clr, self.clr)
                    self.draw_number(my_xpos, my_ypos, t2, fg=self.shader_fg, bg=self.shader_bg)  # draw on the led matrix
                    h_idx = -1  # reset h_idx


    def render(self, draw):
        global hub
        r = 255
        g = 255
        b = 255
        if my_debug:
            print("render(): we passed here")
        """
        dh_draw.rectangle((self.x - (self.width / 2),
                        self.y - (self.height / 2),
                        self.x + (self.width / 2),
                        self.y + (self.height / 2)), (r, g, b))
        """
        #for x in range((self.x - (self.width / 2), self.x + (self.width / 2))):
        for x in range( (self.width // 2)-self.x, (self.width // 2)+self.x ):
            for y in range((self.height // 2)-self.y,(self.height // 2)+self.y):
                set_rgb(int(x), int(y), r, g, b)
        #self.draw_heading()
        self.draw_10degs()
        self.draw_zoom_lvl()
# --------------------------------------------------  +
# Prototypes of the 13 functions in this script file: |
# --------------------------------------------------  +
"""
    def bfr_fnd(int): # (int)
    def ck_uart(): # (nr_bytes)
    def disp_crs(): # ()
    def empty_buffer():  # (void)
    def led_toggle(): # (void)
    def ck_gs(): # (float)
    def ac_is_parked(void): # (bool)
    def loop(): # (void)
    def main():
    def split_msg(): # (bool)
"""

def ck_gs():
    global msg_lst, gs_item
    if len(msg_lst) > 0:
        if my_debug:
            print("msg_lst = {}, gs_item={}".format(msg_lst, gs_item))
        return float(msg_lst[gs_item])
    return 0.0

def ac_is_parked():
    global gs_old, t_parked_init, ribbon, startup
    gs = ck_gs()
    if gs <= 0.2:
        if gs != gs_old:
            gs_old = gs
        t_parked = ribbon.millis()
        if (t_parked - t_parked_init) >= 200:  # twice the FSUIPC7 GPSout2 interval time
            return True
    return False

def loop():
    global startup, led, lp_cnt, ID_s, lstop, previousMillis, led_interval, hub, biLdIsOn, gs_item, gs_old, t_parked_init, \
        biLdIsOn, msg_nr, rx_buffer, msg_lst, nr_msg_items, ribbon,my_platform, dh_left, dh_hdg_row_lst, WIDTH, \
        my_font_title, rx_err_cnt, rx_unicode_err_cnt, rx_serexcept_err_cnt, font8x12, font10x14, font_small, font_large

    TAG = "loop(): "
    chrs_rcvd = 0  # (int) holds the number of characters received via the serial communication line
    c = 0  # (unsigned char)
    
    # gs_item = 6
    gs = 0.0
    gs_old = 0.0
    t_parked_init = 0.0

    # currentMillis (See Adafruit: 'time in CircuitPython')
    currentMillis = ticks_ms()   #monotonic_ns()
    s = "Platform " + my_platform
    sleep_for = 5
    led.set_rgb(0,0,0)
    biLdIsOn = False 
    dh_row = 0
    font_size = 1  # heading font size
    dh_lt = 20 # WIDTH // 2 - (len(dh_name1) //2)
    
    ribbon.hub_clear()
    ribbon.text(dh_name1, font_small)  # (hub, dh_name1, (dh_lt, dh_hdg_row_lst[dh_row]), font_size, dh_white)
    print(dh_name1)
    ribbon.flip()
    sleep(sleep_for)
    
    ribbon.hub_clear()
    ribbon.text(dh_name2, font_small)  # (hub, dh_name2, (dh_lt, dh_hdg_row_lst[dh_row+1]), font_size, dh_white)
    print(dh_name2)
    ribbon.flip()
    sleep(sleep_for)
    
    ribbon.hub_clear()
    ribbon.text(s, font_small)  #  (hub, s       , (dh_lt, dh_hdg_row_lst[dh_row+2], dh_left), font_size, dh_white)
    ribbon.flip()
    sleep(sleep_for)
    ribbon.hub_clear()
    sleep(0.5)
    print(s)   
    print()
    print(ide_name)
    print("\nNumber of loops in this run: {}".format(max_lp_cnt))
    sleep(0.5)  # <=================================== DELAY ===============================<
    """
    # Collect Serial Input NON-BLocking.
    # Continue receiving characters until '\n' (linefeed) is received.
    """
    rx_err_cnt = 0  # reset missers count
    rx_unicode_err_cnt = 0  # idem for rx unicode errors
    rx_serexcept_err_cnt = 0  # idem for serial.exception errors
    lSplitOK = False # flag to indicate the result of split_msg()
    split_err = False
    chrs_rcvd = 0  # reset the nr of characters received
    print("........................", end="\n")

    while True:
        lp_cnt += 1  # increase the loop counter
        ID_s = ''  # Clear the ID string
        # lcd.clear()  # clean the LCD
        print("\nStart of loop {}".format(lp_cnt), end="\n")

        if startup == -1:
            #ribbon.text(dh_draw,"About to RX msgs", (dh_hdg_row_lst[len(dh_hdg_row_lst)-1], dh_left), 1, dh_white)
            #ribbon.text(hub,"About to RX msgs", (dh_hdg_row_lst[len(dh_hdg_row_lst)-1], dh_left), 1, dh_white)
            ribbon.text("Init RX msgs", font_small)
            hub.flip()
            sleep(sleep_for)
        wait_cnt = 0
        # +--------------- RECEPTION ----------------------------------------+
        chrs_rcvd = ck_uart()  # read a complete GPS GPRMC datagram sentence |
        # +------------------------------------------------------------------+
        """
        sleep(0.02) # just a little pause to avoid entry of zeros  # <==================== DELAY =======================================<
        print("GPS data character received: ")
        print(rx_buffer)
        print()
        """
        if chrs_rcvd > 0:
            if not my_debug:
                #  print the rx_buffer less the \r\n at the end
                print(TAG+"Msg nr: {}, ID: {}, characters rcvd from ck_uart() is: {}, contents: \n\"{}\"".format(msg_nr, ID_s, chrs_rcvd, rx_buffer[:-2]), file=sys.stderr)
            n = bfr_fnd(13)  # Check for a CR character
            if n >= 0:
                rx_buffer = rx_buffer[:n]  # slice-off \r\n at end

                if my_debug:
                    print("End-of-text code received.")
                    print("We go to process/print the received GPS datagram.")
                nr_msg_items = split_msg() # split rx_buffer into list of messages: msg_lst
                if not my_debug:
                    print(TAG+"Msg nr: {}, ID: {}, characters rcvd from ck_uart() is: {}, contents (mod): \n\"{}\"".format(msg_nr, ID_s, chrs_rcvd, msg_lst), file=sys.stderr)
                if rx_err_cnt > 0:
                    print("nr of rx misses: ", rx_err_cnt)
                if rx_unicode_err_cnt > 0:
                    print("nr of rx unicode errors: ", rx_unicode_err_cnt)
                if rx_serexcept_err_cnt > 0:
                    print("nr of serial.exception errors: ", rx_serexcept_err_cnt)
                if my_debug:
                    print("loop(): ID_s = {}; nr_msg_items = {}".format(ID_s, nr_msg_items), end="\n")

                if nr_msg_items == 12:
                    gs = ck_gs()
                    print(TAG+"GS = ",gs)
                    if ac_is_parked():
                        print(TAG+"Aircraft is parked")
                        ribbon.hub_clear()
                        hub.flip()
                    else:
                        if gs > 0.2:  # keep margin. Sometimes while parked the gs can be 0.1
                            if gs <= 30.0:
                                print(TAG+"Aircraft is taxying")
                            t_parked_init = 0.0
                            #led_toggle()  # we toggle elsewhere (when receiving msg in ck_uart() )
                            disp_crs()
                        else:
                            if t_parked_init == 0.0:
                                t_parked_init = ribbon.millis()
                else:
                    split_err = True
                if split_err == True:
                    print(TAG+"Error: spliting rx_buffer contents has failed. Exitting...", end="\n")
                    split_err = False
                # Do cleanup and resets
                rx_err_cnt = 0
                rx_unicode_err_cnt = 0
                rx_serexcept_err_cnt = 0
                chrs_rcvd = 0
                empty_buffer()
                #sleep(0.1)  # <============================== DELAY ===============================<
            if msg_nr >= max_lp_cnt:
                #lstop = True  # force end of sketch execution
                pass
            if lstop == True:
                # print("\n\nWe go to stop")
                if biLdIsOn:
                    led_toggle()
            else:
                if startup == -1:
                    print("Waiting for serial com line to become available...")
                    startup = 0  # switch off flag. Showing this text only once.
            if (currentMillis - previousMillis) >= led_interval:
                previousMillis = currentMillis
            print("End of loop {}".format(lp_cnt), end="\n")
            print("........................", end="\n")
            if msg_nr >= max_lp_cnt:
                msg_nr = 0
                lp_cnt = 0
        # End if (chrs_rcvd > 0)
        else:
            # No GPS sentences data received. Exit...
            pass
        if lstop == True:
            break
    # end-of while True

    sleep(0.5)  # <======================================= DELAY ==============================<
    return True
# End of setUp()


"""
ck_uart(void) -> nr_bytes
        This function attempt to read the uart. It filters isolated \x00 byte characters
        Parameters: None
        Return: nr_bytes

"""
def ck_uart():
    global rx_buffer, msg_nr, ID_s, GPRMC_cnt, loop_time, use_button_a, ribbon, rx_err_cnt, \
    rx_unicode_err_cnt, rx_serexcept_err_cnt, uart, my_debug, biLdIsOn
    TAG = "ck_uart(): "
    rx_buffer_s = ""
    if type(rx_buffer) is None:
        rx_buffer = ""
    nr_bytes = i = 0
    t_ID = 0
    delay_ms = 0.4
    rx_buffer_s = b''
    rx_buffer = ""
    c = None
    le = 0
    nTries = 0
    read_option = 0
    delay_ms = 0.2
    if my_debug:
        print("Entering ck_uart()", end="\n")
    if type(uart) is None:
        return 0
    gc.collect() # Run a garbage collection
    if my_debug:
        print("heap allocated:", gc.mem_alloc()) # print the nr of bytes of heap RAM that are allocated
        print("heap free:", gc.mem_free())  # print nr of bytes of available heap RAM, or -1 if not known
    while True:

        try:
            if read_option == 0:
                if uart.any():
                    rx_buffer_s = uart.readline()
                    if rx_buffer_s is None:
                        continue
                    if my_debug:
                        print(TAG+"type(rx_buffer_s): ", type(rx_buffer_s))
                    if type(rx_buffer_s) is bytes and len(rx_buffer_s) > 0:
                        rx_buffer += rx_buffer_s.decode('ascii')
                    if my_debug:
                        print("read_option 0. rx_buffer=", rx_buffer)
                    rx_buffer_s = b''
                else:
                    if use_button_a:
                        ribbon.ck_btns() # use the time to check for button press
            else:
                c = uart.read()
                if c:
                    if len(c) > 0:
                        print("c= ", c)
                        t = c.decode('ascii')
                        rx_buffer += t
      
            le = len(rx_buffer)
            nr_bytes = le
            #if ord(bytes(c)) == 10 and GPRMC_cnt >= 1: # check for LF (end-of-line char) and "GPRMC" already received
            n1 = rx_buffer[-15:].find('*') # find '*' in tail 10 characters. If not found, loop
            if n1 < 0: # no '*' in tail
                sleep(delay_ms)
                continue
            else:
                n2 = rx_buffer.find('$GPRMC')
                if n2 < 0:
                    sleep(delay_ms)
                    continue
                n2 = rx_buffer.find('$GPGGA')
                if n2 >= 0:  # found a GPGGA msg. Not wanted! It can happen that FSUIPC is set for broadcasting also GPGGA msgs.
                    continue

        except Exception as e:  # was: serial.serialutil.SerialException:
            if read_option == 0:
                read_option += 1
            elif read_option == 1:
                print(TAG+"Error: ", e)
                raise
                return 0  # we cannot continue, there is no uart object
            else:
                rx_serexcept_err_cnt+=1
                print(TAG+"error: ", e)
                #print("SerialException occurred. Continuing to receive", end="\n")
                empty_buffer()
                sleep(0.5)  # <<======== DELAY ================================
                continue
        loop_time = ticks_ms() # Record the time of this moment
        if use_button_a:
            ribbon.ck_btns()  # Poll button for button press
  
        try:
            if le > 0:
                if rx_buffer[-1] != '\n':  # check for LF as last character received. This made it work continuously OK !!!
                    sleep(0.2)  # <<======== DELAY ================================
                    continue
                #print(TAG+"msg rcvd: {}, type msg rcvd {}".format(rx_buffer, type(rx_buffer)), end='')
                ID_s = rx_buffer[1:6]
                if ID_s == 'GPRMC':
                    GPRMC_cnt += 1
                    rx_buffer = rx_buffer[7:] # Slice off characters following the ID_s as in b'$GPRMC,173745.00, will become: b'173745.00,
                    msg_nr += 1
                    led_toggle()
                    return len(rx_buffer)
                else:
                    continue
            else:
                rx_err_cnt += 1
                if use_button_a:
                    ribbon.ck_btns()  # Poll button for button press
                nTries += 1
                if nTries > 100:
                    print(TAG+"nothing received for {} tries. Check MS flight simulator 2020/FSUIPC running. Check wiring.".format(nTries), end='\n')
                    nTries = 0
                sleep(0.2)  # <<======== DELAY ================================
        except Exception as e:
            return 0

    # end-of while...

"""
split_msg(void) -> nr_msg_items
        This function attempts to split the comma-delimited rx_buffer items into a msg_lst list.
        It truncates the variation longitudinal indicator letter in the last item
        (a combination of magnetic variation longitudinal indicator and a 2-character checksum).
        It moves the check_sum characters into a 12th element to this list and
        Parameters: None
        Return: le2 (i.e.: len(msg_lst)). If it fails to split, a value of -1 will be returned

"""
def split_msg():
    global my_debug, msg_lst, rx_buffer, nr_msg_items, ID_s
    TAG="split_msg(): "
    retval = -1
    le = le2 = 0
    if my_debug:
        print(TAG+"contents of rx_buffer: {}".format(rx_buffer), end="\n")
    le = len(rx_buffer)
    retval = le  # return value in 1st instance
    """
    if my_debug:
        print(TAG+"tmp_s: {}".format(tmp_s), end="\n")
        print(TAG+"len(tmp_s): {}".format(len(tmp_s)), end="\n")
    """
    if le > 0:
        #msg_lst = tmp_s.split(",") # create a list of message parts split upon comma-delimiter
        msg_lst = rx_buffer.split(",") # create a list of message parts split upon comma-delimiter
        if my_debug:
            print(TAG+"msg_lst is: {}".format(msg_lst), end="\n")
        le2 = len(msg_lst)
        if my_debug:
            print(TAG+"nr of msg parts: {}. original contents msg_lst: {}".format(le2, msg_lst), end="\n")
        if ID_s == "GPRMC":
            s = msg_lst[le2-1][1:] # save the 2nd and 3rd character of the 11th item
            #                                  i.e.: the check_sum value
            msg_lst[le2-1] = msg_lst[le2-1][0:1] # from the 11th element slice off the 2nd and 3rd character,
            #                                    so we're left with only the variation longitude letter 'W' or 'E'
            #print(TAG+"s = {}, msg_lst[le2-1] = {}".format(s, msg_lst[le2-1]), end='\n')
            msg_lst.append(s) # append as 12th element the variation value (e.g.: "5")

            le2 = len(msg_lst)  # re-calculate the number of items in the appended list
            if my_debug:
                print(TAG+"new nr of msg parts: {}. modified contents msg_lst: {}".format(le, msg_lst), end="\n")
        if le2 > 0:
            retval = le2 # return value in 2nd instance
            nr_msg_items = le2
        else:
            print(TAG+"Error: unable to create splitted msg list from rx_buffer", end="\n")
    return retval

"""
bfr_fnd(ck_val) -> int
        This function searches the global rx_buffer
        for the value of parameter byt
        It returns -1 if not found
        or the index to the buffer where byt was found
        Parameters: byte type search value
        Return: integer
"""
def bfr_fnd(ck_val):
    global my_debug, rx_buffer
    TAG = "bfr_fnd(): "
    retval = -1
    if my_debug:
        print("bfr_fnd(): going to search rx_buffer for chr: {}".format(ck_val), end="\n")
    for _ in range(len(rx_buffer)):
        try:
            s_val = rx_buffer[_]
            if type(s_val) == str:
                o_val = ord(s_val)
            elif type(s_val) == int:
                o_val = s_val
        except UnicodeError:  # Happened a few times when receiving errounous code values e.g.: ..."\n\x0cIIIJIIKK\x95"...
        #                       One time caused by baudrate difference between this script and baudrate set in FSUIPC7
            print(TAG+"UnicodeError")
            break
        if o_val == 0:
            break  # no more valid chars in the rx_buffer
        if o_val == 13:
            t = "CR"
        elif o_val == 10:
            t = "LF"
        else:
            t = s_val
        if my_debug:
            print(TAG+"handling rx_buffer[{}], value: {}, (ord)value: \"{}\"".format(_, t, o_val), end="\n")
        if ck_val == o_val:
            if my_debug:
                print(TAG+"found chr: \"{}\" searched for".format(o_val), end="\n")
            retval = _
            break
    return retval

"""
led_toggle(void) -> void
        This function toggles the built-in led
        Sets/resets the global variable biLdIsOn
        Parameters: None
        Return: None
"""
def led_toggle():
    global biLdIsOn, led

    if biLdIsOn:
        led.set_rgb(0, 0, 0)
        biLdIsOn = False
    else:
        led.set_rgb(0, 50, 0)
        biLdIsOn = True       

"""
empty_buffer(void) -> void
        This function clears the rx_buffer
        Parameters: None
        Return: None
"""
def empty_buffer():
    global rx_buffer
    # c = b'\x00'
    rx_buffer = ""

def disp_crs():
    global GPRMC_cnt, loop_time, biLdIsOn
    TAG = "disp_crs(): "
    var_val = float(msg_lst[9])  # floating point value
    print(TAG+"var_val: {}".format(msg_lst[9]), end='\n')
    var_dir = msg_lst[10]
    trk_mag =  float(0.0)  # floating point value
    tmg_true = float(0.0)  # track made good true (floating point value)
    s_tmg_true = ""
    # $GPRMC message parts detail indexes:
    tmg =  7 # index to msg_lst. type float  (e.g.: 338.1)     < used

    if my_debug == 1:
        print(TAG+"GPRMC_cnt: {}".format(GPRMC_cnt), end="\n")

    if my_debug == 1:
        print(TAG, end='')
    le = len(msg_lst)
    if le > 0:
        """
        for _ in range(len(msg_lst)):
            print("{}, ".format(msg_lst[_]), end='')
        print()
        """
        tmg_true = float(msg_lst[tmg])

        #ribbon.hub_clear()
        if var_dir == "E":   # Correct for variation
            
            trk_mag = tmg_true - var_val  # NOTE !!! this is the opposite calculation than from magnetic +- variation to true heading
        elif var_dir == "W":
            trk_mag = tmg_true + var_val  # idem
        print(TAG+"track made good true: {}, var {} {}, track magnetcic: {}".format(tmg_true, var_val, var_dir, trk_mag), end='\n')
        tmg_true = round(tmg_true, 1)
        trk_mag = round(trk_mag, 1)
        s_tmg_true = str(tmg_true)
        s = "CRS " + s_tmg_true + " degs"
        #if my_debug == 1:
        print(TAG+s+" (T)")
            
        finish = ticks_ms()  # monotonic_ns()
        ms_duration = ((finish - loop_time) + 500000) / 1000000
        print("Duration between uart rx and Interstate 75 matrix presentation: {:5.2f} in mSecs".format(ms_duration), end="\n")
        
        ribbon.set_heading_fm_sim(tmg_true)
        ribbon.draw_heading()
        #ribbon.render(hub)
        hub.flip()
        if biLdIsOn:
            led_toggle()
    else:
        print(TAG+"msg_lst is empty!")

"""
main(void) -> void
        This is the start function
        Parameters: None
        Return: None
"""
def main():
    global my_debug, use_button_a, HEIGHT, ribbon, biLdIsOn, set_rgb, led
    lResult = True
    lStart = True
    lUpdate = False
    cnt = 0
    # ----------------------------------------------+
    # Settings for Interstate 75
    #if lStart:
    #    #print("Starting Interstate 75 62x32 clock")
    #    setDT()
    # Lines for use with Heading Ribbon
    if my_debug:
        print("main(): we passed here")
    # Create ribbon object
    ribbon = HdgRibbon()
    
    old_zoom_lvl = ribbon.get_zoom_lvl()
    time_last = ribbon.millis()
    ribbon_pos = HEIGHT / 2
    led.set_rgb(0, 0, 0)
    biLdIsOn = False
    if my_debug:
        print("main(): entering main loop")
    while True:
        # ----------------------------------------------+
        # Settings for Interstate 75
        t = ticks_ms()
        """
        ribbon.hub_clear()
        
        year, month, day, wd, hour, minute, second, _ = rtc.datetime()
        ymd = "{:04}   {:02} {:02}".format(year, month, day)
        hms = "{:02} {:02} {:02}".format(hour, minute, second)

        # Hour / Minute / Second
        ribbon.draw_number(1, 1, hms, fg=ribbon.shader_fg, bg=ribbon.shader_bg)
        #draw_number(1, 1, s_hms, fg=ribbon.shader_fg, bg=ribbon.shader_bg)

        # Year / Month / Day
        ribbon.draw_number(8, 20, ymd, fg=ribbon.shader_fg(), bg=ribbon.shader_bg(), digit_width=5, digit_height=7, digit_spacing=1)
        #draw_number(8, 20, s_ymd, fg=shader_fg, bg=shader_bg, digit_width=5, digit_height=7, digit_spacing=1)
        """
        # ----------------------------------------------+
        
        time_now = ribbon.millis()
        time_delta = time_now - time_last
        if use_button_a:
            lUpdate = ribbon.ck_btns()

        if lStart or lUpdate:
            lStart = False
            lUpdate = False
            ribbon.hub_clear()
            #ribbon.render(hub)
            hub.flip()

        time.sleep(0.001)
        time_last = time_now

        if cnt == 0:
            lResult = loop()
            cnt += 1  # Increase counter
            if lResult == False:
                if my_debug:
                    print("loop(): setup returned with: \"{}\" in line nr: {}".format(lResult))
            break  # Go into the perpetual loop

    # Perpetual loop
    cnt = 0
    while True:
        #print("in perpetual loop...")
        cnt += 1
        if cnt >= 100:
            cnt = 0
            led_toggle()

# Call the main function
if __name__ == '__main__':
    main()

# ----- END-OF-SKETCH -----