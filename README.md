# MSFS2020_FSUIPC_GPRC_to_i75_RGB_LED_panel
Display flown track to i75 RGB LED Matrix panel and use ground speed to control displayed data

Software:
See 'Example'

Used hardware / manuals:

a) i75 hub controller: ```https://shop.pimoroni.com/products/interstate-75```;

b) 64x23 RGB LED Matrix - 3mm pitch: ```https://thepihut.com/products/adafruit-64x32-rgb-led-matrix-3mm-pitch```;

b) Adafruit MCP2221A Breakout: ```https://shop.pimoroni.com/products/adafruit-mcp2221a-breakout-general-purpose-usb-to-gpio-adc-i2c-stemma-qt-qwiic```;

   Manual ```https://learn.adafruit.com/circuitpython-libraries-on-any-computer-with-mcp2221```;

c) adafruit bidirectional i2c isolator: ```https://www.adafruit.com/product/4903```, also available through: 
   ```https://shop.pimoroni.com/products/adafruit-iso1540-bidirectional-i2c-isolator-stemma-qt-qwiic```;

d) a netadapter, able to deliver 5V DC at max 5 Ampère (e.g.: ```https://thepihut.com/products/neopixel-power-brick-5v-5a-25w```).


This project uses micropython instead of circuitpython as described in the Manual listed under b) above.

Goals of this project:

To receive, filter and use certain elements of GPRMC GPS datagram data sent by an add-on called ```FSUIPC7``` to the ```Microsoft Flight Simulator 2020 (FS2020)```.
From the filtered GPRMC GPS type of datagram this project only uses the ```Track made good true``` and the ```groundspeed```. The track flown by the aircraft is displayed on the RGB LED Matrix panel, only when the groundspeed value exceeds a certain minimum value set in the micropython script. If the groundspeed is zero the aircraft is assumed to be halted or be parked. In that case the script will inhibit the display of the track value. As soon as the groundspeed exceeds a set limit the track flown value will be displayed onto the RGB LED Matrix panel.

To be able to achieve what has been described in this project one has to follow the steps described in Adafruit's manual for the MCP2221 (see b). Take care to have the following software items installed on your pc (as described in the ```setup``` section of the Adafruit's manual):
- python3;
- Pip3;
- install hidapi;
- install ```Build Tools for Visual Studio```;
- Install ```adafruit-blinka```
- Set the environment variable: ```BLINKA_MCP2221=1```. Howto do this in Microsoft Windows 10 or 11, follow the steps below and see the screenshot in ```Images```;
  Click on ```Start```; click on ```Definitions```; Click on ```system```; Click on ```About```; Click on ```Advanced system definitions```; Click on ```Environment variables```. In the box ```Variables for user <username>``` click on button ```New```, then in the next pop-up window in the box ```Name of the variable```fill in: ```MCP2221```. In the box ```value of the variable```fill in: ```1```; Click on button ```OK```; Close the windows that had been opened in these steps.
- Perform the checks: ```Check Platform was detected``` in Adafruit's manual.

USB-UART-to-I2C:
To use the MCP2221A's UART-to-I2C functionality work, I connected wires between the following MCP2221A pins:
- ```TX```pin to ```SCL```pin;
- ```RX```pin to ```SDA``` pin.

Data Indicator LED:
In two of the images you can see that I added an external (blue) LED. I connected this LED and a 330 ohm resistor in series between the ```+5V``` pin and the ```SCL``` pin of the i75 hub controller. The external LED blinks at moments that GPS GPRMC datagram messages are being received.

Aside from displaying just numerals on the panel I also wanted to display static texts on the panel, especially during the startup of the script. Later I maybe will add also static text like: ```aircraft parked```. Because I did not find an example to display static texts on these kind of panels, I used the following example as a starting point: ```pimoroni-pico/micropython/examples/interstate75/i75_64x64_scrolling_text.py```. This script I changed drastically. I also modified the font file: ```font8x12.py```.

For more info on the reason why I changed the font file see: ```How I dealt with inconsistent letter spacing using the font8x12.py font file with an i75 RGB LED Panel``` at: ```https://github.com/pimoroni/pimoroni-pico/issues/238```

I used the Thonny app to save, edit and test the two script files: ```main.py``` and ```Interstate75_GPRMC_64x32_matrix_code_v1.py``` and two font files: ```font8x12_v4.py```and ```font10x14.py``` to the i75 hub controller.

The i75 hub controller has three buttons. One of these buttons is called ```A```. This button I used to force the script to exit from running. Micropython looks at boot time for a file called ```main.py```. During the development of this project it happened to me that I had my script named ```main.py```, then there was some error that did not cause the script to crash but I was not able to stop it from running. Also programming a try: ... except KeyboardInterrupt: ... did not work while the script was inside function ck_uart() waiting for data to be received. By using the A button to call SystemExit() I was able to create a suitable backdoor to stop the script from running. At crucial points inside the script, e.g.: inside the ```ck_uart()``` function (which is called very frequently), I put a call to the function ```ck_btns()```. In the case the button A has been pressed the hub object function hub.stop() will be called, clearing the planel to black, then call the SystemExit() function, giving control back to the calling (operating system or for example the Thonny) app.

Disclamer:
This project has been tested and working on pc´s running MS Windows 10 Pro or Windows 11 Pro. The script  ```Interstate75_GPRMC_64x32_matrix_code_v1.py``` has not yet fully been reviewed and cleaned of remains of what the script originally was. I did not want to wait with publishing my project as a repo on Github. The script has some 'remains' that are not used. I am working on that.

Update 2022-07-25. The LED panel now shows stext as: "airplaine parked" (when groundspeed is zero) and "airplaine is taxying" (when groundspeed < 30 knots). See the images.


