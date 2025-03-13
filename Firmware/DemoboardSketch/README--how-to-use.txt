To build and run the Arduino sketch in this directory, you will need:

1. An Arduino SAMD M0+ based board, such as the Arduino Zero.

2. Arduino IDE software, version 1.8.5 or higher, to compile/upload the sketch to the board. Downloads available:

    for Windows  https://www.arduino.cc/download_handler.php?f=/arduino-1.8.5-windows.zip
    for Mac or Linux  https://www.arduino.cc/en/Main/Software

STEPS TO COMPILE AND UPLOAD THIS SKETCH TO YOUR BOARD

Run Arduino IDE and select the Tools -> Board menu. You should see an option for "Arduino/Genuino Zero (Native USB Port)" --- select it.

If you do NOT see Arduino/Genuino Zero listed, click instead on "Boards Manager" (which is the top item on the list), search for "Arduino SAMD Boards (32-bits ARM Cortex-M0+)", and install version 1.6.18 or higher. (Be careful not to choose "Arduino SAM (ARM Cortex-M3)", which is totally unrelated!). You may need to type in your Windows password to allow installing the Arduino Zero USB driver, which appears to be digitally signed by Arduino, LLC.

Now go to File -> Preferences. Set "Sketchbook location" to the parent of the directory containing this README file. If this README file is in "<something>\trunk\DemoboardSketch", then Sketchbook location needs to be "<something>\trunk".

Look for "Show verbose output during: ( ) compilation  ( ) upload", and check both boxes.

Then click "OK" to close the Preferences dialog.

Quit Arduino IDE to ensure all the settings are saved.

Open the DemoboardSketch.ino file in this directory by either double-clicking it in Windows Explorer or by relaunching Arduino IDE and going to File -> Open .

Plug in your Arduino Zero (or compatible SAMD M0+) board.

Go to Tools -> Port. Look for the COM port that says "(Arduino/Genuino Zero (Native USB Port))" and select it. (There should only be one. If more than one, then unplug all boards connected to your PC except the one you're using right now, and try again.)

On the upper left corner of the window under the File, Edit menubar, click on the "->" icon to compile and upload the sketch to the board.

If you get compile time errors, go to Tools -> Board -> Boards Manager, and be sure that version 1.6.18 or later of "Arduino SAMD Boards (32-bits ARM Cortex-M0+)" is installed. If not, upgrade.

--Frank Lee 4/30/2018. Thanks to Zack P. for testing these instructions on his computer.