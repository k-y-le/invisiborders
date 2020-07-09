# Invisiborders: Wearable Politics

##### Created by [Kyle Barnes](https://www.kylebarn.es/) under mentorship of Professor Janet Vertesi.
##### Princeton Computer science independent work project - fall 2019
##### Presented as a WIP at [DIS2020](https://dis.acm.org/2020/)
##### Learn more on the [website](https://www.invisiborders.com/)
## How to use this code:

1. Download the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
2. Navigate to the Arduino Library Manager, and load the following libraries:
  - AdaFruit GPS Library
  - AdaFruit NeoPixel Library
3. Follow [this AdaFruit tutorial](https://learn.adafruit.com/flora-wearable-gps/) to make sure that your GPS is working.
4. Clone this code from GitHub and drag the `invisiborders.ino` file into your Arduino folder
5. Wherever there is a line with `TODO` in the comment, adjust the code for your border data.
  - You will need the latitude and longitude coordinates of points along your border. I suggest using the US Census [TIGER/LINE database](https://www.census.gov/cgi-bin/geo/shapefiles/index.php) and [QGIS](https://qgis.org/en/site/) in order to make this happen.
