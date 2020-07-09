// Flora GPS + LED Pixel Code
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Flora GPS module
//    ------> http://adafruit.com/products/1059
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

//--------------------------------------------------|
//                    DISTANCE                      |
//--------------------------------------------------|
// Please enter the distance (in meters) from your  |
// destination that you want your LEDs to light up: |
#define DESTINATION_DISTANCE   55
//--------------------------------------------------|
// the number of points to be considered
// maximum size (for now) is 1301 for float, long; 2597 for int
// TODO: update this number for the amount of points in your data
#define BORDER_POINTS   392

// Navigation location
// TODO: update these values for the points in your data
float targetLat [BORDER_POINTS] = {40.375205, 40.375543, 40.375886, 40.376213, 40.376552000000004, 40.376858, 40.377023, 40.377174, 40.377319, 40.377474, 40.377776000000004, 40.378094, 40.37811, 40.37828, 40.378603999999996, 40.378929, 40.379257, 40.379334, 40.37936, 40.379521999999994, 40.379733, 40.379946000000004, 40.380179999999996, 40.380418, 40.380626, 40.380722, 40.380793, 40.380828, 40.38093, 40.381071, 40.381209000000005, 40.381336, 40.381454999999995, 40.381584000000004, 40.381793, 40.382018, 40.382218, 40.382394, 40.382565, 40.382658, 40.382751, 40.382844, 40.382978, 40.383159, 40.383295000000004, 40.383378, 40.383439, 40.383553000000006, 40.383722999999996, 40.383853, 40.383965, 40.384077000000005, 40.384201000000004, 40.384378999999996, 40.384639, 40.384993, 40.385534, 40.385934999999996, 40.386161, 40.38641, 40.386661, 40.386925, 40.387192999999996, 40.387469, 40.387741999999996, 40.388011, 40.388276, 40.388504, 40.388736, 40.388984, 40.389271, 40.38957, 40.389856, 40.390134, 40.390397, 40.39066, 40.390925, 40.390946, 40.390758, 40.390569, 40.39038, 40.390190999999994, 40.390001, 40.389811, 40.389621999999996, 40.389432, 40.389241, 40.38905, 40.388856, 40.388661, 40.388467, 40.388272, 40.388078, 40.387883, 40.387688, 40.387491, 40.38728, 40.387096, 40.386919, 40.386741, 40.386564, 40.386387, 40.386183, 40.385975, 40.385768, 40.38556, 40.385352000000005, 40.385144, 40.384935999999996, 40.384728, 40.38452, 40.384313, 40.383899, 40.383395, 40.382803, 40.382211, 40.381629, 40.381021000000004, 40.380414, 40.379768, 40.379089, 40.378382, 40.377669, 40.377005, 40.376385, 40.375772, 40.375119, 40.374565000000004, 40.373907, 40.373267, 40.372566, 40.371846000000005, 40.371138, 40.370487, 40.369865000000004, 40.369245, 40.368598, 40.367951, 40.367277, 40.366603000000005, 40.365926, 40.365241999999995, 40.364554, 40.363864, 40.363168, 40.362456, 40.361743, 40.361026, 40.360306, 40.359586, 40.358866, 40.358145, 40.357425, 40.356705, 40.355989, 40.355277, 40.354572999999995, 40.353868, 40.353163, 40.352451, 40.35174, 40.351031, 40.350321, 40.349612, 40.348902, 40.348193, 40.347483000000004, 40.346773999999996, 40.346064, 40.345387, 40.344807, 40.34435, 40.343896, 40.343188, 40.342893, 40.342652, 40.342503, 40.342414, 40.342358000000004, 40.342271999999994, 40.342205, 40.342133000000004, 40.342045, 40.341944, 40.341871000000005, 40.341767, 40.341649, 40.34153, 40.341351, 40.341035, 40.340595, 40.340121, 40.339622, 40.339051, 40.338457, 40.337849, 40.337241, 40.336607, 40.335964000000004, 40.335319, 40.334659, 40.33399, 40.333331, 40.332673, 40.332011, 40.331346, 40.330678000000006, 40.330011, 40.32935, 40.328692, 40.328029, 40.327385, 40.326768, 40.326183, 40.325606, 40.325140000000005, 40.324638, 40.324118, 40.323603999999996, 40.323089, 40.322565999999995, 40.322047999999995, 40.321540999999996, 40.321038, 40.320557, 40.32014, 40.319742, 40.319344, 40.318953, 40.318560999999995, 40.318157, 40.317746, 40.317328, 40.316909, 40.31647, 40.315983, 40.315406, 40.314803000000005, 40.314203000000006, 40.313601, 40.312996999999996, 40.312386, 40.311785, 40.311173, 40.310565000000004, 40.309991, 40.309473, 40.308972, 40.308523, 40.308066, 40.307611, 40.307142999999996, 40.306671, 40.306226, 40.305775, 40.305303, 40.304925, 40.305589000000005, 40.306248, 40.306908, 40.307567, 40.308229, 40.308902, 40.309574, 40.310247, 40.31092, 40.311592, 40.312264, 40.312938, 40.313612, 40.314287, 40.314961, 40.315630999999996, 40.316302, 40.316972, 40.317643, 40.318313, 40.318983, 40.319654, 40.320324, 40.320995, 40.321665, 40.322334999999995, 40.323006, 40.323676, 40.324346999999996, 40.325016999999995, 40.325683000000005, 40.326347999999996, 40.327014, 40.327678999999996, 40.328343, 40.329006, 40.329669, 40.330333, 40.330996, 40.331659, 40.332322999999995, 40.332986, 40.333649, 40.334313, 40.334976, 40.335644, 40.336304, 40.336971000000005, 40.337651, 40.338324, 40.338988, 40.339651, 40.340313, 40.340976, 40.341640000000005, 40.342307, 40.342975, 40.343643, 40.344309, 40.344975, 40.34564, 40.346306, 40.34697, 40.347633, 40.348296000000005, 40.348964, 40.349633000000004, 40.350297999999995, 40.35096, 40.351622, 40.352286, 40.352948, 40.353613, 40.354281, 40.354948, 40.355610999999996, 40.356274, 40.356944, 40.357611, 40.358277, 40.358944, 40.359609000000006, 40.360276, 40.36094, 40.361609, 40.362272999999995, 40.362937, 40.363601, 40.364265, 40.364929, 40.365592, 40.366256, 40.366919, 40.367581, 40.368247, 40.368913, 40.369571, 40.370235, 40.370906, 40.37158, 40.372254, 40.372927000000004, 40.373599, 40.374272, 40.374942};
float targetLon [BORDER_POINTS] = {-74.722061, -74.72122900000001, -74.7204, -74.71956, -74.71872900000001, -74.717878, -74.716961, -74.716039, -74.71511600000001, -74.714196, -74.71334399999999, -74.712499, -74.711566, -74.710666, -74.709824, -74.70898299999999, -74.708143, -74.707247, -74.706313, -74.705396, -74.704494, -74.703594, -74.702703, -74.701813, -74.700911, -74.699978, -74.69904, -74.698099, -74.69716700000001, -74.69624300000001, -74.695318, -74.69439, -74.693461, -74.692533, -74.691632, -74.690737, -74.689832, -74.688918, -74.68800300000001, -74.68706800000001, -74.686134, -74.685199, -74.684274, -74.683362, -74.682438, -74.681501, -74.680563, -74.679633, -74.678719, -74.67779200000001, -74.676861, -74.67593000000001, -74.675002, -74.67409, -74.673211, -74.672393, -74.67177099999999, -74.670997, -74.670103, -74.669218, -74.668335, -74.667458, -74.666583, -74.665712, -74.66484, -74.663965, -74.663089, -74.662195, -74.661302, -74.66041700000001, -74.659554, -74.65869599999999, -74.657831, -74.656961, -74.65608399999999, -74.655206, -74.65433, -74.653436, -74.652526, -74.651616, -74.650707, -74.64979699999999, -74.648888, -74.64797800000001, -74.64706899999999, -74.64616, -74.645251, -74.64434200000001, -74.643434, -74.642527, -74.64161899999999, -74.640711, -74.639804, -74.638896, -74.637989, -74.63708199999999, -74.63618100000001, -74.63526999999999, -74.634357, -74.633443, -74.632529, -74.63161600000001, -74.630712, -74.629809, -74.628907, -74.628004, -74.62710200000001, -74.626199, -74.625297, -74.624394, -74.623492, -74.62259, -74.62182299999999, -74.62116, -74.620622, -74.620085, -74.619529, -74.619023, -74.618515, -74.618099, -74.617793, -74.617612, -74.617485, -74.6178, -74.61828, -74.618774, -74.619155, -74.619755, -74.620136, -74.62056899999999, -74.620738, -74.620769, -74.620932, -74.62133100000001, -74.621806, -74.62228499999999, -74.622702, -74.62311600000001, -74.623452, -74.62378299999999, -74.624107, -74.624404, -74.624685, -74.624957, -74.6252, -74.625346, -74.62548100000001, -74.625576, -74.62562199999999, -74.62565699999999, -74.625692, -74.62572800000001, -74.625754, -74.625799, -74.62590300000001, -74.626044, -74.626244, -74.62644300000001, -74.626635, -74.626786, -74.626938, -74.627104, -74.62727, -74.627436, -74.627602, -74.627768, -74.627934, -74.6281, -74.628266, -74.62858, -74.62913, -74.629855, -74.630422, -74.63024300000001, -74.631033, -74.63192, -74.632841, -74.633774, -74.634713, -74.635648, -74.636586, -74.637523, -74.638458, -74.639391, -74.640327, -74.64126, -74.642189, -74.643118, -74.644029, -74.644871, -74.645615, -74.646326, -74.647002, -74.647576, -74.648108, -74.648615, -74.649121, -74.649565, -74.649992, -74.650413, -74.650787, -74.651139, -74.651521, -74.651904, -74.65227800000001, -74.65264, -74.652992, -74.65335, -74.653726, -74.65410899999999, -74.654477, -74.65489000000001, -74.655377, -74.65592600000001, -74.656488, -74.657206, -74.65788, -74.65853100000001, -74.659191, -74.65985, -74.660499, -74.661153, -74.661822, -74.662497, -74.663198, -74.663966, -74.664751, -74.66553499999999, -74.666326, -74.66711600000001, -74.667895, -74.66866999999999, -74.669436, -74.670202, -74.670948, -74.67164100000001, -74.67220400000001, -74.672719, -74.673242, -74.67375799999999, -74.674272, -74.67477199999999, -74.675292, -74.675785, -74.676292, -74.676859, -74.677511, -74.678186, -74.678923, -74.67965, -74.68038, -74.681096, -74.68180799999999, -74.682548, -74.68328100000001, -74.683993, -74.684686, -74.684945, -74.685325, -74.685705, -74.686084, -74.68645500000001, -74.68679399999999, -74.68713199999999, -74.687471, -74.68781, -74.688148, -74.688487, -74.688822, -74.689154, -74.689487, -74.689819, -74.69016500000001, -74.690511, -74.690857, -74.691202, -74.691548, -74.69189399999999, -74.692239, -74.692585, -74.692931, -74.693277, -74.69362199999999, -74.693968, -74.694314, -74.69466, -74.695006, -74.695365, -74.695727, -74.696089, -74.696451, -74.69681800000001, -74.697187, -74.69755500000001, -74.697923, -74.69829200000001, -74.69866, -74.699028, -74.699396, -74.699765, -74.700133, -74.700501, -74.700852, -74.70123199999999, -74.701584, -74.7019, -74.702235, -74.7026, -74.70297099999999, -74.703342, -74.703713, -74.70408, -74.704435, -74.704789, -74.70514399999999, -74.70550300000001, -74.705864, -74.706225, -74.706587, -74.706951, -74.70732199999999, -74.70769200000001, -74.708046, -74.70839699999999, -74.70875799999999, -74.709131, -74.70950400000001, -74.709868, -74.71024, -74.710603, -74.710959, -74.711315, -74.711685, -74.712052, -74.7124, -74.712756, -74.71311800000001, -74.713475, -74.713837, -74.71419499999999, -74.714559, -74.71490899999999, -74.715276, -74.715643, -74.71601, -74.71637700000001, -74.71674399999999, -74.717111, -74.717478, -74.717849, -74.71821899999999, -74.71858, -74.71894, -74.719324, -74.71969200000001, -74.720034, -74.72036899999999, -74.720704, -74.72104, -74.72138000000001, -74.721719, -74.72206700000001};

// Trip distance
float tripDistance;

boolean isStarted = false;

// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
// set this to number used in sewing
// TODO: adjust this number for the amount of Pixels you are using
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8);

int i;
int test;

uint8_t LED_Breathe_Table[]  = {   80,  87,  95, 103, 112, 121, 131, 141, 151, 161, 172, 182, 192, 202, 211, 220,
  228, 236, 242, 247, 251, 254, 255, 255, 254, 251, 247, 242, 236, 228, 220, 211,
  202, 192, 182, 172, 161, 151, 141, 131, 121, 112, 103,  95,  87,  80,  73,  66,
  60,  55,  50,  45,  41,  38,  34,  31,  28,  26,  24,  22,  20,  20,  20,  20,
  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  22,  24,  26,  28,
  31,  34,  38,  41,  45,  50,  55,  60,  66,  73 };


  #define BREATHE_TABLE_SIZE (sizeof(LED_Breathe_Table))
  #define BREATHE_CYCLE    5000      /*breathe cycle in milliseconds*/
  #define BREATHE_UPDATE    (BREATHE_CYCLE / BREATHE_TABLE_SIZE)
  uint32_t lastBreatheUpdate = 0;
  uint8_t breatheIndex = 0;

  void setup()
  {
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Serial.begin(115200);

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    delay(1000);
    // Ask for firmware version
    Serial1.println(PMTK_Q_RELEASE);

    // Start up the LED strip
    strip.begin();

    // Update the strip, to start they are all 'off'
    strip.show();
  }

  uint32_t timer = millis();

  void loop()                     // run over and over again
  {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
    if (c) Serial.print(c);

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    if (GPS.fix) {
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 2); Serial.print(GPS.lat);
      //Serial.print(", ");
      //Serial.print(GPS.longitude, 2); Serial.println(GPS.lon);

      float fLat = decimalDegrees(GPS.latitude, GPS.lat);
      float fLon = decimalDegrees(GPS.longitude, GPS.lon);

      if (!isStarted) {
        isStarted = true;
        tripDistance = (double)calc_dist(fLat, fLon, targetLat[0], targetLon[0]);
      }

      //Uncomment below if you want your Flora to navigate to a certain destination.  Then modify the headingDirection function.
      /*if ((calc_bearing(fLat, fLon, targetLat, targetLon) - GPS.angle) > 0) {
      headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-GPS.angle);
    }
    else {
    headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-GPS.angle+360);
  }*/

  int i;
  for (i = 0; i < BORDER_POINTS; i++) {
    headingDistance((double)calc_dist(fLat, fLon, targetLat[i], targetLon[i]));
  }
  //  headingDistance((double)calc_dist(fLat, fLon, targetLat, targetLon));
  //Serial.print("Distance Remaining:"); Serial.println((double)calc_dist(fLat, fLon, targetLat, targetLon));

}
//}

}

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1);
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc;
  }
  return bear_calc;
}

void headingDirection(float heading)
{
  //Use this part of the code to determine which way you need to go.
  //Remember: this is not the direction you are heading, it is the direction to the destination (north = forward).
  if ((heading > 348.75)||(heading < 11.25)) {
    Serial.println("  N");
    //Serial.println("Forward");
  }

  if ((heading >= 11.25)&&(heading < 33.75)) {
    Serial.println("NNE");
    //Serial.println("Go Right");
  }

  if ((heading >= 33.75)&&(heading < 56.25)) {
    Serial.println(" NE");
    //Serial.println("Go Right");
  }

  if ((heading >= 56.25)&&(heading < 78.75)) {
    Serial.println("ENE");
    //Serial.println("Go Right");
  }

  if ((heading >= 78.75)&&(heading < 101.25)) {
    Serial.println("  E");
    //Serial.println("Go Right");
  }

  if ((heading >= 101.25)&&(heading < 123.75)) {
    Serial.println("ESE");
    //Serial.println("Go Right");
  }

  if ((heading >= 123.75)&&(heading < 146.25)) {
    Serial.println(" SE");
    //Serial.println("Go Right");
  }

  if ((heading >= 146.25)&&(heading < 168.75)) {
    Serial.println("SSE");
    //Serial.println("Go Right");
  }

  if ((heading >= 168.75)&&(heading < 191.25)) {
    Serial.println("  S");
    //Serial.println("Turn Around");
  }

  if ((heading >= 191.25)&&(heading < 213.75)) {
    Serial.println("SSW");
    //Serial.println("Go Left");
  }

  if ((heading >= 213.75)&&(heading < 236.25)) {
    Serial.println(" SW");
    //Serial.println("Go Left");
  }

  if ((heading >= 236.25)&&(heading < 258.75)) {
    Serial.println("WSW");
    //Serial.println("Go Left");
  }

  if ((heading >= 258.75)&&(heading < 281.25)) {
    Serial.println("  W");
    //Serial.println("Go Left");
  }

  if ((heading >= 281.25)&&(heading < 303.75)) {
    Serial.println("WNW");
    //Serial.println("Go Left");
  }

  if ((heading >= 303.75)&&(heading < 326.25)) {
    Serial.println(" NW");
    //Serial.println("Go Left");
  }

  if ((heading >= 326.25)&&(heading < 348.75)) {
    Serial.println("NWN");
    //Serial.println("Go Left");
  }
}

void headingDistance(float fDist)
{
  //Use this part of the code to determine how far you are away from the destination.
  //The total trip distance (from where you started) is divided into five trip segments.
  Serial.println(fDist);
  if ((fDist >= DESTINATION_DISTANCE)) { // You are now within X meters of your destination.
    //Serial.println("Trip Distance: 1");
    //Serial.println("Arrived at destination!");
    int i;
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();   // write all the pixels out
  }


  if ((fDist < DESTINATION_DISTANCE)) { // You are now within X meters of your destination.
    //Serial.println("Trip Distance: 0");
    //Serial.println("Arrived at destination!");
    breath();
  }

}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }

  return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}

void breath()
{
  uniformBreathe(LED_Breathe_Table, BREATHE_TABLE_SIZE, BREATHE_UPDATE, 127, 127, 127);
}

void uniformBreathe(uint8_t* breatheTable, uint8_t breatheTableSize, uint16_t updatePeriod, uint16_t r, uint16_t g, uint16_t b)
{
  int i;

  uint8_t breatheBlu;

  if ((millis() - lastBreatheUpdate) > updatePeriod) {
    lastBreatheUpdate = millis();


    for (i=0; i < strip.numPixels(); i++) {
      breatheBlu = (b * breatheTable[breatheIndex]) / 256;
      strip.setPixelColor(i, 0, 0, breatheBlu);
    }
    strip.show();

    breatheIndex++;
    if (breatheIndex > breatheTableSize) {
      breatheIndex = 0;
    }
  }
}
