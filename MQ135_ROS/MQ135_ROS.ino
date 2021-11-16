/** 
 *  This sketch uses MQ135 to read C02 concentration
 *  on an environment and throws the readigns to a ROS
 *  topic called co2_concentration
 *  
 */

//calls DHT library to measure temperature and humidity. This is necessary to make measuremente compensation
//TODO implement temp and humidity compensations
#include <DHT.h>
// calls rosserial_arduino library
#include <ros.h> 
// imports a 32-bit float message. Arduino usually supports only 16bit foat, though
#include <std_msgs/Float32.h>
// imports boolean message to activate the fan
#include <std_msgs/Bool.h>


/* slope of the logarithm response curve
From the datasheet of MQ135, the relationship between gas concentration and the ratio between
measured resistance R_s and a default resistance R_0 is approximatelly exponential
C = b.(R_s/R_0)^a

LOG_SLOPE is a for CO2
*/
#define LOG_SLOPE -2.9331
#define DHTPIN 2                // the digital pin where the DHT sensor is connected
#define DHTTYPE DHT11           // DHT sensor type
#define FAN 5                   // Pin t which the fan is connected
#define FACTORB 115.0758        // this is the default b constant for the exponential sensitivity curve
#define RES_L 1000.             // the R_L resistance value. May vary from sensor to sensor
#define RES_CLEAN 6850.         // sensor resistance at 100 ppm NH3 (considering the datasheet data) - measured after warm-up
#define CLEAN_AIR_RATIO 3.6     // R to R_0 ratio measured on clean air
#define STD_CO2_PPM_AIR 410.0   // default CO2 concentration on clean air. Check at co2.earth
#define PUSH_BUTTON 9           // pin which push button is attached to 
#define CO2_PIN A0              // pin which MQ135 is attached to


// creates the DHT object
// uncomment to enable
//DHT dht(DHTPIN, DHTTYPE);

// ROS node
ros::NodeHandle nh;

// container for the co2 concentration value
std_msgs::Float32 co2conc;

// instantiates the ROS Publisher. The name of the publisher is the ROS topic name
ros::Publisher co2pub("co2_concentration", &co2conc);

// sensor resistance on clean air
float resCleanAir;

/**
 * Callback function that executes when the subscriber receives a message
 */
void thefan(const std_msgs::Bool& msg){
  if (msg.data) {
    digitalWrite(FAN,HIGH);
  }
  else {
    digitalWrite(FAN,LOW);
  }
  
  //Serial.println(msg.data);
}


// instantiates ROS subscriber
ros::Subscriber<std_msgs::Bool> fanstate("turn_fan_on", &thefan);


/*
 * Arduino setup function 
 */
void setup(){
  // initializes ROS node
  nh.initNode();
  nh.advertise(co2pub);
  nh.subscribe(fanstate);

  //sets default resistence for clean air
  resCleanAir = RES_CLEAN;

  
  pinMode(CO2_PIN, INPUT);
  // pinMode(PUSH_BUTTON, INPUT);
  pinMode(FAN,OUTPUT);
  // dht.begin();

  // fan is off 
  digitalWrite(FAN,LOW);

}


/*
 * Arduino loop function
 */
void loop(){

  float Vo = analogRead(A0)/1023.;                   // measured voltage
  float resS = RES_L * (1. / Vo - 1.);               // appearent sensor resistance
  float ratioR = CLEAN_AIR_RATIO * resS / resCleanAir;
  float conc = STD_CO2_PPM_AIR + FACTORB * pow(ratioR , LOG_SLOPE);      // CO2 concentration

  // humidity
  // float humi  = dht.readHumidity();
  // temperature in Celsius degrees
  // float tempC = dht.readTemperature();


  // if button is pushed
  //if (digitalRead(9) == HIGH){ 
  //  resCleanAir = resS;
  //}

  // sets value for the data that will be broadcast to topic
  co2conc.data = conc;
  // publishes valye
  co2pub.publish( &co2conc );
  // update ROS node
  nh.spinOnce();

  // waits for 1 second to restart
  delay(1000);
}
