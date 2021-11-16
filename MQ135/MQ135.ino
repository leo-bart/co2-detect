//calls DHT library to measure temperature and humidity. This is necessary to make measuremente compensation
//TODO implement temp and humidity compensations
#include <DHT.h>


/* slope of the logarithm response curve
From the datasheet of MQ135, the relationship between gas concentration and the ratio between
measured resistance R_s and a default resistance R_0 is approximatelly exponential
C = b.(R_s/R_0)^a

LOG_SLOPE is a for CO2
*/
#define LOG_SLOPE -2.9331
#define DHTPIN 2                // the digital pin where the DHT sensor is connected
#define DHTTYPE DHT11           // DHT sensor type
#define FACTORB 115.0758        // this is the default b constant for the exponential sensitivity curve
#define RES_L 1000.             // the R_L resistance value. May vary from sensor to sensor
#define RES_CLEAN 6750.         // sensor resistance at 100 ppm NH3 (considering the datasheet data) - measured after warm-up
#define CLEAN_AIR_RATIO 3.6     // R to R_0 ratio measured on clean air
#define STD_CO2_PPM_AIR 410.0   // default CO2 concentration on clean air. Check at co2.earth
#define PUSH_BUTTON 9           // pin which push button is attached to 
#define CO2_PIN A0              // pin which MQ135 is attached to


// creates the DHT object
DHT dht(DHTPIN, DHTTYPE);


float resCleanAir;

void setup(){
  resCleanAir = RES_CLEAN;
  
  pinMode(CO2_PIN, INPUT);
  pinMode(PUSH_BUTTON, INPUT);
  dht.begin();

  Serial.begin(9600);
}




void loop(){

  float Vo = analogRead(A0)/1023.;                   // measured voltage
  float resS = RES_L * (1. / Vo - 1.);               // appearent sensor resistance
  float ratioR = CLEAN_AIR_RATIO * resS / resCleanAir;
  float conc = STD_CO2_PPM_AIR + FACTORB * pow(ratioR , LOG_SLOPE);      // CO2 concentration

  // humidity
  float humi  = dht.readHumidity();
  // temperature in Celsius degrees
  float tempC = dht.readTemperature();


  // if button is pushed
  if (digitalRead(9) == HIGH){ 
    resCleanAir = resS;
  }


  
  Serial.print(conc);
  Serial.print("  |  ");
  Serial.print(resS);
  Serial.print("  |  ");
  Serial.print(humi);
  // Serial.print("%");
  Serial.print("  |  "); 
  Serial.println(tempC);
  // Serial.println(" °C");*/
    
  delay(1000);


}
