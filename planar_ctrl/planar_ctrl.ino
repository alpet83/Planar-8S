#define ARDUINO_CPP

#include <DHT.h>
#include <DHT_U.h>

#include <Time.h>
#include <TimerThree.h>
#include <TimeLib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "transit_rs232.hpp"
#include "heater_analyse.hpp"
#include "heater_control.hpp"

#define TX_SOURCE A0
#define RX_SOURCE A1
#define BAUDRATE 2400
// #define DUMP_ALL

// temperature levels for power control
int high_temp  = 22;
int low_temp   = 17;
int curr_power = 100;  

char msg[512];

DHT sensor(2, DHT11);

#ifdef A2D_TRANSIT
 CTransitRS232 tx_transit("TX", 7, 500); 
 CTransitRS232 rx_transit("RX", 8, 400);
#endif 
CHeaterControl heater(&Serial2, &Serial1);


void setup() {
  sensor.begin();
  /* 
  // this need once for levels detection for analog source
  int tx_high = 900;
  int rx_high = 900;
  int tx_low = 400;
  int rx_low = 280;
  
  for (int i = 0; i < 256; i++) {
    int tx = analogRead(A0);
    int rx = analogRead(A1);         
    tx_high = max(tx_high, tx);
    rx_high = max(rx_high, rx);    
    tx_low  = min(tx_low, tx);
    rx_low  = min(rx_low, rx);
    delay(1);    
  }
  tx_trig = tx_high - 170;
  rx_trig = rx_high - 120; 
  sprintf(msg, "TX [%d..%d], RX [%d..%d], target pause %lu ", tx_low, tx_high, rx_low, rx_high, pause_us);
  // */
  
#ifdef ARDUINO_CPP  
  Serial.begin(19200);
  Serial1.begin(2400);
  Serial2.begin(2400);
  
  if(!Serial) 
      delay(100);
  msg_println("#HELLO: Planar 8DM-12S Heater controller via UART."); 
  pinMode(7, OUTPUT); // shorted to RXD2
  pinMode(8, OUTPUT); // shorted to RXD1 
  // Timer3.initialize(pause_us); 
  // Timer3.attachInterrupt(timer_cb);
  Serial.setTimeout(1000);
#endif  
  heater.init_req();    
}

void serialEvent() { // input handler
  String input = Serial.readStringUntil(13);
  input.trim();
  if (input.length() < 4) return;
  sprintf(msg, "Received command [%s]", input.c_str());
  msg_println(msg);
  if (input.startsWith("HSTART_DIAG")) 
      heater.start_req();
  else    
  if (input.startsWith("HSTART_PU5"))
	  heater.start_PU5_req();
  else    
  if (input.startsWith("HSTOP_PU5"))
	  heater.stop_PU5_req();
  else    
  if (input.startsWith("HIGH_POWER"))
	  heater.set_power(100);
  else   
  if (input.startsWith("MID_POWER"))
	  heater.set_power(50);
  else   
  if (input.startsWith("LOW_POWER"))
	  heater.set_power(10);
  else
  // display commands  
  if (input.startsWith("hide_hex")) {
	 heater.hex_output = false;     
  }      
  else 
  if (input.startsWith("show_hex")) {
   heater.hex_output = true;     
  }  
  else 
  if (input.startsWith("set_temp_high")) {
     sscanf(input.c_str(), "set_temp_high %d", &high_temp);     
     sprintf(msg, " high temp level set to %dC", high_temp);
     msg_println(msg);
  } 
  else 
  if (input.startsWith("set_temp_low")) {
     sscanf(input.c_str(), "set_temp_low %d", &low_temp);     
     sprintf(msg, " low temp level set to %dC", low_temp);
     msg_println(msg);
  }    
  else{     
     msg_println("#ERROR: Unsupported command");   
  } 
}


void loop() {

#ifdef A2D_TRANSIT
	tx_level = analogRead(TX_SOURCE);  
	tx_transit.push_bit(tx_level);
	rx_level = analogRead(RX_SOURCE);   
	rx_transit.push_bit(rx_level);
  delayMicroseconds(5);
#else  
  delay(5);
#endif

	heater.dump();

	if (heater.req_elapsed() >= 1000) {
    char tmp[24];      
		heater.info_req();
    if (0 != heater.info_rqnum % 10) return;
    float h = sensor.readHumidity();
    float t = sensor.readTemperature();    
    if (isnan(t) || isnan(h)) return;
    dtostrf(t, 1, 1, tmp);  
    sprintf(msg, "{ 'local_temp':%s,", tmp);
    dtostrf(h, 1, 1, tmp);      
    msg_print(qfixmsg());
    sprintf(msg, " 'local_hum':%s }", tmp);
    msg_println(qfixmsg());
    BYTE st = heater.get_stage();
    if (0 == st || 4 == st) return;
    if (t <= low_temp)  heater.set_power(100);
    if (t >= high_temp) heater.set_power(10);
	}  
}
  
