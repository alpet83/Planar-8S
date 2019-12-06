
// Planar 8DM-12S UART control. Version 0.1b
#include <DHT.h>
#include <DHT_U.h>

#include <Time.h>
#include <TimerThree.h>
#include <TimeLib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define TX_SOURCE A0
#define RX_SOURCE A1
#define BAUDRATE 2400
// #define DUMP_ALL

// not supported > 1 stop bits 

typedef unsigned char  BYTE;
typedef unsigned long  ULONG;
typedef unsigned int   UINT;
typedef unsigned short WORD;


int tx_high = 512;
int rx_high = 512;
int tx_trig = 0;
int rx_trig = 0;

char msg[512];

DHT sensor(2, DHT11);

// used from transit bits from analog pins, to hardware UART port RXD
class CTransitRS232 {
protected:
	int     out_pin;
	int     trig_level;
  int     count;    
	char    tag[8];

public:
	int     idle;
	int		  edges;
  int     last;

	CTransitRS232(const char *stag, int opin, int trig) {
		idle = 0;
		edges = 0;
		out_pin = opin;
		trig_level = trig;				
		strncpy(tag, stag, 7);		
	}

	// For loading from A* inputs without HW-filter
	void push_bit(int lvl) {
		int val = (lvl > 500);  // binnary presentation 
		if (last != val) {
			digitalWrite(out_pin, val);
			last = val;
			edges++;
		}

		if (val)
			idle++;
		else
			idle = 0;
	} // push_bit

};

class CHeaterAnalyze {
protected:
  int     line_count;  
  char    side[4];
 
  HardwareSerial *rx_port;
  
  char    line[1024];
  BYTE    data[128];
  int     count; // rel-to-data
  ULONG   dwtag; // first 4 byte cyclic load
  
  ULONG   swap_endian(ULONG v)
  {
    return (((v >> 24 ) & 0xff )  | ( ( v << 8 ) & 0xff0000 ) | ( ( v >> 8 ) & 0xff00 ) | ( ( v << 24 ) & 0xff000000));
  }
public:
  
  bool    hex_output;

  CHeaterAnalyze (const char* rxtx, HardwareSerial *in_port) {
    strncpy(side, rxtx, 3);
    rx_port = in_port;
    hex_output = true;        
  }

  virtual void process_data() {
    // nothing due abstract
  }
  
  void dump() {
    int bc = 0;
    
    while(rx_port->available() > 0) {      
      BYTE ch = rx_port->read();
      bool eol = false;
      bool ltb = false;
      ULONG ltag = dword_ptr(0, false);
      dwtag = (dwtag << 8) + ch;   
      if (0xaa02e400 == dwtag && count > 4) { // late break 
         count -= 4;
         ltb = true;         
         int pos = strlen(line) - 12; 
         if (pos > 0) line[pos] = 0;         
      }
      
      if (ch == 0xAA) // line fill detections 
         eol = ( count >= 85 && 0x4e02aa == ltag ) || ( count >= 7 && 0x4e02aa != ltag );
      
      if (eol || ltb) {
        
        process_data();    
        
        if (count >= 7) 
        { 
           line_count ++; 
           sprintf(msg, "; bytes = %d ", count);
           strcat(line, msg);           
           if (hex_output) Serial.println(line);
           line[4] = 0;                    
        }   
        
        sprintf(line, "%s: %02x ", side, ch);  // new line initiated
        count = 1;
        memset(data, 0, sizeof(data));        // prevent tag survive
        if (ltb) { 
           dwtag = swap_endian(dwtag);
           memcpy(&data[0], &dwtag, 4);
           count = 4;
        }   
        else
           data[0] = ch;
        
        
      }  // if eol
      else {
        sprintf(msg, "%02x ", ch);      
        strcat(line, msg);           
        data[count] = ch;
        count = min (count + 1, 127);
      }        
      
      // Serial.print(msg);
      bc ++;
      if (bc >= 100) break;
    }
    
    if (count >= 100) {
      if (hex_output) Serial.println(line);
      line[4] = 0;      
      line[64] = 0;
      count = 0;
      line_count ++;
    }   
    
  }  // dump

  WORD word_ptr(int ofs, bool swap) {
     WORD *pw16 = (WORD*)&data[ofs & 127];
     if (swap)
       return (*pw16 >> 8) | (*pw16 << 8);
     else
       return *pw16;
  }
  
  ULONG dword_ptr(int ofs, bool swap) {
     ULONG *pw32 = (ULONG*)&data[ofs & 127];
     ULONG v = *pw32; 
     if (swap) 
       return swap_endian(v);     
     return v;
  }
  
  ULONG dword_ptr(int ofs) {
     return dword_ptr(ofs, true);    
  }

}; // class CHeaterAnalyze

class CHeaterControl: 
  public CHeaterAnalyze {
protected:
	
		
	ULONG   req_last;
	BYTE    _mode;
	BYTE    _stage;
	BYTE    _pulse;
	BYTE    _power;
 
	HardwareSerial *tx_port;



   void    set_mode(BYTE m) {
     if (_mode == m) return; 
     sprintf(msg, "------------ mode changed %d => %d -------------", _mode, m);
     Serial.println(msg);
     _mode = m;
   }
   void    set_stage(BYTE s) {
     if (_stage == s) return;
     sprintf(msg, "------------ stage changed %d => %d -------------", _stage, s);
     Serial.println(msg);
     _stage = s;
   }

   
public:
   int     info_rqnum;

   CHeaterControl(HardwareSerial *in_port, HardwareSerial *out_port): CHeaterAnalyze("RX", in_port) {
	   strcpy(line, "RX: ");
	   hex_output = false;
	   info_rqnum = 0;	   
	   tx_port = out_port;
   }

   const char* qfixmsg() { // replace single quote to double
     char *ch = msg;
     while (*ch) {
       if (0x27 == *ch) *ch = 0x22;
       ch++;
     }    
     return msg;    
   }

  virtual void process_data() {
    if (count >= 7 && 0xaa == data[0])
        decode_rx();
  }
 
  void decode_rx() {        
     ULONG ltag = dword_ptr(0, false);
     if (strstr(line, "aa 03 00 00 1c 95 3d"))  
         strcat(line, "[Planar 8S init response]");
     else 
     if (strstr(line, "aa 04 01 00 11 7f 9d a5"))         
        strcat(line, "[Planar 8S ping response]");
     else
      
     if (strstr(line, "aa 04 06 00 02") && 13 == count)
       decode_info1();
     else
     if (strstr(line, "aa 04 0a 00 0f") && 17 == count)
       decode_info2();     
     else
     if (0x4e02aa == ltag && count >= 75) 
       decode_diag();     
     else  {
        sprintf(msg, "Ignored line tag 0x%lx", ltag);
        Serial.println(msg);    
        return; // tag of info-reply
     }
  } 
 void decode_info1() { // may be current settings of POWER only    
    _pulse = data[8];
    _power = data[10];
    sprintf(msg, "P8S: power settings %d, %d ", _pulse, _power);
    strcat(line, msg);
 }
 
 void decode_info2() {
     set_stage (data[5]);
     set_mode  (data[6]);     
     sprintf(msg, "P8S: stage:%d, mode:%d ", _stage, _mode);
 }
 void decode_diag() { // only 85 byte vectors
     char tmp[24];      
     if (0 == (line_count & 7) && hex_output)  {     
       // print offset header
       strcpy(msg, "OF: ");
       for (int i = 0; i < count; i ++) {
         sprintf(tmp, "%02x ", i);
         strcat(msg, tmp); 
       }  
       Serial.println(msg);  
     }  
     
     // formatting JSON string output
     set_stage (data[5]);
     set_mode  (data[6]);
     sprintf(msg, "{ 'id':%d, 'stage':%d, 'mode':%d, ", line_count, (int)_stage, (int)_mode);     
     Serial.print(qfixmsg());
     sprintf(msg, "'work_t':%lu, 'stage_t':%lu, 'mode_t':%lu, ", dword_ptr(7) >> 8, dword_ptr(0x0a) >> 8, dword_ptr(0x0d) >> 8); // used 24-bit counter
     Serial.print(qfixmsg());
     sprintf(msg, "'target_rpm':%d, 'curr_rpm':%d, ", (int)data[0x10], (int)data[0x11]);
     Serial.print(qfixmsg());          
     long val = word_ptr(0x12, true);
     if (val >= 49716L) {
        val -= 49716L;
        val = val * 60L / 3657L;        
     }
     sprintf(msg, "'fuel_pump':%ld, ", val); // impulse per minute
     Serial.print(qfixmsg());
     float vf = 0.2f * (float)data[0x18];
     dtostrf(vf, 1, 1, tmp);
     sprintf(msg, "'candle_pwm':%s, ", tmp); // candle voltage can be calculated via batt_v and this value
     Serial.print(qfixmsg());     
     
     // temperatures in K 
     sprintf(msg, "'flame_ind':%d, 'exhaust_temp':%d, ", word_ptr(0x1b, true) - 273, word_ptr(0x1d, true) - 273);  
     Serial.print(qfixmsg());
      
     float vdf = 0.02f * (float)data[0x2c];     
     if (data[0x2c] > 0xd0) // over 15V?
        vdf = 0.02f * (float)(char)data[0x2c]; // signed value
     
     
     dtostrf(vdf + 10.9f, 1, 2, tmp);
     sprintf(msg, "'batt_v':%s }", tmp);
     Serial.println(qfixmsg());    
  }
 

  void send_req(const BYTE *req, int cb) {
     for (int i = 0; i < cb; i ++)
       tx_port->write(req[i]);
     req_last = millis();  
  }

  void init_req() { 
    const BYTE IRR1[] = { 0xaa, 0x03, 0x01, 0x00, 0x07, 0x00, 0xdd, 0x5f };
    const BYTE IRR2[] = { 0xaa, 0x03, 0x00, 0x00, 0x04, 0x9f, 0x3d };
    const BYTE IRR3[] = { 0xaa, 0x03, 0x00, 0x00, 0x06, 0x5e, 0xbc };
    const BYTE IRR4[] = { 0xaa, 0x03, 0x01, 0x00, 0x07, 0x01, 0x1d, 0x9e };
    send_req(IRR1, 8);
    delay(250);
    send_req(IRR1, 7);
    delay(250);
    send_req(IRR2, 7);
    delay(250);
    send_req(IRR3, 7);
    delay(1000);
    send_req(IRR4, 8);    
  } 
  
  void init_PU5_req() { // initialize as from pult PU-5
    const BYTE IRR1[] = { 0xaa, 0x03, 0x00, 0x00, 0x1c, 0x95, 0x3d };
    const BYTE IRR2[] = { 0xaa, 0x03, 0x00, 0x00, 0x06, 0x5e, 0xbc };
    const BYTE IRR3[] = { 0xaa, 0x03, 0x00, 0x00, 0x23, 0x85, 0x7d };
    const BYTE IRR4[] = { 0xaa, 0x03, 0x00, 0x00, 0x1e, 0x54, 0xbc };
    const BYTE HPWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x1e, 0x02, 0x09, 0x1a, 0x7d }; // maximum performance select    
    for (int i = 0; i < 3; i++) {
      send_req(IRR1, 7);
      delay(1000);
    }
    send_req(IRR2, 7);
    delay(1000);
    send_req(IRR3, 7);    
    delay(1000);
    send_req(IRR4, 7);
    delay(1000);
    send_req(HPWR, sizeof(HPWR));
  } 

  
  void info_req() {
     const BYTE PING_REQ[]  = { 0xaa, 0x03, 0x01, 0x00, 0x11, 0x7f, 0x5d, 0x10 };
     const BYTE DIAG_REQ[]  = { 0xaa, 0x02, 0x00, 0x00, 0x00, 0xa0, 0x3d };  
     const BYTE INFO_REQ1[] = { 0xaa, 0x03, 0x00, 0x00, 0x02, 0x9d, 0xbd };  // power settings request?
     const BYTE INFO_REQ2[] = { 0xaa, 0x04, 0x01, 0x00, 0x11, 0x7f, 0xa5 };  // mode/stage/status request?
     switch (info_rqnum % 4) {
      case 0: 
         send_req(PING_REQ, 8);
         break;
      case 1: 
         send_req(INFO_REQ1, 7);
         break;
      case 2: 
         send_req(INFO_REQ2, 7);
         break;
      case 3:
         send_req(DIAG_REQ, 7);
         break;   
      
     };
     info_rqnum ++;
  }

  ULONG req_elapsed() {
     return millis() - req_last;
  }

  void start_req() {
     const BYTE START_REQ[] = { 0xaa, 0x03, 0x00, 0x00, 0x01, 0x9c, 0xfd }; 
     while (req_elapsed() < 500) delay(50);
     send_req(START_REQ, 7);
  }
  
  void start_PU5_req() {
     const BYTE START_REQ[] = { 0xaa, 0x03, 0x06, 0x00, 0x01, 0xff, 0xff, 0x04, 0x1e, 0x02, 0x09, 0x1a, 0x4e };
     while (req_elapsed() < 500) delay(50);
     send_req(START_REQ, sizeof(START_REQ));
  }
  
  void stop_PU5_req() {
     const BYTE STOP_REQ[] = { 0xaa, 0x03, 0x00, 0x00, 0x03, 0x5d, 0x7c };
     while (req_elapsed() < 500) delay(50);
     send_req(STOP_REQ, sizeof(STOP_REQ));
  }

  void set_power(int pwr) {
     const BYTE LOW_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x0e, 0x02, 0x04, 0x1a, 0xbd };
     const BYTE MID_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x17, 0x02, 0x07, 0xdc, 0x2c };
     const BYTE HGH_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x1e, 0x02, 0x09, 0x1a, 0x7d };
     while (req_elapsed() < 500) delay(50);
     if (pwr <= 10) 
        send_req(LOW_PWR, sizeof(LOW_PWR));
     else
     if (pwr <= 50) 
        send_req(MID_PWR, sizeof(MID_PWR));
     else
     if (pwr <= 100) 
        send_req(HGH_PWR, sizeof(HGH_PWR));     
  }
};


CTransitRS232 tx_transit("TX", 7, 500); 
CTransitRS232 rx_transit("RX", 8, 400);

CHeaterControl heater(&Serial2, &Serial1);


void setup() {
  sensor.begin();
  Serial.begin(19200);
  Serial1.begin(2400);
  Serial2.begin(2400);
  
  /* 
  // this need once for levels detection for analog source
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
    sprintf(msg, "TX [%d..%d], RX [%d..%d], target pause %lu ", tx_low, tx_high, rx_low, rx_high, pause_us);
  }
  //initialize the variables we're linked to */
  if(!Serial) 
      delay(100);
      
  Serial.println("#HELLO: Planar 8DM-12S Heater controller via UART."); 
  tx_trig = tx_high - 170;
  rx_trig = rx_high - 120;
  pinMode(7, OUTPUT); // shorted to RXD2
  pinMode(8, OUTPUT); // shorted to RXD1 
  // Timer3.initialize(pause_us); 
  // Timer3.attachInterrupt(timer_cb);
  heater.init_req();  
  Serial.setTimeout(1000);
}

int low_lvl = 0;
int hi_lvl = 0;
int tx_last = -1;
int rx_last = -1;
int tx_level = 0;
int  rx_level = 0;
UINT tx_edges = 0;
UINT rx_edges = 0;



void serialEvent() { // input handler
  String input = Serial.readStringUntil(13);
  input.trim();
  if (input.length() < 4) return;
  sprintf(msg, "Received command [%s]", input.c_str());
  Serial.println(msg);
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
  else{     
     Serial.println("#ERROR: Unsupported command");   
  } 
}


void loop() {
	delayMicroseconds(500);


#ifdef A2D_TRANSIT
	tx_level = analogRead(TX_SOURCE);  
	tx_transit.push_bit(tx_level);
	rx_level = analogRead(RX_SOURCE);   
	rx_transit.push_bit(rx_level);
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
    Serial.print(heater.qfixmsg());
    sprintf(msg, " 'local_hum':%s }", tmp);
    Serial.println(heater.qfixmsg());
	}  
}
  
