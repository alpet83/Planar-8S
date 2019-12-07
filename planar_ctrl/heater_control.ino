
void    CHeaterControl::set_mode(BYTE m) {
   if (_mode == m) return; 
   sprintf(msg, "------------ mode changed %d => %d -------------", _mode, m);
   msg_println(msg);
   _mode = m;
}

void    CHeaterControl::set_stage(BYTE s) {
   if (_stage == s) return;
   sprintf(msg, "------------ stage changed %d => %d -------------", _stage, s);
   msg_println(msg);
   _stage = s;
}

 
CHeaterControl::CHeaterControl(HardwareSerial *in_port, HardwareSerial *out_port): 
                    CHeaterAnalyze("RX", in_port) {
  strcpy(line, "RX: ");
  hex_output = false;
  info_rqnum = 0;	   
  tx_port = out_port;
}

BYTE    CHeaterControl::get_stage() {
  return _stage;
}
BYTE    CHeaterControl::get_mode() {
  return _mode; 
}

void CHeaterControl::process_data() {
  if (count >= 7 && 0xaa == data[0])
      decode_rx();
}

void CHeaterControl::decode_rx() {        
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
      msg_println(msg);    
      return; // tag of info-reply
   }
} 
void CHeaterControl::decode_info1() { // may be current settings of POWER only    
  _pulse = data[8];
  _power = data[10];
  sprintf(msg, "P8S: power settings %d, %d ", _pulse, _power);
  strcat(line, msg);
}

void CHeaterControl::decode_info2() {
   set_stage (data[5]);
   set_mode  (data[6]);     
   sprintf(msg, "P8S: stage:%d, mode:%d ", _stage, _mode);
}
void CHeaterControl::decode_diag() { // only 85 byte vectors
   char tmp[24];      
   if (0 == (line_count & 7) && hex_output)  {     
     // print offset header
     strcpy(msg, "OF: ");
     for (int i = 0; i < count; i ++) {
       sprintf(tmp, "%02x ", i);
       strcat(msg, tmp); 
     }  
     msg_println(msg);  
   }  
   
   // formatting JSON string output
   set_stage (data[5]);
   set_mode  (data[6]);
   sprintf(msg, "{ 'id':%d, 'stage':%d, 'mode':%d, ", line_count, (int)_stage, (int)_mode);     
   msg_print(qfixmsg());
   sprintf(msg, "'work_t':%lu, 'stage_t':%lu, 'mode_t':%lu, ", dword_ptr(7) >> 8, dword_ptr(0x0a) >> 8, dword_ptr(0x0d) >> 8); // used 24-bit counter
   msg_print(qfixmsg());
   sprintf(msg, "'target_rpm':%d, 'curr_rpm':%d, ", (int)data[0x10], (int)data[0x11]);
   msg_print(qfixmsg());          
   long val = word_ptr(0x12, true);
   if (val >= 49716L) {
      val -= 49716L;
      val = val * 60L / 3657L;        
   }
   sprintf(msg, "'fuel_pump':%ld, ", val); // impulse per minute
   msg_print(qfixmsg());
   float vf = 0.2f * (float)data[0x18];
   dtostrf(vf, 1, 1, tmp);
   sprintf(msg, "'candle_pwm':%s, ", tmp); // candle voltage can be calculated via batt_v and this value
   msg_print(qfixmsg());     
   
   // temperatures in K 
   sprintf(msg, "'flame_ind':%d, 'exhaust_temp':%d, ", word_ptr(0x1b, true) - 273, word_ptr(0x1d, true) - 273);  
   msg_print(qfixmsg());
    
   float vdf = 0.02f * (float)data[0x2c];     
   if (data[0x2c] > 0xd0) // over 15V?
      vdf = 0.02f * (float)(char)data[0x2c]; // signed value
   
   
   dtostrf(vdf + 10.9f, 1, 2, tmp);
   sprintf(msg, "'batt_v':%s }", tmp);
   msg_println(qfixmsg());    
}


void CHeaterControl::send_req(const BYTE *req, int cb) {
   for (int i = 0; i < cb; i ++)
     tx_port->write(req[i]);
   req_last = millis();  
}

void CHeaterControl::init_req() { 
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

void CHeaterControl::init_PU5_req() { // initialize as from pult PU-5
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


void CHeaterControl::info_req() {
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

ULONG CHeaterControl::req_elapsed() {
   return millis() - req_last;
}

void CHeaterControl::start_req() {
   const BYTE START_REQ[] = { 0xaa, 0x03, 0x00, 0x00, 0x01, 0x9c, 0xfd }; 
   while (req_elapsed() < 500) delay(50);
   send_req(START_REQ, 7);
}

void CHeaterControl::start_PU5_req() {
   const BYTE START_REQ[] = { 0xaa, 0x03, 0x06, 0x00, 0x01, 0xff, 0xff, 0x04, 0x1e, 0x02, 0x09, 0x1a, 0x4e };
   while (req_elapsed() < 500) delay(50);
   send_req(START_REQ, sizeof(START_REQ));
}

void CHeaterControl::stop_PU5_req() {
   const BYTE STOP_REQ[] = { 0xaa, 0x03, 0x00, 0x00, 0x03, 0x5d, 0x7c };
   while (req_elapsed() < 500) delay(50);
   send_req(STOP_REQ, sizeof(STOP_REQ));
}

void CHeaterControl::set_power(int pwr) {
   const BYTE LOW_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x0e, 0x02, 0x04, 0x1a, 0xbd };
   const BYTE MID_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x17, 0x02, 0x07, 0xdc, 0x2c };
   const BYTE HGH_PWR[] = { 0xaa, 0x03, 0x06, 0x00, 0x02, 0xff, 0xff, 0x04, 0x1e, 0x02, 0x09, 0x1a, 0x7d };
   if (pwr == curr_power) return;
   sprintf(msg, "#CONTROL: trying set power level %d", pwr);
   msg_println(msg);
   while (req_elapsed() < 500) delay(50);
   if (pwr <= 10) 
      send_req(LOW_PWR, sizeof(LOW_PWR));
   else
   if (pwr <= 50) 
      send_req(MID_PWR, sizeof(MID_PWR));
   else
   if (pwr <= 100) 
      send_req(HGH_PWR, sizeof(HGH_PWR));     
   curr_power = pwr;
}
