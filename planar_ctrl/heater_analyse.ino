#ifndef ARDUINO_CPP
  extern char *msg;
#endif

  
  CHeaterAnalyze::CHeaterAnalyze (const char* rxtx, HardwareSerial *in_port) {
    strncpy(side, rxtx, 3);
    rx_port = in_port;
    hex_output = true;        
  }

  void CHeaterAnalyze::process_data() {
    // nothing due abstract
  }
  
  void CHeaterAnalyze::dump() {
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

      if (ch == 0x00) // line fill detections 
         eol = ( 64 == count && 0x4e02aa == ltag );
         
      if (ch == 0xAA) 
         eol = ( count >= 64 && 0x4e02aa == ltag ) || ( count >= 7 && 0x4e02aa != ltag );
      
      if (eol || ltb) {
        
        process_data();    
        
        if (count >= 7) 
        { 
           line_count ++; 
           sprintf(msg, "; bytes = %d ", count);
           strcat(line, msg);           
           if (hex_output) msg_println(line);
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
      
      // msg_print(msg);
      bc ++;
      if (bc >= 100) break;
    }
    
    if (count >= 100) {
      if (hex_output) msg_println(line);
      line[4] = 0;      
      line[64] = 0;
      count = 0;
      line_count ++;
    }   
    
  }  // dump

  WORD CHeaterAnalyze::word_ptr(int ofs, bool swap) {
     WORD *pw16 = (WORD*)&data[ofs & 127];
     if (swap)
       return (*pw16 >> 8) | (*pw16 << 8);
     else
       return *pw16;
  }
  
  ULONG CHeaterAnalyze::dword_ptr(int ofs, bool swap) {
     ULONG *pw32 = (ULONG*)&data[ofs & 127];
     ULONG v = *pw32; 
     if (swap) 
       return swap_endian(v);     
     return v;
  }
  
  ULONG CHeaterAnalyze::dword_ptr(int ofs) {
     return dword_ptr(ofs, true);    
  }
  
  const char* qfixmsg() { // replace single quote to double
     char *ch = msg;
     while (*ch) {
       if (0x27 == *ch) *ch = 0x22;
       ch++;
     }    
     return msg;    
  }

  ULONG   swap_endian(ULONG v)  {
    return (((v >> 24 ) & 0xff )  | ( ( v << 8 ) & 0xff0000 ) | ( ( v >> 8 ) & 0xff00 ) | ( ( v << 24 ) & 0xff000000));
  }

  void    msg_print (const char* str) {
    Serial.print (str);
  }
  void    msg_println (const char* str) {
    Serial.println (str);    
  }
  
