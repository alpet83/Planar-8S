
typedef unsigned char  BYTE;
typedef unsigned long  ULONG;
typedef unsigned int   UINT;
typedef unsigned short WORD;


class CHeaterAnalyze {
protected:
  int     line_count;  
  char    side[4];
 
  HardwareSerial *rx_port;
  
  char    line[1024];
  BYTE    data[128];
  int     count; // rel-to-data
  ULONG   dwtag; // first 4 byte cyclic load
    
public:
  
  bool    hex_output;

  CHeaterAnalyze (const char* rxtx, HardwareSerial *in_port);

  virtual void process_data();
  
  void dump();

  WORD word_ptr(int ofs, bool swap);
  
  ULONG dword_ptr(int ofs, bool swap);
  
  ULONG dword_ptr(int ofs);

}; // class CHeaterAnalyze

const   char* qfixmsg();
ULONG   swap_endian(ULONG v);  
// portability functions
void    msg_print (const char* str); 
void    msg_println (const char* str); 
