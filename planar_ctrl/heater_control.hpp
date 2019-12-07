
class CHeaterControl: 
  public CHeaterAnalyze {
protected:
		
	ULONG   req_last;
	BYTE    _mode;
	BYTE    _stage;
	BYTE    _pulse;
	BYTE    _power;
 
	HardwareSerial *tx_port;



  void    set_mode(BYTE m);
  void    set_stage(BYTE s);
   
public:
  int     info_rqnum;

  CHeaterControl(HardwareSerial *in_port, HardwareSerial *out_port);

  virtual void process_data();

  BYTE  get_stage();
  BYTE  get_mode();   
  void  decode_rx(); 
  void  decode_info1(); 
  void  decode_info2();
  
  void  decode_diag();

  void  send_req(const BYTE *req, int cb);

  void  init_req();
  
  void  init_PU5_req();
  
  void  info_req();

  ULONG req_elapsed();

  void  start_req();  
  void  start_PU5_req();  
  void  stop_PU5_req();
  void  set_power(int pwr);
  
};
