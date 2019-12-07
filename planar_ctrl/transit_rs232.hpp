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

	CTransitRS232(const char *stag, int opin, int trig); 
	// For loading from A* inputs without HW-filter
	void push_bit(int lvl); 
};
