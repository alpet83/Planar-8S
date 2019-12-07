
	CTransitRS232::CTransitRS232(const char *stag, int opin, int trig) {
		idle = 0;
		edges = 0;
		out_pin = opin;
		trig_level = trig;				
		strncpy(tag, stag, 7);		
	}

	// For loading from A* inputs without HW-filter
	void CTransitRS232::push_bit(int lvl) {
		int val = (lvl > trig_level);  // binnary presentation 
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
