"""
Basic wrapper for 2-channel Roboclaws.

This receives a serial port and optional mutex to transmit and recieve using.
"""

class RoboclawX2:
    
    def __init__(self, port, addr, lock=None):
        if not port.is_open:
            raise Exception("Serial Port is down!")
        
        self._port = port
        self._addr = addr
        self._lock = lock
        self._crc = 0
    
    # rough enum for command numbers 
    class CMD:
		M1FORWARD = 0
		M1BACKWARD = 1
		SETMINMB = 2
		SETMAXMB = 3
		M2FORWARD = 4
		M2BACKWARD = 5
		M17BIT = 6
		M27BIT = 7
		MIXEDFORWARD = 8
		MIXEDBACKWARD = 9
		MIXEDRIGHT = 10
		MIXEDLEFT = 11
		MIXEDFB = 12
		MIXEDLR = 13
		GETM1ENC = 16
		GETM2ENC = 17
		GETM1SPEED = 18
		GETM2SPEED = 19
		RESETENC = 20
		GETVERSION = 21
		SETM1ENCCOUNT = 22
		SETM2ENCCOUNT = 23
		GETMBATT = 24
		GETLBATT = 25
		SETMINLB = 26
		SETMAXLB = 27
		SETM1PID = 28
		SETM2PID = 29
		GETM1ISPEED = 30
		GETM2ISPEED = 31
		M1DUTY = 32
		M2DUTY = 33
		MIXEDDUTY = 34
		M1SPEED = 35
		M2SPEED = 36
		MIXEDSPEED = 37
		M1SPEEDACCEL = 38
		M2SPEEDACCEL = 39
		MIXEDSPEEDACCEL = 40
		M1SPEEDDIST = 41
		M2SPEEDDIST = 42
		MIXEDSPEEDDIST = 43
		M1SPEEDACCELDIST = 44
		M2SPEEDACCELDIST = 45
		MIXEDSPEEDACCELDIST = 46
		GETBUFFERS = 47
		GETPWMS = 48
		GETCURRENTS = 49
		MIXEDSPEED2ACCEL = 50
		MIXEDSPEED2ACCELDIST = 51
		M1DUTYACCEL = 52
		M2DUTYACCEL = 53
		MIXEDDUTYACCEL = 54
		READM1PID = 55
		READM2PID = 56
		SETMAINVOLTAGES = 57
		SETLOGICVOLTAGES = 58
		GETMINMAXMAINVOLTAGES = 59
		GETMINMAXLOGICVOLTAGES = 60
		SETM1POSPID = 61
		SETM2POSPID = 62
		READM1POSPID = 63
		READM2POSPID = 64
		M1SPEEDACCELDECCELPOS = 65
		M2SPEEDACCELDECCELPOS = 66
		MIXEDSPEEDACCELDECCELPOS = 67
		SETM1DEFAULTACCEL = 68
		SETM2DEFAULTACCEL = 69
		SETPINFUNCTIONS = 74
		GETPINFUNCTIONS = 75
		SETDEADBAND = 76
		GETDEADBAND = 77
		RESTOREDEFAULTS = 80
		GETTEMP = 82
		GETTEMP2 = 83
		GETERROR = 90
		GETENCODERMODE = 91
		SETM1ENCODERMODE = 92
		SETM2ENCODERMODE = 93
		WRITENVM = 94
		READNVM = 95
		SETCONFIG = 98
		GETCONFIG = 99
		SETM1MAXCURRENT = 133
		SETM2MAXCURRENT = 134
		GETM1MAXCURRENT = 135
		GETM2MAXCURRENT = 136
		SETPWMMODE = 148
		GETPWMMODE = 149
		READEEPROM = 252
		WRITEEEPROM = 253
		FLAGBOOTLOADER = 255

	def crc_clear(self):
		self._crc = 0
		return
		
	def crc_update(self,data):
		self._crc = self._crc ^ (data << 8)
		for bit in range(0, 8):
			if (self._crc&0x8000)  == 0x8000:
				self._crc = ((self._crc << 1) ^ 0x1021)
			else:
				self._crc = self._crc << 1
		return

    def _write_byte(self,val):
		self.crc_update(val&0xFF)
		self._port.write(chr(val&0xFF))

    def _send_command(self,cmd,*data_bytes,recv_bytes=0)
        crc = 0
        for i in range(self.tries):
            # write address/command
            self._write_byte(address)
            self._write_byte(command)
            # write any data bytes
            for val in data_bytes:
                self._write_byte(val)
            # write checksum
            self._write_byte(self._crc&0xFFFF)
            # check checksum
            val = self._read_byte()
            if len(val) and val[0]:
                return True
        return False

    def _read()


	def DutyM1(self,address,val):
		return self._writeS2(address,self.Cmd.M1DUTY,val)
        return self._send_command(self.CMD.M1DUTY,val)

	def DutyM2(self,address,val):
		return self._writeS2(address,self.Cmd.M2DUTY,val)

	def DutyM1M2(self,address,m1,m2):
		return self._writeS2S2(address,self.Cmd.MIXEDDUTY,m1,m2)

	def ReadCurrents(self,address):
		val = self._read4(address,self.Cmd.GETCURRENTS)
		if val[0]:
			cur1 = val[1]>>16
			cur2 = val[1]&0xFFFF
			if cur1&0x8000:
				cur1-=0x10000
			if cur2&0x8000:
				cur2-=0x10000
			return (1,cur1,cur2)
		return (0,0,0)

	def ReadTemp(self,address):
		return self._read2(address,self.Cmd.GETTEMP)

	def ReadTemp2(self,address):
		return self._read2(address,self.Cmd.GETTEMP2)

	def ReadError(self,address):
		return self._read4(address,self.Cmd.GETERROR)    