"""
Basic wrapper for 2-channel Roboclaws.

This receives a serial port and optional mutex to transmit and recieve using.
"""

from math import cos, sqrt, pi
import serial

class RoboclawX2Serial:
    
    def __init__(self, port):
        self._port = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1,
            interCharTimeout=0.01
            )
        if not self._port.is_open:
            raise Exception("Serial Port is down!")

        self._crc = 0
        self._tries = 3
    
    # rough enum for command numbers 
    class CMD():
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
        GETTEMP1 = 82
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

    def _crc_clear(self):
        self._crc = 0
        return
        
    def _crc_update(self,data):
        self._crc = self._crc ^ (data << 8)
        for _ in range(0, 8):
            if (self._crc&0x8000)  == 0x8000:
                self._crc = ((self._crc << 1) ^ 0x1021)
            else:
                self._crc = self._crc << 1
        #print(self._crc)
        return

    def _write_byte(self,val):
        try:
            self._port.write(chr(int(val)&0xFF))
        except Exception as err:
            #print(val)
            raise err

    def _read_byte(self):
        recv = self._port.read(1)
        if len(recv):
            val = ord(recv)
            #self._crc_update(val)
            return (1,val)
        else:
            return (0,0)

    def _send_command(self,addr, cmd, send=tuple([]), bytes_per_send=1, num_recv=0, bytes_per_recv=1):
        if type(send) is not tuple:
            raise Exception("send_command needs a tuple for send data!")
        # initialize CRC for write
        for i in range(self._tries):
            self._crc_clear()
            # Write address and update CRC
            self._write_byte(addr)
            self._crc_update(addr)
            # Write command and update CRC
            self._write_byte(cmd)
            self._crc_update(cmd)
            # decision based on read or write command:
            if len(send):
                # write command
                # write any data bytes
                for data in send:
                    #print(data)
                    for i in range(bytes_per_send):
                        val = data>>(8*(bytes_per_send-i-1))
                        #print(val&0xFF)
                        self._write_byte(val)
                        self._crc_update(val&0xFF)

                # write CRC
                self._write_byte(self._crc>>8)
                self._write_byte(self._crc)
                # read return byte
                recv = self._read_byte()
                # TODO check return byte is 0xFF
                if recv[0]:
                    return (True,)
                    
            else:
                # read command
                recv_data = [True]+[0]*(num_recv)
                for i in range(num_recv):
                    recv = 0
                    for j in range(bytes_per_recv):
                        # read byte and update CRC
                        _, recv_byte = self._read_byte()
                        #print(j,recv_byte, (recv_byte<<(8*(bytes_per_recv-j-1))))
                        self._crc_update(recv_byte)
                        # TODO error checking
                        # stitch bytes
                        recv = recv|(recv_byte<<(8*(bytes_per_recv-j-1)))
                    #print(recv)
                    # add stitched value to list
                    recv_data[i+1] = recv
                #print(recv_data)
                # read CRC
                recv_crc_1 = self._read_byte()
                recv_crc_2 = self._read_byte()
                # TODO CRC read error checking
                recv_crc = recv_crc_1[1]<<8 | recv_crc_2[1]
                # check CRC
                #print(recv_data,recv_crc,self._crc&0xFFFF)
                if self._crc&0xFFFF==recv_crc&0xFFFF:
                    recv_data[0] = True
                    return tuple(recv_data)

        return (False,)
                


    def DutyM1(self,address,val):
        return self._send_command(
            cmd=self.CMD.M1DUTY,
            addr=address,
            send=(int(val),),
            bytes_per_send=2
            )

    def DutyM2(self,address,val):
        return self._send_command(
            cmd=self.CMD.M2DUTY,
            addr=address,
            send=(int(val),),
            bytes_per_send=2
            )

    def DutyM1M2(self,address,m1,m2):
        return self._send_command(
            cmd=self.CMD.MIXEDDUTY,
            addr=address,
            send=(int(m1),int(m2)),
            bytes_per_send=2
            )

    def ReadCurrents(self,address):
        ret = self._send_command(
            cmd=self.CMD.GETCURRENTS,
            addr=address,
            num_recv=2,
            bytes_per_recv=2
            )
        #print(ret)
        if ret[0]:
            cur1 = ret[1]
            cur2 = ret[2]
            # TODO Investigate this
            # looks like dumb stuff for signed 16-bit overflow...?
            if cur1&0x8000:
                cur1-=0x10000
            if cur2&0x8000:
                cur2-=0x10000
            return (ret[0],float(cur1)/100.0,float(cur1)/100.0)
        return (ret[0],0,0)

    def ReadTemp1(self,address):
        err, temp = self._send_command(
            cmd=self.CMD.GETTEMP1,
            addr=address,
            num_recv=1,
            bytes_per_recv=2
            )
        return (err,float(temp)/10.0)

    def ReadTemp2(self,address):
        err, temp = self._send_command(
            cmd=self.CMD.GETTEMP2,
            addr=address,
            num_recv=1,
            bytes_per_recv=2
            )
        return (err,float(temp)/10.0)

    def ReadError(self,address):
        err, status = self._send_command(
            cmd=self.CMD.GETERROR,
            addr=address,
            num_recv=1,
            bytes_per_recv=1
            )
        # TODO error codes
        return (err,status)
