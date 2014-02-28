#!/usr/bin/env python
# coding=utf8

import spidev
import time, datetime


""" {{{ """

class PCD:
    """
    /////////////////////////////////////////////////////////////////////
    //MF522 command word
    /////////////////////////////////////////////////////////////////////
    """
    IDLE       = 0x00 #NO action; cancel the current command
    MEM        = 0x01
    AUTHENT    = 0x0E #authentication key
    RECEIVE    = 0x08 #receive data
    TRANSMIT   = 0x04 #Transmit Data
    TRANSCEIVE = 0x0C #Send and receive data
    RESETPHASE = 0x0F #Reset
    CALCCRC    = 0x03 #CRC calculation

class PICC:
    """
    /////////////////////////////////////////////////////////////////////
    //Mifare_One card command word
    /////////////////////////////////////////////////////////////////////
    """
    REQIDL    = 0x26 #find the antenna area does not enter hibernation
    REQALL    = 0x52 #find all the cards antenna area
    ANTICOLL1 = 0x93 #anti-collision
    ANTICOLL2 = 0x95 #Anti-collision
    AUTHENT1A = 0x60 #authentication key A
    AUTHENT1B = 0x61 #authentication key B
    READ      = 0x30 #Read Block
    WRITE     = 0xA0 #write block
    DECREMENT = 0xC0 #debit?
    INCREMENT = 0xC1 #recharge?
    RESTORE   = 0xC2 #transfer block data to the buffer
    TRANSFER  = 0xB0 #save the data in the buffer
    HALT      = 0x50 #Sleep

class MF522:
    """
    /////////////////////////////////////////////////////////////////////
    //MF522 register
    /////////////////////////////////////////////////////////////////////
    """
    # PAGE 0
    RFU00               = 0x00    
    CommandReg          = 0x01    
    ComIEnReg           = 0x02    
    DivlEnReg           = 0x03    
    ComIrqReg           = 0x04    
    DivIrqReg           = 0x05
    ErrorReg            = 0x06    
    Status1Reg          = 0x07    
    Status2Reg          = 0x08    
    FIFODataReg         = 0x09
    FIFOLevelReg        = 0x0A
    WaterLevelReg       = 0x0B
    ControlReg          = 0x0C
    BitFramingReg       = 0x0D
    CollReg             = 0x0E
    RFU0F               = 0x0F
      
    # PAGE 1
    RFU10               = 0x10
    ModeReg             = 0x11
    TxModeReg           = 0x12
    RxModeReg           = 0x13
    TxControlReg        = 0x14
    TxAutoReg           = 0x15
    TxSelReg            = 0x16
    RxSelReg            = 0x17
    RxThresholdReg      = 0x18
    DemodReg            = 0x19
    RFU1A               = 0x1A
    RFU1B               = 0x1B
    MifareReg           = 0x1C
    RFU1D               = 0x1D
    RFU1E               = 0x1E
    SerialSpeedReg      = 0x1F
     
    # PAGE 2
    RFU20               = 0x20  
    CRCResultRegM       = 0x21
    CRCResultRegL       = 0x22
    RFU23               = 0x23
    ModWidthReg         = 0x24
    RFU25               = 0x25
    RFCfgReg            = 0x26
    GsNReg              = 0x27
    CWGsCfgReg          = 0x28
    ModGsCfgReg         = 0x29
    TModeReg            = 0x2A
    TPrescalerReg       = 0x2B
    TReloadRegH         = 0x2C
    TReloadRegL         = 0x2D
    TCounterValueRegH   = 0x2E
    TCounterValueRegL   = 0x2F
       
    # PAGE 3
    RFU30               = 0x30
    TestSel1Reg         = 0x31
    TestSel2Reg         = 0x32
    TestPinEnReg        = 0x33
    TestPinValueReg     = 0x34
    TestBusReg          = 0x35
    AutoTestReg         = 0x36
    VersionReg          = 0x37
    AnalogTestReg       = 0x38
    TestDAC1Reg         = 0x39  
    TestDAC2Reg         = 0x3A   
    TestADCReg          = 0x3B   
    RFU3C               = 0x3C   
    RFU3D               = 0x3D   
    RFU3E               = 0x3E   
    RFU3F               = 0x3F

""" }}} """


class Reader :

    

    def __init__(self):
        self.spi = spidev.SpiDev()

	self.verbose = False
		
    def open(self, bus, device):
        self.spi.open(0,0)
        # spi.cshigh = True
        # spi.max_speed_hz = 488000

    def wr(self, addr, val):
        a = ( ( addr << 1 ) & 0x7e )
        v = [a, val]
        if self.verbose:
            print '>>>' , self.hexf(v)

        r = self.spi.xfer(v)
        if self.verbose:
            print '<<<' , self.hexf(r)

    def wr2(self, addr, vall):
        a = ( ( addr << 1 ) & 0x7e )
        v = [a] + vall
        if self.verbose:
            print '>>>' , self.hexf(v)

        r = self.spi.xfer(v)
        if self.verbose:
            print '<<<' , self.hexf(r)


    def rd(self, addr):
        a = ( ( ( addr<<1 ) & 0x7e ) | 0x80 )
        v = [a, 0x00]
        if self.verbose:
            print '>>>' , self.hexf(v)
    
        r = self.spi.xfer(v)
        if self.verbose:
            print '<<<' , self.hexf(r)
    
        d = r[1]
        return d
    
    def xfer(self, addr, vall):
        a = ( ( addr << 1 ) & 0x7e )
        b = [a] + vall
        if self.verbose:
            print '>>>' , self.hexf( b )
    
        r = self.spi.xfer(b)
        if self.verbose:
            print '<<<' , self.hexf([r])
    
        return r
    
    def clearBitMask(self, reg, mask):
        tmp = self.rd(reg)
        self.wr(reg, tmp & ~mask)
    
    def setBitMask(self, reg, mask):
        tmp = self.rd(reg)
        self.wr(reg, tmp | mask)
    
    
    def hexf(self, r):
        return [('%02x' % i ) for i in r]
    
    
    def reset(self):
		
        if self.verbose:
            print 'reset'
        self.wr(MF522.CommandReg, PCD.RESETPHASE)
    
        self.wr(MF522.ModeReg, 0x3d)
    
        self.wr(MF522.TReloadRegL, 0x1e)
        self.wr(MF522.TReloadRegH, 0x00)
    
        self.wr(MF522.TModeReg, 0x8d)
    
        self.wr(MF522.TPrescalerReg, 0x3e)
    
        self.wr(MF522.TxAutoReg, 0x40)
    
    def antennaOff(self):
        if self.verbose:
            print 'antenna off'
        self.clearBitMask(MF522.TxControlReg, 0x03)
    
    def antennaOn(self):
        if self.verbose:
            print 'antenna on'
        i = self.rd(MF522.TxControlReg)
        if(not(i & 0x03)) :
            if self.verbose:
                print 'set antenna on'
            self.setBitMask(MF522.TxControlReg, 0x03)
        # rd(MF522.TxControlReg)
    
    def configISOType(self):
		
        if self.verbose:
            print 'config type'
        self.clearBitMask(MF522.Status2Reg, 0x08)
        
        self.wr(MF522.ModeReg      , 0x3d)
        self.wr(MF522.RxSelReg     , 0x86)
        self.wr(MF522.RFCfgReg     , 0x7f)
        self.wr(MF522.TReloadRegL  , 0x1e)
        self.wr(MF522.TReloadRegH  , 0x00)
        self.wr(MF522.TModeReg     , 0x8d)
        self.wr(MF522.TPrescalerReg, 0x3e)
        
        self.antennaOn()
    
    def cmd(self, command, req):
        status = False
    
        n = 0x00
        irqEn = 0x00
        waitFor = 0x00
    
        if command == PCD.AUTHENT :
            irqEn = 0x12
            waitFor = 0x10
        elif command == PCD.TRANSCEIVE :
            irqEn = 0x77
            waitFor = 0x30
    
        self.wr(MF522.ComIEnReg,irqEn|0x80)
        self.clearBitMask(MF522.ComIrqReg, 0x80)
        self.wr(MF522.CommandReg, PCD.IDLE)
        self.setBitMask(MF522.FIFOLevelReg, 0x80)
        
        self.wr2(MF522.FIFODataReg, req)
        self.wr(MF522.CommandReg, command)
       
        if command == PCD.TRANSCEIVE :
            self.setBitMask(MF522.BitFramingReg, 0x80)  
    
        i = 100
        while True:
            n = self.rd(MF522.ComIrqReg)   
            i = i - 1
            if ( i > 0 ) and ( not ( n & 0x01 ) ) and ( not ( n & waitFor ) ) :
                continue
            else:
                break
        # print 'n', n
    
        e = self.rd(MF522.ErrorReg)
        # print 'e', e
    
        self.clearBitMask(MF522.BitFramingReg, 0x80)
    
        # print 'i', i
        if i > 0 :
            e = self.rd(MF522.ErrorReg)
            # print 'e', e
            if not ( e & 0x1b ) :
                status = True
    
                if n & irqEn & 0x01 :
                    status = [] # no tag
    
                elif command == PCD.TRANSCEIVE :
                    n = self.rd(MF522.FIFOLevelReg)
                    # print 'fifo level', n
                    lastBits = self.rd(MF522.ControlReg) & 0x07
                    
                    status = []
                    n = max(min(n, 18), 1)
                    for k in xrange(n) :
                        v = self.rd(MF522.FIFODataReg)
                        status.append(v)
    
            else:
                status = False
    
        self.setBitMask(MF522.ControlReg, 0x80)
        self.wr(MF522.CommandReg, PCD.IDLE)
    
        return status
    
    def reqidl_cmd(self):
        self.clearBitMask(MF522.Status2Reg, 0x08)
        self.wr(MF522.BitFramingReg, 0x07)
        self.setBitMask(MF522.TxControlReg, 0x03)
    
        status = self.cmd(PCD.TRANSCEIVE, [PICC.REQIDL])
    
        return status
    
    def anticoll_cmd(self):
        self.clearBitMask(MF522.Status2Reg,0x08)
        self.wr(MF522.BitFramingReg,0x00)
        self.clearBitMask(MF522.CollReg,0x80)
    
        status = self.cmd(PCD.TRANSCEIVE, [PICC.ANTICOLL1, 0x20])
    
        self.setBitMask(MF522.CollReg, 0x80)
    
        if len(status) == 5 :
            snr_check = 0
            for i in xrange(4):
                snr_check ^= status[i] 
            print 'snr_check', snr_check
            if snr_check != status[4] :
                # what's this ?
                status = False
            else :
                status = status[:4]
    
        return status
    
    def select_cmd(self, card_no):
    
        msg = [0] * 9
        msg[0] = PICC.ANTICOLL1
        msg[1] = 0x70
        msg[6] = 0x00
        for i in xrange(4):
            msg[i+2] = card_no[i]
            msg[6]  ^= card_no[i]
    
        msg[7], msg[8] = self.cal_crc(msg[:7])
    
        self.clearBitMask(MF522.Status2Reg, 0x08)
    
        status = self.cmd(PCD.TRANSCEIVE, msg)
    
        return status
    
    def cal_crc(self, data):
        self.clearBitMask(MF522.DivIrqReg, 0x04)
        self.wr(MF522.CommandReg, PCD.IDLE)
        self.setBitMask(MF522.FIFOLevelReg, 0x80)
    
        for v in data:
            self.wr(MF522.FIFODataReg, v)
    
        self.wr(MF522.CommandReg, PCD.CALCCRC)
    
        i = 0xff
        while True :
            n = self.rd(MF522.DivIrqReg)
            i = i - 1
            if ( i > 0 ) and (not ( n & 0x04 ) ):
                continue
            else :
                break
    
        l = self.rd(MF522.CRCResultRegL)
        m = self.rd(MF522.CRCResultRegM)
    
        return (l, m)

# def selftest():
#     ''' not working... '''
#     print 'start self test'
#     e = rd(MF522.CommandReg)
#     e = rd(MF522.ComIEnReg)
# 
#     time.sleep(5)
#     return
# 
#     wr(MF522.CommandReg, PCD.RESETPHASE)
#     e = rd(MF522.ErrorReg)
# 
# 
#     wr(MF522.CommandReg, PCD.MEM)
#     for i in xrange(25):
#         wr(MF522.FIFODataReg, 0x00)
#     
#     wr(MF522.AutoTestReg, 0x09)
#     wr(MF522.FIFODataReg, 0x00)
#     wr(MF522.CommandReg, PCD.CALCCRC)
#     time.sleep(1)
#     print 'result'
#     for i in xrange(64) :
#         n = rd(MF522.FIFODataReg)
# 
#     time.sleep(5)

if __name__ == '__main__' :

    reader = Reader()
    reader.open(0, 0)

    while True:
        print 'run'
        #time.sleep(1)
    
        """
        selftest()
        continue
        """
    
        """
        wr(MF522.CommandReg, 0x0f)
    
        e = rd(MF522.CommandReg)
        e = rd(MF522.ErrorReg)
        e = rd(MF522.ErrorReg)
        continue
        """
    
        reader.reset()    
        # time.sleep(1)
    
    
        reader.antennaOff()
        # time.sleep(1)
        reader.antennaOn()
        # time.sleep(1)
        #reader.configISOType()
        # time.sleep(1)
    
        card_type = reader.reqidl_cmd()
        # res = request_cmd(PICC.REQALL)
        print 'card type', card_type
        if card_type :
            card_no = reader.anticoll_cmd()
            print 'coll no', card_no
            if card_no :
                sel = reader.select_cmd(card_no)
                print 'sel', sel
        #time.sleep(0.1)
    
        reader.antennaOff()
    
        print 'next...'
        #time.sleep(0.5)

