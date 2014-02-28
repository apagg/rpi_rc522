#!/usr/bin/env python
# coding=utf8

import time, datetime

import reader

reader = reader.Reader()
reader.open(0, 0)

print 'run'
while True:
    
    
    reader.reset()    

    reader.antennaOff()

    reader.antennaOn()

    card_type = reader.reqidl_cmd()
    
    if card_type :
        print 'card type', card_type
        card_no = reader.anticoll_cmd()
        print 'card no', card_no
        if card_no :
            sel = reader.select_cmd(card_no)
            print 'sel', sel

    reader.antennaOff()

    time.sleep(0.5)
