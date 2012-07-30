#! /usr/bin/env python

import sys
import random
import unittest
import argparse
import logging
import time

import socket
import threading
import asynchat
import asyncore
import struct

from pysimavr.avr import Avr
from pysimavr.firmware import Firmware
from pysimavr.sim import ArduinoSim
from pysimavr.udp import Udp
from pysimavr.udpreader import UdpHandler
from pysimavr.udpreader import UdpRepeater
from pyavrutils.arduino import Arduino

log = logging.getLogger(__name__)

ELFNAME = None;

def calculateChecksum(data):
    sum = 0;
    for b in data:
        sum = sum + b;
        sum = sum & 0xFF
    sum = ~sum+1
    sum = sum & 0xFF
    return sum;



def stringToBytes(s):
    bytes = []
    for c in s:
        b = ord(c);
        bytes.append(b)
    return bytes;
    
def printBytes(bytes):
    for b in bytes:
        print hex(b),
    print

UDP_IP = "127.0.0.1"
UDP_PORT = 4321
TIMEOUT = 1.0

class UartBridge(asynchat.async_chat):
    """
    Handler of UDP pipe from AVR simulator. It provides basic network
    facilities. It shall be subclassed to implement specific behavior.
    """
    def __init__(self, ip=UDP_IP, port=UDP_PORT, timeout=TIMEOUT):
        asynchat.async_chat.__init__(self)
        self.set_terminator(None)
        self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)
        self.socket.setblocking(False)
        self.ac_out_buffer_size = 1
        self.ac_in_buffer_size = 1
        self.ip = ip
        self.port = port
        
    def start(self):
        self.socket.sendto('', (self.ip, self.port))
    
    def collect_incoming_data(self, data):
        self.ibuffer += data

    def terminate(self):
        self.close()

    def read(self):
        res = ''
        while (True):
            try:
                s = self.socket.recv(1024)
            except socket.error:
                break;
            res = res + s; 
        print "Read:", 
        printBytes(stringToBytes(res))
        return res 

    def found_terminator(self):
        print 'got data'
        pass
    
    def send_str(self, s):
        print "Sent:", 
        printBytes(stringToBytes(s))
        while (True):
            n = self.socket.sendto(s, (self.ip, self.port));
            if (n == len(s)): break;
            s = s[n:]

      

class TestBase(unittest.TestCase):

    def setUp(self):
        self.startAvr();
        pass

    def tearDown(self):
        self.uartBridge.terminate()        
        self.avr.terminate()
        pass

    def messageSanityCheck(self, bytes, commandId, expectedSize, first):
        '''
        Check that the response has correct size, message Id, checksum 
        @param bytes: list of bytes
        @param commandId: message identifier
        @param expectedSize: expected size of the response
        @param first: is is a first response  
        @raise exception: unittest.TestCase.assertEqual 
        @return: None
        '''
        if (expectedSize != None):
            self.assertEqual(len(bytes), expectedSize, "Message length is "+str(len(bytes)));
        self.assertEqual(bytes[0], 0x7f, "1st sync byte is "+str(bytes[0]));                             
        self.assertEqual(bytes[1], 0xef, "2nd sync byte is "+str(bytes[1]));                             
        if (first):                                                                                      
            self.assertEqual(bytes[2], commandId, "Message id "+str(bytes[2]));                               
        else:                                                                                            
            self.assertEqual(bytes[2], commandId | 0x80, "Message id "+str(bytes[2]));                               
        self.assertEqual(bytes[3], expectedSize-4, "Payload size is "+str(bytes[3]));                              
                                                                                                         
        bytesWithoutChecksum = bytes[:-1]                                                                
        csExpected = calculateChecksum(bytesWithoutChecksum)                                             
        csActual = bytes[4]                                                                              
        self.assertEqual(csActual, csExpected, "Checksum is "+hex(csActual)+" instead "+hex(csExpected));

    def startAvr(self):
        self.avr = Avr(mcu='atmega88',f_cpu=8000000)
        
        self.udp = Udp(self.avr)
        self.udp.connect()

        self.uartBridge = UartBridge()
        
        firmware = Firmware(ELFNAME)
        self.avr.load_firmware(firmware);

        self.uartBridge.start();
        self.avr.run()
        #self.time_passed();
        
    def time_passed(self, timeout=0.99):
        while self.avr.time_passed() < 0.1 * timeout:
            time.sleep(0.05)
    

    def sendCommand(self, command, payload):
        syncBytes = [0x7f, 0xef];
        command = syncBytes + command + [len(payload)+1] + payload;
        cs = calculateChecksum(command)
        command = command + [cs];
        
        s = str(bytearray(command))
        self.uartBridge.send_str(s);
        self.time_passed(0.05*len(s))

              
class TestUart(TestBase):

    def test_sendping(self):
        self.sendping(True);
        self.sendping(False);

    def test_getstatus(self):
        self.sendCommand([0x04], [])
        serial = self.uartBridge.read()
        bytes = stringToBytes(serial)
        self.messageSanityCheck(bytes, 0x3, 3, first)

    def test_getfirmwareversion(self):
        self.sendCommand([0x13], [])
        serial = self.uartBridge.read()
        bytes = stringToBytes(serial)
        self.messageSanityCheck(bytes, 0x3, 3, first)

    def test_getstatistics(self):
        self.sendCommand([0x14], [])
        serial = self.uartBridge.read()
        bytes = stringToBytes(serial)
        # size of the statistics is going to change frequently
        # I do not want to check the message size  
        self.messageSanityCheck(bytes, 0x3, None, first)
    
    def test_accessmemoryread8(self):
        self.sendCommand([0x15], [0xd7, 0x03, 0x80])
        serial = self.uartBridge.read()
        bytes = stringToBytes(serial)
        self.messageSanityCheck(bytes, 0x3, None, first)
    
    def sendping(self, first):
        self.sendCommand([0x03], [])
        serial = self.uartBridge.read()
        
        bytes = stringToBytes(serial)
        self.messageSanityCheck(bytes, 0x3, 5, first)
        
    def gettestaddress(self):
        TEST_ADDRESS = 0x8000 + 0x1000 - 2 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--elf', help='ELF filename', default=None, required=True)
    parser.add_argument('unittest_args', nargs='*')

    args = parser.parse_args()
    sys.argv[1:] = args.unittest_args
    
    while True:
        ELFNAME = args.elf;
        if (ELFNAME == None):
            optionsParser.print_help();
            break;
    
        unittest.main();

        break;

