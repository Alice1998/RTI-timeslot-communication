#!/usr/bin/python
# -*-coding: utf-8 -*-

import serial
import threading
import binascii
from datetime import datetime
import struct
import csv

class SerialPort:
    def __init__(self, port, buand):
        self.port = serial.Serial(port, buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self):
        self.port.write('')

    def read_data(self):
        global is_exit
        global data_bytes
        while not is_exit:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.readline(count)[:-1]
                if len(rec_str)==0:
                    continue
                dt=datetime.now()
                nowtime_str=dt.strftime('%H-%M-%S.%f')
                loc_str=[nowtime_str[3:-3],rec_str]
                csv_writer.writerow(loc_str)
                print nowtime_str[3:-3],rec_str
                #print('当前数据接收总字节数：'+str(len(data_bytes))+' 本次接收字节数：'+str(len(rec_str)))
                #print str(datetime.now()),':',rec_str


serialPort = 'COM5'  # 串口 5
baudRate = 1000000  # 波特率 
is_exit=False
data_bytes=bytearray()

if __name__ == '__main__':
    #打开串口
    mSerial = SerialPort(serialPort, baudRate)

    #文件写入操作
    filename="central.csv"
    dt=datetime.now()
    nowtime_str=dt.strftime('%H-%M-%S.%f')  #时间
    filename='data/'+nowtime_str+'_'+filename
    out=open(filename,'a+')
    csv_writer=csv.writer(out)

    #开始数据读取线程
    t1 = threading.Thread(target=mSerial.read_data)
    t1.setDaemon(True)
    t1.start()
    while not is_exit:
        i=0
    '''
    i=0
    while not is_exit:
        #主线程:对读取的串口数据进行处理
        while i<len(data_bytes) and data_bytes[i]!=10:
            i+=1
        if i<len(data_bytes) and data_bytes[i]==10:
            dt=datetime.now()
            nowtime_str=dt.strftime('%H-%M-%S.%f')
            loc_str=[nowtime_str[3:-3],data_bytes[1:i-1]]
            csv_writer.writerow(loc_str)
            print nowtime_str[3:-3],data_bytes[1:i-1]
            data_bytes[:i+1]=b''
            i=0
    '''