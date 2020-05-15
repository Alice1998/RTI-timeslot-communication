# -*-coding: utf-8 -*-
#python3

import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

def get_timestamp(input):
    m_s=input.split('-')
    s_micro=m_s[1].split('.')
    return int(m_s[0],10)*60+int(s_micro[0],10)+int(s_micro[1],10)*0.001

def central_get_timestamp(input):
    h_m_s=input.split('-')
    s_micro=h_m_s[2].split('.')
    return int(h_m_s[0],10)*3600+int(h_m_s[1],10)*60+int(s_micro[0],10)+int(s_micro[1],10)*0.000001

data=[]
def analysis_sensor():
    global data
    time_stamp=0
    last_send_rsp=0
    #with open("09-38-41.306000_sensor1.csv",'r') as f:
    with open("09-38-37.931000_sensor2.csv",'r') as f:
        reader=csv.reader(f)
        for row in reader:
            if len(row)==2:
                if row[1]=="20 \n":
                    time_stamp=get_timestamp(row[0])
                    #print(time_stamp)
                elif len(row[1])==len("1 \n"):
                    index=int(row[1][0],10)
                    if index>0 and index<9:
                       time=get_timestamp(row[0])
                       if index!=1 and time-last_send_rsp<0.02:
                          data.append(time-last_send_rsp)
                       last_send_rsp=time
                elif row[1]=="50 \n":
                    time=round(get_timestamp(row[0])-time_stamp,3)
                    if time<0.01:
                        data.append(time)
                elif row[1]=="10 \n":
                    time=round(get_timestamp(row[0])-last_send_rsp,3)
                    #if time<0.025 and last_send_rsp!=0:
                    #    data.append(time)
    #print(round_counter)

def analysis_central():
    time_stamp=0
    with open("09-33-46.819000_central.csv",'r') as f:
        reader=csv.reader(f)
        for row in reader:
            if len(row)==2 and len(row[1])>0:
                if row[1][:3]=='"C ':
                    time=get_timestamp(row[0])
                    #print(time,time_stamp,time-time_stamp)
                    if time-time_stamp<0.1:
                        data.append(time-time_stamp)
                    time_stamp=time
analysis_sensor()
#analysis_central()

plt.hist(data, bins=5000, normed=0, facecolor="blue", edgecolor="black", alpha=0.7)
# 显示横轴标签
plt.xlabel("区间")
# 显示纵轴标签
plt.ylabel("频数/频率")
# 显示图标题
plt.title("频数/频率分布直方图")
plt.show()

