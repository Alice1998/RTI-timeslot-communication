# -*-coding: utf-8 -*-
#python3

import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

def get_timestamp(input):
    h_m_s=input.split('-')
    s_micro=h_m_s[2].split('.')
    return int(h_m_s[0],10)*3600+int(h_m_s[1],10)*60+int(s_micro[0],10)+int(s_micro[1],10)*0.000001

time_stamp=0
last_send_rsp=0
round_counter=[{},{},{}]
data=[]
with open("14-30-18.641000_sensor2.csv",'r') as f:
    reader=csv.reader(f)
    for row in reader:
        if len(row)==2:
            if row[1]=="[R]: 20 ":
                time_stamp=get_timestamp(row[0])
                #print(time_stamp)
            elif len(row[1])==len("[R]: 1 "):
                index=int(row[1][5],16)
                #if index==3 and time<0.1:
                #    data.append(time)
                if index>0 and index<4:
                    time=get_timestamp(row[0])
                    if round(time-last_send_rsp,3)<0.006 and index!=1:
                        data.append(round(time-last_send_rsp,3))
                    last_send_rsp=time
                '''
                if time in round_counter[index-1]:
                    round_counter[index-1][time]+=1
                else:
                    round_counter[index-1][time]=1
                '''
            elif row[1]=="[R]: 50 ":
                time=round(get_timestamp(row[0])-time_stamp,3)
                #if time<0.08:
                #    data.append(time)
#print(round_counter)

plt.hist(data, bins=5000, normed=0, facecolor="blue", edgecolor="black", alpha=0.7)
# 显示横轴标签
plt.xlabel("区间")
# 显示纵轴标签
plt.ylabel("频数/频率")
# 显示图标题
plt.title("频数/频率分布直方图")
plt.show()

