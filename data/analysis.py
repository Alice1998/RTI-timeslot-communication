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
    sensor_counter=[0 for i in range(8)]
    rssi_matrix=[[0 for i in range(8)] for j in range(8)]
    with open("15-51-51.740000_central.csv",'r') as f:
        reader=csv.reader(f)
        for row in reader:
            if len(row)==2 and len(row[1])>0:
                content=row[1].split('-')
                if content[0]=='C':
                    for i in range(1,len(content)):
                        index_rssi=content[i].split(' ')
                elif content[0]=='S':
                    time=get_timestamp(row[0])
                    if time-time_stamp<0.04:
                        data.append(time-time_stamp)
                    time_stamp=time
                    i_index=min(8,len(content))
                    break_flag=0
                    for i in range(1,i_index):
                        if break_flag==1:
                            break
                        rssi=content[i].split(' ')
                        index=min(8,len(rssi))
                        for j in range(index):
                            if 'C' in rssi[j] or 'S' in rssi[j] or '\n' in rssi[j]:
                                break_flag=1
                                break
                            if  rssi[j]=="":
                                continue
                            value=int(rssi[j],10)
                            # to be modified
                            if value!=0 and value<100 and value>10:
                                rssi_matrix[i-1][j]=int(rssi[j],10)
                    print(row)
                    for i in range(8):
                        for j in range(i+1,8):
                            if rssi_matrix[i][j]==0 or rssi_matrix[j][i]==0:
                                rssi_matrix[i][j]+=rssi_matrix[j][i]
                            else:
                                rssi_matrix[i][j]=(rssi_matrix[j][i]+rssi_matrix[i][j])//2
                            rssi_matrix[j][i]=rssi_matrix[i][j]
                        print(rssi_matrix[i])
                    print('\n')
                    # to be modified
                    rssi_matrix=[[0 for i in range(8)] for j in range(8)]
                    
                else:
                    print(content)


#analysis_sensor()
analysis_central()

plt.hist(data, bins=4000, normed=0, facecolor="blue", edgecolor="black", alpha=0.7)
# 显示横轴标签
plt.xlabel("区间")
# 显示纵轴标签
plt.ylabel("频数/频率")
# 显示图标题
plt.title("频数/频率分布直方图")
plt.show()