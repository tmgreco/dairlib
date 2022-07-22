import csv
import numpy as np
import matplotlib.pyplot as plt

def get_whole_period(begins,ends,angles,angular_vs,q_12_torques,times):
    
    begins[0]+=[x+times[-1] for x in begins[1]]
    begins[1]+=[x+times[-1] for x in begins[0]]
    begins[2]+=[x+times[-1] for x in begins[3]]
    begins[3]+=[x+times[-1] for x in begins[2]]
    ends[0]+=[x+times[-1] for x in ends[1]]
    ends[1]+=[x+times[-1] for x in ends[0]]
    ends[2]+=[x+times[-1] for x in ends[3]]
    ends[3]+=[x+times[-1] for x in ends[2]]

    angles+=[-x for x in angles]
    angular_vs+=[-x for x in angular_vs]
    q_12_torques+=[-x for x in q_12_torques]
    times+=[x+times[-1] for x in times]
    return begins,ends,angles,angular_vs,q_12_torques,times

for j in range(14,15):
  with open('./data/trot_half/test%d.csv'%j, mode ='r')as file:
    # reading the CSV file
    csvFile = csv.reader(file)
    
    times=[]
    begins=[[],[],[],[]]
    ends=[[],[],[],[]]
    q_12s=[]
    q_12_dots=[]
    q_12_torques=[]

    last_contact=[False,False,False,False]
    counter=0
    for lines in csvFile:
        if counter<2:
            counter+=1
            continue
        
        times.append(float(lines[0]))
        for i in range(4):
            if float(lines[3+3*i])==0:
                if last_contact[i]==True: ends[i].append(float(lines[0]))
                last_contact[i]=False
            else:
                if last_contact[i]==False: begins[i].append(float(lines[0]))
                last_contact[i]=True
        q_12s.append(float(lines[20]))
        q_12_dots.append(float(lines[21]))
        q_12_torques.append(float(lines[22]))
    # Detact if the contact end at the end of the time
    for i in range(4):
        if last_contact[i]==True: ends[i].append(times[-1]+0.01)
    
    # Get the whole period
    begins,ends,q_12s,q_12_dots,q_12_torques,times=get_whole_period(begins,ends,q_12s,q_12_dots,q_12_torques,times)
    ## Plot
    legs=["LF","RF","LB","RB"]
    colors=['#d62728','#1f77b4','#1f77b4','#d62728']
    fig, (ax0, ax1) = plt.subplots(nrows=2, sharex=True)

    ax0.set_title('Joint 12')
    ax1.set_title('Footfall diagram')
    ax1.set_yticks([])

    # ax0.plot(times, q_12s,label='angle')
    # ax0.plot(times, q_12_dots,'r--',label='angular velocity')
    ax0.plot(times, q_12_torques,'g')
    for i in range(4):
        ax1.text(-0.1*times[-1],0.5*i,legs[i])
        ax1.hlines(y=0.5*i, xmin=begins[i], xmax=ends[i],linewidth=10, color=colors[i])
    plt.legend()
    # plt.show()

    plt.savefig("./figs/trot_half/footfall_diag%d"%j)
    plt.close()

