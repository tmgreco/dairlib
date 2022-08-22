import csv
import numpy as np
import matplotlib.pyplot as plt

displacements=[]
displacements2=[]
works=[]
works2=[]
# for j in range(26,47):
#   with open('./data/long_jump/no_perturbation/test%d.csv'%j, mode ='r')as file:
#     # reading the CSV file
#     csvFile = csv.reader(file)
    
#     times=[]
#     friction_ratios=[[],[],[],[]]
#     positions=[[],[],[]]
#     counter=0
#     for lines in csvFile:
#       if counter<2:
#         if counter==0:
#           displacements.append(float(lines[1]))
#           works.append(float(lines[3]))
#         counter+=1
#         continue
      
#     #   times.append(float(lines[0]))
#     #   for i in range(4):
#     #     friction=np.sqrt(float(lines[1+3*i])**2+float(lines[2+3*i])**2)
#     #     if float(lines[3+3*i])!=0: friction_ratios[i].append(friction/float(lines[3+3*i]))
#     #     else: friction_ratios[i].append(0)
#     #   for i in range(3):
#     #     positions[i].append(float(lines[14+i]))
#     # plt.figure()
#     # plt.plot(times,friction_ratios[0],'b',label='LF')
#     # plt.plot(times,friction_ratios[1],'y',label='RF')
#     # plt.plot(times,friction_ratios[2],'r--',label='LB')
#     # plt.plot(times,friction_ratios[3],'g--',label='RB')
#     # plt.legend()
#     # plt.savefig("./figs/long jump/friction ratios%d"%j)
#     # plt.close()

#     # plt.figure()
#     # plt.plot(times,positions[0],'r',label='x')
#     # plt.plot(times,positions[1],'g',label='y')
#     # plt.plot(times,positions[2],'b',label='z')
#     # plt.legend()
#     # plt.savefig("./figs/long jump/positions%d"%j)
#     # plt.close()

for p in range(1,14):
  best_work=1000
  displacement=0
  for s in range(1,6):
    # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
    with open('./data/trot_half/twisting_p5/trot_trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
      # reading the CSV file
      csvFile = csv.reader(file)
      counter=0
      for lines in csvFile:
        if counter==0:
          displacement=float(lines[1])
          if best_work>float(lines[5]): best_work=float(lines[5])
          if s==1: work=float(lines[5])
        counter+=1
        continue
  works.append(work)
  displacements.append(displacement)
  works2.append(best_work)
      

plt.figure()
# plt.plot(displacements2,np.array(works)-np.array(works2),'b')
plt.plot(displacements,works,'b')
plt.plot(displacements,works2,'r')
# plt.show()
plt.savefig("./figs/bounding_gait/electrical_work_10_perturbed_trajs")

    
    