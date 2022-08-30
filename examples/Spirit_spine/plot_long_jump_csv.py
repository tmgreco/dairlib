import csv
import numpy as np
import matplotlib.pyplot as plt

displacements=[]
displacements2=[]
twisting_works=[]
twisting_works2=[]
rigid_works=[]
rigid_works2=[]
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
#           twisting_works.append(float(lines[3]))
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

for p in range(1,41):
  best_work=1000
  displacement=0
  for s in range(1,21):
    # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
    with open('./data/trot_half/twisting_p20/trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('/home/feng/Downloads/data/twisting/data/bounding_gait5_p%d_s%d.csv'%(p,s), mode ='r')as file:  
      csvFile = csv.reader(file)
      counter=0
      for lines in csvFile:
        if counter==0:
          displacement=float(lines[1])
          if best_work>float(lines[7]): best_work=float(lines[7])
          if s==1: work=float(lines[7])
        counter+=1
        continue
  twisting_works.append(work)
  displacements.append(displacement)
  twisting_works2.append(best_work)
      
for p in range(1,41):
  best_work=1000
  displacement=0
  for s in range(1,21):
    # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
    with open('./data/trot_half/rigid_p20/trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('./data/bounding_gait/rigid/bounding_gait4_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('/home/feng/Downloads/data/rigid/data/bounding_gait5_p%d_s%d.csv'%(p,s), mode ='r')as file:  
      # reading the CSV file
      csvFile = csv.reader(file)
      counter=0
      for lines in csvFile:
        if counter==0:
          displacement=float(lines[1])
          if best_work>float(lines[7]): best_work=float(lines[7])
          if s==1: work=float(lines[7])
        counter+=1
        continue
  rigid_works.append(work)
  rigid_works2.append(best_work)

plt.figure()
# plt.plot(displacements2,np.array(twisting_works)-np.array(twisting_works2),'b')
plt.plot(displacements,rigid_works2,'b',label="rigid")
plt.plot(displacements,twisting_works2,'r',label="twisting")
plt.legend()
# plt.show()
plt.savefig("./figs/trot_half/rigid_vs_twisting_p40_s20")
# plt.savefig("./figs/bounding_gait/electrical_work_20_perturbed_trajs")

    
    