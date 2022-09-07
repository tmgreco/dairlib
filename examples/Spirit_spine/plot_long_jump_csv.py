import csv
import numpy as np
import matplotlib.pyplot as plt

parameters=[]
measurements=[]
constraintLimits=[]

parameters_for_min=[]
measurements_min=[]
abscissaIndex = 1
ordinateIndex = 7
constraintIndex = 3
isSuccessIndex = 9
for p in range(1,41+1):
  min_measurement=1000000
  for s in range(1,20+1):
    # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('./data/trot_half/twisting_p5/trot_trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('./data/bounding_gait/rigid/bounding_gait4_p%d_s%d.csv'%(p,s), mode ='r')as file:
    try:
      with open('/home/kodlab/Desktop/morphology-study-data-linked/bounding_gait_2/rigid/data/bounding_gait6_p%d_s%d.csv'%(p,s), mode ='r')as file:
        # reading the CSV file
        csvFile = csv.reader(file)
        line = csvFile.__next__()

        parameterName = line[abscissaIndex-1]
        measurementName = line[ordinateIndex-1]
        constraintLimitName = line[constraintIndex-1]

        parameter = float(line[abscissaIndex])
        measurement = float(line[ordinateIndex])
        constraintLimit = float(line[constraintIndex])
        if (not int(line[isSuccessIndex])):
          continue
    except Exception as error:
      print(error)
      continue
    if min_measurement>float(line[ordinateIndex]): 
      min_measurement = measurement
    measurements.append(measurement)
    parameters.append(parameter)
    constraintLimits.append(constraintLimit)
  if min_measurement==1000000:
    continue
  measurements_min.append(min_measurement)
  parameters_for_min.append(parameter)
      

plt.figure()
# plt.plot(displacements2,np.array(works)-np.array(works2),'b')
plt.scatter(parameters,measurements,c=constraintLimits,label = "Successful Optimizations")
plt.plot(parameters_for_min,measurements_min,c='r',label = "Minimum")
plt.xlabel(parameterName)
plt.ylabel(measurementName)
plt.legend()
plt.colorbar()
plt.show()
# plt.savefig("./figs/bounding_gait/electrical_work_5_perturbed_trajs")

    
    
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