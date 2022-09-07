from cmath import asin
import csv
from os import times
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math



def GetMirrorError(pitches):
  return np.array(pitches) - (-np.array(pitches[::-1]))
# parameters=[]
# measurements=[]
# constraintLimits=[]

# parameters_for_min=[]
# measurements_min=[]
# abscissaIndex = 1
# ordinateIndex = 7
# constraintIndex = 3
# isSuccessIndex = 9
# for p in range(1,41+1):
#   min_measurement=1000000
#   for s in range(1,20+1):
#     # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
#     # with open('./data/trot_half/twisting_p5/trot_trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
#     # with open('./data/bounding_gait/rigid/bounding_gait4_p%d_s%d.csv'%(p,s), mode ='r')as file:
#     try:
#       with open('/home/kodlab/Desktop/morphology-study-data-linked/bounding_gait_2/rigid/data/bounding_gait6_p%d_s%d.csv'%(p,s), mode ='r')as file:
#         # reading the CSV file
#         csvFile = csv.reader(file)
#         line = csvFile.__next__()

#         parameterName = line[abscissaIndex-1]
#         measurementName = line[ordinateIndex-1]
#         constraintLimitName = line[constraintIndex-1]

#         parameter = float(line[abscissaIndex])
#         measurement = float(line[ordinateIndex])
#         constraintLimit = float(line[constraintIndex])
#         if (not int(line[isSuccessIndex])):
#           continue
#     except Exception as error:
#       print(error)
#       continue
#     if min_measurement>float(line[ordinateIndex]): 
#       min_measurement = measurement
#     measurements.append(measurement)
#     parameters.append(parameter)
#     constraintLimits.append(constraintLimit)
#   if min_measurement==1000000:
#     continue
#   measurements_min.append(min_measurement)
#   parameters_for_min.append(parameter)
      

# plt.figure()
# # plt.plot(displacements2,np.array(works)-np.array(works2),'b')
# plt.scatter(parameters,measurements,c=constraintLimits,label = "Successful Optimizations")
# plt.plot(parameters_for_min,measurements_min,c='r',label = "Minimum")
# plt.xlabel(parameterName)
# plt.ylabel(measurementName)
# plt.legend()
# plt.colorbar()
# plt.show()
# # plt.savefig("./figs/bounding_gait/electrical_work_5_perturbed_trajs")
#################################################################################################################################
    
"""
Calculate the pitch of every run and plot over one another 
plot max pitch vs pitch constraint
"""

parameters=[]
measurements=[]
constraintLimits=[]

parameters_for_min=[]
measurements_min=[]
experiment_index_min = []
abscissaIndex = 1
ordinateIndex = 7
constraintIndex = 3
isSuccessIndex = 9
pitchesAll = []
timesAll = []
constraintsAll = []
measurementsAll = []
numParams = 21
numPerturbations=10
params_unique=[]
for p in range(1,numParams+1):
  min_measurement=1000000
  pitchesTemp = []
  timesTemp = []
  constraintsTemp = []
  measurementsTemp = []
  experiment_index_min_temp = (p,-1)
  for s in range(1,numPerturbations+1):
    # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('./data/trot_half/twisting_p5/trot_trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
    # with open('./data/bounding_gait/rigid/bounding_gait4_p%d_s%d.csv'%(p,s), mode ='r')as file:

    headers = []
    quats = []
    pitches= []
    timesIn = []
    try:
      with open('/home/kodlab/Desktop/morphology-study-data-linked/bounding_gait_5/twisting/data/bounding_gait6_p%d_s%d.csv'%(p,s), mode ='r')as file:
        # reading the CSV file
        csvFile = csv.reader(file)
        # Read the first line of the CSV which contains the summary
        counter = 0
        line = csvFile.__next__()
        if (int(line[isSuccessIndex])): #Check the first line for failure, parse if success, continue if fail
          parameterName = line[abscissaIndex-1]
          measurementName = line[ordinateIndex-1]
          constraintLimitName = line[constraintIndex-1]

          parameter = float(line[abscissaIndex])
          measurement = float(line[ordinateIndex])
          constraintLimit = float(line[constraintIndex])
        else:
          print("run", p , s, "failed")
          continue
        # Now read the rest of the csv starting at the second line (index 1)
        for line in csvFile: 
          counter += 1
          if (counter == 1): #Ignore the first two header lines
            headers = line
            quat_indices = [headers.index("qw"),headers.index("qx"),headers.index("qy"),headers.index("qz")]
            continue
          # print(line)
          time = float(line[0])
          quat = [float(line[ele]) for ele in quat_indices]
          quats.append(quat)
          # print(quats)
          pitch = math.asin( min(max(2.0 *(quat[0] * quat[2] - quat[3] * quat[1]),-.999),.999))
          # print(pitch)
          pitches.append(pitch)
          timesIn.append(time)
        pitchesTemp.append(pitches)
        timesTemp.append(timesIn)
        constraintsTemp.append(constraintLimit)
        measurementsTemp.append(measurement)
    except Exception as error:
      print(error)
      continue
    if min_measurement > measurement: 
      min_measurement = measurement
      experiment_index_min_temp = (p,s)
    measurements.append(measurement)
    parameters.append(parameter)
    constraintLimits.append(constraintLimit)

  
  pitchesAll.append(pitchesTemp)
  timesAll.append(timesTemp)
  constraintsAll.append(constraintsTemp)
  measurementsAll.append(measurementsTemp)
  if min_measurement==1000000:
    continue
  measurements_min.append(min_measurement)
  parameters_for_min.append(parameter)
  experiment_index_min.append(experiment_index_min_temp)
measurements_clipped = np.array(measurements)
measurements_clipped[measurements_clipped>300] = 300
periodsAll = []
print(len(timesAll))
for tts in timesAll:
  pTemp = []  
  for tt in tts:
    pTemp.append(tt)
  print(len(pTemp))
  periodsAll.append(pTemp)
# print(periodsAll)
# input("PAUSE")
#################################################
print("PLOTTING ###################################################")
#################################################
plt.figure()
plt.title("Avg Power Cost Scatter Plot")
# plt.plot(displacements2,np.array(works)-np.array(works2),'b')
plt.scatter(parameters,measurements,c=constraintLimits,label = "Successful Optimizations")
plt.plot(parameters_for_min,measurements_min,c='r',label = "Minimum")
plt.xlabel(parameterName)
plt.ylabel(measurementName)
plt.legend()
plt.colorbar(label="Max Pitch Constraint")
#################################################


# print("test",len(pitchesAll))
actual_pitch_maxes = []
for parameter_pitches in pitchesAll:
  actual_pitch_maxes_temp = []
  for perturbation_pitches in parameter_pitches:
    actual_pitch_maxes_temp.append(np.array(perturbation_pitches).max())
  actual_pitch_maxes.append(actual_pitch_maxes_temp)
# print(parameters)
parametersExpanded = []
for (c_temp,param) in zip(constraintsAll, parameters):
#   print(param)
  p_temp = []
  for c in c_temp:
    p_temp.append(param)
  parametersExpanded.append(p_temp)

#####################################################

plt.figure()
plt.title("Actual Max Pitch vs Constraint\nScatter of all final optimizations")
constraintsStacked = np.array(sum(constraintsAll,[]))
actual_pitch_maxesStacked = np.array(sum(actual_pitch_maxes,[]))
plt.scatter(constraintsStacked,actual_pitch_maxesStacked,c=parameters)
plt.plot([0,1.5],[0,1.5],c="gray")
# plt.plot(parameters_for_min, measurements_min,c='r',label = "Minimum")
plt.xlabel("Max Pitch Constraint")
plt.ylabel("Actual Max Pitch")
# plt.legend()
plt.colorbar(label=parameterName)

###################################################

plt.figure()
plt.title("Actual Max Pitch vs Constraint\nScatter of all final optimizations")
constraintsStacked = np.array(sum(constraintsAll,[]))
actual_pitch_maxesStacked = np.array(sum(actual_pitch_maxes,[]))
plt.scatter(constraintsStacked,actual_pitch_maxesStacked,c=measurements_clipped)
plt.plot([0,1.5],[0,1.5],c="gray")
# plt.plot(parameters_for_min, measurements_min,c='r',label = "Minimum")
plt.xlabel("Max Pitch Constraint")
plt.ylabel("Actual Max Pitch")
# plt.legend()
# plt.colorbar(label=measurementName)

cmap = plt.get_cmap('viridis')

norm = mpl.colors.Normalize(vmin=0, vmax=300)

plt.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), label=measurementName)
#################################################
# print(len(pitchesAll))
# print(len(constraintsAll))
fig, axs = plt.subplots(6,2,figsize=(9, 15),sharex=True,sharey=True)


cmap = plt.get_cmap('viridis')
constraints_MAX  = np.max(np.max(constraintsAll))
constraints_MIN  = np.min(np.min(constraintsAll))
fig.suptitle("Pitch Magnitude for all Perturbations\nColor is max pitch constraint")


# Now remove axes[1,5] from the grouper for xaxis
axs[5,1].get_shared_x_axes().remove(axs[5,1])
axs[5,1].get_shared_y_axes().remove(axs[5,1])

# Create and assign new ticker
xticker = mpl.axis.Ticker()
axs[5,1].xaxis.major = xticker

# The new ticker needs new locator and formatters
xloc = mpl.ticker.AutoLocator()
xfmt = mpl.ticker.ScalarFormatter()

axs[5,1].xaxis.set_major_locator(xloc)
axs[5,1].xaxis.set_major_formatter(xfmt)
axs[5,1].axis('off')


# print(constraints_MAX)
# print(cmap(1))
for index in range(np.size(axs)-1):
  # Currently full range is 41 want to take them all but too many pltos so we are dividing it up into ten subplots with all the s
  [axs[index%6,index//6].plot(timesDum,pitchesDum,c=cmap(constraintDum/constraints_MAX)) for (timesDum, pitchesDum,constraintDum) in zip(timesAll[index * 2], pitchesAll[index * 2],constraintsAll[index*2])]
  axs[index%6,index//6].set_title("Speed "+ str(parameters_for_min[2*index]))

  if(index==5 or index==10):
    axs[index%6,index//6].set_xlabel("Time [s]")
  if ( not index//6):
    axs[index%6,index//6].set_ylabel("Pitch Magnitude")
  



norm = mpl.colors.Normalize(vmin=constraints_MIN, vmax=constraints_MAX)

fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), orientation='horizontal', label='Max Pitch Constraint')

fig.tight_layout(rect=[0, 0.03, 1, 0.95])
############################################
fig2, axs2 = plt.subplots(6,2,figsize=(9, 15),sharex=True,sharey=True)
# fig.subplots_adjust(bottom=0.5)
cmap = plt.get_cmap('viridis')
measurement_MAX  = 300#np.max(np.max(measurementsAll))
measurement_MIN  = np.min(np.min(measurementsAll))
fig2.suptitle("Pitch Magnitude for all Perturbations\nColor is avg power")


# Now remove axes[1,5] from the grouper for xaxis
axs2[5,1].get_shared_x_axes().remove(axs2[5,1])
axs2[5,1].get_shared_y_axes().remove(axs2[5,1])

# Create and assign new ticker
xticker = mpl.axis.Ticker()
axs2[5,1].xaxis.major = xticker

# The new ticker needs new locator and formatters
xloc = mpl.ticker.AutoLocator()
xfmt = mpl.ticker.ScalarFormatter()

axs2[5,1].xaxis.set_major_locator(xloc)
axs2[5,1].xaxis.set_major_formatter(xfmt)
axs2[5,1].axis('off')



# print(constraints_MAX)
# print(cmap(1))
for index in range(np.size(axs2)-1):
  [axs2[index%6,index//6].plot(timesDum,pitchesDum,c=cmap(measureDum/measurement_MAX)) for (timesDum, pitchesDum,measureDum) in zip(timesAll[index * 2], pitchesAll[index * 2],measurementsAll[index*2])]
  axs2[index%6,index//6].set_title("Speed "+ str(parameters_for_min[2*index]))
  axs2[index%6,index//6].grid(axis='x', color='0.95')
  if(index==5 or index==10):
    axs2[index%6,index//6].set_xlabel("Time [s]")
  if ( not index//6):
    axs2[index%6,index//6].set_ylabel("Pitch Magnitude")

norm = mpl.colors.Normalize(vmin=measurement_MIN, vmax=measurement_MAX)

fig2.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), orientation='horizontal', label='AveragePower')
fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
###########################################################
print("Minimum Runs:", experiment_index_min)
plt.show()



