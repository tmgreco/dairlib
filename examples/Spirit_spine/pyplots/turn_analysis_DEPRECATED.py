#
# NOW IN /home/kodlab/dairlib/bazel-bin/bindings/pydairlib/lcm/dircon_trajectory_plotter.runfiles/dairlib/bindings/pydairlib/lcm/turning_analysis.py
from cmath import asin
import csv
from os import times
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import collections


class SpiritData:
  
  def __init__(self, filename, directory = "", toe_force_headers_missing = True ):

    self.csv_filename = ""
    self.is_success = False
    self.index_success = 9
    self.preamble_dict = {}
    self.headers_list = []
    self.header_indices_dict = {}
    self.data_dict = collections.defaultdict(list)

    try:
      self.csv_filename = directory + filename
      with open(self.csv_filename, mode ='r') as file:
        csv_file = csv.reader(file)
        preamble = csv_file.__next__()
        self.is_success = int(preamble[self.index_success])
        if(not self.is_success):
          raise Exception(filename + " did not succeed and so will not be read")
        self.process_csv_preamble(preamble)
        headers_list = csv_file.__next__()
        self.process_headers(headers_list)

        for data_row in csv_file:
          for header in self.header_indices_dict.keys():
            column_index = self.header_indices_dict[header]
            if (header.strip() in [ "Front L" ,"Back L" ,"Front R" ,"Back R" ] and toe_force_headers_missing):
              self.data_dict[header+"x"].append(self.get_float(data_row[column_index])) 
              self.data_dict[header+"y"].append(self.get_float(data_row[column_index+1])) 
              self.data_dict[header+"z"].append(self.get_float(data_row[column_index+2])) 
            else:
              self.data_dict[header].append(self.get_float(data_row[column_index])) 
      self.data_dict.default_factory = None
    except Exception as error:
      print(error)

  def process_csv_preamble(self, csv_row):
    for (ind, description) in enumerate(csv_row[::2]):
      self.preamble_dict[description] = csv_row[2*ind+1]

  def process_headers(self, headers):
    for (ind, header) in enumerate(headers):
      if( header ):
        print(header.strip())
        self.header_indices_dict[header.strip()] = ind

  def calculate_avg_power(self,with_smooth_relu = False):
    ind = 0
    joint_vels_list = []
    joint_torque_list = []
    Q_list = []
    while True:
      try:
        joint_vels_list.append( self.data_dict["dq" + str(ind)] )
        joint_torque_list.append( self.data_dict["f" + str(ind)] )

        if ind in [1,3,5,7]:
          Q_list.append( 0.561 )
        else:
          Q_list.append( 0.249 )
      except Exception as e:
        break
      ind +=1
    last_index = ind

    joint_vels = np.array(joint_vels_list)
    joint_torque = np.array(joint_torque_list)
    Q_vector = np.transpose(np.array(Q_list,ndmin=2))
    thermal_loss = np.multiply(Q_vector, np.power(joint_torque,2))
    mechanical_work = np.multiply(joint_torque, joint_vels)
    power_actual = mechanical_work + thermal_loss
    if (with_smooth_relu):
      relu_smooth_vectorize = np.vectorize(self.relu_smooth)
      power_positive = relu_smooth_vectorize(power_actual)
    else:
      power_positive = np.maximum(power_actual,0)
    # print(power_positive)

    #Integrate
    # print(np.shape(power_positive)[1])
    timesteps = np.diff(np.array(self.data_dict["Time"]))
    power_trapz = np.zeros([last_index,len(timesteps)])
    for index in range(np.shape(power_positive)[1]-1):
        power_trapz[:,index] = timesteps[index]*(power_positive[:,index] + power_positive[:,index+1])/2.0
    

    # print(power_trapz)
    # print(timesteps)
    return np.sum(power_trapz)/sum(timesteps)
  def get_float(self,input):
    if input:
      return float(input)
    else:
      return math.nan

  def relu_smooth(self, value, alpha = 4):
    # print(value)
    if (value>5):
      return value
    else:
      return 1/alpha * np.log(1+ np.exp( alpha * value )) 





s = SpiritData('turn1_ss_p3_s3.csv',"/home/kodlab/Desktop/morphology-study-data-linked/bounding_turn/twisting/data/")
# s1 = SpiritData('trot6_p4_s2.csv',"/home/kodlab/Desktop/")
# s2 = SpiritData('trot6_p4_s2_knot.csv',"/home/kodlab/Desktop/")
# print(s1.calculate_avg_power())
# print(s2.calculate_avg_power())
# print(s1.calculate_avg_power(with_smooth_relu=True))
# print(s2.calculate_avg_power(with_smooth_relu=True))
print(s.preamble_dict.keys())
print(s.data_dict.keys())
print(s.preamble_dict["Electrical power"])
print(s.calculate_avg_power())
print(s.calculate_avg_power(with_smooth_relu=True))
fig = plt.figure()
plt.plot(s.data_dict["x"],s.data_dict["y"])
plt.quiver(s.data_dict["x"],s.data_dict["y"],s.data_dict["vx"],s.data_dict["vy"] )
plt.show()

# def getData(directory, pre)



print(s.data_dict['q733'])
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
# for spine in ["rigid","twisting"]
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
      with open('/home/kodlab/Desktop/morphology-study-data-linked/bounding_turn/twisting/data/bounding_gait6_p%d_s%d.csv'%(p,s), mode ='r')as file:
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



