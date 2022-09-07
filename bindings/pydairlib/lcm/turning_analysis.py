import sys
import os
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from cmath import asin
import csv
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from pydairlib.lcm.spirit_data import SpiritData

def draw_turn_quiver(spirit, data_source = "knotpoints", quiver_type="", axis = [], *args, **kwargs):
  if(not axis):
    fig, axis = plt.subplots()
  data_x = []
  data_y = []
  data_vx = []
  data_vy = []
  if(data_source == "knotpoints"):
    for mode_index in range(spirit.dircon_traj.GetNumModes()):
        data_temp =  spirit.modes_state_knotpoints_data[mode_index]
        data_x.append(data_temp[spirit.map_state_name_to_index["base_x"],:])
        data_y.append(data_temp[spirit.map_state_name_to_index["base_y"],:])
        data_vx.append(data_temp[spirit.map_state_name_to_index["base_vx"],:])
        data_vy.append(data_temp[spirit.map_state_name_to_index["base_vy"],:])
        # axis.plot(data_temp[spirit.map_state_name_to_index["base_x"],:],data_temp[spirit.map_state_name_to_index["base_y"],:])
        # axis.quiver(data_temp[spirit.map_state_name_to_index["base_x"],:],data_temp[spirit.map_state_name_to_index["base_y"],:], data_temp[spirit.map_state_name_to_index["base_vx"],:],data_temp[spirit.map_state_name_to_index["base_vy"],:])
    # axis.axis('equal')
  elif(data_source == "csv"):
    data_x.append(spirit.csv_data_dict["x"])
    data_y.append(spirit.csv_data_dict["y"])
    data_vx.append(spirit.csv_data_dict["vx"])
    data_vy.append(spirit.csv_data_dict["vy"])
    # axis.plot(spirit.csv_data_dict["x"],spirit.csv_data_dict["y"])
    # axis.quiver(spirit.csv_data_dict["x"],spirit.csv_data_dict["y"],spirit.csv_data_dict["vx"],spirit.csv_data_dict["vy"] )
    # axis.axis('equal')
  elif(data_source == "traj"):
    data_x.append(spirit.state_samples[spirit.map_state_name_to_index["base_x"],:])
    data_y.append(spirit.state_samples[spirit.map_state_name_to_index["base_y"],:])
    data_vx.append(spirit.state_samples[spirit.map_state_name_to_index["base_vx"],:])
    data_vy.append(spirit.state_samples[spirit.map_state_name_to_index["base_vy"],:])
    # plt.plot(spirit.state_samples[spirit.map_state_name_to_index["base_x"],:],spirit.state_samples[spirit.map_state_name_to_index["base_y"],:])
    # plt.quiver(spirit.state_samples[spirit.map_state_name_to_index["base_x"],:],spirit.state_samples[spirit.map_state_name_to_index["base_y"],:],spirit.state_samples[spirit.map_state_name_to_index["base_vx"],:],spirit.state_samples[spirit.map_state_name_to_index["base_vy"],:])
    # plt.axis('equal')
  for (dat_x,dat_y,dat_vx,dat_vy) in zip(data_x,data_y,data_vx,data_vy):
    axis.plot(dat_x,dat_y, *args, **kwargs)
    if(quiver_type=="ends"):
      axis.quiver([dat_x[0],dat_x[-1]],[dat_y[0],dat_y[-1]],[dat_vx[0],dat_vx[-1]],[dat_vy[0],dat_vy[-1]])
    elif(quiver_type=="all"):
      axis.quiver(dat_x,dat_y,dat_vx,dat_vy)
  axis.axis('equal')

def safe_save_fig(figure_name, figure_dir):
  try:
    os.mkdir(figure_dir)
  except Exception as error:
    print("Directory exists ",error)
  suffix = ""
  index = 0 
  while (figure_name + suffix + ".png")  in os.listdir(figure_dir):
    suffix = str(index)
    index +=1
  plt.savefig(figure_dir +"/"+ figure_name + suffix + ".png")


def draw_yaw(spirit):

  qw = spirit.state_samples[spirit.map_state_name_to_index["base_qw"],:]
  qx = spirit.state_samples[spirit.map_state_name_to_index["base_qx"],:]
  qy = spirit.state_samples[spirit.map_state_name_to_index["base_qy"],:]
  qz = spirit.state_samples[spirit.map_state_name_to_index["base_qz"],:]
  temp0 = 2.0 * (np.multiply(qw,qz) - np.multiply(qx,qy) ) 
  temp1 = 1.0 - 2.0 * (np.multiply(qy,qy) + np.multiply(qz,qz)) 
  

  plt.plot(spirit.t,np.arctan2(temp0,temp1))

def parse_turn(run):
  return run[4:].split("_")

def main():
  directory = "../Desktop/morphology-study-data-linked/bounding_turn_corrected/"
  run_prefix = "turn"
  stride_lengths = ["ss","ms","ls"]
  fig, axs = plt.subplots(2,3,sharex=True)
  for type in ["rigid","twisting"]:
    available_files = os.listdir(directory + type + "/saved_trajectories/")
    for run in available_files:
      if ( not run[0:len(run_prefix)] == run_prefix):
        continue
      
      settings = parse_turn(run)
      s = SpiritData(run, directory, is_twisting=(type =="twisting"))
      if (not s.is_success):
        continue
      # print(s.preamble_dict["speed"])
      cmap = plt.get_cmap('viridis')

      
      draw_turn_quiver(s, axis=axs[int(s.is_twisting)][stride_lengths.index(settings[1])], quiver_type="ends",data_source= "traj", color=cmap(float(s.preamble_dict["speed"])/5))
  
  figure_name = "Test"
  figure_dir = directory + "/test_figs"
  safe_save_fig(figure_name, figure_dir)
#   s.KILL()
#   fig = plt.figure()
#   plt.plot(s.csv_data_dict["x"],s.csv_data_dict["y"])
#   plt.quiver(s.csv_data_dict["x"],s.csv_data_dict["y"],s.csv_data_dict["vx"],s.csv_data_dict["vy"] )
#   plt.show()

#   # def getData(directory, pre)
#   fig = plt.figure()
#   plt.plot(spirit.data_dict["x"],spirit.data_dict["y"])
#   plt.quiver(spirit.data_dict["x"],spirit.data_dict["y"],spirit.data_dict["vx"],spirit.data_dict["vy"] )
#   # plt.show()
#   state_traj = spirit.dircon_traj.ReconstructStateTrajectory()
#   fig = plt.figure()

#   n_points = 100
#   t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
#   state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
#   for i in range(n_points):
#     state_samples[i] = state_traj.value(t[i])[:, 0]
#   state_samples = np.transpose(state_samples)
#   print(np.shape(state_samples))
  
#   print(map_state_name_to_index["base_x"])

#   plt.plot(state_samples[map_state_name_to_index["base_x"],:],state_samples[map_state_name_to_index["base_y"],:])
#   plt.quiver(state_samples[map_state_name_to_index["base_x"],:],state_samples[map_state_name_to_index["base_y"],:],state_samples[map_state_name_to_index["base_vx"],:],state_samples[map_state_name_to_index["base_vy"],:])
#   plt.axis('equal')

#   plt.figure()
#   for mode_index in range(spirit.dircon_traj.GetNumModes()):
#     mode_state_traj = spirit.dircon_traj.GetTrajectory("state_traj" + str(mode_index))
#     data_temp = mode_state_traj.datapoints
#     plt.plot(data_temp[map_state_name_to_index["base_x"],:],data_temp[map_state_name_to_index["base_y"],:])
#     plt.quiver(data_temp[map_state_name_to_index["base_x"],:],data_temp[map_state_name_to_index["base_y"],:], data_temp[map_state_name_to_index["base_vx"],:],data_temp[map_state_name_to_index["base_vy"],:])
#   plt.axis('equal')

  
#   plt.figure()

  
  # plt.show()


  # auto t0=2.0 * (x0(0) * x0(3) - x0(1)* x0(2));
  # auto t1=1 - 2* (x0(2) * x0(2) + x0(3)* x0(3));
  # auto yaw0 = atan2(t0, t1);




#   print(s.csv_data_dict['q733'])
#   """
#   Calculate the pitch of every run and plot over one another 
#   plot max pitch vs pitch constraint
#   """

#   parameters=[]
#   measurements=[]
#   constraintLimits=[]

#   parameters_for_min=[]
#   measurements_min=[]
#   experiment_index_min = []
#   abscissaIndex = 1
#   ordinateIndex = 7
#   constraintIndex = 3
#   isSuccessIndex = 9
#   pitchesAll = []
#   timesAll = []
#   constraintsAll = []
#   measurementsAll = []
#   numParams = 21
#   numPerturbations=10
#   params_unique=[]
#   # for spine in ["rigid","twisting"]
#   for p in range(1,numParams+1):
#     min_measurement=1000000
#     pitchesTemp = []
#     timesTemp = []
#     constraintsTemp = []
#     measurementsTemp = []
#     experiment_index_min_temp = (p,-1)
#     for s in range(1,numPerturbations+1):
#       # with open('./data/long_jump/5_perturbed_trajs2/jump_c3_p%d_s%d.csv'%(p,s), mode ='r')as file:
#       # with open('./data/trot_half/twisting_p5/trot_trot6_p%d_s%d.csv'%(p,s), mode ='r')as file:
#       # with open('./data/bounding_gait/rigid/bounding_gait4_p%d_s%d.csv'%(p,s), mode ='r')as file:

#       headers = []
#       quats = []
#       pitches= []
#       timesIn = []
#       try:
#         with open('/home/kodlab/Desktop/morphology-study-data-linked/bounding_turn/twisting/data/bounding_gait6_p%d_s%d.csv'%(p,s), mode ='r')as file:
#           # reading the CSV file
#           csvFile = csv.reader(file)
#           # Read the first line of the CSV which contains the summary
#           counter = 0
#           line = csvFile.__next__()
#           if (int(line[isSuccessIndex])): #Check the first line for failure, parse if success, continue if fail
#             parameterName = line[abscissaIndex-1]
#             measurementName = line[ordinateIndex-1]
#             constraintLimitName = line[constraintIndex-1]

#             parameter = float(line[abscissaIndex])
#             measurement = float(line[ordinateIndex])
#             constraintLimit = float(line[constraintIndex])
#           else:
#             print("run", p , s, "failed")
#             continue
#           # Now read the rest of the csv starting at the second line (index 1)
#           for line in csvFile: 
#             counter += 1
#             if (counter == 1): #Ignore the first two header lines
#               headers = line
#               quat_indices = [headers.index("qw"),headers.index("qx"),headers.index("qy"),headers.index("qz")]
#               continue
#             # print(line)
#             time = float(line[0])
#             quat = [float(line[ele]) for ele in quat_indices]
#             quats.append(quat)
#             # print(quats)
#             pitch = math.asin( min(max(2.0 *(quat[0] * quat[2] - quat[3] * quat[1]),-.999),.999))
#             # print(pitch)
#             pitches.append(pitch)
#             timesIn.append(time)
#           pitchesTemp.append(pitches)
#           timesTemp.append(timesIn)
#           constraintsTemp.append(constraintLimit)
#           measurementsTemp.append(measurement)
#       except Exception as error:
#         print(error)
#         continue
#       if min_measurement > measurement: 
#         min_measurement = measurement
#         experiment_index_min_temp = (p,s)
#       measurements.append(measurement)
#       parameters.append(parameter)
#       constraintLimits.append(constraintLimit)

    
#     pitchesAll.append(pitchesTemp)
#     timesAll.append(timesTemp)
#     constraintsAll.append(constraintsTemp)
#     measurementsAll.append(measurementsTemp)
#     if min_measurement==1000000:
#       continue
#     measurements_min.append(min_measurement)
#     parameters_for_min.append(parameter)
#     experiment_index_min.append(experiment_index_min_temp)
#   measurements_clipped = np.array(measurements)
#   measurements_clipped[measurements_clipped>300] = 300
#   periodsAll = []
#   print(len(timesAll))
#   for tts in timesAll:
#     pTemp = []  
#     for tt in tts:
#       pTemp.append(tt)
#     print(len(pTemp))
#     periodsAll.append(pTemp)
#   # print(periodsAll)
#   # input("PAUSE")
#   #################################################
#   print("PLOTTING ###################################################")
#   #################################################
#   plt.figure()
#   plt.title("Avg Power Cost Scatter Plot")
#   # plt.plot(displacements2,np.array(works)-np.array(works2),'b')
#   plt.scatter(parameters,measurements,c=constraintLimits,label = "Successful Optimizations")
#   plt.plot(parameters_for_min,measurements_min,c='r',label = "Minimum")
#   plt.xlabel(parameterName)
#   plt.ylabel(measurementName)
#   plt.legend()
#   plt.colorbar(label="Max Pitch Constraint")
#   #################################################


#   # print("test",len(pitchesAll))
#   actual_pitch_maxes = []
#   for parameter_pitches in pitchesAll:
#     actual_pitch_maxes_temp = []
#     for perturbation_pitches in parameter_pitches:
#       actual_pitch_maxes_temp.append(np.array(perturbation_pitches).max())
#     actual_pitch_maxes.append(actual_pitch_maxes_temp)
#   # print(parameters)
#   parametersExpanded = []
#   for (c_temp,param) in zip(constraintsAll, parameters):
#   #   print(param)
#     p_temp = []
#     for c in c_temp:
#       p_temp.append(param)
#     parametersExpanded.append(p_temp)

#   #####################################################

#   plt.figure()
#   plt.title("Actual Max Pitch vs Constraint\nScatter of all final optimizations")
#   constraintsStacked = np.array(sum(constraintsAll,[]))
#   actual_pitch_maxesStacked = np.array(sum(actual_pitch_maxes,[]))
#   plt.scatter(constraintsStacked,actual_pitch_maxesStacked,c=parameters)
#   plt.plot([0,1.5],[0,1.5],c="gray")
#   # plt.plot(parameters_for_min, measurements_min,c='r',label = "Minimum")
#   plt.xlabel("Max Pitch Constraint")
#   plt.ylabel("Actual Max Pitch")
#   # plt.legend()
#   plt.colorbar(label=parameterName)

#   ###################################################

#   plt.figure()
#   plt.title("Actual Max Pitch vs Constraint\nScatter of all final optimizations")
#   constraintsStacked = np.array(sum(constraintsAll,[]))
#   actual_pitch_maxesStacked = np.array(sum(actual_pitch_maxes,[]))
#   plt.scatter(constraintsStacked,actual_pitch_maxesStacked,c=measurements_clipped)
#   plt.plot([0,1.5],[0,1.5],c="gray")
#   # plt.plot(parameters_for_min, measurements_min,c='r',label = "Minimum")
#   plt.xlabel("Max Pitch Constraint")
#   plt.ylabel("Actual Max Pitch")
#   # plt.legend()
#   # plt.colorbar(label=measurementName)

#   cmap = plt.get_cmap('viridis')

#   norm = mpl.colors.Normalize(vmin=0, vmax=300)

#   plt.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), label=measurementName)
#   #################################################
#   # print(len(pitchesAll))
#   # print(len(constraintsAll))
#   fig, axs = plt.subplots(6,2,figsize=(9, 15),sharex=True,sharey=True)


#   cmap = plt.get_cmap('viridis')
#   constraints_MAX  = np.max(np.max(constraintsAll))
#   constraints_MIN  = np.min(np.min(constraintsAll))
#   fig.suptitle("Pitch Magnitude for all Perturbations\nColor is max pitch constraint")


#   # Now remove axes[1,5] from the grouper for xaxis
#   axs[5,1].get_shared_x_axes().remove(axs[5,1])
#   axs[5,1].get_shared_y_axes().remove(axs[5,1])

#   # Create and assign new ticker
#   xticker = mpl.axis.Ticker()
#   axs[5,1].xaxis.major = xticker

#   # The new ticker needs new locator and formatters
#   xloc = mpl.ticker.AutoLocator()
#   xfmt = mpl.ticker.ScalarFormatter()

#   axs[5,1].xaxis.set_major_locator(xloc)
#   axs[5,1].xaxis.set_major_formatter(xfmt)
#   axs[5,1].axis('off')


#   # print(constraints_MAX)
#   # print(cmap(1))
#   for index in range(np.size(axs)-1):
#     # Currently full range is 41 want to take them all but too many pltos so we are dividing it up into ten subplots with all the s
#     [axs[index%6,index//6].plot(timesDum,pitchesDum,c=cmap(constraintDum/constraints_MAX)) for (timesDum, pitchesDum,constraintDum) in zip(timesAll[index * 2], pitchesAll[index * 2],constraintsAll[index*2])]
#     axs[index%6,index//6].set_title("Speed "+ str(parameters_for_min[2*index]))

#     if(index==5 or index==10):
#       axs[index%6,index//6].set_xlabel("Time [s]")
#     if ( not index//6):
#       axs[index%6,index//6].set_ylabel("Pitch Magnitude")
    



#   norm = mpl.colors.Normalize(vmin=constraints_MIN, vmax=constraints_MAX)

#   fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), orientation='horizontal', label='Max Pitch Constraint')

#   fig.tight_layout(rect=[0, 0.03, 1, 0.95])
#   ############################################
#   fig2, axs2 = plt.subplots(6,2,figsize=(9, 15),sharex=True,sharey=True)
#   # fig.subplots_adjust(bottom=0.5)
#   cmap = plt.get_cmap('viridis')
#   measurement_MAX  = 300#np.max(np.max(measurementsAll))
#   measurement_MIN  = np.min(np.min(measurementsAll))
#   fig2.suptitle("Pitch Magnitude for all Perturbations\nColor is avg power")


#   # Now remove axes[1,5] from the grouper for xaxis
#   axs2[5,1].get_shared_x_axes().remove(axs2[5,1])
#   axs2[5,1].get_shared_y_axes().remove(axs2[5,1])

#   # Create and assign new ticker
#   xticker = mpl.axis.Ticker()
#   axs2[5,1].xaxis.major = xticker

#   # The new ticker needs new locator and formatters
#   xloc = mpl.ticker.AutoLocator()
#   xfmt = mpl.ticker.ScalarFormatter()

#   axs2[5,1].xaxis.set_major_locator(xloc)
#   axs2[5,1].xaxis.set_major_formatter(xfmt)
#   axs2[5,1].axis('off')



#   # print(constraints_MAX)
#   # print(cmap(1))
#   for index in range(np.size(axs2)-1):
#     [axs2[index%6,index//6].plot(timesDum,pitchesDum,c=cmap(measureDum/measurement_MAX)) for (timesDum, pitchesDum,measureDum) in zip(timesAll[index * 2], pitchesAll[index * 2],measurementsAll[index*2])]
#     axs2[index%6,index//6].set_title("Speed "+ str(parameters_for_min[2*index]))
#     axs2[index%6,index//6].grid(axis='x', color='0.95')
#     if(index==5 or index==10):
#       axs2[index%6,index//6].set_xlabel("Time [s]")
#     if ( not index//6):
#       axs2[index%6,index//6].set_ylabel("Pitch Magnitude")

#   norm = mpl.colors.Normalize(vmin=measurement_MIN, vmax=measurement_MAX)

#   fig2.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), orientation='horizontal', label='AveragePower')
#   fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
#   ###########################################################
#   print("Minimum Runs:", experiment_index_min)
#   plt.show()



if __name__ == "__main__":
  main()
