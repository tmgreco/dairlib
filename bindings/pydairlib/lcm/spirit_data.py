
import collections
import sys
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

class SpiritData:
  ''' Gets the CSV data and the data from the DirconTrajectory
  '''
  def __init__(self, run_name,  run_directory = "", use_csv_data = False, is_twisting = False, toe_force_headers_missing = True, index_success = 9 ):

    self.is_success = False
    self.index_success = index_success
    self.preamble_dict = {}
    self.headers_list = []
    self.header_indices_dict = {}
    self.csv_data_dict = collections.defaultdict(list)
    self.toe_force_headers_missing = toe_force_headers_missing
    self.is_twisting = is_twisting

    self.use_csv_data = use_csv_data

    try:    
      self.csv_filename = FindResourceOrThrow(run_directory + ("rigid","twisting")[int(is_twisting)] +"/data/" + run_name + ".csv")
      with open(self.csv_filename, mode ='r') as file:
        csv_file = csv.reader(file)
        preamble = csv_file.__next__()
        self.is_success = int(preamble[self.index_success])
        if(not self.is_success):
          raise Exception(self.csv_filename  + " did not succeed and so will not be read")
        self.process_csv_preamble(preamble)
        if(use_csv_data):
          headers_list = csv_file.__next__()
          self.process_headers(headers_list)
          self.process_csv_data(csv_file)
          self.duration = self.csv_data_dict["Time"][-1]      
    except Exception as error:
      print(error)

    self.csv_data_dict.default_factory = None

    # SETUP DIRCON TRAJ DATA
    self.traj_filename = FindResourceOrThrow(run_directory + ("rigid","twisting")[int(is_twisting)] +"/saved_trajectories/" + run_name) 
    self.dircon_traj = lcm_trajectory.DirconTrajectory(self.traj_filename)
    self.state_traj = self.dircon_traj.ReconstructStateTrajectory()
    self.input_traj = self.dircon_traj.GetTrajectory("input_traj")
    self.input_knotpoints_data = np.array(self.input_traj .datapoints)
    self.map_input_name_to_index = dict(zip(self.input_traj .datatypes,range(len(self.input_traj .datatypes))))
    self.modes_state_traj = []
    self.modes_state_knotpoints_data = []
    self.modes_state_knotpoints_time = []
    for mode_index in range(self.dircon_traj.GetNumModes()):
      self.modes_state_traj.append(self.dircon_traj.GetTrajectory("state_traj" + str(mode_index)))
      self.modes_state_knotpoints_data.append(np.array(self.modes_state_traj[-1].datapoints))
      self.modes_state_knotpoints_time.append(np.array(self.modes_state_traj[-1].time_vector))    
    self.map_state_name_to_index = dict(zip(self.modes_state_traj[0].datatypes,range(len(self.modes_state_traj[0].datatypes))))
    self.calculate_samples()
    

  def load_csv_data(self):
    try:
      with open(self.csv_filename, mode ='r') as file:
        csv_file = csv.reader(file)
        preamble = csv_file.__next__()
        self.is_success = int(preamble[self.index_success])
        if(not self.is_success):
          raise Exception(self.csv_filename  + " did not succeed and so will not be read")
        self.process_csv_preamble(preamble)
        headers_list = csv_file.__next__()
        self.process_headers(headers_list)
        self.process_csv_data(csv_file)
        self.duration = self.csv_data_dict["Time"][-1]  
        self.use_csv_data = True
    except Exception as error:
      print(error)

  def process_csv_preamble(self, csv_row):
    for (ind, description) in enumerate(csv_row[::2]):
      self.preamble_dict[description] = csv_row[2*ind+1]

  def process_csv_data(self, csv_data_iter):#csv_file is the iterator after the headers
    for data_row in csv_data_iter:
      for header in self.header_indices_dict.keys():
        column_index = self.header_indices_dict[header]
        if (header.strip() in [ "Front L" ,"Back L" ,"Front R" ,"Back R" ] and self.toe_force_headers_missing):
          self.csv_data_dict[header+"x"].append(self.get_float(data_row[column_index])) 
          self.csv_data_dict[header+"y"].append(self.get_float(data_row[column_index+1])) 
          self.csv_data_dict[header+"z"].append(self.get_float(data_row[column_index+2])) 
        else:
          self.csv_data_dict[header].append(self.get_float(data_row[column_index])) 

  def process_headers(self, headers):
    for (ind, header) in enumerate(headers):
      if( header ):
        print(header.strip())
        self.header_indices_dict[header.strip()] = ind


  def calculate_samples(self, num_samples=100):
      n_points = num_samples
      self.t = np.linspace(self.state_traj.start_time(), self.state_traj.end_time(), n_points)
      state_samples = np.zeros((n_points, self.state_traj.value(0).shape[0]))
      for i in range(n_points):
          state_samples[i] = self.state_traj.value(self.t [i])[:, 0]
      self.state_samples = np.transpose(state_samples)
      self.num_samples_ = num_samples

  def calculate_avg_power_csv(self, with_smooth_relu = False):
    if (not self.use_csv_data):
      print("CSV not loaded")
    ind = 0
    joint_vels_list = []
    joint_torque_list = []
    Q_list = []
    while True:
      try:
        joint_vels_list.append( self.csv_data_dict["dq" + str(ind)] )
        joint_torque_list.append( self.csv_data_dict["f" + str(ind)] )
        if ind in [1,3,5,7]:
          Q_list.append( 0.249 )
        else:
          Q_list.append( 0.561 )
      except Exception as e:
        # print("Number of Joints:",ind)
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
    timesteps = np.diff(np.array(self.csv_data_dict["Time"]))
    power_trapz = np.zeros([last_index,len(timesteps)])
    for index in range(np.shape(power_positive)[1]-1):
        power_trapz[:,index] = timesteps[index]*(power_positive[:,index] + power_positive[:,index+1])/2.0
    

    # print(power_trapz)
    # print(timesteps)
    return np.sum(power_trapz)/sum(timesteps)
  
  def calculate_avg_power_traj(self, with_smooth_relu = False):
    
    input_index_offset = 0
    energy_vector = []
    for mode_index in range(self.dircon_traj.GetNumModes()):
      mode_elec_energy = 0
      joint_index = 0
      num_knots = len(self.modes_state_knotpoints_time[mode_index])
      while ("joint_" + str(joint_index)) in self.map_state_name_to_index:
        for knot_index in range(1,num_knots):
         
          joint_vels = self.modes_state_knotpoints_data[mode_index][self.map_state_name_to_index["joint_" + str(joint_index)+"dot"]][(knot_index-1):(knot_index+1)]
          joint_inputs = self.input_knotpoints_data[self.map_input_name_to_index["motor_" + str(joint_index)]][(input_index_offset+knot_index-1):(input_index_offset+knot_index+1)]
          dt = self.modes_state_knotpoints_time[mode_index][knot_index] - self.modes_state_knotpoints_time[mode_index][knot_index-1]
          
          if joint_index in [1,3,5,7]:
            Qgain = 0.249 #Knee
          else:
            Qgain = 0.561 #Hip

          power_heat = Qgain * np.multiply(joint_inputs,joint_inputs)
          power_mech =  np.multiply(joint_vels,joint_inputs)

          joint_energy = (power_heat + power_mech)

          if (with_smooth_relu):
            relu_smooth_vectorize = np.vectorize(self.relu_smooth)
            joint_energy = relu_smooth_vectorize(joint_energy)
          else:
            joint_energy = np.maximum(joint_energy,0)

          mode_elec_energy += np.sum(joint_energy)/2 * dt
        joint_index+=1
      input_index_offset += (num_knots-1)
      energy_vector.append(mode_elec_energy)
    return sum(energy_vector)/self.modes_state_knotpoints_time[-1][-1]

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

