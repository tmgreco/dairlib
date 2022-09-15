
import collections
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

#Jillian Functions
if(True):

  # Define rotation matrice functions
  def RotX( a ):
      # rotation about the x-axis
      a = float(a)
      
      return np.array([
          [ 1,             0,              0 ],
          [ 0, math.cos( a ), -math.sin( a ) ],
          [ 0, math.sin( a ),  math.cos( a ) ]
      ])


  def RotY( a ):
      #rotation about the y-axis
      return np.array([
          [  math.cos( a ), 0, math.sin( a ) ],
          [  0,             1,             0 ],
          [ -math.sin( a ), 0, math.cos( a ) ]
      ])


  def Offsets(leg_num):
      
      # Provide Offsets for particular robot
      d_l = [0, 0, 0]
      d_0 = [173.25, 70, 0]
      d_1 = [56.5, 7.75, 0]
      d_2 = [-206, 69.5, 0]
      d_3 = [ 226, 0,    0]  
      
      # PROVIDE OFFSETS
      d_l = np.array([d_l]).transpose()/1000
      d_0 = np.array([d_0]).transpose()/1000
      d_1 = np.array([d_1]).transpose()/1000
      d_2 = np.array([d_2]).transpose()/1000
      d_3 = np.array([d_3]).transpose()/1000
      
      # begin with back_leg / right_leg conditions being reset
      back_leg = False
      right_leg = False
      
      # Check if leg given is a back or right leg
      if leg_num == 1 or leg_num == 3:
          back_leg = True
      if leg_num == 2 or leg_num == 3:
          right_leg = True
        
      # If it is a back leg, negate the x-variables of offstes
      if back_leg == True:
          d_0[0] = -d_0[0]
          d_1[0] = -d_1[0]
        
      # If it is a right leg, negate the y-variables of offsets
      if right_leg == True:
          d_0[1] = -d_0[1]
          d_1[1] = -d_1[1]
          d_2[1] = -d_2[1]
      
      # make alpha negative for back legs
      # if back_leg == True:
      #     alpha = -alpha
      
      # QUESTION: WONT ALPHAS BE DIFFERENT BASED ON WHICH LEG_NUM THEY ARE?
      # so should we still use an array the same size as the abd, hip, knee arrays?
      
      return (d_l, d_0, d_1, d_2, d_3)
   
  def ForwardKin(alpha, abd, hip, knee, leg_num):
      
      # Get offsets from running inputs through the offsets function:
      d_l, d_0, d_1, d_2, d_3 = Offsets(leg_num)
      
      # begin with back_leg condition being reset
      back_leg = False
      
      # Check if leg indicated is a back leg
      if leg_num == 1 or leg_num == 3:
          back_leg = True
      
      # make alpha negative for back legs
      if back_leg == True:
          alpha = 0
      
      # provide rotation matrices
      B_R_F = RotX(alpha)
      F_R_abd = RotX(abd)
      abd_R_hip = RotY(hip)
      hip_R_knee = RotY(knee)
      
      
      # Provide equations for finding coordinates of each joint:
      # TOE
      B_r_toe =   d_l + B_R_F @ (d_0 + F_R_abd @ (d_1 + abd_R_hip @ (d_2 + hip_R_knee @ d_3)))
      # KNEE
      # B_r_knee =   d_l + B_R_F @ (d_0 + F_R_abd @ (d_1 + abd_R_hip @ (d_2)))
      # # HIP 
      # B_r_hip =   d_l + B_R_F @ (d_0 + F_R_abd @ (d_1))
      # # ABDUCTION
      # B_r_abd =   d_l + B_R_F @ (d_0)
      

      # return('x-coords=', leg[0,:], 'y-coords=', leg[1,:], 'z-coords=', leg[2,:])
      return  B_r_toe #, B_r_knee, B_r_hip, B_r_abd)

#   def JacLimb(abd, hip, knee, leg_num):
      
#       # d_l.reshape((1,1))
#       # print(d_l.transpose())
      
#       # Retrieve Offsets for specified leg
#       d_l, d_0, d_1, d_2, d_3 = Of.Offsets(leg_num)
      
#       # Initialize Jacobian matrix to be filled with values:
#       Jac = np.zeros([3,3])
      
#       # Creat xhat, yhat, and zhat vectors:
#       x_hat = skew(1, 0, 0)
#       y_hat = skew(0, 1, 0)
      
#       # provide rotation matrices
#       F_R_abd = Of.RotX(abd)
#       abd_R_hip = Of.RotY(hip)
#       hip_R_knee = Of.RotY(knee)
      
#       # Add data to each row of the jacobian matrix:
#       # define the columns:
#       col1 = F_R_abd @ x_hat @ (d_1 + abd_R_hip @ (d_2 + hip_R_knee @ d_3))
#       col2 = F_R_abd @ abd_R_hip @ y_hat @ (d_2 + hip_R_knee @ d_3)
#       col3 = F_R_abd @ abd_R_hip @ hip_R_knee @ y_hat @ (d_3)
      
#       # print(col1)
#       # print(col2)
#       # print(col3)

      
#       # Add the columns to the matrix:
#       Jac[:, 0] = col1[:,0]
#       Jac[:, 1] = col2[:,0]
#       Jac[:, 2] = col3[:,0]
      
#       # Jac[:, {1}] = np.array([col2])
      
#       return(Jac)
    
# #  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  JacBody Function  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ 

# # Takes in a data point for alpha, abd, hip, and knee as well as 
# # the offsest values for the front, left leg 

# # Results in a Jacobian Body matrix, of shape [12, 13] for the entire robot

# def JacBody(alpha, abd, hip, knee):

#     # Retrieve Offsets for specified leg
#     # d_l, d_0, d_1, d_2, d_3 = Of.Offsets(leg_num)
    
#     # Initialize the [12, 13] Jacobian matrix to be filled with values:
#     JacBod = np.zeros([12,13], dtype = object)
#     # Create xhat vector:
#     x_hat = skew(1, 0, 0)
    
#     # Provide rotation matrices
#     B_R_F = Of.RotX(alpha/2)
#     B_R_R = Of.RotX(-alpha/2)
    
#     # Go through each leg, create it's Jacobian, and add it to the Jacobian body matrix
#     for legNum in range(4):
        
#         # Use index to help specify at what indices to place new matrix
#         index = legNum * 3
        
#         # check if leg is a front leg, if not, make it a back leg (rotation matrix)
#         if legNum % 2 == 0:
#             B_R_R = B_R_F 
#             x_var = 1/2 * B_R_F @ x_hat
#         else: 
#             B_R_R = Of.RotX(-alpha/2)
#             x_var = -1/2 * B_R_R @ x_hat
    
#         # Define Jac as the jacobian at specified legNum
#         Jac = JacLimb(abd[legNum], hip[legNum], knee[legNum], legNum)
            
#         # Add (rotation matrix) * Jac to JacBody full matrix
#         JacBod[index : index+Jac.shape[0], index : index+Jac.shape[1]]  +=  B_R_R @ Jac
        
#         # Run Forward kinematics function to get coordinate of toe 
#         toe, knees, hips, abduction = ForwardKin(alpha, abd[legNum], hip[legNum], knee[legNum], legNum)
        
#         # Create 'New' matrix which will be added to the last column of JacBody,
#         # this will be a (rotation matrix * x_hat matrix * the toe coords).
#         New = x_var @ toe
        
#         # Now, add this to the last column of JacBod matrix: 
#         JacBod[index : index+New.shape[0], 12 : 12+New.shape[1] ]  += New
        
    
#     return(JacBod)

class SpiritData:
  ''' Gets the CSV data and the data from the DirconTrajectory
  '''
  def __init__(self, run_name,  run_directory = "", use_csv_data = False, is_twisting = False, toe_force_headers_missing = True, index_success = 9, quiet=True ):

    self.is_success = False
    self.index_success = index_success
    self.preamble_dict = {}
    self.headers_list = []
    self.header_indices_dict = {}
    self.csv_data_dict = collections.defaultdict(list)
    self.toe_force_headers_missing = toe_force_headers_missing
    self.is_twisting = is_twisting
    self.run_name = run_name
    self.csv_filename = ""
    
    self.use_csv_data = use_csv_data
    

    try:    
      self.csv_filename = FindResourceOrThrow(run_directory + ("rigid","twisting")[int(is_twisting)] +"/data/" + run_name + ".csv")
      self.traj_filename = FindResourceOrThrow(run_directory + ("rigid","twisting")[int(is_twisting)] +"/saved_trajectories/" + run_name) 
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
      if (not quiet):
        print(error)
    if(not self.is_success):
      os.rename(self.csv_filename, run_directory + ("rigid","twisting")[int(is_twisting)] +"/data/" + "zzFAIL_" + run_name + ".csv")
      os.rename(self.traj_filename, run_directory + ("rigid","twisting")[int(is_twisting)] +"/saved_trajectories/" + "zzFAIL_" + run_name)
      return



    self.csv_data_dict.default_factory = None

    # SETUP DIRCON TRAJ DATA
    # self.traj_filename = FindResourceOrThrow(run_directory + ("rigid","twisting")[int(is_twisting)] +"/saved_trajectories/" + run_name) 
    self.dircon_traj = lcm_trajectory.DirconTrajectory(self.traj_filename)
    self.state_traj = self.dircon_traj.ReconstructStateTrajectory()
    
    self.input_traj = self.dircon_traj.ReconstructInputTrajectory()
    input_data_traj = self.dircon_traj.GetTrajectory("input_traj")
    self.input_knotpoints_data = np.array( input_data_traj.datapoints)
    self.map_input_name_to_index = dict(zip(input_data_traj.datatypes, range(len(input_data_traj.datatypes))))
    self.modes_state_traj = []
    self.modes_state_knotpoints_data = []
    self.modes_state_knotpoints_time = []
    for mode_index in range(self.dircon_traj.GetNumModes()):
      self.modes_state_traj.append(self.dircon_traj.GetTrajectory("state_traj" + str(mode_index)))
      self.modes_state_knotpoints_data.append(np.array(self.modes_state_traj[-1].datapoints))
      self.modes_state_knotpoints_time.append(np.array(self.modes_state_traj[-1].time_vector))    
    self.map_state_name_to_index = dict(zip(self.modes_state_traj[0].datatypes,range(len(self.modes_state_traj[0].datatypes))))
    self.calculate_samples()
    self.get_toe_positions_body_frame()
  
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
      input_samples = np.zeros((n_points, self.input_traj.value(0).shape[0]))
      for i in range(n_points):
          state_samples[i] = self.state_traj.value(self.t [i])[:, 0]
          input_samples[i] = self.input_traj.value(self.t [i])[:, 0]
      self.state_samples = np.transpose(state_samples)
      self.input_samples = np.transpose(input_samples)
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

  def get_toe_positions_body_frame(self):
    #Pose is fixed in the base
    abd_index = [8,9,10,11]
    hip_index = [0,2,4,6]
    kne_index = [1,3,5,7]
    self.toe_positions_data = []
    for mode_i in range(self.dircon_traj.GetNumModes()):
      toe_poses_per_mode = []
      for toe_i in range(4): 
        toe_i_pos_per_mode = []
        for state_knot in self.modes_state_knotpoints_data[mode_i].transpose():
          if self.is_twisting:
            alpha = state_knot[self.map_state_name_to_index["joint_12"]]
          else:
            alpha = 0
          toe_i_pos_per_mode.append( ForwardKin( alpha ,
                      state_knot[self.map_state_name_to_index["joint_" + str(abd_index[toe_i])] ],
                      -state_knot[self.map_state_name_to_index["joint_" + str(hip_index[toe_i])] ],
                      state_knot[self.map_state_name_to_index["joint_" + str(kne_index[toe_i])] ],
                      toe_i) )
        toe_poses_per_mode.append(toe_i_pos_per_mode)
      self.toe_positions_data.append(toe_poses_per_mode)
                      
          # # print(ForwardKin( 0 ,
          # #             0,
          # #             -1,
          # #             2,
          # #             toe_i))
          # print(state_knot)
          # print(self.modes_state_traj[0].datatypes)
          # print("joint_" + str(abd_index[toe_i]), self.map_state_name_to_index["joint_" + str(abd_index[toe_i])])
          # print("joint_" + str(hip_index[toe_i]), self.map_state_name_to_index["joint_" + str(hip_index[toe_i])])
          # print("joint_" + str(kne_index[toe_i]), self.map_state_name_to_index["joint_" + str(kne_index[toe_i])])
          # print(     [state_knot[self.map_state_name_to_index["joint_" + str(abd_index[toe_i])] ],
          #             state_knot[self.map_state_name_to_index["joint_" + str(hip_index[toe_i])] ],
          #             state_knot[self.map_state_name_to_index["joint_" + str(kne_index[toe_i])] ]] )
          
      # ForwardKin(alpha, abd, hip, knee, toe_i)
    # d = np.array()
    

if __name__ == "__main__":
  print("spirit_data main() not implemented")