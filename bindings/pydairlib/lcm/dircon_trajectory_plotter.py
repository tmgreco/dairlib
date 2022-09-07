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
from spirit_data import SpiritData

def main():
  
  run_name = "turn5_ss_p1_s1"
  run_directory = "../Desktop/morphology-study-data-linked/bounding_turn_corrected/"

  spirit = SpiritData(run_name, run_directory, is_twisting=False)

  # Reconstructing state and input trajectory as piecewise polynomials
  # state_traj = dircon_traj.ReconstructStateTrajectory()
  # input_traj = dircon_traj.ReconstructInputTrajectory()
  # state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  # state_der_datatypes = dircon_traj.GetTrajectory("state_derivative_traj0").datatypes
  # input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes
  # force_samples = dircon_traj.GetTrajectory("force_vars0").datapoints
  # force_t_samples = dircon_traj.GetStateBreaks(0)
  # force_traj = dircon_traj.ReconstructLambdaTrajectory()
  # force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes
  # force_datatypes = dircon_traj.GetTrajectory("force_vars1").datatypes
  
  print(sorted(spirit.dircon_traj.GetTrajectoryNames()))
  


  np.set_printoptions(suppress=True)
  np.set_printoptions(precision=3)


  mode_input_traj = spirit.dircon_traj.GetTrajectory("input_traj")
  knotpoints_inputs = np.array(mode_input_traj.datapoints)
  map_input_name_to_index = dict(zip(mode_input_traj.datatypes,range(len(mode_input_traj.datatypes))))

  print(map_input_name_to_index)
#   print(knotpoints_inputs)
  energy_vector = []
  input_index_offset = 0
  for mode_index in range(spirit.dircon_traj.GetNumModes()):
    mode_elec_energy = 0
    mode_state_traj = spirit.dircon_traj.GetTrajectory("state_traj" + str(mode_index))
    knotpoints_states = np.array(mode_state_traj.datapoints)
    num_knots = len(knotpoints_states[0])
    mode_time_vector = mode_state_traj.time_vector
    map_state_name_to_index = dict(zip(mode_state_traj.datatypes,range(len(mode_state_traj.datatypes))))
    joint_index = 0
    while ("joint_" + str(joint_index)) in map_state_name_to_index:
      for knot_index in range(1,num_knots):
        joint_vels = knotpoints_states[map_state_name_to_index["joint_" + str(joint_index)+"dot"]][(knot_index-1):(knot_index+1)]
        joint_inputs = knotpoints_inputs[map_input_name_to_index["motor_" + str(joint_index)]][(input_index_offset+knot_index-1):(input_index_offset+knot_index+1)]
        dt = mode_time_vector[knot_index] - mode_time_vector[knot_index-1]
        if joint_index in [1,3,5,7]:
          Qgain = 0.249 #Knee
        else:
          Qgain = 0.561 #Hip
        power_heat = Qgain * np.multiply(joint_inputs,joint_inputs)
        power_mech =  np.multiply(joint_vels,joint_inputs)

        # print(input_index_offset + knot_index, joint_vels, joint_inputs,power_heat,power_mech)

        joint_energy = (power_heat + power_mech)
        joint_energy = [max(jNRG,0) for jNRG in joint_energy]
        mode_elec_energy += np.sum(joint_energy)/2 * dt
      joint_index+=1
    input_index_offset += (num_knots-1)
    energy_vector.append(mode_elec_energy)
    print(mode_time_vector)

  print("Work:")
  
  print(spirit.preamble_dict["Electrical work"])
  print(np.sum(energy_vector))
  print(spirit.calculate_avg_power()*spirit.duration)


  print("Power:")

  print(spirit.preamble_dict["Electrical power"])
  print(np.sum(energy_vector)/mode_time_vector[-1])
  print(spirit.calculate_avg_power())
  # print(np.sum(energy_vector)/float(line[5]))
  # print(np.sum(energy_vector)/mode_time_vector[-1]/float(line[7]))
  fig = plt.figure()
  plt.plot(spirit.data_dict["x"],spirit.data_dict["y"])
  plt.quiver(spirit.data_dict["x"],spirit.data_dict["y"],spirit.data_dict["vx"],spirit.data_dict["vy"] )
  plt.axis('equal')
  # plt.show()
  state_traj = spirit.dircon_traj.ReconstructStateTrajectory()
  fig = plt.figure()

  n_points = 100
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[:, 0]
  state_samples = np.transpose(state_samples)
  print(np.shape(state_samples))
  
  print(map_state_name_to_index["base_x"])

  plt.plot(state_samples[map_state_name_to_index["base_x"],:],state_samples[map_state_name_to_index["base_y"],:])
  plt.quiver(state_samples[map_state_name_to_index["base_x"],:],state_samples[map_state_name_to_index["base_y"],:],state_samples[map_state_name_to_index["base_vx"],:],state_samples[map_state_name_to_index["base_vy"],:])
  plt.axis('equal')

  plt.figure()
  for mode_index in range(spirit.dircon_traj.GetNumModes()):
    mode_state_traj = spirit.dircon_traj.GetTrajectory("state_traj" + str(mode_index))
    data_temp = mode_state_traj.datapoints
    plt.plot(data_temp[map_state_name_to_index["base_x"],:],data_temp[map_state_name_to_index["base_y"],:])
    plt.quiver(data_temp[map_state_name_to_index["base_x"],:],data_temp[map_state_name_to_index["base_y"],:], data_temp[map_state_name_to_index["base_vx"],:],data_temp[map_state_name_to_index["base_vy"],:])
  plt.axis('equal')

  
  plt.figure()

  
  # plt.show()


  # auto t0=2.0 * (x0(0) * x0(3) - x0(1)* x0(2));
  # auto t1=1 - 2* (x0(2) * x0(2) + x0(3)* x0(3));
  # auto yaw0 = atan2(t0, t1);

  qw = state_samples[map_state_name_to_index["base_qw"],:]
  qx = state_samples[map_state_name_to_index["base_qx"],:]
  qy = state_samples[map_state_name_to_index["base_qy"],:]
  qz = state_samples[map_state_name_to_index["base_qz"],:]
  temp0 = 2.0 * (np.multiply(qw,qz) - np.multiply(qx,qy) ) 
  temp1 = 1.0 - 2.0 * (np.multiply(qy,qy) + np.multiply(qz,qz)) 
  

  plt.plot(t,np.arctan2(temp0,temp1))
  plt.show()


  

def CalculateAveragePower():
  pass
if __name__ == "__main__":
  main()


#   dircon_traj.
#   import pdb; pdb.set_trace()
#   # M = reflected_joints()
#   #
#   # mirror_traj = lcm_trajectory.Trajectory()
#   # mirror_traj.traj_name = 'mirror_matrix'
#   # mirror_traj.time_vector = np.zeros(M.shape[0])
#   # mirror_traj.datapoints = M
#   # mirror_traj.datatypes = [''] * M.shape[0]
#   #
#   # dircon_traj.AddTrajectory('mirror_matrix', mirror_traj)
#   # dircon_traj.WriteToFile(filename)

#   n_points = 500
#   t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
#   state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
#   input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
#   # force_samples = np.zeros((n_points, force_traj[0].value(0).shape[0]))
#   for i in range(n_points):
#     state_samples[i] = state_traj.value(t[i])[:, 0]
#     input_samples[i] = input_traj.value(t[i])[:, 0]
#     # force_samples[i] = force_traj[0].value(t[i])[:, 0]

#   # reflected_state_samples = state_samples @ M
#   # Plotting reconstructed state trajectories
#   plt.figure("state trajectory")
#   plt.plot(t, state_samples[:, 0:7])
#   # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 0:7])
#   # plt.plot(t, state_samples[:, -18:])
#   # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 7:13])
#   # plt.plot(t, state_samples[:, 25:31])
#   # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 25:31])
#   plt.legend(state_datatypes[0:7])

#   plt.figure("input trajectory")
#   plt.plot(t, input_samples[:, :])
#   plt.legend(input_datatypes[:])

#   plt.figure("force trajectory")
#   # plt.plot(t, force_samples[:, :12])
#   plt.plot(force_t_samples, force_samples.T)
#   plt.legend(force_datatypes)

#   plt.show()

# def reflected_joints():

#   mirror = np.zeros((37, 37))
#   mirror[0:7, 0:7] = np.eye(7)
#   mirror[19:25, 19:25] = np.eye(6)
#   joint_slice = range(7, 19, 2)
#   joint_vel_slice = range(19 + 6, 19 + 18, 2)
#   asy_indices = {7, 9, 25, 27}
#   mirror[1, 1] = -1
#   mirror[3, 3] = -1
#   mirror[5, 5] = -1

#   mirror[19, 19] = -1
#   mirror[21, 21] = -1
#   mirror[23, 23] = -1
#   for i in joint_slice:
#     if(i in asy_indices):
#       mirror[i,i+1] = -1
#       mirror[i+1,i] = -1
#     else:
#       mirror[i,i+1] = 1
#       mirror[i+1,i] = 1
#   for i in joint_vel_slice:
#     if(i in asy_indices):
#       mirror[i,i+1] = -1
#       mirror[i+1,i] = -1
#     else:
#       mirror[i,i+1] = 1
#       mirror[i+1,i] = 1
#   return mirror