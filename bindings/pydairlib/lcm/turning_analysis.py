import sys
import os
import copy
from xml.etree.ElementTree import PI
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

def safe_save_fig(figure_name, figure_dir,overwrite = False, figure=None):
  if not figure:
    figure = plt.gcf()
  try:
    os.mkdir(figure_dir)
  except Exception as error:
    print("Directory exists ",error)
  suffix = ""
  index = 0 
  while ((not overwrite) and ((figure_name + suffix + ".png")  in os.listdir(figure_dir))):
    suffix = str(index)
    index +=1
  final_figure_name = figure_name + suffix + ".png"
  if (overwrite and os.path.isfile(final_figure_name) ):
    os.remove(final_figure_name)   # Opt.: os.system("rm "+strFile)
  figure.savefig(figure_dir +"/"+ final_figure_name)
  print("Figure saved to " + final_figure_name + " in " + figure_dir)

def draw_yaw(spirit):

  qw = spirit.state_samples[spirit.map_state_name_to_index["base_qw"],:]
  qx = spirit.state_samples[spirit.map_state_name_to_index["base_qx"],:]
  qy = spirit.state_samples[spirit.map_state_name_to_index["base_qy"],:]
  qz = spirit.state_samples[spirit.map_state_name_to_index["base_qz"],:]
  temp0 = 2.0 * (np.multiply(qw,qz) - np.multiply(qx,qy) ) 
  temp1 = 1.0 - 2.0 * (np.multiply(qy,qy) + np.multiply(qz,qz)) 
  

  plt.plot(spirit.t,np.arctan2(temp0,temp1))

def draw_zheight(spirit, axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  axis.plot(spirit.t, spirit.state_samples[spirit.map_state_name_to_index["base_z"],:], *args, **kwargs)

def draw_spine(spirit_twisting,axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  axis.plot(spirit_twisting.t, spirit_twisting.state_samples[spirit_twisting.map_state_name_to_index["joint_12"],:], *args, **kwargs)
  
def draw_spine_torque(spirit_twisting,axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  axis.plot(spirit_twisting.t, spirit_twisting.input_samples[spirit_twisting.map_input_name_to_index["motor_12"],:], *args, **kwargs)
  
def draw_stance_phases(spirit, modes, axis = [], facecolor = 'gray', *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  for mode in modes:
    time_interval = [spirit.modes_state_knotpoints_time[mode][0],spirit.modes_state_knotpoints_time[mode][-1]]
    axis.axvspan(time_interval[0], time_interval[1], facecolor=facecolor, alpha=0.2, zorder=-100,*args,**kwargs)

def parse_turn(run):
  return run[4:].split("_")

def main():
  directory = "../Desktop/morphology-study-data-linked/bounding_turn_final/"
  run_prefix = "turn"
  stride_lengths = ["ss","ms","ls"]
  num_velocity = 5
  num_yaws = 10
  num_perturbs_per_period = 10
  # periods = [(hp+1)*0.1 for hp in range(len(stride_lengths))]

  velocity_map = dict( [( str(label+1) , label +1 ) for label in range(num_velocity) ] )
  yaw_map = dict( [( "p" + str(plabel + 1), 0.1*plabel + 0.1 ) for plabel in range(num_yaws) ] )
  perturbation_map = dict( [ ( "s" + str(slabel + 1) , (slabel%num_perturbs_per_period ) ) for slabel in range(num_perturbs_per_period) ] )
  
  # velocity_indices = range(1,num_params+1)
  # param_indices = range(1,10+1)
  # spirit_lists = {stride:{} for stride in stride_lengths}
  # spirit_lists_optimal = {stride:{} for stride in stride_lengths}
  
  run_none = None
  energy_default = 10000
  spirit_lists =  {v_i: {y_i: {hp_i:[ run_none for p_i in range(1,num_perturbs_per_period + 1) ] for hp_i in stride_lengths } for y_i in yaw_map.values()} for v_i in velocity_map.values() }
  spirit_lists_optimal =  {v_i: {y_i: {hp_i: (run_none, energy_default) for hp_i in  stride_lengths } for y_i in yaw_map.values()} for v_i in velocity_map.values() }

  cmap = plt.get_cmap('viridis')
  cmap_reds = plt.get_cmap('Reds')
  cmap_blues = plt.get_cmap('Blues')
  
  spirit_lists = dict(zip(["rigid","twisting"],[spirit_lists,copy.deepcopy(spirit_lists)]))
  spirit_lists_optimal = dict(zip(["rigid","twisting"],[spirit_lists_optimal,copy.deepcopy(spirit_lists_optimal)]))
  # print(spirit_lists)

  # fig, axs = plt.subplots(2,3,sharex=True)
  print("READ DATA")
  count = 0
  for spine_type in ["rigid","twisting"]:
    available_files = os.listdir(directory + spine_type + "/saved_trajectories/")
    for run in available_files:
      if ( not run[0:len(run_prefix)] == run_prefix):
        continue
      settings = parse_turn(run)
      print(settings)
      if(len(settings)==3):
        continue
      #Ex. ['4', 'ls', 'p2', 's4']

      s = SpiritData(run, directory, is_twisting=(spine_type =="twisting"))
      if (not s.is_success):
        continue
      
      spirit_lists[spine_type][velocity_map[settings[0]]][yaw_map[settings[2]]][settings[1]][perturbation_map[settings[3]]] = s
      # spirit_lists[spine_type][velocity_map[settings[2]]][settings[1]][perturbation_map[settings[3]]].append(s)
      run_energy = s.calculate_avg_power_traj() * s.modes_state_knotpoints_time[-1][-1]
      if not spirit_lists_optimal[spine_type][velocity_map[settings[0]]][yaw_map[settings[2]]][settings[1]][0] or spirit_lists_optimal[spine_type][velocity_map[settings[0]]][yaw_map[settings[2]]][settings[1]][1] > run_energy:
        spirit_lists_optimal[spine_type][velocity_map[settings[0]]][yaw_map[settings[2]]][settings[1]] = (s, run_energy)
      # draw_turn_quiver(s, axis=axs[int(s.is_twisting)][stride_lengths.index(settings[1])], quiver_type="ends",data_source= "traj", color=cmap(float(s.preamble_dict["speed"])/5))
  for vel in spirit_lists_optimal["twisting"]:
    for yaw in spirit_lists_optimal["twisting"][vel]:
      for stride in spirit_lists_optimal["twisting"][vel][yaw]:
        if spirit_lists_optimal["twisting"][vel][yaw][stride][0]:
          print(spirit_lists_optimal["twisting"][vel][yaw][stride][0].run_name)
  s.kill()
  # directory = "../Desktop/morphology-study-data-linked/trot_search/"
  # run_prefix = "trot5"

  # run_none = None
  # energy_default = 10000
  # spirit_lists =  {v_i: {hp_i:[ run_none for p_i in range(1,num_perturbs_per_period + 1) ] for hp_i in periods } for v_i in velocity_map.values() }
  # spirit_lists_optimal =  {v_i: {hp_i: (run_none, energy_default) for hp_i in  periods } for v_i in velocity_map.values() }


  # spirit_lists = dict( zip( ["rigid","twisting"], [spirit_lists,copy.deepcopy(spirit_lists)] ) )
  # spirit_lists_optimal = dict(zip(["rigid","twisting"], [spirit_lists_optimal,copy.deepcopy(spirit_lists_optimal)]))

  # # fig, axs = plt.subplots(2,3,sharex=True)
  
  # count = 0
  
  # for spine_type in ["rigid","twisting"]:
  #   available_files = os.listdir(directory + spine_type + "/saved_trajectories/")
  #   for run in available_files:
  #     if ( not run[:len(run_prefix)] == run_prefix):
  #       continue
      
  #     try:
  #       [run_param_name, run_perturb_name] = parse_trot(run)
  #     except:
  #       continue
  #     print([run_param_name, run_perturb_name] )
  #     #Ex. ['p2', 's4']
      
  #     s = SpiritData(run, directory, is_twisting=(spine_type =="twisting"))
  #     if (not s.is_success):
  #       continue
      
  #     # print(s.modes_state_knotpoints_time)
  #     # s.kill()
  #     # print(s.preamble_dict["speed"])

  #     # vel_ind = int(settings[0][1:])//2 
  #     # pert_ind = int(settings[1][1:])-1

  #     spirit_lists[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][perturbation_map[run_perturb_name][1] -1]
  #     run_power = s.calculate_avg_power_traj()
  #     if run_power<600 and (not spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][0] or  spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][1] > run_power):
  #       spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]] = (s, run_power)
  


############################### NRG PLOT 
  if(False):
    # spirit_lists[spine_type][velocity_map[settings[0]]][yaw_map[settings[2]]][settings[1]]
    print("Building plots")
    fig, axs = plt.subplots(3, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    counter = 0
    fig_perc, axs_perc = plt.subplots(1, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    print("Finished plots")
    period_energies = [[[None for stride in stride_lengths] for yaw in yaw_map ] for spine_type in ["rigid","twisting"] ] 
    
    ## PLOT YAW
    spine_list = []
    for spine_type in ["rigid","twisting"]:
      vel_list = []
      for vel_key in velocity_map.values():
        stride_list = []
        for (stride_i, stride) in enumerate(stride_lengths):
          yaw_list = []
          for y_key in yaw_map.values():
            if y_key>3.14159/4:
              continue
            if spirit_lists_optimal[spine_type][vel_key][y_key][stride][0]:
              yaw_list.append([y_key, spirit_lists_optimal[spine_type][vel_key][y_key][stride][1]])
          nrg_for_plot = np.array(yaw_list)
          axs[stride_i,0].plot( nrg_for_plot[:,0], nrg_for_plot[:,1], ("-","--")[spine_type=="twisting"], color = (cmap(vel_key/5)) )

    stride_labels = dict([("ss", "Short Initial Stride (XX)"),("ms", "Medium Initial Stride (XX)"),("ls", "Long Initial Stride (XX)")])
    for (stride_i, stride) in enumerate(stride_lengths):
      axs[stride_i,0].set_ylabel(stride_labels[stride] + "\nEnergy [J]")
      axs[stride_i,0].grid(visible=True, which="both")
    axs[2,0].set_xlabel("Yaw Displacement [rad]")
      


############################### NRG PLOT PERCENTAGE
  if(True):
    print("Building plots")
    fig_power, axs = plt.subplots(3, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    counter = 0
    fig_perc, axs_perc = plt.subplots(3, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    print("Finished plots")
    
    ## PLOT YAW
    vel_list = []
    for vel_key in velocity_map.values():
      stride_list = []
      for (stride_i, stride) in enumerate(stride_lengths):
        yaw_list_twist = []
        yaw_list_rigid = []
        for y_key in yaw_map.values():
          if y_key>3.14159/4:
            continue
          if vel_key==1:
            if stride=="ss":
              if y_key>0.39:
                continue
              
          if spirit_lists_optimal["twisting"][vel_key][y_key][stride][0]:
            yaw_list_twist.append([y_key, spirit_lists_optimal["twisting"][vel_key][y_key][stride][1]])
          if spirit_lists_optimal["rigid"][vel_key][y_key][stride][0]:
            yaw_list_rigid.append([y_key, spirit_lists_optimal["rigid"][vel_key][y_key][stride][1]])
        nrg_for_plot_twist = np.array(yaw_list_twist)
        nrg_for_plot_rigid = np.array(yaw_list_rigid)

        axs[stride_i,0].plot( nrg_for_plot_twist[:,0], nrg_for_plot_twist[:,1], "--", color = (cmap(vel_key/5)) )
        axs[stride_i,0].plot( nrg_for_plot_rigid[:,0], nrg_for_plot_rigid[:,1], "-", color = (cmap(vel_key/5)) )
        nrg_for_dict_rigid = {pair[0]:pair[1] for pair in nrg_for_plot_rigid}
        yaw_list_twist_perc = []
        for twist_yaw_nrg in yaw_list_twist:
          try: 
            yaw_list_twist_perc.append([twist_yaw_nrg[0],twist_yaw_nrg[1]/nrg_for_dict_rigid[twist_yaw_nrg[0]]-1])
          except KeyError:
            continue
        print(yaw_list_twist_perc)
        perc_nrg_for_plot_twist = np.array(yaw_list_twist_perc)        
        axs_perc[stride_i,0].plot( perc_nrg_for_plot_twist[:,0], perc_nrg_for_plot_twist[:,1], "-", color = (cmap(vel_key/5)) )

    stride_labels = dict([("ss", "Short Stride"),("ms", "Medium Stride"),("ls", "Long Stride")])
    for (stride_i, stride) in enumerate(stride_lengths):
      axs[stride_i,0].set_ylabel(stride_labels[stride] + "\nEnergy [J]")
      axs[stride_i,0].grid(visible=True, which="both")

      axs_perc[stride_i,0].grid( visible=True, which="both")
      axs_perc[stride_i,0].set_ylabel(stride_labels[stride] + "\n Difference ")
      axs_perc[stride_i,0].set_ylim([-.3,.3])
      vals = axs_perc[stride_i,0].get_yticks()
      axs_perc[stride_i,0].set_yticklabels(['{:,.1%}'.format(x) for x in vals])
    axs[2,0].set_xlabel("Yaw Displacement [rad]")
    axs_perc[2,0].set_xlabel("Yaw Displacement [rad]")    
    
    
    figure_dir = directory + "/figs"
    figure_name = "TurnPowerPercentage"
    safe_save_fig(figure_name, figure_dir, overwrite = True, figure=fig_perc)
    figure_name = "TurnPowerAll"
    safe_save_fig(figure_name, figure_dir, overwrite = True, figure=fig_power)


    #   # ax.set_xlabel("Velocity [m/s]")
    #   # ax.set_ylabel("Energy [J]")
    #   # for (spine_type,nrg_by_spine) in zip(["rigid","twisting"], period_energies):
    #   #   counter = 0
    #   #   style = ("-d","--X")[int(spine_type =="twisting" )]
    #   #   for nrg_by_period in nrg_by_spine:
    #   #     nrg_for_plot = np.array(nrg_by_period)
    #   #     ax.plot( nrg_for_plot[:,0], nrg_for_plot[:,1], style, color=cmap((counter+1)*.3), zorder=-1, label = spine_type + ", " + stride_lengths[counter] )
    #   #     counter += 1
    
    # ax.legend(title="Spine, Stride Period")
    # # fig.suptitle("Average Power for Trot\nwith Different Stride Periods")
    
    # counter = 0
    # style = ("-d","--X")[int(spine_type =="twisting" )]
    # print("##########################")
    # for (rigid_nrg_by_period, twist_nrg_by_period) in zip(period_energies[0],period_energies[1]):
    #   print(rigid_nrg_by_period)
    #   num_runs_rigid = len(rigid_nrg_by_period)
    #   num_runs_twist = len(twist_nrg_by_period)
    #   num_runs = min(num_runs_rigid,num_runs_twist)
    #   print(num_runs_rigid,num_runs_twist,num_runs)

    #   rigid_nrg_for_plot = np.array(rigid_nrg_by_period)
    #   twist_nrg_for_plot = np.array(twist_nrg_by_period)
    #   # print(rigid_nrg_for_plot)
    #   # print(twist_nrg_for_plot)

    #   ax_perc.plot( twist_nrg_for_plot[:num_runs,0], twist_nrg_for_plot[:num_runs,1]/rigid_nrg_for_plot[:num_runs,1], style, color=cmap((counter+1)*.3), zorder=-1, label = spine_type + ", " + stride_lengths[counter] )
    
    #   counter += 1
    # ax_perc.legend(title="Spine, Stride Period")
    # ax_perc.plot([0,4],[1,1],'--',color='gray')
    # ax_perc.set_ylim([0,4])

    # vals = ax_perc.get_yticks()
    # ax_perc.set_yticklabels(['{:,.1%}'.format(x) for x in vals])
    # fig.suptitle("Average Power for Trot\nwith Different Stride Periods")
    
    # figure_name = "Trot_Power"
    # figure_dir = directory + "/test_figs"
    # safe_save_fig(figure_name, figure_dir, overwrite = True)



  ######################### PLOT THE TURN SPINE ACTUATION FOR MOST "OPTIMAL" RUNS
  if(False): # Turn this plot on 
    for stride in stride_lengths:
      print("Building the subplots figure for " + stride)
      fig2, axs2 = plt.subplots(len(velocity_indices),len(param_indices),squeeze=False, sharex=True, sharey=True,figsize=(16, 10))
      print("Figure and axes built")
      fig2.suptitle("SpineTorque vs. Time\nStride Length:"  + stride)
      for (index_vel, vel_dict) in enumerate(spirit_lists_optimal["twisting"][stride]):
        # print(spirit_lists["twisting"]["ss"][vel_dict])
        for (index_turn,turn_disp) in enumerate(spirit_lists_optimal["twisting"][stride][vel_dict]):
          # print(turn_disp)
          # for s in turn_disp:
          if (not turn_disp[0]):
            continue
          s, run_power = turn_disp #spirit_lists_optimal["twisting"][stride][vel_dict][index_turn]
          draw_spine_torque(s,axis=axs2[index_vel, index_turn])
          draw_stance_phases(s, [1,4], axis=axs2[index_vel, index_turn])
          axs2[index_vel, index_turn].grid(which='both')

          velStr = "{vel:.1f}"
          axs2[index_vel, 0].set_ylabel("Velocity " + velStr.format(vel=(index_vel+1)))
          dispStr = "{disp:.2f}"
          axs2[len(velocity_indices)-1, index_turn].set_xlabel("Yaw Disp " + dispStr.format(disp=(index_turn+1)*0.1))
          
          ################################

  ######################### PLOT THE TURN SPINE ACTUATION FOR MOST "OPTIMAL" RUNS
  if(False): # Turn this plot on 
    
      
    for stride in stride_lengths:
      print("Building the subplots figure for " + stride)
      fig2, axs2 = plt.subplots(len(velocity_indices),len(param_indices) - 5,squeeze=False, sharex=True, sharey=True,figsize=(16, 10))
      print("Figure and axes built")
      fig2.suptitle("Friction vs. Time\nStride Length:"  + stride)
      for spine_type in ["rigid","twisting"]:
        for (index_vel, vel_dict) in enumerate(spirit_lists_optimal[spine_type][stride]):

          # print(spirit_lists["twisting"]["ss"][vel_dict])
          for (index_turn,turn_disp) in enumerate(spirit_lists_optimal[spine_type][stride][vel_dict]):
            if (index_turn >= len(axs2[0])):
              continue
            # print(turn_disp)
            # for s in turn_disp:
            if (not turn_disp[0]):
              continue
            s, run_power = turn_disp #spirit_lists_optimal["twisting"][stride][vel_dict][index_turn]
            # draw_spine_torque(s,axis=axs2[index_vel, index_turn])
            # draw_stance_phases(s, [1,4], axis=axs2[index_vel, index_turn])
            # axs2[index_vel, index_turn].grid(which='both')

            velStr = "{vel:.1f}"
            # axs2[index_vel, 0].set_ylabel("Velocity " + velStr.format(vel=(index_vel+1)))
            dispStr = "{disp:.2f}"
            # axs2[len(velocity_indices)-1, index_turn].set_xlabel("Yaw Disp " + dispStr.format(disp=(index_turn+1)*0.1))
            
            ################################
            print(s.run_name)
            for mode_i in range(s.dircon_traj.GetNumModes()):
              force_traj = s.dircon_traj.GetTrajectory("force_vars" + str(mode_i))
              if (not force_traj.datapoints.any()):
                continue
              time_vector = force_traj.time_vector
              # left_forces = force_traj.datapoints[:3,:]
              # right_forces = force_traj.datapoints[3:,:]
              # left_lateral_magnitude = np.sqrt(np.sum(np.square(left_forces[:2,:]),0))
              # left_mu = left_lateral_magnitude/left_forces[2,:]
              # right_lateral_magnitude = np.sqrt(np.sum(np.square(right_forces[:2,:]),0))
              # right_mu = right_lateral_magnitude/right_forces[2,:]
              # axs2[index_vel, index_turn].plot( time_vector, left_lateral_magnitude, ("b-","r-")[int(s.is_twisting )],
              #                                   time_vector, right_lateral_magnitude, ("b--","r--")[int(s.is_twisting )])

            draw_stance_phases(s, [1,4], axis=axs2[index_vel, index_turn], facecolor=('blue','red')[int(s.is_twisting )])
            # axs2[index_vel,index_turn].set_ylim([0,0.6])
      # figure_name = "TurnTorque_"+ stride
      # figure_dir = directory + "/test_figs"
      # safe_save_fig(figure_name, figure_dir, overwrite = True)

  plt.show()
  






if __name__ == "__main__":
  main()
