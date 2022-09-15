import sys
import os
import copy
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

def skew(vector3):
  return np.array([[          0, -vector3[2],  vector3[1]],
                   [ vector3[2],           0, -vector3[0]],
                   [-vector3[1],  vector3[0],           0]])

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

def draw_spine(spirit_twisting, axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  axis.plot(spirit_twisting.t, spirit_twisting.state_samples[spirit_twisting.map_state_name_to_index["joint_12"],:], *args, **kwargs)
  
def draw_spine_torque(spirit_twisting,axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  axis.plot(spirit_twisting.t, spirit_twisting.input_samples[spirit_twisting.map_input_name_to_index["motor_12"],:], *args, **kwargs)
  
def draw_stance_phases(spirit, modes, axis = [], *args, **kwargs):
  if not axis:
    fig,axis = plt.subplots()
  for mode in modes:
    time_interval = [spirit.modes_state_knotpoints_time[mode][0],spirit.modes_state_knotpoints_time[mode][-1]]
    axis.axvspan(time_interval[0], time_interval[1], facecolor='gray', alpha=0.2, zorder=-100)

def parse_trot(run):
  return run[6:].split("_")

def main():
  directory = "../Desktop/morphology-study-data-linked/trot_search/"
  run_prefix = "trot5"
  num_params = 9
  num_perturbs = 40
  num_perturbs_per_period = 10
  periods = [(hp+1)*0.1 for hp in range(num_perturbs//num_perturbs_per_period)]

  velocity_map = dict( [( "p" + str(plabel + 1), plabel * 0.5 ) for plabel in range(num_params) ] )
  perturbation_map = dict( [ ( "s" + str(slabel + 1) , ( periods[slabel//num_perturbs_per_period ], (slabel%num_perturbs_per_period + 1) ) ) for slabel in range(num_perturbs) ] )
  

  run_none = None
  energy_default = 10000
  spirit_lists =  {v_i: {hp_i:[ run_none for p_i in range(1,num_perturbs_per_period + 1) ] for hp_i in periods } for v_i in velocity_map.values() }
  spirit_lists_optimal =  {v_i: {hp_i: (run_none, energy_default) for hp_i in  periods } for v_i in velocity_map.values() }



  cmap = plt.get_cmap('viridis')
  cmap_reds = plt.get_cmap('Reds')
  cmap_blues = plt.get_cmap('Blues')
  # spirit_lists_optimal = [[ (run_none, energy_default, velocity_default, period_default) for hp_i in half_period_indices] for v_i in  velocity_indices ]
  
  spirit_lists = dict( zip( ["rigid","twisting"], [spirit_lists,copy.deepcopy(spirit_lists)] ) )
  spirit_lists_optimal = dict(zip(["rigid","twisting"], [spirit_lists_optimal,copy.deepcopy(spirit_lists_optimal)]))

  # fig, axs = plt.subplots(2,3,sharex=True)
  
  count = 0
  
  for spine_type in ["rigid","twisting"]:
    available_files = os.listdir(directory + spine_type + "/saved_trajectories/")
    for run in available_files:
      if ( not run[:len(run_prefix)] == run_prefix):
        continue
      
      try:
        [run_param_name, run_perturb_name] = parse_trot(run)
      except:
        continue
      print([run_param_name, run_perturb_name] )
      #Ex. ['p2', 's4']
      
      s = SpiritData(run, directory, is_twisting=(spine_type =="twisting"))
      if (not s.is_success):
        continue
      
      # print(s.modes_state_knotpoints_time)
      # s.kill()
      # print(s.preamble_dict["speed"])

      # vel_ind = int(settings[0][1:])//2 
      # pert_ind = int(settings[1][1:])-1

      spirit_lists[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][perturbation_map[run_perturb_name][1] -1] = s
      run_power = s.calculate_avg_power_traj()
      if run_power<600 and (not spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][0] or  spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]][1] > run_power):
        spirit_lists_optimal[spine_type][velocity_map[run_param_name]][perturbation_map[run_perturb_name][0]] = (s, run_power)
  # for vel in spirit_lists_optimal["twisting"]:
  #   for period in spirit_lists_optimal["twisting"][vel]:
  #     if spirit_lists_optimal["twisting"][vel][period][0]:
  #       print(spirit_lists_optimal["twisting"][vel][period][0].run_name)
  # s.kill()

  if(False): #Average Power for Trot\nwith Different Stride Periods
    print("Building plots")
    fig_power, axs = plt.subplots(1, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    fig_perc, axs_perc = plt.subplots(1, 1, squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    print("Finished plots")
    period_energies = [[[],[],[],[]],[[],[],[],[]]]
    for runs_vel in spirit_lists["rigid"]:
      ax = axs[0][0]
      ax_perc = axs_perc[0][0]
      for spine_type in ["rigid","twisting"]:
        for (hp_i,hp_key) in enumerate(spirit_lists[spine_type][runs_vel]):
          # Take mode 1 an normalize time
          s = spirit_lists_optimal[spine_type][runs_vel][hp_key][0]
          if (not s or not s.is_success):
            continue
          force_traj = s.dircon_traj.GetTrajectory("force_vars1")
          final_flight = s.dircon_traj.GetTrajectory("state_traj2")

          t_stance =force_traj.time_vector #np.linspace(force_traj.time_vector[0],force_traj.time_vector[-1],num_points)
          stance_period = force_traj.time_vector[-1]-force_traj.time_vector[0]
          trot_half_period = final_flight.time_vector[-1]
          period_energies[int(s.is_twisting )][hp_i].append([runs_vel, spirit_lists_optimal[spine_type][runs_vel][hp_key][1] ])
          force_samples = force_traj.datapoints
          
          # ax.plot( runs_vel, spirit_lists_optimal[spine_type][runs_vel][hp_key][1] ,  ("d","X")[int(s.is_twisting )], color=((cmap,cmap)[int(s.is_twisting )])(hp_key*2) )


      perStr = "{per:.1f}"
      velStr = "{vel:.1f}"
      
      ax.set_xlabel("Velocity [m/s]")
      ax.set_ylabel("Average Power [W]")
    for (spine_type,nrg_by_spine) in zip(["rigid","twisting"], period_energies):
      counter = 0
      style = ("-d","--X")[int(spine_type =="twisting" )]
      for nrg_by_period in nrg_by_spine:
        counter += 1
        nrg_for_plot = np.array(nrg_by_period)
        ax.plot( nrg_for_plot[:,0], nrg_for_plot[:,1], style, color=cmap(counter*.1*2), zorder=-1, label = spine_type + ", " + perStr.format(per=counter*.1*2) + "s")
    ax.legend(title="Spine, Stride Period")
    # fig.suptitle("Average Power for Trot\nwith Different Stride Periods")
    
    counter = 0
    style = ("-d","--X")[int(spine_type =="twisting" )]
    print("##########################")
    for (rigid_nrg_by_period, twist_nrg_by_period) in zip(period_energies[0],period_energies[1]):
      counter += 1
      print(rigid_nrg_by_period)
      num_runs_rigid = len(rigid_nrg_by_period)
      num_runs_twist = len(twist_nrg_by_period)
      num_runs = min(num_runs_rigid,num_runs_twist)
      print(num_runs_rigid,num_runs_twist,num_runs)

      rigid_nrg_for_plot = np.array(rigid_nrg_by_period)
      twist_nrg_for_plot = np.array(twist_nrg_by_period)
      # print(rigid_nrg_for_plot)
      # print(twist_nrg_for_plot)

      ax_perc.plot( twist_nrg_for_plot[:num_runs,0], twist_nrg_for_plot[:num_runs,1]/rigid_nrg_for_plot[:num_runs,1], style, color=cmap(counter*.1*2), zorder=-1, label = spine_type + ", " + perStr.format(per=counter*.1*2) + "s")
    ax_perc.legend(title="Spine, Stride Period")
    ax_perc.plot([0,4],[1,1],'--',color='gray')
    ax_perc.set_ylim([0,4])

    vals = ax_perc.get_yticks()
    ax_perc.set_yticklabels(['{:,.1%}'.format(x) for x in vals])
    # fig.suptitle("Average Power for Trot\nwith Different Stride Periods")
    
    figure_dir = directory + "/figs"
    figure_name = "TrotPowerPercentage"
    safe_save_fig(figure_name, figure_dir, overwrite = True, figure=fig_perc)
    figure_name = "TrotPowerAll"
    safe_save_fig(figure_name, figure_dir, overwrite = True, figure=fig_power)
#########################################################
#########################################################
  if(False):
    counter = -1
    print("Building plots")
    fig,axs = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    fig2,axs2 = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    fig3,axs3 = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    print("Finished plots")
    for runs_vel in spirit_lists["rigid"]:
      counter +=1
      ax = axs[counter//3][counter%3]
      ax2 = axs2[counter//3][counter%3]
      ax3 = axs3[counter//3][counter%3]
      for spine_type in ["rigid","twisting"]:
        for hp_key in spirit_lists[spine_type][runs_vel]:
          # Take mode 1 an normalize time
          s = spirit_lists_optimal[spine_type][runs_vel][hp_key][0]
          if (not s or not s.is_success):
            continue
          force_traj = s.dircon_traj.GetTrajectory("force_vars1")
          final_flight = s.dircon_traj.GetTrajectory("state_traj2")

          t_stance =force_traj.time_vector #np.linspace(force_traj.time_vector[0],force_traj.time_vector[-1],num_points)
          stance_period = force_traj.time_vector[-1]-force_traj.time_vector[0]
          trot_half_period = final_flight.time_vector[-1]

          force_samples = force_traj.datapoints
          # GET THE MOMENT FROM TOE
          effective_moment = []
          for knot_i in range(len(s.modes_state_knotpoints_time[1])):
            print(skew(s.toe_positions_data[1][0][knot_i].transpose()[0]))
            print(skew(s.toe_positions_data[1][3][knot_i].transpose()[0]))
            print(force_samples[:3,[knot_i]])
            print(force_samples[3:,[knot_i]])
            
            FL_moment = skew(s.toe_positions_data[1][0][knot_i].transpose()[0])@force_samples[:3,[knot_i]]
            BR_moment = skew(s.toe_positions_data[1][3][knot_i].transpose()[0])@force_samples[3:,[knot_i]]
            print(FL_moment[0,0])
            print(BR_moment[0,0])
            effective_moment.append(FL_moment[0,0]-BR_moment[0,0])
          
          ax.plot(  (t_stance ), (force_samples[0,:] + force_samples[3,:])/2, ("bo","ro")[int(s.is_twisting )],
                    (t_stance ), (force_samples[1,:] + force_samples[4,:])/2, ("bx","rx")[int(s.is_twisting )],
                    (t_stance ), (force_samples[2,:] + force_samples[5,:])/2, ( "b", "r")[int(s.is_twisting )],
                    label=trot_half_period
          )
          ax2.plot( (t_stance/trot_half_period ), (force_samples[0,:] + force_samples[3,:])/2, ("bo","ro")[int(s.is_twisting )],
                    (t_stance/trot_half_period ), (force_samples[1,:] + force_samples[4,:])/2, ("bx","rx")[int(s.is_twisting )],
                    (t_stance/trot_half_period ), (force_samples[2,:] + force_samples[5,:])/2, ( "b", "r")[int(s.is_twisting )],
                    label=trot_half_period)

          ax3.plot(t_stance/trot_half_period, effective_moment)

      velStr = "{vel:.1f}"
      ax.set_title("Velocity " + velStr.format(vel=float(runs_vel)) )
      ax.set_xlabel("Time")
      ax.set_ylabel("Total Force")
      ax2.set_title("Velocity " + velStr.format(vel=float(runs_vel)) )
      ax2.set_xlabel("Phase in Stance")
      ax2.set_ylabel("Total Force")
    fig.suptitle("Forces vs. Time\nBlue is rigid spine")
    fig2.suptitle("Forces vs. Phase\nBlue is rigid spine")
  
#############################################################
#############################################################
  if(False):
    counter = -1
    print("Building plots")
    fig,axs = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    fig2,axs2 = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    fig3,axs3 = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    print("Finished plots")
    for runs_vel in spirit_lists["rigid"]:
      counter +=1
      ax = axs[counter//3][counter%3]
      ax2 = axs2[counter//3][counter%3]
      ax3= axs3[counter//3][counter%3]
      for spine_type in ["twisting"]:
        for hp_key in spirit_lists[spine_type][runs_vel]:
          # Take mode 1 an normalize time
          s = spirit_lists_optimal[spine_type][runs_vel][hp_key][0]
          if (not s or not s.is_success):
            continue
          force_traj = s.dircon_traj.GetTrajectory("force_vars1")
          final_flight = s.dircon_traj.GetTrajectory("state_traj2")

          t_stance =force_traj.time_vector #np.linspace(force_traj.time_vector[0],force_traj.time_vector[-1],num_points)
          stance_period = force_traj.time_vector[-1]-force_traj.time_vector[0]
          trot_half_period = final_flight.time_vector[-1]

          force_samples = force_traj.datapoints
          effective_moment = []
          for knot_i in range(len(s.modes_state_knotpoints_time[1])):
            print(skew(s.toe_positions_data[1][0][knot_i].transpose()[0]))
            print(skew(s.toe_positions_data[1][3][knot_i].transpose()[0]))
            print(force_samples[[[1],[0],[2]],knot_i])
            print(force_samples[[[4],[3],[5]],knot_i])
            print("HERE")
            # s.kill()
            FL_moment = skew(s.toe_positions_data[1][0][knot_i].transpose()[0])@force_samples[[[1],[0],[2]],knot_i]
            BR_moment = skew(s.toe_positions_data[1][3][knot_i].transpose()[0])@force_samples[[[4],[3],[5]],knot_i]
            print(FL_moment)
            print(BR_moment)
            effective_moment.append((FL_moment-BR_moment)[0])
          print(effective_moment)
          # draw_spine(s, axis = ax,color='red')
          ax.plot(s.t, s.state_samples[s.map_state_name_to_index["joint_12"],:] , 'r')
          # ax.plot(s.t + 0.5 ,- s.state_samples[s.map_state_name_to_index["joint_12"],:] , 'r')
          ax2.plot(0.5*s.t/s.t[-1], s.input_samples[s.map_input_name_to_index["motor_12"],:] , 'r')
          ax2.plot(0.5*s.t/s.t[-1] + 0.5 ,- s.input_samples[s.map_input_name_to_index["motor_12"],:] , 'r')
          
          stance_interval = [s.modes_state_knotpoints_time[1][0],s.modes_state_knotpoints_time[1][-1]]

          ax2.axvspan(0.5 * stance_interval[0]/s.t[-1] , 0.5 * stance_interval[1]/s.t[-1]  , facecolor='gray', alpha=0.2, zorder=-100)
          ax2.axvspan(0.5 * stance_interval[0]/s.t[-1] + 0.5 , 0.5 * stance_interval[1]/s.t[-1] + 0.5 , facecolor='gray', alpha=0.2, zorder=-100)
          
          ax3.plot(t_stance/trot_half_period, effective_moment)
        ax2.set_ylim([-15,15])
        ax2.grid(axis='x')
        ax2.grid(axis='y', which='both')
      
      velStr = "{vel:.1f}"
      # ax.set_title("Velocity " + velStr.format(vel=float(runs_vel)) )
      # ax.set_xlabel("Time")
      # ax.set_ylabel("Total Force")
    # fig.suptitle("Spine vs. Time")
  if(True):
    fig_spine_nom, axs_spine_nom = plt.subplots(1,1,squeeze=False, sharex=True, sharey=True,figsize=(8, 4.5))
    axs_spine_nom = axs_spine_nom[0,0]
    axs_spine_nom_2 = axs_spine_nom.twinx()
    
    example_trot = spirit_lists_optimal["twisting"][2.0][0.1*3][0]
    print(example_trot.run_name)
    example_period = example_trot.t[-1]
    
    axs_spine_nom.plot(0.5*example_trot.t/example_period, example_trot.state_samples[example_trot.map_state_name_to_index["joint_12"],:], 'b' , label="position")
    axs_spine_nom.plot(0.5*(example_trot.t/example_period + 1),-example_trot.state_samples[example_trot.map_state_name_to_index["joint_12"],:], 'b' )
    
    axs_spine_nom_2.plot(0.5*example_trot.t/example_period, example_trot.input_samples[example_trot.map_input_name_to_index["motor_12"],:], 'r' , label="torque")
    axs_spine_nom_2.plot(0.5*(example_trot.t/example_period + 1),-example_trot.input_samples[example_trot.map_input_name_to_index["motor_12"],:], 'r' )

    stance_interval = [example_trot.modes_state_knotpoints_time[1][0], example_trot.modes_state_knotpoints_time[1][-1]]
    axs_spine_nom.axvspan(0.5 * stance_interval[0]/example_period, 0.5 * stance_interval[1]/example_period, facecolor='gray', alpha=0.2, zorder=-100)
    axs_spine_nom.axvspan(0.5 * stance_interval[0]/example_period + 0.5 , 0.5 * stance_interval[1]/example_period + 0.5 , facecolor='gray', alpha=0.2, zorder=-100)
    
    vals = axs_spine_nom.get_xticks()
    axs_spine_nom.set_xticklabels(['{:,.0%}'.format(x) for x in vals])
    axs_spine_nom.set_ylabel("Spine Displacement [rad]")
    axs_spine_nom_2.set_ylabel("Spine Torque [N m]")
    axs_spine_nom.legend(loc="upper left")
    axs_spine_nom_2.legend(loc="upper right")
    axs_spine_nom.grid(which="both")


    figure_dir = directory + "/figs"
    figure_name = "TrotNominal"
    safe_save_fig(figure_name, figure_dir, overwrite = True, figure=fig_spine_nom)


  plt.show()

#   #\\\\\################################
#     ######################################
# if(False):
#   counter = -1
#   print("Building plots")
#   fig,axs = plt.subplots(3,3,squeeze=False, sharex=True, sharey=True,figsize=(16, 10))
#   print("Finished plots")
#   for runs_vel in spirit_lists["rigid"]:
#     counter +=1
#     ax = axs[counter//3][counter%3]
#     for spine_type in ["twisting"]:
#       for hp_key in spirit_lists[spine_type][runs_vel]:
#         # Take mode 1 an normalize time
#         s = spirit_lists_optimal[spine_type][runs_vel][hp_key][0]
#         if (not s or not s.is_success):
#           continue
#         force_traj = s.dircon_traj.GetTrajectory("force_vars1")
#         final_flight = s.dircon_traj.GetTrajectory("state_traj2")

#         t_stance =force_traj.time_vector #np.linspace(force_traj.time_vector[0],force_traj.time_vector[-1],num_points)
#         stance_period = force_traj.time_vector[-1]-force_traj.time_vector[0]
#         trot_half_period = final_flight.time_vector[-1]

#         force_samples = force_traj.datapoints
        
#         # ax.plot( (t_stance ), (force_samples[0,:] + force_samples[3,:])/2, ("bo","ro")[int(s.is_twisting )],
#         #           (t_stance ), (force_samples[1,:] + force_samples[4,:])/2, ("bx","rx")[int(s.is_twisting )],
#         #           (t_stance ), (force_samples[2,:] + force_samples[5,:])/2, ("b","r")[int(s.is_twisting )]
#         # )
#         draw_spine(s, axis = ax,color='red')

#     velStr = "{vel:.1f}"
#     ax.set_title("Velocity " + velStr.format(vel=float(runs_vel)) )
#     ax.set_xlabel("Time")
#     ax.set_ylabel("Deflection [rad]")
#     fig.sup_title("Spine Position vs. Time")
#     plt.show()
#     s.kill()
#   #\\\\\################################

#   ######################### PLOT THE TURN SPINE ACTUATION FOR MOST "OPTIMAL" RUNS
#   if(False): # Turn this plot on 
#     for stride in stride_lengths:
#       print("Building the subplots figure for " + stride)
#       fig2, axs2 = plt.subplots(len(velocity_indices),len(param_indices),squeeze=False, sharex=True, sharey=True,figsize=(16, 10))
#       print("Figure and axes built")
#       fig2.suptitle("SpineTorque vs. Time\nStride Length:"  + stride)
#       for (index_vel, vel_dict) in enumerate(spirit_lists_optimal["twisting"][stride]):
#         # print(spirit_lists["twisting"]["ss"][vel_dict])
#         for (index_turn,turn_disp) in enumerate(spirit_lists_optimal["twisting"][stride][vel_dict]):
#           # print(turn_disp)
#           # for s in turn_disp:
#           if (not turn_disp[0]):
#             continue
#           s, run_power = turn_disp #spirit_lists_optimal["twisting"][stride][vel_dict][index_turn]
#           draw_spine_torque(s,axis=axs2[index_vel, index_turn])
#           draw_stance_phases(s, [1,4], axis=axs2[index_vel, index_turn])
#           axs2[index_vel, index_turn].grid(which='both')

#           velStr = "{vel:.1f}"
#           axs2[index_vel, 0].set_ylabel("Velocity " + velStr.format(vel=(index_vel+1)))
#           dispStr = "{disp:.2f}"
#           axs2[len(velocity_indices)-1, index_turn].set_xlabel("Yaw Disp " + dispStr.format(disp=(index_turn+1)*0.1))
      
#       figure_name = "TurnTorque_"+ stride
#       figure_dir = directory + "/test_figs"
#       safe_save_fig(figure_name, figure_dir, overwrite = True)


plt.show()
  
  # fig = plt.subplots()
  # spirit_lists["twisting"]["ms"]
  # fig = plt.subplots()
  # spirit_lists["twisting"]["ls"]
  # plt.show()


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
