

#include <vector>
#include <string>
#include <unordered_map>
#include "lcmt_trajectory.h"


using std::vector;
using std::string;
using std::unordered_map;
using Eigen::VectorXd;
using Eigen::MatrixXd;


class LcmTrajectory{
public:
	struct Trajectory{
		string traj_name;
		VectorXd time_vector;
		MatrixXd datapoints;
		vector<string> datatypes;
	};

	struct TrajectoryMetadata{
		bool git_dirty_flag;
		string datatime;
		string name;
		string description;
		string git_commit_hash;
	};

	LcmTrajectory(	vector<Trajectory> trajectories, 
					vector<string> trajectory_names, 
					string name = "DEFAULT_NAME", 
					string description = "DEFAULT_DESCRIPTION"
					){

	}

	LcmTrajectory(lcmt_trajectory trajectory){

	}

	lcmt_trajectory generateLcmObject() const{

	}




private:
	TrajectoryMetadata metadata_;
	unordered_map<string, Trajectory> trajectories_;
	vector<string> trajectory_names_;

};