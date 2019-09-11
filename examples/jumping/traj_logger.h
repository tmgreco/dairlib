#pragma once

#include <fstream>
#include <string>
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib{
namespace examples{

class TrajLogger{
public:
	TrajLogger(){

	}

	~TrajLogger(){

	}

	void writeTrajToFile(drake::trajectories::PiecewisePolynomial<double> traj, std::string filename){
		std::ofstream* fout = new std::ofstream(filename);
		double timesteps = 500.0;
		for(double t = 0; t < traj.end_time(); t += traj.end_time() / timesteps){
			(*fout) << t << " ";
			(*fout) << traj.value(t).transpose();
			(*fout) << "\n";
			// row = ;
			// for(int i = 0; i < traj.rows(); ++i){
			// 	(*fout) << traj.value()
			// }
		}
		fout->flush();
		fout->close();
		delete fout;
	}
};

}
}