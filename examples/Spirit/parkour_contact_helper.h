// Better mode sequence helper
  //TAKE CONTACT SURFACES AND CREATE
    // Bounding mode sequence
    // Pronking mode sequence
// Better constraint management
  // What are the constraints for position?

//need to change the bound to include  
#include <assert.h>
#include <algorithm>



// bool* boolStr2QuadArr(bool quadArr[4], string boolstring){

//   N = boolstring.length();
//   transform(boolstring.begin(), boolstring.end(), boolstring.begin(), ::tolower);
//   bool dummyArr[N];
//   for( int i = 0; i<N; i++){
//     assert(boolstring[i]=="t"||boolstring[i]=="f"||boolstring[i]=="0"||boolstring[i]=="1") && "String must be made up of t's and f's or 1's and 0's";
//     dummyArr[i] = (boolstring[i]!="f"||boolstring[i]!="0");
//   }
//   return dummyArr;
// }

void addOffsetConstraint(MultibodyPlant<T>& plant,
                dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars, 
                Eigen::Vector3d offset,
                double dist_along_normal = -1, 
                Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(), 
                double eps ){
  
  Eigen::Matrix3d wRc = drake::math::ComputeBasisFromAxis(2, normal);//Rotation world to contact frame
  Eigen::Vector3d offset_c = wRc.Tranpose()*offset;
  Eigen::Vector3d lb; //Lower Bound
  Eigen::Vector3d ub; //Upper Bound
  Eigen::Vector3d tangent_error = Eigen::Vector3d::UnitX() * eps + Eigen::Vector3d::UnitY() * eps;
  if (dist_along_normal==std::numeric_limits<double>::infinity() || dist_along_normal < 0 ){
    lb = offset_c - tangent_error;
    ub = offset_c + tangent_error + Eigen::Vector3d::UnitZ()*std::numeric_limits<double>::infinity();
  }
  else{
    Eigen::Vector3d height_vect = Eigen::Vector3d::UnitZ()*dist_along_normal;
    lb = offset_c + height_vect - tangent_error - Eigen::Vector3d::UnitZ()*eps;
    ub = offset_c + height_vect + tangent_error + Eigen::Vector3d::UnitZ()*eps;
  }
  trajopt.AddLinearConstraint(wRc.Transpose(), lb, ub);
  
}


class QuadrupedalMode {
  private:
    bool contact_set_[4];
    int num_knots_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d offset_;
    double mu_;
    double minT_;
    double maxT_;

  public:
    QuadrupedalMode(  bool contact_set[4],
                      int num_knots,
                      Eigen::Vector3d normal,
                      Eigen::Vector3d offset,
                      double mu = std::numeric_limits<double>::infinity() ,
                      double minT = 0,
                      double maxT = std::numeric_limits<double>::infinity()   ) {
      for (size_t i = 0; i < 4; i++)
      {
        contact_set_[i] = contact_set[i];
      }
      num_knots_ = num_knots;
      normal_ = normal;
      offset_ = offset;
      mu_ = mu;
      minT_ = minT;
      maxT_ = maxT;
      }
      
    QuadrupedalMode(std::string contact_set_str,
                    int num_knots,
                    Eigen::Vector3d normal,
                    Eigen::Vector3d offset,
                    double mu = std::numeric_limits<double>::infinity(),
                    double minT = 0,
                    double maxT = std::numeric_limits<double>::infinity()) {
      assert(contact_set_str.length() == 4  && "String must be four characters long");
      transform(contact_set_str.begin(), contact_set_str.end(), contact_set_str.begin(), ::tolower);
      for( int i = 0; i<4; i++){
        assert(contact_set_str[i]=='t'||contact_set_str[i]=='f'||contact_set_str[i]=='0'||contact_set_str[i]=='1' && "String must be made up of t's and f's or 1's and 0's");
        contact_set_[i] = (contact_set_str[i]!='f'&&contact_set_str[i]!='0');
        }      
      num_knots_ = num_knots;
      normal_ = normal;
      normal.normalize();
      offset_ = offset;
      mu_ = mu;
      minT_ = minT;
      maxT_ = maxT;
      }

    static QuadrupedalMode flight(int num_knots,
                                 double minT = 0,
                                  double maxT = std::numeric_limits<double>::infinity()){
      return QuadrupedalMode( {false,false,false,false},
                              num_knots,
                              Eigen::Vector3d::UnitZ(), 
                              Eigen::Vector3d::Zero(), 
                              std::numeric_limits<double>::infinity(),
                              minT,
                              maxT);
    }
    // Simple verbose printing method
    void print(){
      std::cout << "************** Quadrupedal Mode *****" << std::endl;
      std::cout << std::boolalpha;
      std::cout << "Active Toe Contacts: " << contact_set_[0] << ", " << contact_set_[1] << ", " << contact_set_[2] << ", " << contact_set_[3] << std::endl;
      std::cout << "Number of Knots: " << num_knots_ << std::endl;
      std::cout << "Contact Normal: " << "(" << normal_.transpose() << ")^T" << std::endl;
      std::cout << "Contact Offset: " << "(" << offset_.transpose() << ")^T" << std::endl;
      std::cout << "Friction coefficient, mu: " << mu_ << std::endl;
      std::cout << "Time Range: " << "min: " << minT_ << " max: "<<  maxT_ << std::endl;
      std::cout << "*************************************" << std::endl;
    }
    // Get and Set methods
    Eigen::Vector3d normal(){return normal_;}
    Eigen::Vector3d offset(){return offset_;}
    double mu(){return mu_;}
    double minT(){return minT_;}
    double maxT(){return maxT_;}
    int num_knots(){return num_knots_;}

    void set_normal(Eigen::Vector3d new_normal){ normal_ = new_normal;}
    void set_offset(Eigen::Vector3d new_offset){ offset_ = new_offset;}
    void set_mu(double mu){ mu_ = mu;}
    void set_minT(double minT){ minT_ = minT;}
    void set_maxT(double maxT){ maxT_ = maxT;}
    void set_num_knots(int num_knots){num_knots_ = num_knots;}

    Eigen::Vector3d affinePosition(double height){ return (offset() + (normal() * height)); }
    
};


class QuadrupedalModeSequence {
  public:
    std::vector<dairlib::QuadrupedalMode> mode_sequence;
    void addMode(QuadrupedalMode qMode){ mode_sequence.push_back(qMode); }
    void addFlight(int num_knots,
                   double minT = 0,
                   double maxT = std::numeric_limits<double>::infinity()){
      mode_sequence.push_back(dairlib::QuadrupedalMode::flight(num_knots,
                                                               minT,
                                                               maxT));
    }
    void print(){ for(auto mode : mode_sequence){mode.print();} }
};

class Surface {
 public:
  Eigen::Vector3d normal;
  Eigen::Vector3d offset;
  double mu;
  Surface(Eigen::Vector3d normal_ ,Eigen::Vector3d offset_, double mu_){
    normal = normal_;
    offset = offset_;
    mu = mu_;
  }
}

void getParkourBoundModeSequence(QuadrupedalModeSequence quad_mode_seq, 
                                 double mu,
                                 int num_knots, 
                                 double min_T,
                                 double max_T,
                                 Eigen::Vector3d init_surf_normal, 
                                 Eigen::Vector3d init_surf_offset, 
                                 Eigen::Vector3d mid_surf_normal, 
                                 Eigen::Vector3d mid_surf_offset, 
                                 Eigen::Vector3d final_surf_normal, 
                                 Eigen::Vector3d final_surf_normal){
  Eigen::Vector3d unitZ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  double min_stance_T = 0.02;
  int num_knots = 7;

  Eigen::Vector3d init_surf_normal; init_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d init_surf_offset; init_surf_offset << 0.0, 0.0, 0.0;

  Eigen::Vector3d mid_surf_normal; mid_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d mid_surf_offset; mid_surf_offset << 0.5, 0.0, 0.1;

  Eigen::Vector3d final_surf_normal; final_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d final_surf_offset; final_surf_offset << 1.0, 0.0, 0.0;

  //Front Left, Back Left, Front Right, Back Right
  quad_mode_seq.addMode( QuadrupedalMode( "1111", num_knots, init_surf_normal, init_surf_offset, mu, 0.02, 0.5) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", num_knots, init_surf_normal, init_surf_offset, mu, 0.02, 0.5) );
  quad_mode_seq.addFlight(num_knots, 0.02, 1.0);
  quad_mode_seq.addFlight(num_knots, 0.02, 1.0);
  quad_mode_seq.addMode( QuadrupedalMode( "1010", num_knots, mid_surf_normal, mid_surf_offset, mu, 0.02, 0.2) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", num_knots, mid_surf_normal, mid_surf_offset, mu, 0.02, 0.2) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", num_knots, mid_surf_normal, mid_surf_offset, mu, 0.02, 0.2) );
  quad_mode_seq.addFlight(num_knots, 0.02, 1.0);
  quad_mode_seq.addFlight(num_knots, 0.02, 1.0);
  quad_mode_seq.addMode( QuadrupedalMode( "1010", num_knots, final_surf_normal, final_surf_offset, mu, 0.02, 0.5) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", num_knots, final_surf_normal, final_surf_offset, mu, 0.02, 0.5) );
}