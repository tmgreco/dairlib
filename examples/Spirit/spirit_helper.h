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
  Surface(Eigen::Vector3d normal_ ,Eigen::Vector3d offset_){
    normal = normal_;
    offset = offset_;
    mu = std::numeric_limits<double>::infinity();
  }
};


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
                      Eigen::Vector3d normal,
                      Eigen::Vector3d offset,
                      double mu = std::numeric_limits<double>::infinity() ,
                      int num_knots = 7,
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
                    Eigen::Vector3d normal,
                    Eigen::Vector3d offset,
                    double mu = std::numeric_limits<double>::infinity(),
                    int num_knots = 7,
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
                              Eigen::Vector3d::UnitZ(), 
                              Eigen::Vector3d::Zero(), 
                              std::numeric_limits<double>::infinity(),
                              num_knots,
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
    void addMode(dairlib::QuadrupedalMode qMode){ mode_sequence.push_back(qMode); }
    void addFlight(int num_knots,
                   double minT = 0,
                   double maxT = std::numeric_limits<double>::infinity()){
      mode_sequence.push_back(dairlib::QuadrupedalMode::flight(num_knots,
                                                               minT,
                                                               maxT));
    }
    void print(){ for(auto mode : mode_sequence){mode.print();} }
};