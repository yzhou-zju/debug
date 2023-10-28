/*** 
 * @Author: ztr
 * @Date: 2022-05-12 00:00:09
 * @LastEditTime: 2022-05-12 00:00:10
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /Pipline-Swarm-Formation/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h
 */
#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
// #include <path_searching/dyn_a_star.h>
// #include <path_searching/kinodynamic_astar.h>
// #include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"
#include <frob_test/swarm_graph.hpp>
#include <fstream>
using namespace std;
namespace ego_planner
{
  
  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    
    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class FixedTimePoints
  {
  public:
    double start_time;
    double sampling_time_step;
    Eigen::MatrixXd points;            // visualization
    bool enable_decouple_swarm_graph;  // if calculate local min of swarm graph in advance

    double duration;
    int    sampling_num;
    int    idx_of_sets; 
    
    Eigen::VectorXd relative_time_ahead;
    std::vector<SwarmGraph> swarm_graph_advance_sets;
    std::vector<std::vector<Eigen::Vector3d>> swarm_pos_advance_sets;
    std::vector<std::vector<Eigen::Vector3d>> swarm_vel_advance_sets;
    std::vector<Eigen::Vector3d> local_min_advance_sets;
    
    void resetBuffer(){
      start_time = 0.0;
      duration   = 0.0;
      sampling_num = 0; 
      idx_of_sets  = 0;

      relative_time_ahead.resize(0);
      swarm_graph_advance_sets.clear();
      swarm_pos_advance_sets.clear();
      swarm_vel_advance_sets.clear();
      local_min_advance_sets.clear();
      return;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
  

  class PolyTrajOptimizer
  {

  private:
    // GridMap::Ptr grid_map_;
    bool use_kino_;
    // AStar::Ptr a_star_;
    // KinodynamicAstar::Ptr kino_a_star_;

    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstrainPoints cps_;              // uniform sampling需要修改这部分?????
    FixedTimePoints time_cps_;         // fixed time step sampling
    SwarmGraph::Ptr swarm_graph_;
    int drone_id_;
    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_;     // optimization variables
    int piece_num_;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    bool enable_fix_step_ = false;

    string result_fn_;
    fstream result_file_;

    bool record_once_ = true;  // debug for calculating the local min of swarm graph
    std::ofstream log_;
    int opt_local_min_loop_num_;
    int opt_local_min_loop_sum_num_ = 0;
    double collision_check_time_end_=0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    enum FORMATION_TYPE
    {
      NONE_FORMATION        = 0,
      RECTANGLE_WITH_CENTER = 1,
      REGULAR_TETRAHEDRON   = 2,
      HEXAGON               = 3,
      Z_LETTER              = 4,
      J_LETTER              = 5,
      U_LETTER              = 6,
      REGULAR_HEXAGON       = 7,
      ARROW                 = 8,
      HEART                 = 9,
      RECTANGLE             = 10,
      OCTAHEDRON            = 11,
      DOUBLE_TEN            = 20,
      TWENTY_FIVE           = 25,
      CUBE                  =88
    };

    enum FORMATION_METHOD_TYPE
    {
      SWARM_GRAPH           = 0, // (default)
      LEADER_POSITION       = 1,
      RELATIVE_POSITION     = 2,
      VRB_METHOD            = 3
    };

    /* optimization parameters */
    double wei_smooth_;                      // smooth weight
    double wei_obs_;                         // obstacle weight
    double wei_swarm_;                       // swarm weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double wei_formation_;                   // swarm formation simllarity
    double wei_gather_;                      // swarm gathering penalty
    
    double obs_clearance_;                   // safe distance between uav and obstacles
    double swarm_clearance_;                 // safe distance between uav and uav
    double swarm_gather_threshold_;          // threshold distance between uav and swarm center
    double max_vel_, max_acc_;               // dynamic limits
    
    bool is_set_swarm_advanced_;
    
    int    formation_type_;
    int    formation_method_type_;
    int    formation_size_;
    bool   use_formation_ = true;
    bool   is_other_assigning_ = false;

    std::vector<Eigen::Vector3d> swarm_des_;
    std::vector<int> adj_in_, adj_out_;
    Eigen::VectorXi assignment_;

    double t_now_;
    // benchmark 
    vector<Eigen::Vector3d> formation_relative_dist_;

  public:
    
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {ztr_log3.close();}

    /* set variables */
    void setParam(bool m_or_not_, vector<int> leader_id_);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    // void setSwarmTrajs();
    void setDroneId(const int drone_id);
    double error_dist(poly_traj::Trajectory traj_final);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; }
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    inline int getFormationSize() { return formation_size_; }
    std::vector<Eigen::Vector3d> getSwarmGraphInit() { return swarm_graph_->getDesNodesInit(); }
    double getFormationError(std::vector<Eigen::Vector3d> swarm_graph_pos);
    void updateSwarmGraph(Eigen::VectorXi assignment);
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }
    
    /* main planning API */
    bool optimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, const bool use_formation);
    
  
  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    /* 
      calculate cost of trajectory with fixed control points sampling
    */
    template <typename EIGENVEC>
    void addPVAGradCost2CTwithFixedCtrlPoints(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);
    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);
    
    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool swarmGraphGradCostP(const int i_dp,
                             const double t,
                             const Eigen::Vector3d &p,
                             const Eigen::Vector3d &v,
                             Eigen::Vector3d &gradp,
                             double &gradt,
                             double &grad_prev_t,
                             double &costp);
    
    bool swarmGatherCostGradP(const int i_dp,
                              const double t,
                              const Eigen::Vector3d &p,
                              const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradp,
                              double &gradt,
                              double &grad_prev_t,
                              double &costp);
    
    // benchmark
    bool leaderPosFormationCostGradP(const int i_dp,
                                    const double t,
                                    const Eigen::Vector3d &p,
                                    const Eigen::Vector3d &v,
                                    Eigen::Vector3d &gradp,
                                    double &gradt,
                                    double &grad_prev_t,
                                    double &costp);
    
    bool relativePosFormationCostGradP(const int i_dp,
                                       const double t,
                                       const Eigen::Vector3d &p,
                                       const Eigen::Vector3d &v,
                                       Eigen::Vector3d &gradp,
                                       double &gradt,
                                       double &grad_prev_t,
                                       double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    /* 
      calculate cost of trajectory with fixed time step sampling
    */
    template <typename EIGENVEC>
    void addPVAGradCost2CTwithFixedTimeSteps(EIGENVEC &gdT, Eigen::VectorXd &costs);

    /* use L-NFGS optimizer to calculate the local minimum of swarm graph */
    void setSwarmGraphInAdavanced(const Eigen::VectorXd initT);
    static double swarmGraphCostCallback(void *func_data, const double *x, double *grad, const int n);

    double obstacleGradCostP(Eigen::VectorXd &gdT);
    bool decide_contin(int id_get);
    double swarmGradCostP(Eigen::VectorXd &gdT);

    double swarmGraphGradCostP(Eigen::VectorXd &gdT);
    
    void feasibilityGradCostVandA(Eigen::VectorXd &gdT, double &vel_cost, double acc_cost);
    
    /* useful function */

    bool getFormationPos(std::vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos);

    void setDesiredFormation(int type, bool m_or_not, vector<int> _leader_id){
      switch (type)
      {
        case FORMATION_TYPE::CUBE:
        {  // set the desired formation
          std::vector<Eigen::Vector3d> v;
          v.resize(80);
          v[0] = {1.0, -2.0, 1.0};
          v[1] = {-1.0, -2.0,1.0};
          v[2] = {1.0,  0.0, 1.0}; 
          v[3] = {-1.0, 0.0, 1.0};
          v[4] = {1.0,  2.0, 1.0};
          v[5] = {-1.0,  2.0,1.0};
          v[6] = {1.0,  4.0, 1.0};
          v[7] = {-1.0, 4.0, 1.0};

          v[8] = {1.0, -2.0, 3.0};
          v[9] = {-1.0, -2.0,3.0};
          v[10] = {1.0,  0.0, 3.0}; 
          v[11] = {-1.0, 0.0, 3.0};
          v[12] = {1.0,  2.0, 3.0};
          v[13] = {-1.0,  2.0,3.0};
          v[14] = {1.0,  4.0, 3.0};
          v[15] = {-1.0, 4.0, 3.0};

          v[16] = {1.0, -2.0, -1.0};
          v[17] = {-1.0, -2.0,-1.0};
          v[18] = {1.0,  0.0, -1.0}; 
          v[19] = {-1.0, 0.0, -1.0};
          v[20] = {1.0,  2.0, -1.0};
          v[21] = {-1.0,  2.0,-1.0};
          v[22] = {1.0,  4.0, -1.0};
          v[23] = {-1.0, 4.0, -1.0};

          v[24] = {1.0, -2.0, 5.0};
          v[25] = {-1.0, -2.0,5.0};
          v[26] = {1.0,  0.0, 5.0}; 
          v[27] = {-1.0, 0.0, 5.0};
          v[28] = {1.0,  2.0, 5.0};
          v[29] = {-1.0,  2.0,5.0};
          v[30] = {1.0,  4.0, 5.0};
          v[31] = {-1.0, 4.0, 5.0};

          v[32] = {1.0, -2.0, -3.0};
          v[33] = {-1.0, -2.0,-3.0};
          v[34] = {1.0,  0.0, -3.0}; 
          v[35] = {-1.0, 0.0, -3.0};
          v[36] = {1.0,  2.0, -3.0};
          v[37] = {-1.0,  2.0,-3.0};
          v[38] = {1.0,  4.0, -3.0};
          v[39] = {-1.0, 4.0, -3.0};

          v[40] = {1.0, -2.0, -5.0};
          v[41] = {-1.0, -2.0,-5.0};
          v[42] = {1.0,  0.0, -5.0}; 
          v[43] = {-1.0, 0.0, -5.0};
          v[44] = {1.0,  2.0, -5.0};
          v[45] = {-1.0,  2.0,-5.0};
          v[46] = {1.0,  4.0, -5.0};
          v[47] = {-1.0, 4.0, -5.0};

          v[48] = {1.0, -2.0,  -7.0};
          v[49] = {-1.0, -2.0, -7.0};
          v[50] = {1.0,  0.0, -7.0}; 
          v[51] = {-1.0, 0.0, -7.0};
          v[52] = {1.0,  2.0, -7.0};
          v[53] = {-1.0,  2.0,-7.0};
          v[54] = {1.0,  4.0, -7.0};
          v[55] = {-1.0, 4.0, -7.0};

          v[56] = {1.0, -2.0, 7.0};
          v[57] = {-1.0, -2.0,7.0};
          v[58] = {1.0,  0.0, 7.0}; 
          v[59] = {-1.0, 0.0, 7.0};
          v[60] = {1.0,  2.0, 7.0};
          v[61] = {-1.0,  2.0,7.0};
          v[62] = {1.0,  4.0, 7.0};
          v[63] = {-1.0, 4.0, 7.0};

          v[64] = {1.0, -2.0, 9.0};
          v[65] = {-1.0, -2.0,9.0};
          v[66] = {1.0,  0.0, 9.0}; 
          v[67] = {-1.0, 0.0, 9.0};
          v[68] = {1.0,  2.0, 9.0};
          v[69] = {-1.0,  2.0,9.0};
          v[70] = {1.0,  4.0, 9.0};
          v[71] = {-1.0, 4.0, 9.0};
          swarm_des_.push_back(v[0]);
          swarm_des_.push_back(v[1]);
          swarm_des_.push_back(v[2]);
          swarm_des_.push_back(v[3]);
          swarm_des_.push_back(v[4]);
          swarm_des_.push_back(v[5]);
          swarm_des_.push_back(v[6]);
          swarm_des_.push_back(v[7]);
          if(m_or_not)
          {
            swarm_des_.push_back(v[_leader_id[0]]);
            swarm_des_.push_back(v[_leader_id[1]]);
            swarm_des_.push_back(v[_leader_id[2]]);
            swarm_des_.push_back(v[_leader_id[3]]);
            swarm_des_.push_back(v[_leader_id[4]]);
            swarm_des_.push_back(v[_leader_id[5]]);
            swarm_des_.push_back(v[_leader_id[6]]);
            swarm_des_.push_back(v[_leader_id[7]]);
          }else{
          
          swarm_des_.push_back(v[8]);
          swarm_des_.push_back(v[9]);
          swarm_des_.push_back(v[10]);
          swarm_des_.push_back(v[11]);
          swarm_des_.push_back(v[12]);
          swarm_des_.push_back(v[13]);
          swarm_des_.push_back(v[14]);
          swarm_des_.push_back(v[15]);

          swarm_des_.push_back(v[16]);
          swarm_des_.push_back(v[17]);
          swarm_des_.push_back(v[18]);
          swarm_des_.push_back(v[19]);
          swarm_des_.push_back(v[20]);
          swarm_des_.push_back(v[21]);
          swarm_des_.push_back(v[22]);
          swarm_des_.push_back(v[23]);

          swarm_des_.push_back(v[24]);
          swarm_des_.push_back(v[25]);
          swarm_des_.push_back(v[26]);
          swarm_des_.push_back(v[27]);
          swarm_des_.push_back(v[28]);
          swarm_des_.push_back(v[29]);
          swarm_des_.push_back(v[30]);
          swarm_des_.push_back(v[31]);

          swarm_des_.push_back(v[32]);
          swarm_des_.push_back(v[33]);
          swarm_des_.push_back(v[34]);
          swarm_des_.push_back(v[35]);
          swarm_des_.push_back(v[36]);
          swarm_des_.push_back(v[37]);
          swarm_des_.push_back(v[38]);
          swarm_des_.push_back(v[39]);

          swarm_des_.push_back(v[40]);
          swarm_des_.push_back(v[41]);
          swarm_des_.push_back(v[42]);
          swarm_des_.push_back(v[43]);
          swarm_des_.push_back(v[44]);
          swarm_des_.push_back(v[45]);
          swarm_des_.push_back(v[46]);
          swarm_des_.push_back(v[47]);

          swarm_des_.push_back(v[48]);
          swarm_des_.push_back(v[49]);
          swarm_des_.push_back(v[50]);
          swarm_des_.push_back(v[51]);
          swarm_des_.push_back(v[52]);
          swarm_des_.push_back(v[53]);
          swarm_des_.push_back(v[54]);
          swarm_des_.push_back(v[55]);

          swarm_des_.push_back(v[56]);
          swarm_des_.push_back(v[57]);
          swarm_des_.push_back(v[58]);
          swarm_des_.push_back(v[59]);
          swarm_des_.push_back(v[60]);
          swarm_des_.push_back(v[61]);
          swarm_des_.push_back(v[62]);
          swarm_des_.push_back(v[63]);

          swarm_des_.push_back(v[64]);
          swarm_des_.push_back(v[65]);
          swarm_des_.push_back(v[66]);
          swarm_des_.push_back(v[67]);
          swarm_des_.push_back(v[68]);
          swarm_des_.push_back(v[69]);
          swarm_des_.push_back(v[70]);
          swarm_des_.push_back(v[71]);
          }
          // Eigen::Vector3d v82(1.0, -2.0, 11.0);
          // Eigen::Vector3d v83(-1.0, -2.0,11.0);
          // Eigen::Vector3d v84(1.0,  0.0, 11.0); 
          // Eigen::Vector3d v85(-1.0, 0.0, 11.0);
          // Eigen::Vector3d v86(1.0,  2.0, 11.0);
          // Eigen::Vector3d v87(-1.0,  2.0,11.0);
          // Eigen::Vector3d v88(1.0,  4.0, 11.0);
          // Eigen::Vector3d v89(-1.0, 4.0, 11.0);
          // swarm_des_.push_back(v82);
          // swarm_des_.push_back(v83);
          // swarm_des_.push_back(v84);
          // swarm_des_.push_back(v85);
          // swarm_des_.push_back(v86);
          // swarm_des_.push_back(v87);
          // swarm_des_.push_back(v88);
          // swarm_des_.push_back(v89);

          formation_size_ = swarm_des_.size();
          // construct the desired swarm graph
          // adj_in_ =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
          // adj_out_ = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
          for (int i=0; i<formation_size_; i++){
            for ( int j=0; j<i; j++){
              adj_in_.push_back(i);
              adj_out_.push_back(j);
            }
          }
          assignment_ = Eigen::VectorXi::LinSpaced(formation_size_, 0, formation_size_ - 1 );
          swarm_graph_->setDesiredForm(swarm_des_, adj_in_, adj_out_);
          break;
              
            }

        default:
          break;
      }
    }

  public:
    std::ofstream ztr_log3;
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
    
  };

} // namespace ego_planner
#endif