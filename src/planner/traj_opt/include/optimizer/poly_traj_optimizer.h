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
    void setParam();
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

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

    double swarmGradCostP(Eigen::VectorXd &gdT);

    double swarmGraphGradCostP(Eigen::VectorXd &gdT);
    
    void feasibilityGradCostVandA(Eigen::VectorXd &gdT, double &vel_cost, double acc_cost);
    
    /* useful function */

    bool getFormationPos(std::vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos);

    void setDesiredFormation(int type){
      switch (type)
      {
        case FORMATION_TYPE::NONE_FORMATION :
        {
          use_formation_  = false;
          formation_size_ = 0;
          break;
        }

        case FORMATION_TYPE::REGULAR_HEXAGON :
        {
          // set the desired formation
          Eigen::Vector3d v0(0,0,0);
          Eigen::Vector3d v1(1.7321,-1,0);
          Eigen::Vector3d v2(0,-2,0);
          Eigen::Vector3d v3(-1.7321,-1,0);
          Eigen::Vector3d v4(-1.7321,1,0);
          Eigen::Vector3d v5(0,2,0);
          Eigen::Vector3d v6(1.7321,1,0);

          swarm_des_.push_back(v0);
          swarm_des_.push_back(v1);
          swarm_des_.push_back(v2);
          swarm_des_.push_back(v3);
          swarm_des_.push_back(v4);
          swarm_des_.push_back(v5);
          swarm_des_.push_back(v6);

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

        case FORMATION_TYPE::TWENTY_FIVE :
        {
          // set the desired formation
          Eigen::Vector3d v0(2,-2,0);
          Eigen::Vector3d v1(1,-2,0);
          Eigen::Vector3d v2(0,-2,0);
          Eigen::Vector3d v3(-1,-2,0);
          Eigen::Vector3d v4(-2,-2,0);
          Eigen::Vector3d v5(2,-1,0);
          Eigen::Vector3d v6(1,-1,0);
          Eigen::Vector3d v7(0,-1,0);
          Eigen::Vector3d v8(-1,-1,0);
          Eigen::Vector3d v9(-2,-1,0);
          Eigen::Vector3d v10(2,0,0);
          Eigen::Vector3d v11(1,0,0);
          Eigen::Vector3d v12(0,0,0);
          Eigen::Vector3d v13(-1,0,0);
          Eigen::Vector3d v14(-2,0,0);
          Eigen::Vector3d v15(2,1,0);
          Eigen::Vector3d v16(1,1,0);
          Eigen::Vector3d v17(0,1,0);
          Eigen::Vector3d v18(-1,1,0);
          Eigen::Vector3d v19(-2,1,0);
          // Eigen::Vector3d v20(2,2,0);
          // Eigen::Vector3d v21(1,2,0);
          // Eigen::Vector3d v22(0,2,0);
          // Eigen::Vector3d v23(-1,2,0);
          // Eigen::Vector3d v24(-2,2,0);
          

          swarm_des_.push_back(v0);
          swarm_des_.push_back(v1);
          swarm_des_.push_back(v2);
          swarm_des_.push_back(v3);
          swarm_des_.push_back(v4);
          swarm_des_.push_back(v5);
          swarm_des_.push_back(v6);
          swarm_des_.push_back(v7);
          swarm_des_.push_back(v8);
          swarm_des_.push_back(v9);
          swarm_des_.push_back(v10);
          swarm_des_.push_back(v11);
          swarm_des_.push_back(v12);
          swarm_des_.push_back(v13);
          swarm_des_.push_back(v14);
          swarm_des_.push_back(v15);
          swarm_des_.push_back(v16);
          swarm_des_.push_back(v17);
          swarm_des_.push_back(v18);
          swarm_des_.push_back(v19);
          // swarm_des_.push_back(v20);
          // swarm_des_.push_back(v21);
          // swarm_des_.push_back(v22);
          // swarm_des_.push_back(v23);
          // swarm_des_.push_back(v24);

          formation_size_ = swarm_des_.size();
          // construct the desired swarm graph
          // adj_in =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
          // adj_out = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
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

        case FORMATION_TYPE::DOUBLE_TEN:
        {
          // set the desired formation
          Eigen::Vector3d v0(6,0,0);
          Eigen::Vector3d v1(4,-1,0);
          Eigen::Vector3d v2(4,1,0);
          Eigen::Vector3d v3(2,-2,0);
          Eigen::Vector3d v4(2,0,0);
          Eigen::Vector3d v5(2,2,0);
          Eigen::Vector3d v6(0,-3,0);
          Eigen::Vector3d v7(0,-1,0);
          Eigen::Vector3d v8(0,1,0);
          Eigen::Vector3d v9(0,3,0);
          Eigen::Vector3d v10(6,0,2);
          Eigen::Vector3d v11(4,-1,2);
          Eigen::Vector3d v12(4,1,2);
          Eigen::Vector3d v13(2,-2,2);
          Eigen::Vector3d v14(2,0,2);
          Eigen::Vector3d v15(2,2,2);
          Eigen::Vector3d v16(0,-3,2);
          Eigen::Vector3d v17(0,-1,2);
          Eigen::Vector3d v18(0,1,2);
          Eigen::Vector3d v19(0,3,2);

          swarm_des_.push_back(v0);
          swarm_des_.push_back(v1);
          swarm_des_.push_back(v2);
          swarm_des_.push_back(v3);
          swarm_des_.push_back(v4);
          swarm_des_.push_back(v5);
          swarm_des_.push_back(v6);
          swarm_des_.push_back(v7);
          swarm_des_.push_back(v8);
          swarm_des_.push_back(v9);
          swarm_des_.push_back(v10);
          swarm_des_.push_back(v11);
          swarm_des_.push_back(v12);
          swarm_des_.push_back(v13);
          swarm_des_.push_back(v14);
          swarm_des_.push_back(v15);
          swarm_des_.push_back(v16);
          swarm_des_.push_back(v17);
          swarm_des_.push_back(v18);
          swarm_des_.push_back(v19);

          formation_size_ = swarm_des_.size();
          // construct the desired swarm graph
          // adj_in =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
          // adj_out = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
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
        
        case FORMATION_TYPE::OCTAHEDRON:
        {  // set the desired formation
          Eigen::Vector3d v0(-2,0,0);
          Eigen::Vector3d v1(2,0,0);
          Eigen::Vector3d v2(0,-2,0);
          Eigen::Vector3d v3(0,2,0);
          Eigen::Vector3d v4(0,0,-2);
          Eigen::Vector3d v5(0,0,2);


          swarm_des_.push_back(v0);
          swarm_des_.push_back(v1);
          swarm_des_.push_back(v2);
          swarm_des_.push_back(v3);
          swarm_des_.push_back(v4);
          swarm_des_.push_back(v5);


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
        case FORMATION_TYPE::CUBE:
        {  // set the desired formation
          Eigen::Vector3d v0(0,0,0);
          Eigen::Vector3d v1(0,2,0);
          Eigen::Vector3d v2(2,2,0);
          Eigen::Vector3d v3(2,0,0);
          Eigen::Vector3d v4(0,0,2);
          Eigen::Vector3d v5(0,2,2);
          Eigen::Vector3d v6(2,2,2);
          Eigen::Vector3d v7(2,0,2);


          swarm_des_.push_back(v0);
          swarm_des_.push_back(v1);
          swarm_des_.push_back(v2);
          swarm_des_.push_back(v3);
          swarm_des_.push_back(v4);
          swarm_des_.push_back(v5);
          swarm_des_.push_back(v6);
          swarm_des_.push_back(v7);



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