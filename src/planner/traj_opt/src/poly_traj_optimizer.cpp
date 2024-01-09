
#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState_init, const Eigen::MatrixXd &finState_init,
      const Eigen::MatrixXd &initInnerPts_init, const Eigen::VectorXd &initT_init,
      Eigen::MatrixXd &optimal_points, const bool use_formation, const double t_now_get_)
  {
    Eigen::Matrix3d iniState;
    Eigen::Matrix3d finState;
    Eigen::MatrixXd initInnerPts;
    Eigen::VectorXd initT;
    Eigen::Vector3d start_pos = iniState_init.col(0);
    Eigen::Vector3d end_pos = finState_init.col(0);
    // std::cout<<"drone_id_@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@:"<<drone_id_<<std::endl;
    std::vector<Eigen::Vector3d> a_star_path;
    std::vector<Eigen::Vector3d> simple_path;
    bool success = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos, simple_path, a_star_path);
    astar_path = simple_path;
    // for (size_t i = 0; i < simple_path.size(); i++)
    // {
    //   std::cout << "第" << i << "个路径点是:" << simple_path[i].transpose() << " " << std::endl;
    // }
    if(success)
    {
      iniState << simple_path[0], iniState_init.col(1), iniState_init.col(2);
      finState << simple_path.back(), finState_init.col(1), finState_init.col(2);
      initT.resize(simple_path.size()-1);
      for(int i=0;i<simple_path.size()-1;i++)
      {
        initT[i] = sqrt((simple_path[i+1][0]-simple_path[i][0])*(simple_path[i+1][0]-simple_path[i][0])+
                      (simple_path[i+1][1]-simple_path[i][1])*(simple_path[i+1][1]-simple_path[i][1])+
                      (simple_path[i+1][2]-simple_path[i][2])*(simple_path[i+1][2]-simple_path[i][2]))/max_vel_;
      }
      Eigen::MatrixXd combinedPath(3, simple_path.size()-2);
      for(int i=1;i<simple_path.size()-1;i++)
      {
        // initInnerPts << simple_path[i];
        combinedPath.col(i-1) = simple_path[i];
      }
      initInnerPts = combinedPath;
        
    }else{
      // std::cout<<"a star failed:"<<std::endl;
      // std::cout<<"start:"<<start_pos<<std::endl;
      // std::cout<<"end:"<<end_pos<<std::endl;
      // initT = initT_init;
      iniState = iniState_init;
      finState = finState_init;
      // initInnerPts = initInnerPts_init;
      Eigen::MatrixXd combinedPath(3, 12);
      initT.resize(13);
      for(int i=0;i<13;i++)
      {
        initT[i] = 0.9*3/max_vel_;
      }
      for(int i=1;i<13;i++)
      {
        // initInnerPts << simple_path[i];
        combinedPath.col(i-1) = start_pos + (i)*(end_pos - start_pos)/(initT.size());
        // std::cout<<"waypoint::"<<combinedPath.col(i-1)<<std::endl;
      }
      initInnerPts = combinedPath;
    }
    
    // initT = initT_init;
    // iniState = iniState_init;
    // finState = finState_init;
    // initInnerPts = initInnerPts_init;
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = t_now_get_;
    // t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);
    // jerkOpt_.generate(iniState, initT);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    ros::Time t0 = ros::Time::now(), t1, t2;
    bool use_formation_temp = use_formation_;

    double q[variable_num_];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    Eigen::Map<Eigen::VectorXd> Vt(q + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.0;     //0.1
    lbfgs_params.min_step = 1e-24;    //1e-32
    lbfgs_params.max_linesearch = 120; // default:60
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-4;
    lbfgs_params.line_search_type = 0;
    
    /* trick : for real-time optimization */
    if (use_formation)
    {
      // consider formation
      // so we use less iterations for real time optimization
      lbfgs_params.max_iterations = 100; //20      
    } else{
      // do not consider formation
      // so we use more iterations for more precise optimization results
      lbfgs_params.max_iterations = 100;
      use_formation_ = false;
    }
  
    // calculate in advance a part of swarm graph in other agents' local traj for each replanning
    opt_local_min_loop_sum_num_ = 0;
    if (enable_fix_step_&&swarm_trajs_->size()>=2)
      setSwarmGraphInAdavanced(initT);

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */
    t1 = ros::Time::now();
    lbfgs::line_time_log_close();
    int result = lbfgs::lbfgs_optimize(variable_num_,
                                       q,
                                       &final_cost,
                                       PolyTrajOptimizer::costFunctionCallback,
                                       NULL,
                                       PolyTrajOptimizer::earlyExitCallback,
                                       this,
                                       &lbfgs_params);
    
    use_formation_ = use_formation_temp;

    t2 = ros::Time::now();
    double optimize_time_ms = (t2 - t1).toSec() * 1000;
    double total_time_ms = (t2 - t0).toSec() * 1000;

    // debug
    // cout<<"final_cost:"<<final_cost<<endl;
    // cout<<"result:"<<result<<endl;
    // cout << "[debug] final total: " << jerkOpt_.get_T1().sum() << ", T: " << jerkOpt_.get_T1().transpose() << endl;
    printf("\033[32m[Optimization]: iter=%d, use_formation=%d, optimize time(ms)=%5.3f, total time(ms)=%5.3f, graph num=%i \n\033[0m", iter_num_, use_formation, optimize_time_ms, total_time_ms, opt_local_min_loop_sum_num_);
    
    // print the optimization result
    // if (result != lbfgs::LBFGS_STOP && result != lbfgs::LBFGSERR_MAXIMUMITERATION)
    //   printf("\033[33m[Optimization]: %s\n\033[0m", lbfgs::lbfgs_strerror(result));
    
    // optimal_points for visulization
    if (!enable_fix_step_)
      optimal_points = cps_.points;
    else
      optimal_points = time_cps_.points;

    if (enable_fix_step_)
      time_cps_.resetBuffer();
    
    // check collision
    bool occ = false;
    double collision_relative_time;


    if (occ){
      printf("\033[31m[Optimization]: Failed! Trajectory is collied in the relative time=%5.3f \n\033[0m", collision_relative_time);
      return false;
    }
    else
      return true;
  }

  void PolyTrajOptimizer::setSwarmGraphInAdavanced(const Eigen::VectorXd initT)
  {
    is_set_swarm_advanced_ = false;
    if (use_formation_)
    {
      int size = swarm_trajs_->size();
      if (drone_id_ == formation_size_ - 1&&!first_planner_)
      {
        size = formation_size_;
      }
      if (size < formation_size_)
      {
        return;
      }
      time_cps_.start_time = t_now_;

      // calculate the effective time duration
      time_cps_.duration = initT.sum();
      time_cps_.sampling_num = floor(time_cps_.duration / time_cps_.sampling_time_step) + 1; // should be >= 1

      // calculate the effective time duration of swarm
      time_cps_.relative_time_ahead.resize(size);

      for (int id = 0; id < size; id++)
      {
        if (id == drone_id_)
        {
          time_cps_.relative_time_ahead(id) = 0.0;
        }
        else
        {
          time_cps_.relative_time_ahead(id) = time_cps_.start_time - swarm_trajs_->at(id).start_time;
        }
      }
      // calculate in advance for swarm graph
      time_cps_.swarm_graph_advance_sets.resize(time_cps_.sampling_num);
      time_cps_.swarm_pos_advance_sets.resize(time_cps_.sampling_num);
      time_cps_.swarm_vel_advance_sets.resize(time_cps_.sampling_num);

      double accumulated_step_swarm(0.0);
      for (int i=0; i<time_cps_.sampling_num; 
           i++, accumulated_step_swarm += time_cps_.sampling_time_step) 
      {
        SwarmGraph swarm_graph;
        swarm_graph.setDesiredForm(swarm_des_, adj_in_, adj_out_);
        swarm_graph.setAssignment(assignment_);
        std::vector<Eigen::Vector3d> swarm_pos(size), swarm_vel(size);
        // get swarm pos and vel
        for (int id = 0; id < size; id++)
        {
          // std::cout<<"opt !!!!!!!!!!!!!!!!!!!22222:"<<id<<std::endl;
          // std::cout<<"opt !traj:"<<id<<"   "<<swarm_trajs_->at(id).duration<<std::endl;
          Eigen::Vector3d pos, vel;
          if (id == drone_id_)
          {
            // own pos and vel, set as zero now. And they will be updated in the optimized loop
            pos = Eigen::Vector3d::Zero();
            vel = Eigen::Vector3d::Zero();
          }
          else
          {
            // others pos and vel
            double t_in_st = time_cps_.relative_time_ahead(id) + accumulated_step_swarm;
            if (t_in_st < swarm_trajs_->at(id).duration)
            {
              // std::cout<<"t_in_st init::"<<t_in_st<<std::endl;
              pos = swarm_trajs_->at(id).traj.getPos(t_in_st);
              vel = swarm_trajs_->at(id).traj.getVel(t_in_st);
            }
            else
            {
              double exceed_time = t_in_st - swarm_trajs_->at(id).duration;
              vel = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
              pos = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) + exceed_time * vel;
            }
          }
          swarm_pos[id] = pos;
          swarm_vel[id] = vel;
        }
        time_cps_.swarm_pos_advance_sets[i] = swarm_pos;
        time_cps_.swarm_vel_advance_sets[i] = swarm_vel;
        // ztr_log3<<"第"<<i<<"架飞机"<<""
        // set swarm graph
        // swarm_graph.updateGraph(swarm_pos);   // wait to change to calculate local minimum
        time_cps_.swarm_graph_advance_sets[i] = swarm_graph;
      }
      if (time_cps_.enable_decouple_swarm_graph) {
      // test : calculate the local minimum of swarm graph
        time_cps_.local_min_advance_sets.resize(time_cps_.sampling_num);
      
        // record
        if (record_once_)
          log_.open("/home/lunquan/paper_ws/TRO2022_ws/Pipline-Swarm-Formation/src/planner/traj_opt/log/drone_" + to_string(drone_id_)+".txt");      

        for (int i=0; i<time_cps_.sampling_num; i++){            
          if (record_once_)
            log_<< i << endl;
          
          // init optimizer
          lbfgs::lbfgs_parameter_t lbfgs_params;
          lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
          lbfgs_params.mem_size = 8;
          lbfgs_params.g_epsilon = 1e-5;     //0.0
          lbfgs_params.min_step = 1e-10;    //1e-32
          lbfgs_params.delta = 1.0e-4;
          lbfgs_params.line_search_type = 1;      // 1: continues
          lbfgs_params.max_iterations = 20;

          /* get init value */
          double q[3];
          
          // Average of other swarm pos, if there is any other more efficient trick?
          Eigen::Vector3d average_pos = Eigen::Vector3d::Zero();
          for (int id=0; id<size; id++){
            if (id != drone_id_){
              average_pos += time_cps_.swarm_pos_advance_sets[i][id];

              if (record_once_)
                log_ << time_cps_.swarm_pos_advance_sets[i][id].transpose() << endl;
            }
          }
          average_pos /= (size-1);
          q[0] = average_pos(0);
          q[1] = average_pos(1);
          q[2] = average_pos(2);
          if (record_once_)
            log_ << average_pos.transpose() << endl;

          // get the pos from own trajectory
          // poly_traj::Trajectory traj = jerkOpt_.getTraj();
          // Eigen::Vector3d pos = traj.getPos(i * time_cps_.sampling_time_step);
          // q[0] = pos(0);
          // q[1] = pos(1);
          // q[2] = pos(2);
          // if (record_once_)
          //   log_ << pos.transpose() << endl;

          /* optimize */ 
          double final_cost;
          time_cps_.idx_of_sets = i;
          opt_local_min_loop_num_ = 0;
          int result = lbfgs::lbfgs_optimize( 3, 
                                              q, 
                                              &final_cost,
                                              PolyTrajOptimizer::swarmGraphCostCallback,
                                              NULL,
                                              NULL,
                                              this,
                                              &lbfgs_params);
          // save the local minimum
          Eigen::Vector3d local_min;
          local_min << q[0], q[1], q[2];
          time_cps_.local_min_advance_sets[i] = local_min;

          // debug: print the optimization result
          // if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_ALREADY_MINIMIZED)
          //   printf("\033[33m[advanced] idx: %i, result: %s\n\033[0m", time_cps_.idx_of_sets, lbfgs::lbfgs_strerror(result));
        }

        if (record_once_){
          record_once_ = false;
          log_.close();
        }
      }

      is_set_swarm_advanced_ = true;
    }
    return;
  }

  double PolyTrajOptimizer::swarmGraphCostCallback(void *func_data, const double *x, double *grad, const int n){
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Vector3d local_min_pos;
    local_min_pos << x[0], x[1], x[2];
    /* calculate cost and grad of swarm graph */
    double cost;
    Eigen::Vector3d gradP;
    int idx = opt->time_cps_.idx_of_sets;
    // std::cout<<"idx idx idx :"<<opt->formation_size_<<std::endl;
    // get swarm pos
    vector<Eigen::Vector3d> swarm_pos(opt->formation_size_);
    for (int id = 0; id < opt->formation_size_; id++) {
      if (id == opt->drone_id_)
        swarm_pos[id] = local_min_pos;
      else
        swarm_pos[id] = opt->time_cps_.swarm_pos_advance_sets[idx][id];
    }
    // get cost and gradP
    opt->time_cps_.swarm_graph_advance_sets[idx].updatePartGraphAndGetGrad(opt->drone_id_, swarm_pos, gradP);
    opt->time_cps_.swarm_graph_advance_sets[idx].calcFNorm2(cost);
    grad[0] = gradP(0);
    grad[1] = gradP(1);
    grad[2] = gradP(2);

    // record
    if (opt->record_once_){
      opt->log_ << opt->opt_local_min_loop_num_ << ", pos: " << local_min_pos.transpose();
      opt->log_ << ", cost: " << cost << ", grad: " << -gradP.transpose() << endl;
    }
      
    opt->opt_local_min_loop_num_++;
    opt->opt_local_min_loop_sum_num_++;

    return cost;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    // std::cout<<"opt->piece_num_::::"<<opt->piece_num_<<"   "<<gradT(0)<<std::endl;
    double smoo_cost = 0, time_cost = 0;

    opt->jerkOpt_.generate(P, T);

    // debug
    // if (opt->iter_num_ == 0)
    //   cout << "[debug] init total: " << T.sum() << ", T: " << T.transpose() << endl;
    
    // if (T.sum() > 20.00){
    //   gradP.setZero();
    //   gradt.setZero();
    //   return 9999999.0;
    // }

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost
    Eigen::VectorXd obs_swarm_feas_qvar_costs;

    if (opt->enable_fix_step_) {
      obs_swarm_feas_qvar_costs.resize(5);
      opt->addPVAGradCost2CTwithFixedTimeSteps(gradT, obs_swarm_feas_qvar_costs);
    } else {
      obs_swarm_feas_qvar_costs.resize(6);
      opt->addPVAGradCost2CTwithFixedCtrlPoints(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_);
    }
    
    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g,
                                           const double fx, const double xnorm, const double gnorm,
                                           const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }
  
  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    // std::cout<<" jerkOpt_.initGradCost(gdT, cost):::"<<gdT[0]<<"    "<<cost<<std::endl;
    jerkOpt_.initGradCost(gdT, cost);
    gdT *= wei_smooth_;
    cost *= wei_smooth_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CTwithFixedCtrlPoints(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0;
    // Eigen::MatrixXd constrain_pts(3, N * K + 1);

    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        /* Formation method chosing */
        if (use_formation_){
          if (swarmGraphGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
            {
              gradViolaPc = beta0 * gradp.transpose();
              gradViolaPt = alpha * gradt;
              jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
              gdT(i) += omg * (costp / K + step * gradViolaPt);
              if (i > 0)
              {
                gdT.head(i).array() += omg * step * grad_prev_t;
              }
              costs(2) += omg * step * costp;
            }
        }

        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(4) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(4) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(5) += var;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CTwithFixedTimeSteps(EIGENVEC &gdT, Eigen::VectorXd &costs){
    costs.setZero();
    ros::Time t0,t1,t2,t3,t4,t5;
    t0 = ros::Time::now();

    double obs_cost = obstacleGradCostP(gdT);   
    costs(0) = obs_cost;

    t1 = ros::Time::now();

    // costs(0) = 0;
    double swarm_obs_cost;
    if(swarm_trajs_->size()>1)
    {
      swarm_obs_cost = swarmGradCostP(gdT);
    }else{
      swarm_obs_cost = 0;
    }
    costs(1) = swarm_obs_cost;

    t2 = ros::Time::now();

    // costs(1) = 0;
    double swarm_formation_cost = 0.0;
    if (use_formation_&&swarm_trajs_->size()>1)
      swarm_formation_cost = swarmGraphGradCostP(gdT);
    costs(2) = swarm_formation_cost;

    t3 = ros::Time::now();

    double vel_cost = 0.0, acc_cost = 0.0;
    feasibilityGradCostVandA(gdT, vel_cost, acc_cost);
    costs(3) = vel_cost;
    costs(4) = acc_cost;
    
    t4 = ros::Time::now();

    // cout << "[debug] time ----------------" << endl;
    // cout << "[obs]: " << (t1-t0).toSec()*1000.0 
    //      << " ms, [swarm]: " << (t2-t1).toSec()*1000.0
    //      << " ms, [formation]: " << (t3-t2).toSec()*1000.0
    //      << " ms, [dynamic]: " << (t4-t3).toSec()*1000.0
    //      << endl;
  }


  
  bool PolyTrajOptimizer::decide_contin(int id_get)
  {
    int leader_id1  = 8;
    int leader_id2  = 16;
    int leader_id3  = 24;
    int leader_id4  = 32;
    int leader_id5  = 40;
    int leader_id6  = 48;
    int leader_id7  = 56;
    int leader_id8  = 64;
    int leader_id9  = 72;
    bool id_need_ignored = false;
    if(id_get!=0&&id_get!=2&&id_get!=3&&id_get!=4&&id_get!=5&&id_get!=6&&id_get!=7
    &&id_get!=leader_id1 && id_get!=leader_id2 && id_get!=leader_id3 && id_get!=leader_id4 
    && id_get!=leader_id5 && id_get!=leader_id6 && id_get!=leader_id7 && id_get!=leader_id8 && id_get!=leader_id9)
    {
      id_need_ignored = true;
    }
    else{
      id_need_ignored = false;
    }
    return id_need_ignored;
  }

  double PolyTrajOptimizer::obstacleGradCostP(Eigen::VectorXd &gdT) {
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    /* Calculate the gdC, gdT and obs_cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double obs_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx=0; idx<=k; idx++, accumulated_dur+=step) {
      // if (idx >= k * 4 / 5)
      //   break;
      
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);
      
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      
      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // calculate the cost and grad of obstacle avoidance
      double dJ_df, df_dt, grad_prev_t;
      Eigen::Matrix<double, 6, 3> gradViolaPc;
      Eigen::Vector3d df_dp;
      
      // use esdf
      double dist;
      Eigen::Vector3d dist_grad;
      grid_map_->getDisWithGradI(pos, dist, dist_grad);
      double dist_err = obs_clearance_ - dist;

      if (dist_err > 0){
        dJ_df = 3 * wei_obs_ * step * omg * pow(dist_err, 2);
        df_dp = - dist_grad;
        gradViolaPc = dJ_df * beta0 * df_dp.transpose();
        df_dt = df_dp.dot(vel);
        grad_prev_t = -1 * dJ_df * df_dt;

        // cost 
        obs_cost += wei_obs_ * step * omg * pow(dist_err, 3);

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }
    }
    return obs_cost;
  }

  double PolyTrajOptimizer::swarmGradCostP(Eigen::VectorXd &gdT){   
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    /* Calculate the gdC, gdT and swarm_obs_cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double swarm_obs_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx=0; idx<=k; idx++, accumulated_dur+=step) {
      // if (idx >= k * 2 / 3)
      //   break;
      
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);
      
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      
      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;
      int size = swarm_trajs_->size();
      for (int id=0; id < size; id++) {
        if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
          continue;
        // get swarm pos
        double t_in_st = accumulated_dur + (t_now_ - swarm_trajs_->at(id).start_time);
        Eigen::Vector3d swarm_p, swarm_v;
        if (t_in_st < swarm_trajs_->at(id).duration){
          swarm_p = swarm_trajs_->at(id).traj.getPos(t_in_st);
          swarm_v = swarm_trajs_->at(id).traj.getVel(t_in_st);
        } else {
          double exceed_time = t_in_st - swarm_trajs_->at(id).duration;
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration)
                    + exceed_time * vel;
        }

        // calculate the cost and grad of swarm reciprocal avoidance
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        Eigen::Vector3d dist_vec = pos - swarm_p;
        double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
        double dist2_err = CLEARANCE2 - ellip_dist2;
        double dist2_err2 = dist2_err * dist2_err;
        double dist2_err3 = dist2_err2 * dist2_err;

        if (dist2_err > 0){
          dJ_df = 3 * wei_swarm_ * step * omg * dist2_err2;
          df_dp = -2 * dist_vec;
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          df_dt = df_dp.dot(vel);
          grad_prev_t = -1 * dJ_df * df_dt;

          // cost 
          swarm_obs_cost += wei_swarm_ * step * omg * dist2_err3;

          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
      }
    }
    return swarm_obs_cost;
  }

  double PolyTrajOptimizer::swarmGraphGradCostP(Eigen::VectorXd &gdT){
    if (!is_set_swarm_advanced_)
      return 0.0;
    
    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;
    if (size < formation_size_)
      return 0.0;
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k;

    /* Expand swarm_graph_advance_sets */
    if (T_sum > time_cps_.duration){
      k = floor(T_sum / step) + 1;
      for (int i = time_cps_.sampling_num; i < k; i++)
      {
        SwarmGraph swarm_graph;
        // std::cout<<"swarm_des_ size:"<<swarm_des_.size()<<std::endl;
        swarm_graph.setDesiredForm(swarm_des_, adj_in_, adj_out_);
        swarm_graph.setAssignment(assignment_);
        std::vector<Eigen::Vector3d> swarm_pos(size), swarm_vel(size);

        int size = swarm_trajs_->size();
        for (int id=0; id<size; id++){
          Eigen::Vector3d pos, vel;
          if (id == drone_id_)
          {
            // own pos and vel, set as zero now. And they will be updated in the optimized loop
            pos = Eigen::Vector3d::Zero();
            vel = Eigen::Vector3d::Zero();
          }
          else
          {
            // others pos and vel
            double t_in_st = time_cps_.relative_time_ahead(id) + i * step;
            if (t_in_st < swarm_trajs_->at(id).duration)
            {
              // std::cout<<"t_in_st::"<<t_in_st<<std::endl;
              pos = swarm_trajs_->at(id).traj.getPos(t_in_st);
              vel = swarm_trajs_->at(id).traj.getVel(t_in_st);
            }
            else
            {
              double exceed_time = t_in_st - swarm_trajs_->at(id).duration;
              vel = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
              pos = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) + exceed_time * vel;
            }
          }
          swarm_pos[id] = pos;
          swarm_vel[id] = vel;
        }
        time_cps_.swarm_pos_advance_sets.emplace_back(swarm_pos);
        time_cps_.swarm_vel_advance_sets.emplace_back(swarm_vel);
        // set swarm graph, need to change: calculate the local optimal pos of swarm graph
        swarm_graph.updateGraph(swarm_pos);
        time_cps_.swarm_graph_advance_sets.emplace_back(swarm_graph);
      }/*************************3333333333333333333333333333333333333333333333333333333333333333333333333333*****************8*/
      if (time_cps_.enable_decouple_swarm_graph){
        for (int i=time_cps_.sampling_num; i<k; i++) {
          // init optimizer
          lbfgs::lbfgs_parameter_t lbfgs_params;
          lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
          lbfgs_params.mem_size = 8;
          lbfgs_params.g_epsilon = 1e-5;     //0.0
          lbfgs_params.min_step = 1e-10;    //1e-32
          lbfgs_params.delta = 1.0e-4;
          lbfgs_params.line_search_type = 1;      // 1: continues
          lbfgs_params.max_iterations = 20;

          /* get init value */
          double q[3];
          // Average of other swarm pos, if there is any other more efficient trick?
          Eigen::Vector3d average_pos = Eigen::Vector3d::Zero();
          for (int id=0; id<size; id++) {
            if (id != drone_id_)
              average_pos += time_cps_.swarm_pos_advance_sets[i][id];
          }
          average_pos /= (size-1);
          q[0] = average_pos(0);
          q[1] = average_pos(1);
          q[2] = average_pos(2);

          // get the pos from own trajectory
          // poly_traj::Trajectory traj = jerkOpt_.getTraj();
          // Eigen::Vector3d pos = traj.getPos(i * time_cps_.sampling_time_step);
          // q[0] = pos(0);
          // q[1] = pos(1);
          // q[2] = pos(2);

          /* optimize */ 
          double final_cost;
          time_cps_.idx_of_sets = i;
          opt_local_min_loop_num_ = 0;
          lbfgs::lbfgs_optimize( 3,
                                 q, 
                                 &final_cost,
                                 PolyTrajOptimizer::swarmGraphCostCallback,
                                 NULL,
                                 NULL,
                                 this,
                                 &lbfgs_params);
          // save the local minimum
          Eigen::Vector3d local_min;
          local_min << q[0], q[1], q[2];
          time_cps_.local_min_advance_sets.emplace_back(local_min);
        }
      }
      time_cps_.sampling_num = k;
      time_cps_.duration = T_sum;
    }
    else
    {
      k = time_cps_.sampling_num;
    }
    /* Calculate the gdC, gdT and swarm cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double swarm_formation_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx=0; idx<=k; idx++, accumulated_dur+=step){
      
      if (idx >= k * 2 / 3)
        break; 
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      
      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;
      
      // calculate the cost and grad of swarm graph
      // decoupled swarm graph: calculate the local min of swarm graph in advanced 
      // coupled swarm graph: calculate the grad and cost of swarm graph in each loop of optimization
      if (time_cps_.enable_decouple_swarm_graph){
        // update swarm graph and calculate cost and grad
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        Eigen::Vector3d dist_vec = pos - time_cps_.local_min_advance_sets[idx];
        double dist  = dist_vec.norm();
        double dist2 = dist_vec.squaredNorm(); 
        // std::cout<<"dist2:"<<dist2<<std::endl;
        if (dist2 > 0){
          // double dist2_2 = dist2 * dist2;
          // double dist2_3 = dist2_2 * dist2;

          // dJ_df = 3 * wei_formation_ * step * omg * dist2_2;
          // df_dp = 2 * dist_vec;
          // gradViolaPc = dJ_df * beta0 * df_dp.transpose(); 
          // df_dt = df_dp.dot(vel);
          // grad_prev_t = -1 * dJ_df * df_dt;

          dJ_df = 2 * wei_formation_ * step * omg * dist;
          df_dp << dist_vec(0) / dist, 
                   dist_vec(1) / dist,
                   dist_vec(2) / dist;
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          df_dt = df_dp.dot(vel);
          grad_prev_t = -1 * dJ_df * df_dt;

          // cost 
          swarm_formation_cost += wei_formation_ * step * omg * dist2;

          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
        
      } else{
        // get the swarm pos 
        vector<Eigen::Vector3d> swarm_graph_pos(size);
        for (int id=0; id < size; id++){
          if (id == drone_id_)
            swarm_graph_pos[id] = pos;
          else
            swarm_graph_pos[id] = time_cps_.swarm_pos_advance_sets[idx][id];      
        }
        // update swarm graph and calculate cost and grad
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        time_cps_.swarm_graph_advance_sets[idx].updatePartGraphAndGetGrad(drone_id_, swarm_graph_pos, df_dp);
        // log_zy[drone_id_]<<drone_id_<<"       ###########################################"<<std::endl;
        // log_zy_x[drone_id_]<<abs(df_dp[0])<<std::endl;
        // log_zy_y[drone_id_]<<abs(df_dp[1])<<std::endl;
        // log_zy_z[drone_id_]<<abs(df_dp[2])<<std::endl;
        double similarity_error;
        time_cps_.swarm_graph_advance_sets[idx].calcFNorm2(similarity_error);

        if (similarity_error > 0.0){
          dJ_df = wei_formation_ * step * omg;
          df_dt = df_dp.dot(vel);
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          grad_prev_t = -1 * dJ_df * df_dt;
          
          // cost 
          swarm_formation_cost += wei_formation_ * step * omg * similarity_error;
          
          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;
          
          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
      }
    }
    return swarm_formation_cost;
  }
    
  void PolyTrajOptimizer::feasibilityGradCostVandA(Eigen::VectorXd &gdT, double &vel_cost, double acc_cost){
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    /* Calculate the gdC, gdT and vel_cost */
    // consider the whole trajectory
    vel_cost = 0.0;
    acc_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4;
    Eigen::Matrix<double, 6, 1> beta1, beta2, beta3;
    Eigen::Vector3d vel, acc, jer;
    double omg;

    for (int idx=0; idx<=k; idx++, accumulated_dur+=step) {
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;

      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      
      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // calculate the cost and grad of velocity feasibility
      double dJ_df = 0.0, df_dt = 0.0, grad_prev_t = 0.0;
      Eigen::Matrix<double, 6, 3> gradViolaPc;

      /* velocity feasibility*/
      double vpen = vel.squaredNorm() - max_vel_ * max_vel_; 
      if (vpen > 0){
        dJ_df = 3 * wei_feas_ * step * omg * vpen * vpen;
        gradViolaPc = dJ_df * 2 * beta1 * vel.transpose();
        df_dt = 2 * vel.dot(acc);
        grad_prev_t = -1 * dJ_df * df_dt;
        
        // cost 
        vel_cost += wei_feas_ * step * omg * vpen * vpen * vpen;

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }

      /* acc feasibility*/
      double apen = acc.squaredNorm() - max_acc_ * max_acc_;
      if (apen > 0){
        dJ_df = 3 * wei_feas_ * step * omg * apen * apen;
        gradViolaPc = dJ_df * 2 * beta2 * acc.transpose();
        df_dt = 2 * acc.dot(jer);

        // cost 
        acc_cost += wei_feas_ * step * omg * apen * apen * apen;

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }
    }

    // calculate the rest part
    // ...... it is not need

    return;
  }

  bool PolyTrajOptimizer::swarmGraphGradCostP(const int i_dp,
                                              const double t,
                                              const Eigen::Vector3d &p,
                                              const Eigen::Vector3d &v,
                                              Eigen::Vector3d &gradp,
                                              double &gradt,
                                              double &grad_prev_t,
                                              double &costp)
  {
    // wait all the drones have trajectories
    // if (swarm_trajs_->size() < formation_size_ && drone_id_ != formation_size_-1)
    //   return false;
    // if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
      return false;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    
    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    // update the swarm graph
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;
    
    for (int id = 0; id < size; id++){
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
        return false;
      // {
      //   double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
      //   swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
      //   swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
      //             exceed_time * swarm_v;
      // }

      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    swarm_graph_->updateGraph(swarm_graph_pos);

    // calculate the swarm graph cost and gradp
    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);

    if (similarity_error > 0)
    {
      ret = true;

      // double similarity_error2 = similarity_error * similarity_error;
      // double similarity_error3 = similarity_error2 * similarity_error;

      costp = wei_formation_ * similarity_error;
      vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);
      // double dJ_df = wei_formation_ * 3 * similarity_error2;
      gradp = wei_formation_ * swarm_grad[drone_id_];

      for (int id=0; id<size; id++){
        // log_zy<<id<<"    #################################################"<<std::endl;
        // log_zy<<swarm_grad[id]<<std::endl;
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGatherCostGradP(const int i_dp,
                                               const double t,
                                               const Eigen::Vector3d &p,
                                               const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradp,
                                               double &gradt,
                                               double &grad_prev_t,
                                               double &costp)
  {
    // there is a bug still need to fix: when drone_id = formation_size_ - 1,
    // swarm_trajs_->size() = formation_size_ -1 but not swarm_trajs_->size() = formation_size_
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
      return false;

    // for the formation problem, we may optimize the whole trajectory ?
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    // get the swarm pos and vel
    Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (int id = 0; id < size; id++){
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    double alpha1 = double(1.0 - formation_size_) / double(formation_size_);
    double alpha2 = double(1.0 / formation_size_);

    for (int id = 0; id < size; id++)
      center_pos += swarm_graph_pos[id];

    center_pos = center_pos * alpha2;

    // calculate the swarm gathering cost and grad
    const double CLEARANCE2 = swarm_gather_threshold_ * swarm_gather_threshold_;
    Eigen::Vector3d dist_vec = center_pos - p;
    double dist2 = dist_vec.norm() * dist_vec.norm();
    double dist_error = dist2 - CLEARANCE2;
    double dist_error2 = dist_error * dist_error;
    double dist_error3 = dist_error2 * dist_error;
    ret = true;

    if (dist_error3 > 0)
    {
      costp = wei_gather_ * dist_error3;
      double dJ_df = wei_gather_ * 3 * dist_error2;
      gradp = dJ_df * 2 * alpha1 * dist_vec;
      
      for (int id=0; id<size; id++){
        if (id == drone_id_)
          gradt += dJ_df * 2 * alpha1 * dist_vec.dot(swarm_graph_vel[id]);
        else
          gradt += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);

        if (id != drone_id_)
          grad_prev_t += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);
      }
    }
    return ret;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;
    
    // use esdf
    double dist;
    Eigen::Vector3d dist_grad;
    // grid_map_->evaluateEDTWithGrad(p, dist, dist_grad);
    double dist_err = obs_clearance_ - dist;

    if (dist_err > 0)
    {
      ret = true;
      costp = wei_obs_ * pow(dist_err, 3);
      gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    // if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;
    if (i_dp <= 0)
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    int size = swarm_trajs_->size();
    for (int id = 0; id < size; id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }
    }

    return ret;    
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }



  bool PolyTrajOptimizer::getFormationPos(vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos){
    int size = swarm_trajs_->size();
    if (size < formation_size_ || !use_formation_){
      return false;  
    } 
    else
    {
      double pt_time = t_now_;

      swarm_graph_pos[drone_id_] = pos;

      for (int id = 0; id < size; id++){
        if (swarm_trajs_->at(id).drone_id < 0 || swarm_trajs_->at(id).drone_id == drone_id_)
          continue;

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

        Eigen::Vector3d swarm_p, swarm_v;
        if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
          swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                    exceed_time * swarm_v;
        }
        swarm_graph_pos[id] = swarm_p;
      }
      return true;
    }
  }
  
  //尚未看
  double PolyTrajOptimizer::getFormationError(vector<Eigen::Vector3d> swarm_graph_pos)
  {
    swarm_graph_->updateGraph(swarm_graph_pos);
    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);
    return similarity_error;
  }

  
  void PolyTrajOptimizer::updateSwarmGraph(Eigen::VectorXi assignment)
  {
    swarm_graph_->setAssignment(assignment);
    assignment_ = assignment;
  }

  
  //设置控制点cps_为points
  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) {
     swarm_trajs_ = swarm_trajs_ptr; 

  }

  void PolyTrajOptimizer::fisrt_planner_or_not(const bool first_p_){
    first_planner_ = first_p_;
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id)
  {
    drone_id_ = drone_id;
  }
  void PolyTrajOptimizer::get_path(std::vector<Eigen::Vector3d> &simple_path_)
  {
    simple_path_ = astar_path;
  }
  void PolyTrajOptimizer::get_esdf(sensor_msgs::PointCloud2& esdf_vis, pcl::PointCloud<pcl::PointXYZ> &point_obs_)
  {
    grid_map_->get_esdf_map(esdf_vis);
    grid_map_->get_obs(point_obs_);
  }
 double PolyTrajOptimizer::error_dist(std::vector<Eigen::Vector3d> f_cur , std::vector<Eigen::Vector3d> f_des)
 {
  double error_d_;
  error_d_ = swarm_graph_->getBigForm_error(f_des, f_cur);
  error_d_ = error_d_ * 1000;
  log_zy<<error_d_<<std::endl;

 }
  /* helper functions */
  void PolyTrajOptimizer::setlog(const int d_id_, const int s_num)
  {
    // if(s_num>2)
    //  {
    //    log_zy_x.resize(s_num);
    //    log_zy_y.resize(s_num);
    //    log_zy_z.resize(s_num);
    //  }
    if(d_id_==0)
    {
      log_zy.open("/home/zy/debug/1/error_" + to_string(d_id_)+"_.txt");
    }
    // log_zy_x[d_id_].open("/home/zy/debug/1/error_" + to_string(d_id_)+"_x.txt");
    // log_zy_y[d_id_].open("/home/zy/debug/1/error_" + to_string(d_id_)+"_y.txt");
    // log_zy_z[d_id_].open("/home/zy/debug/1/error_" + to_string(d_id_)+"_z.txt");
  }
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh_opt ,vector<int> leader_id_, std::vector<Eigen::Vector3d> v_des)
  {
    nh_opt.param("wei_smooth_", wei_smooth_, 1.0);
    nh_opt.param("wei_obs_", wei_obs_, 5000.0);
    nh_opt.param("wei_swarm_", wei_swarm_, 1000.0);
    nh_opt.param("wei_feas_", wei_feas_, 10.0);
    nh_opt.param("wei_sqrvar_", wei_sqrvar_, 0.0);
    nh_opt.param("wei_time_", wei_time_, 80.0);
    nh_opt.param("wei_formation_", wei_formation_, 100.0);
    nh_opt.param("wei_gather_", wei_gather_, 0.0);
    nh_opt.param("obs_clearance_", obs_clearance_, 0.5);
    nh_opt.param("swarm_clearance_", swarm_clearance_, 0.1);
    nh_opt.param("formation_type_", formation_type_, 1);
    nh_opt.param("formation_method_type_", formation_method_type_, 0);
    nh_opt.param("max_vel_", max_vel_, 2.0);
    nh_opt.param("max_acc_", max_acc_, 3.0);
    nh_opt.param("enable_fix_step_", enable_fix_step_, true);
    nh_opt.param("sampling_time_step", time_cps_.sampling_time_step, 0.05);
    nh_opt.param("enable_decouple_swarm_graph", time_cps_.enable_decouple_swarm_graph, true);
 

    Eigen::Vector3i pool_size = grid_map_->get_mapsize();
    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, pool_size);
    
    swarm_graph_.reset(new SwarmGraph);
    setDesiredFormation(formation_type_, leader_id_, v_des);

    // benchmark position-based formation setting
  }
} // namespace ego_planner