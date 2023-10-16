/*** 
 * @Author: ztr
 * @Date: 2022-05-12 00:00:43
 * @LastEditTime: 2022-05-12 00:00:43
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /Pipline-Swarm-Formation/src/planner/plan_manage/src/ego_planner_node.cpp
 */
#include "ego_planner_node.h"
using namespace ego_planner;
using namespace poly_traj;
TrajContainer traj_;
void deal_traj(int kkk, double b)
{
  /*************************************************************00000000000************************************/
  double t_now_0 = ros::Time::now().toSec();
  int piece_nums = 4;
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  int recv_id = 0+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time = t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;

  cMats[0].row(0) <<-0.000411463, 0.00262856, -0.00482534, 0.00192251, 1.00824, -17.1803;
  cMats[0].row(1) <<0.000227876,-0.00163438,0.0028976, 0.00414863, -0.00871011, -2.00708;
  cMats[0].row(2) <<0.000134042, -0.000122678, -0.00360272, 0.0081438, 0.0704743, 1.2073+b;
  dura[0] = 1.83508;
  cMats[1].row(0) <<0.000246434,-0.00114678,0.000613024 ,0.0030411  ,1.00819,-15.3322;
  cMats[1].row(1) <<-5.61373e-05 ,0.00045648 ,-0.00142546,0.00115986,0.00831037,-2.00498;
  cMats[1].row(2) <<-0.000223125,0.00110721 ,1.06812e-05,-0.00588538 ,0.0685346,1.34318+b;
  dura[1] = 1.86858;
  cMats[2].row(0) <<-0.000368983,0.00115562,0.000646085 ,-0.00146881 ,1.01107 ,-13.4421;
  cMats[2].row(1) <<1.95903e-05 ,-6.8004e-05 ,2.63325e-05 ,-0.000930438 ,0.00620453,-1.99042;
  cMats[2].row(2) <<0.000178149,-0.000977423 ,0.000495723 ,0.00281267 ,0.0619462,1.45918+b;
  dura[2] = 1.87471;
  cMats[3].row(0) <<0.00100066 ,-0.00230306 ,-0.00365615,0.00222239 ,1.02004 ,-11.54179;
  cMats[3].row(1) <<-4.00929e-05,0.000115627 ,0.00020489 ,-0.000925602 ,0.00241122 ,-1.98227;
  cMats[3].row(2) <<-0.000123335 ,0.000692466,-0.000572704 ,-0.0032727,0.0629613 ,1.58052+b;
  dura[3] = 1.89703;
  poly_traj::Trajectory trajectory0(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory0;
  traj_.swarm_traj[recv_id].duration = trajectory0.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory0.getPos(0.0);
  /*************************************************************111111111************************************/
  recv_id = 1+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0)<< -4.97468e-05 ,-2.77387e-05  ,0.00135211 ,-0.00105728  ,1.00402  ,-15.1829;
  cMats[0].row(1)<< 3.00566e-05,-0.000362528,0.00147098 ,-0.00154369,-0.000871422,0.013369;
  cMats[0].row(2)<< -0.000133146 ,0.00122418,-0.00399823 ,0.00261465 ,0.0747863 ,1.22002+b;
  dura[0] = 1.83325;
  cMats[1].row(0)<< 0.000143357 ,-0.00048373 ,-0.00052319 ,0.00275466 ,1.01028,-13.3389;
  cMats[1].row(1)<< 1.94947e-05 ,-8.70214e-05 ,-0.000177288 ,0.00108789 ,0.00106274 ,0.012174;
  cMats[1].row(2)<< -9.47188e-06 ,3.73411e-06 ,0.000503924 ,-0.00289256 ,0.0667114 ,1.35235+b;
  dura[1] = 1.86752;
  cMats[2].row(0)<< -0.000309179 ,0.000854881,0.000863072 ,-0.000961826 ,1.01122,-11.4486;
  cMats[2].row(1)<< -1.09368e-05,9.50123e-05 ,-0.000147442 ,-0.000456642 ,0.00218955,0.0161825;
  cMats[2].row(2)<< 6.95198e-06 ,-8.47106e-05 ,0.000201473,-0.000608083 ,0.0607012 ,1.46996+b;
  dura[2] = 1.87629;
  cMats[3].row(0)<< 0.000922475 ,-0.00204566 ,-0.00360543,0.00153123,1.02015 ,-9.54552;
  cMats[3].row(1)<< -1.25635e-05 ,-7.59005e-06 ,0.000180617,-2.06786e-06 ,0.000751424 ,0.0186324;
  cMats[3].row(2)<< 3.35805e-05 ,-1.94911e-05,-0.000189552 ,-0.000804134 ,0.0587398,1.58215+b;
  dura[3] = 1.90132;
  poly_traj::Trajectory trajectory_1(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory_1;
  traj_.swarm_traj[recv_id].duration = trajectory_1.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory_1.getPos(0.0);
  /*************************************************************222222222************************************/
  recv_id = 2+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0)<< -4.97468e-05 ,-2.77387e-05  ,0.00135211 ,-0.00105728  ,1.00402  ,-15.1829;
  cMats[0].row(1)<< 3.00566e-05,-0.000362528,0.00147098 ,-0.00154369,-0.000871422,0.013369;
  cMats[0].row(2)<< -0.000133146 ,0.00122418,-0.00399823 ,0.00261465 ,0.0747863 ,1.22002+b;
  dura[0] = 1.83325;
  cMats[1].row(0)<< 0.000143357 ,-0.00048373 ,-0.00052319 ,0.00275466 ,1.01028,-13.3389;
  cMats[1].row(1)<< 1.94947e-05 ,-8.70214e-05 ,-0.000177288 ,0.00108789 ,0.00106274 ,0.012174;
  cMats[1].row(2)<< -9.47188e-06 ,3.73411e-06 ,0.000503924 ,-0.00289256 ,0.0667114 ,1.35235+b;
  dura[1] = 1.86752;
  cMats[2].row(0)<< -0.000309179 ,0.000854881,0.000863072 ,-0.000961826 ,1.01122,-11.4486;
  cMats[2].row(1)<< -1.09368e-05,9.50123e-05 ,-0.000147442 ,-0.000456642 ,0.00218955,0.0161825;
  cMats[2].row(2)<< 6.95198e-06 ,-8.47106e-05 ,0.000201473,-0.000608083 ,0.0607012 ,1.46996+b;
  dura[2] = 1.87629;
  cMats[3].row(0)<< 0.000922475 ,-0.00204566 ,-0.00360543,0.00153123,1.02015 ,-9.54552;
  cMats[3].row(1)<< -1.25635e-05 ,-7.59005e-06 ,0.000180617,-2.06786e-06 ,0.000751424 ,0.0186324;
  cMats[3].row(2)<< 3.35805e-05 ,-1.94911e-05,-0.000189552 ,-0.000804134 ,0.0587398,1.58215+b;
  dura[3] = 1.90132;
  poly_traj::Trajectory trajectory1(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory1;
  traj_.swarm_traj[recv_id].duration = trajectory1.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory1.getPos(0.0);
  /*************************************************************33333333333************************************/
  recv_id = 3+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0) << -0.000365218,0.00237253,-0.00471081,0.00322043 ,1.00819,-17.1927;
  cMats[0].row(1) << -0.000346181,0.00207964,-0.0021763 ,-0.00520912 ,0.00679615,0.00240974;
  cMats[0].row(2) << -0.00076856 ,0.00505819 ,-0.00785851,-0.00893965 ,0.0857453,1.23181+b;
  dura[0] = 1.83624;
  cMats[1].row(0) << 0.000217828 ,-0.0009806,0.000401023 ,0.00265573 ,1.01037 ,-15.3404;
  cMats[1].row(1) << 0.00016831 ,-0.00109871 ,0.00142612 ,0.00344124 ,-0.00252334,0.000267061;
  cMats[1].row(2) << 0.000291084 ,-0.0019981 ,0.00337958 ,0.00251589 ,0.0550037 ,1.35193+b;
  dura[1] = 1.86948;
  cMats[2].row(0) << -0.000347048 ,0.00105553 ,0.000681164 ,-0.00142573 ,1.01218 ,-13.4466;
  cMats[2].row(1) << -7.08559e-05 ,0.000474546 ,-0.000907602,-0.000603167,0.00686048,0.00731753;
  cMats[2].row(2) << -0.000105467 ,0.000722777 ,-0.00138879 ,-0.00141081 ,0.0654022 ,1.46787+b;
  dura[2] = 1.87582;
  cMats[3].row(0) << 0.000969392 ,-0.00219947 ,-0.00361049 ,0.00178525 ,1.0204 ,-11.5435;
  cMats[3].row(1) << 5.2131e-05 ,-0.000190017 ,0.000159846 ,-0.000368755 ,0.00315932 ,0.0163034;
  cMats[3].row(2) << 7.08331e-05 ,-0.000266404 ,0.000323356 ,-0.000928023 ,0.0580027 ,1.58292+b;
  dura[3] = 1.89874;
  poly_traj::Trajectory trajectory2(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory2;
  traj_.swarm_traj[recv_id].duration = trajectory2.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory2.getPos(0.0);
  /*************************************************************444444444************************************/
  recv_id = 4+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0) << -0.000216395,0.00122065,-0.00172462,0.00106573,1.00544,-15.1773;
  cMats[0].row(1) << 3.55111e-05,-0.000267207,0.000802169,-0.00177553,0.00289523,2.01887;
  cMats[0].row(2) << -0.000214041,0.00171846,-0.00457073,0.0017161,0.0747211,1.22142+b;
  dura[0] = 1.83270;
  cMats[1].row(0) << 0.000186287,-0.000762288,-4.45585e-05,0.00286241,1.00981,-13.3324;
  cMats[1].row(1) << -1.01721e-05,5.81976e-05,3.60674e-05,-0.000564144,-0.000106103,2.02087;
  cMats[1].row(2) << 2.63551e-05,-0.000242904,0.000837745,-0.00195826,0.0651944,1.35095+b;
  dura[1] = 1.86572;
  cMats[2].row(0) << -0.000332044,0.000975511,0.00075107,-0.00120948,1.01151,-11.4437;
  cMats[2].row(1) << 5.03061e-06,-3.66934e-05,0.000116309,0.000192599,-0.000938959,2.01942;
  cMats[2].row(2) << -2.04693e-06,2.95195e-06,-5.76213e-05,-0.000630826,0.0619222,1.46886+b;
  dura[2] = 1.87404;
  cMats[3].row(0) << 0.000949244,-0.00213581,-0.0035978,0.00171519,1.0201,-9.54306;
  cMats[3].row(1) << -1.06996e-05,1.04444e-05,1.79255e-05,0.00040439,0.000352583,2.01876;
  cMats[3].row(2) << 2.90711e-05,-1.62282e-05,-0.000107382,-0.0010273,0.0589022,1.5823+b;
  dura[3] = 1.89873;
  poly_traj::Trajectory trajectory3(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory3;
  traj_.swarm_traj[recv_id].duration = trajectory3.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory3.getPos(0.0);

  /*************************************************************55555555555************************************/
  recv_id = 5+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0) << -4.43091e-06,-0.000403771,0.00245761,-0.00222761,1.00451,-17.1841;
  cMats[0].row(1) << 0.000100546,-0.000816277,0.00226231,-0.00150024,-0.00149066,2.00954;
  cMats[0].row(2) << -0.000136435,0.00135236,-0.00473611,0.00429541,0.0725133,1.21929+b;
  dura[0] = 1.83298;
  cMats[1].row(0) << 0.000138134,-0.00044438,-0.000651666,0.00287421,1.01092,-15.3399;
  cMats[1].row(1) << -1.2515e-05,0.000105218,-0.000344395,0.000676971,0.00137932,2.00857;
  cMats[1].row(2) << -3.78094e-05,0.000101956,0.000595352,-0.00288833,0.0661361,1.34991+b;
  dura[3] = 1.86747;
  cMats[2].row(0) << -0.000304961,0.000845429,0.000846231,-0.00107892,1.01166,-13.4485;
  cMats[2].row(1) << -1.96372e-07,-1.16387e-05,5.11754e-06,0.000134115,0.00228455,2.01226;
  cMats[2].row(2) << 4.95235e-05,-0.000251083,3.83712e-05,0.000118065,0.061934,1.46761+b;
  dura[3] = 1.87617;
  cMats[3].row(0) << 0.000909689,-0.00201537,-0.00354381,0.0013995,1.01998,-11.5453;
  cMats[3].row(1) << 1.30457e-05,-1.34809e-05,-8.91399e-05,-9.58609e-05,0.00252222,2.0169;
  cMats[3].row(2) << -3.41418e-05,0.00021349,-0.000102691,-0.00169826,0.0592175,1.58252+b;
  dura[3] = 1.90137;
  poly_traj::Trajectory trajectory4(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory4;
  traj_.swarm_traj[recv_id].duration = trajectory4.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory4.getPos(0.0);

  /*************************************************************66666666************************************/
  recv_id = 6+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0) << -4.43091e-06,-0.000403771,0.00245761,-0.00222761,1.00451,-17.1841;
  cMats[0].row(1) << 0.000100546,-0.000816277,0.00226231,-0.00150024,-0.00149066,2.00954;
  cMats[0].row(2) << -0.000136435,0.00135236,-0.00473611,0.00429541,0.0725133,1.21929+b;
  dura[0] = 1.83298;
  cMats[1].row(0) << 0.000138134,-0.00044438,-0.000651666,0.00287421,1.01092,-15.3399;
  cMats[1].row(1) << -1.2515e-05,0.000105218,-0.000344395,0.000676971,0.00137932,2.00857;
  cMats[1].row(2) << -3.78094e-05,0.000101956,0.000595352,-0.00288833,0.0661361,1.34991+b;
  dura[1] = 1.86747;
  cMats[2].row(0) << -0.000304961,0.000845429,0.000846231,-0.00107892,1.01166,-13.4485;
  cMats[2].row(1) << -1.96372e-07,-1.16387e-05,5.11754e-06,0.000134115,0.00228455,2.01226;
  cMats[2].row(2) << 4.95235e-05,-0.000251083,3.83712e-05,0.000118065,0.061934,1.46761+b;
  dura[2] = 1.87617;
  cMats[3].row(0) << 0.000909689,-0.00201537,-0.00354381,0.0013995,1.01998,-11.5453;
  cMats[3].row(1) << 1.30457e-05,-1.34809e-05,-8.91399e-05,-9.58609e-05,0.00252222,2.0169;
  cMats[3].row(2) << -3.41418e-05,0.00021349,-0.000102691,-0.00169826,0.0592175,1.58252+b;
  dura[3] = 1.90137;
  poly_traj::Trajectory trajectory5(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory5;
  traj_.swarm_traj[recv_id].duration = trajectory5.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory5.getPos(0.0);

  /*************************************************************77777777777************************************/
  recv_id = 7+kkk;
  traj_.swarm_traj[recv_id].drone_id = recv_id;
  traj_.swarm_traj[recv_id].traj_id = 1;
  traj_.swarm_traj[recv_id].start_time =  t_now_0;
  traj_.swarm_traj[recv_id].constraint_aware = 0;
  cMats[0].row(0) << -0.000114895,0.000449812,0.000194672,-0.000462773,1.00709,-17.1839;
  cMats[0].row(1) << 9.01858e-05,-0.000263987,-0.00124583,0.00361961,0.0015333,4.01734;
  cMats[0].row(2) << -0.000717661,0.00527405,-0.0111377,0.000587644,0.0754915,1.21854+b;
  dura[0] = 1.83277;
  cMats[1].row(0) << 0.000163984,-0.000603069,-0.000367098,0.00259988,1.01195,-15.3358;
  cMats[1].row(1) << -9.94928e-05,0.000562464,-0.000151746,-0.00299863,0.000833873,4.02353;
  cMats[1].row(2) << 0.000133864,-0.0013025,0.00342022,0.00146209,0.0547978,1.33497+b;
  dura[1] = 1.86594;
  cMats[2].row(0) << -0.000323479,0.000926854,0.00084123,-0.00139985,1.01209,-13.4445;
  cMats[2].row(1) << 5.13673e-05,-0.000365774,0.000582277,0.00143825,-0.00335553,4.01822;
  cMats[2].row(2) << 6.1235e-05,-5.3588e-05,-0.00164052,0.00209494,0.0702451,1.45177+b;
  dura[2] = 1.8739;
  cMats[3].row(0) << 0.00093878,-0.00210398,-0.00357042,0.00157162,1.02015,-11.5434;
  cMats[3].row(1) << -1.25356e-05,0.000115513,-0.000355652,0.000385221,0.00170826,4.01749;
  cMats[3].row(2) << -0.000111609,0.000520154,0.000108076,-0.00422725,0.0631792,1.58072+b;
  dura[3] = 1.89909;
  poly_traj::Trajectory trajectory6(dura, cMats);
  traj_.swarm_traj[recv_id].traj = trajectory6;
  traj_.swarm_traj[recv_id].duration = trajectory6.getTotalDuration();
  traj_.swarm_traj[recv_id].start_pos = trajectory6.getPos(0.0);

  /*************************************************************************************************/
}
int main(int argc, char **argv)
{
  ros::Time::init();
  PolyTrajOptimizer::Ptr formation_optim;
  formation_optim.reset(new PolyTrajOptimizer);
  formation_optim->setDroneId(1);
  /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
  /*******一共九组，一组8个飞机，默认leader为0 ，8，16，24，32，40，48，56，64号飞机******/
  std::vector<int> leader_id;
  leader_id.resize(8);
  /*drone_*/
  leader_id.at(0) = 8;                              /*from 8 to 15*/
  leader_id.at(1) = 16;                           /*from 16 to 23*/
  leader_id.at(2) = 24;                            /*from 24 to 31*/
  leader_id.at(3) = 32;                            /*from 32 to 39*/
  leader_id.at(4) = 40;                            /*from 40 to 47*/
  leader_id.at(5) = 48;                            /*from 48 to 55*/
  leader_id.at(6) = 56;                            /*from 56 to 63*/
  leader_id.at(7) = 64;                            /*from 64 to 71*/

  bool m_or_not_get = true;
  // bool m_or_not_get = false;
  /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
  formation_optim->setParam(m_or_not_get,leader_id);
  if(m_or_not_get){
  traj_.swarm_traj.resize(16);
  deal_traj(0,0.0);
  deal_traj(8,2.0);
  }else{
  traj_.swarm_traj.resize(72);
  deal_traj(0,0.0);
  deal_traj(8,2.0);
  deal_traj(16,4.0);
  deal_traj(24,-2.0);
  deal_traj(32,-4.0);
  deal_traj(40,6.0);
  deal_traj(48,8.0);
  deal_traj(56,-6.0);
  deal_traj(64,-8.0);
  }

  formation_optim->setSwarmTrajs(&traj_.swarm_traj);
  
  bool flag_;
  Eigen::MatrixXd cstr_pts;
  Eigen::VectorXd initT_;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  // Eigen::MatrixXd intState;
  Eigen::Matrix<double, 3, 3> intState;
  Eigen::Vector3d head_pos;
  Eigen::Vector3d head_vel;
  Eigen::Vector3d head_acc;
  Eigen::Vector3d tail_pos;
  Eigen::Vector3d tail_vel;
  Eigen::Vector3d tail_acc;
  Eigen::Vector3d int_pos;
  Eigen::Vector3d int_vel;
  Eigen::Vector3d int_acc;
  head_pos = {-17.1896,-2.01374,1.2061};
  head_vel = {1.00248,0.0031434,0.0801646 };
  head_acc = {0.00153082,0.0194509,0.0231634};
  tail_pos = {-9.62895, -1.97912,1.69021};
  tail_vel = {0.990905,0.00167276 ,0.0552846 };
  tail_acc = {-1.11022e-16,3.46945e-18,1.04083e-17};

  int_pos = {-15.3564,-2.00246,1.34311};
  int_vel = {-13.454,-1.99745,1.45262};
  int_acc = {-11.5558,-1.98311,1.58101};

  intState <<  int_pos,int_vel,int_acc;

  headState << head_pos, head_vel, head_acc;
  tailState << tail_pos, tail_vel, tail_acc;
  initT_.resize(4);
  initT_<<1.87849, 1.87849, 1.87849, 1.87849;
  poly_traj::Trajectory traj_optim;
  flag_ =  formation_optim->optimizeTrajectory_lbfgs(headState, tailState,
                                                            intState, initT_,
                                                            cstr_pts, true);
  std::cout<<"fuck you1!!!!!!!!!!"<<std::endl;
  if(flag_)
  {
    std::cout<<"fuck you all!!!!!!!!!!"<<std::endl;
    traj_optim = formation_optim->getMinJerkOptPtr()->getTraj();
  }else{
    std::cout<<"fuck you!!!!!!!!!!"<<std::endl;
  }
  return 0;
}
