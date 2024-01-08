
#include "optimizer/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    std::cout<<"pool_size::::::::::"<<pool_size<<std::endl;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
    resolution_ = grid_map_->getResolution();
    offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - grid_map_->getOrigin() / resolution_;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {   
        ROS_WARN("Start point is insdide an obstacle.");
        cout << "[Astar] start point : " << start_pt.transpose() << endl;
        cout << "Index2Coord(start_idx) : " << Index2Coord(start_idx).transpose() << endl;
        cout << "start_idx: " << start_idx.transpose() << endl;
        return false;
    }

    // if (checkOccupancy(Index2Coord(start_idx)))
    // {
    //     ROS_WARN("Start point is insdide an obstacle.");
    //     do
    //     {
    //         start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
    //         if (!Coord2Index(start_pt, start_idx))
    //             return false;
    //     } while (checkOccupancy(Index2Coord(start_idx)));
    // }

    // if (checkOccupancy(Index2Coord(end_idx)))
    // {
    //     ROS_WARN("End point is insdide an obstacle.");
    //     return false;
    // }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        ROS_WARN("End point is insdide an obstacle, try to find a nearest end_point");
        Vector3d temp_end_pt = end_pt;
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx)){
                ROS_WARN("can't find a right end point");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx)));
        cout <<"[Astar] : init end point: " << temp_end_pt.transpose() << " , real end point: " << end_pt.transpose() << endl;
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator_Astar> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter=%d, time(ms)=%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if(use_esdf_check){
                        if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                            continue;
                    } else {
                        if (checkOccupancy(Index2Coord(neighborPtr->index)))
                            continue;
                    }
                    
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 1.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::Time time_2 = ros::Time::now();

    // if ((time_2 - time_1).toSec() > 0.1)
    //     ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

bool AStar::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, 
                                        Vector3d end_pt, vector<Vector3d> &final_simple_path,
                                        std::vector<Eigen::Vector3d> &path)
{
    bool is_show_debug = false;

    // call astar search and get the path
    bool success = AstarSearch(step_size, start_pt, end_pt, true);
    if (!success){
        ROS_WARN("A* failed!");
        return false;
    }
  path = getPath();

    // debug
    if (is_show_debug){
        cout << "[ show init path ] : " << endl;
        int n4 = path.size();
        for (int i=0; i<n4; i++)
            cout << path[i].transpose() << endl;
    }
    
    if ((path[0]-start_pt).norm() > 0.5){
        ROS_WARN("Unknown situation in A* path finding");
        cout <<"path[0] : " << path[0].transpose() << ", strat_pt : " << start_pt.transpose() << ", end_pt : "<< end_pt.transpose() << endl;
        return false;
    }
    
    // generate the simple path
    vector<Vector3d> simple_path;
    int size = path.size();
    if (size <= 2){
        ROS_WARN("the path only have two points");
        final_simple_path = path;
        return true;
    }

    if (is_show_debug)
        cout << "[ start simplify the path ] " << endl;
    

    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    simple_path.clear();
    simple_path.push_back(path.front());
    

    for (int i=1; i<path.size(); ++i){
        if (lineVisib(simple_path.back(), path[i], resolution_, colli_pt)) continue;
        // if (!grid_map_->isLineOccupancy(simple_path.back(), path[i])) continue;
        // grid_map_->getDisWithGradI(colli_pt, dist, grad);
        // if (grad.norm() > 1e-3) {
        //     grad.normalize();
        //     dir = (path[i] - simple_path.back()).normalized();
        //     push_dir = grad - grad.dot(dir) * dir;
        //     push_dir.normalize();
        //     colli_pt = colli_pt + resolution_ * push_dir;
        // }
        simple_path.push_back(path[i]);
    }
    simple_path.push_back(path.back());

    // // debug
    // if (is_show_debug){
    //     cout << "[simple A* path] : --------- " << endl;
    //     int n1 = simple_path.size();
    //     cout << "simple A* path size : " << n1 << endl;
    //     for (int i=0; i<n1; i++)
    //         cout << simple_path[i].transpose() << endl;
    // }
    
    // // check the near points and delete it
    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <=2){
            near_flag = false;
            break;
        }

        int num_same_check = simple_path.size();
        for (int i=0; i<num_same_check-1; i++){
            double len = (simple_path[i+1] - simple_path[i]).norm();
            if (len < 1.5){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    // // debug
    // if (is_show_debug){
    //     cout << "[delete simple path] : --------- " << endl;
    //     int n2 = simple_path.size();
    //     cout << "delete simple path size : " << n2 << endl;
    //     for (int i=0; i<n2; i++)
    //         cout << simple_path[i].transpose() << endl;
    // }
    
    // // check the path and add a point if two of them are too far away
    bool too_long_flag;
    const double length_threshold = 2;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = (simple_path[i+1] - simple_path[i]).norm();
            if (leng > length_threshold){
                Vector3d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 30);

    // // debug
    // if (is_show_debug){
    //     cout << "[final simple path] : --------- " << endl;
    //     int n3 = simple_path.size();
    //     cout << "final simple path size : " << n3 << endl;
    //     for (int i=0; i<n3; i++)
    //         cout << simple_path[i].transpose() << endl;
    // }
    
    final_simple_path = simple_path;
    return true;
}

bool AStar::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                      double thresh, Eigen::Vector3d& pc)
{
    Eigen::Vector3d ray_pt;
    Eigen::Vector3i pt_id;
    double dist;

    RayCaster casters;
    casters.setInput(p1 / resolution_, p2 / resolution_);
    while (casters.step(ray_pt)) {
        pt_id(0) = ray_pt(0) + offset_(0);
        pt_id(1) = ray_pt(1) + offset_(1);
        pt_id(2) = ray_pt(2) + offset_(2);
        
        grid_map_->getDistance_i(pt_id, dist);

        if (dist <= thresh) {
        grid_map_->indexToPos(pt_id, pc);
        return false;
        }
    }
    return true;
}
