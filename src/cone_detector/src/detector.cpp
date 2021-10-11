#include <ros/ros.h>
#include "detector.h"

namespace ns_detector
{

void BoundaryDetector::loadParameters() 
{
	//parameters
	if (!nh_.param("/planning/max_iter_num", max_iter_num_, 5)) {
		ROS_WARN_STREAM("Did not load max_iter_num. Standard value is: " << max_iter_num_);
	}
	if (!nh_.param("/planning/max_search_num", max_search_num_, 3)) {
		ROS_WARN_STREAM("Did not load max_search_num. Standard value is: " << max_search_num_);
	}
	if (!nh_.param("/planning/max_beam_cost", max_beam_cost_, 1.5)) {
		ROS_WARN_STREAM("Did not load max_beam_cost. Standard value is: " << max_beam_cost_);
	}

	beam_weight_.w_c = nh_.param("/planning/beam_weight/w_c", beam_weight_.w_c);
    ROS_INFO("true is %lf",beam_weight_.w_c);
	beam_weight_.w_k = nh_.param("/planning/beam_weight/w_k", beam_weight_.w_k);
	beam_weight_.w_w = nh_.param("/planning/beam_weight/w_w", beam_weight_.w_w);
	beam_weight_.w_d = nh_.param("/planning/beam_weight/w_d", beam_weight_.w_d);

	path_weight_.w_k = nh_.param("/planning/path_weight/w_k", path_weight_.w_k);
	path_weight_.w_w = nh_.param("/planning/path_weight/w_w", path_weight_.w_w);
	path_weight_.w_c = nh_.param("/planning/path_weight/w_c", path_weight_.w_c);
	path_weight_.w_b = nh_.param("/planning/path_weight/w_b", path_weight_.w_b);
	path_weight_.w_r = nh_.param("/planning/path_weight/w_r", path_weight_.w_r);
}

BoundaryDetector::BoundaryDetector(ros::NodeHandle &nh)
{
	nh_=nh;
	loadParameters();
	//Publisher
	boundaryDetectionsPub = nh_.advertise<fsd_common_msgs::Map>(
		"/planning/base/localmap", 2);
    coneInMapPub = nh_.advertise<fsd_common_msgs::Map>(
        "/planning/map/localmap",2);
  	visualTrianglesPub = nh_.advertise<visualization_msgs::Marker>(
  		"/visualization/visual_triangles", 2);
  	visualBoundaryPub = nh_.advertise<visualization_msgs::MarkerArray>(
  		"/visualization/visual_boundary", 2);
  	visualTreePub = nh_.advertise<visualization_msgs::MarkerArray>(
  		"/visualization/visual_tree", 2);
  	visualPathPub = nh_.advertise<visualization_msgs::Marker>(
  		"/visualization/visual_path", 2);
    //Subscriber
	localMapSub = nh_.subscribe("/localmap", 2, 
		&BoundaryDetector::localMapCallback, this);
    poseSub = nh_.subscribe("/slam/carstate",5,
        &BoundaryDetector::poseCallback,this);
    ROS_INFO("subscibe topics successful!");
    carstate.car_state.x = -1.0;
    carstate.car_state.y = -1.0;
    carstate.car_state.theta = -1.0;
    ROS_INFO("starting working ...");

}

void BoundaryDetector::localMapCallback(const fsd_common_msgs::Map &msg)
{
	map_current = msg;
}

void BoundaryDetector::poseCallback(const fsd_common_msgs::CarState &state)
{
    carstate = state;
}


void BoundaryDetector::sendMsg() 
{
	boundaryDetectionsPub.publish(boundaryDetections);
	visualTrianglesPub.publish(visualTriangles);
	visualBoundaryPub.publish(visualBoundary);
	visualTreePub.publish(visualTree);
	visualPathPub.publish(visualPath);
    coneInMapPub.publish(coneInMap);
}

void BoundaryDetector::runAlgorithm()
{
	// Init Map
    map = map_current;
    if(map.cone_red.size() == 0 || map.cone_blue.size() == 0) {return;}
    if(!filter(map)) {return;}
    //	Build delaunay triangles
   	cv::Rect rect(-200, -200, 400, 400);
    cv::Subdiv2D coneSet(rect);
    std::map<ConePos, char> colorMap;    // Key: Cones Position -> Map: Cones Color ( r, b, u )
    std::map<int, PathPoint> MidSet;
    initSet(map, coneSet, colorMap);
    getMidPoint(coneSet, colorMap, MidSet); 
    //ROS_INFO("mid point successful");
    // Tree-Search Best Path
    SearchTree Path;
    std::vector<PathPoint> BestPath;
    searchPath(MidSet, Path);
    selectBestPath(Path, BestPath);
    //ROS_INFO("best path successful");
    generateBoundary(BestPath, boundaryDetections);
    getConeInMap();
    //visualization
    FSD::visual(coneSet, Path, boundaryDetections,BestPath,
    	visualTriangles, visualTree, visualBoundary,visualPath);

}

bool BoundaryDetector::filter(fsd_common_msgs::Map &init_map)
{
	fsd_common_msgs::Map map;
	// filter cones that x <= 0
    for(const auto &red: init_map.cone_red) {
        if (red.position.x > 0 && std::hypot(red.position.x, red.position.y) < 15) {
            map.cone_red.push_back(red);
        }
    }
    for(const auto &blue: init_map.cone_blue) {
        if (blue.position.x > 0 && std::hypot(blue.position.x, blue.position.y) < 15) {
            map.cone_blue.push_back(blue);
        }
    }
    for(const auto &unknow: init_map.cone_unknow) {
        if (unknow.position.x > 0 && std::hypot(unknow.position.x, unknow.position.y) < 15) {
            map.cone_unknow.push_back(unknow);
        }
    }

    init_map = map;

    if(init_map.cone_red.size() >= 2 && init_map.cone_blue.size() >= 2)
        return true;
    else 
        return false;	

}

void BoundaryDetector::initSet(fsd_common_msgs::Map map, cv::Subdiv2D &coneSet, std::map<ConePos,char> &colorMap) {

    // insert points to create delaunay triangles and Pos2color Map
    for (const auto &iter: map.cone_red) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'r';
    }
    for (const auto &iter: map.cone_blue) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'b';
    }
    for (const auto &iter: map.cone_unknow) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'u';
    }

    return;
}

void BoundaryDetector::getMidPoint(cv::Subdiv2D coneSet, std::map<ConePos,char> colorMap, std::map<int, PathPoint> &MidSet) {

    std::vector<cv::Vec4f> edges;
    coneSet.getEdgeList(edges);
    
    // the virtual triangle three edges
    cv::Point2f outer_vtx[3];
    for (int i = 0; i < 3; i++)
        outer_vtx[i] = coneSet.getVertex(i + 1);

    // add car itself to Mid Point
    {
        PathPoint car;
        car.id = 0;
        car.wrong_color_cnt = 0;
        car.x = 0;
        car.y = 0;
        car.edge_dst = 0;
        MidSet[0] = car;
    }

    // get Mid Point
    for (int i = 0, j = 0; i < edges.size(); i++) {

        PathPoint tmp;

        // abandon the points of the virtual triangle
        if (edges[i][0] == outer_vtx[0].x && edges[i][1] == outer_vtx[0].y ||
            edges[i][0] == outer_vtx[1].x && edges[i][1] == outer_vtx[1].y ||
            edges[i][0] == outer_vtx[2].x && edges[i][1] == outer_vtx[2].y ||
            edges[i][2] == outer_vtx[0].x && edges[i][3] == outer_vtx[0].y ||
            edges[i][2] == outer_vtx[1].x && edges[i][3] == outer_vtx[1].y ||
            edges[i][2] == outer_vtx[2].x && edges[i][3] == outer_vtx[2].y)
            continue;
        
        if(colorMap[ConePos(edges[i][0], edges[i][1])] == colorMap[ConePos(edges[i][2], edges[i][3])]) {
            tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
            tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            tmp.wrong_color_cnt += 1;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'r') {
            tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
            tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'u') 
                tmp.wrong_color_cnt += 0.5;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'b') {
            tmp.left_cone = ConePos(edges[i][2], edges[i][3]);
            tmp.right_cone = ConePos(edges[i][0], edges[i][1]);
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'u')
                tmp.wrong_color_cnt += 0.5;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'u') {
            tmp.wrong_color_cnt += 0.5;
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'r') {
                tmp.left_cone = ConePos(edges[i][2], edges[i][3]);
                tmp.right_cone = ConePos(edges[i][0], edges[i][1]); 
            }
            else {
                tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
                tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            }
        }

        tmp.CalculateDst();
        tmp.CalculateMidPoint();
        tmp.id = 2*j+1;
        MidSet[2*j+1] = tmp;
        MidSet[2*j+2] = tmp.ReverseCone();
        j++;
    }
    
    return;
}

void BoundaryDetector::searchPath(std::map<int, PathPoint> MidSet, SearchTree &Path) {
    Path.history.push_back(Path.Node);
    
    Path.node_cost_weight = beam_weight_;
    Path.path_cost_weight = path_weight_;

    std::vector<SearchTree*> next_set;
    std::vector<SearchTree*> next_tmp;

    next_set.push_back(&Path);

    for(int iter = 0; iter < max_iter_num_; iter++) {
        next_tmp.clear();
        std::vector<Cost_index> v_c;
        for(auto &leaf: next_set) {

            // Calculate each cost which current node connects to next lead to
            v_c.clear();
            for(int i = 1; i < MidSet.size(); i++) {
                if(leaf->CheckExist(MidSet[i]))
                    continue;
                double cost_curr_tmp = leaf->CalculateCurrentCost(MidSet[i]);

                if(cost_curr_tmp > max_beam_cost_)
                    continue;
                v_c.push_back(Cost_index(i, cost_curr_tmp));
            }

            // Select max_search_num_ smallest cost as next node
            sort(v_c.begin(),v_c.end(),
                    [&](const Cost_index &a, const Cost_index &b) {
                        return a.cost < b.cost;
                    });

            leaf->next.resize(std::min(max_search_num_, int(v_c.size())));

            for(int i = 0; i < std::min(max_search_num_, int(v_c.size())); i++) {
                SearchTree branch;
                branch.Node = MidSet[v_c[i].index];
                branch.history = leaf->history; branch.history.push_back(MidSet[v_c[i].index]);
                branch.node_cost_weight = leaf->node_cost_weight;
                branch.path_cost_weight = leaf->path_cost_weight;
                leaf->next[i] = branch;
                next_tmp.push_back(&(leaf->next[i]));

                leaf->CalculateCurrentCost(MidSet[v_c[i].index]);

            }
        }
        if(v_c.size() == 0)
            break;
        next_set = next_tmp;
    }

    return;
}

void BoundaryDetector::selectBestPath(SearchTree Path, std::vector<PathPoint> &BestPath) {
    std::vector<std::vector<PathPoint>> tree_path;
    std::vector<SearchTree*> next;
    std::vector<SearchTree*> tmp;
    std::vector<Cost_index> costSet;
    next.push_back(&Path);

    int index_i = 0;

    while(next.size() != 0) {
        tmp.clear();
        for(auto &iter :next) {
            if(iter->next.size() == 0) {
                if(iter->history.size() <= 1)
                    continue;
                tree_path.push_back(iter->history);
                costSet.push_back(Cost_index(index_i, iter->CalculateAllCost()));
                index_i++;
            }
            else {
                for(auto &it:iter->next)
                    tmp.push_back(&it);
            }
        }
        next = tmp;
    }
    sort(costSet.begin(),costSet.end(),
                    [&](const Cost_index &a, const Cost_index &b) {
                        return a.cost < b.cost;
                    });
    if(costSet.size()>0)
        BestPath = tree_path[costSet[0].index];

    return;
}

void BoundaryDetector::generateBoundary(std::vector<PathPoint> BestPath,
					fsd_common_msgs::Map &Boundary)
{
	std::vector<ConePos> bound_l;
	std::vector<ConePos> bound_r;

	Boundary.cone_red.clear();
	Boundary.cone_blue.clear();

	fsd_common_msgs::Cone tmp;
	for(int i=1;i<BestPath.size();i++)
	{
		if(i==1)
		{
			bound_l.push_back(BestPath[i].left_cone);
			bound_r.push_back(BestPath[i].right_cone);
			tmp.position.x = BestPath[i].left_cone.x;
			tmp.position.y = BestPath[i].left_cone.y;
			tmp.color.data = "r";
			Boundary.cone_red.push_back(tmp);

			tmp.position.x = BestPath[i].right_cone.x;
            tmp.position.y = BestPath[i].right_cone.y;
            tmp.color.data = "b";
            Boundary.cone_blue.push_back(tmp);
        }
        else
        {
        	if(BestPath[i].left_cone != bound_l[bound_l.size()-1]) {
                bound_l.push_back(BestPath[i].left_cone);
                tmp.position.x = BestPath[i].left_cone.x;
                tmp.position.y = BestPath[i].left_cone.y;
                tmp.color.data = "r";
                Boundary.cone_red.push_back(tmp);
            }
            if(BestPath[i].right_cone != bound_r[bound_r.size()-1]) {
                bound_r.push_back(BestPath[i].right_cone);
                tmp.position.x = BestPath[i].right_cone.x;
                tmp.position.y = BestPath[i].right_cone.y;
                tmp.color.data = "b";
                Boundary.cone_blue.push_back(tmp);
            }
		}
	}
}

void BoundaryDetector::getConeInMap()
{
    // if(carstate.car_state.x==-1.0)
    //     return;
    //ROS_INFO("carstate.car_state.x is %lf",carstate.car_state.x);
    coneInMap.cone_red.clear();
    coneInMap.cone_blue.clear();
    coneInMap.header.frame_id = "/map";
    double px = carstate.car_state.x;
    double py = carstate.car_state.y;
    double psi = carstate.car_state.theta;
    // double px=0;
    // double py=0;
    // double psi=0;
    
    while ((psi) >= M_PI)
      psi -= M_PI * 2.0;
    while ((psi) <= -1.0 * M_PI)
      psi += M_PI * 2.0;

    fsd_common_msgs::Cone tmp;
    double tempx,tempy;

    for(const auto& iter:boundaryDetections.cone_red)
    {
        tempx = iter.position.x;
        tempy = iter.position.y;

        tmp.position.x = px+tempx*cos(psi)-tempy*sin(psi);
        tmp.position.y = py+tempx*sin(psi)+tempy*cos(psi);
        tmp.color.data = "r";
        coneInMap.cone_red.push_back(tmp);
    }
    for(const auto& iter:boundaryDetections.cone_blue)
    {
        tempx = iter.position.x;
        tempy = iter.position.y;
        tmp.position.x = px+tempx*cos(psi)-tempy*sin(psi);
        tmp.position.y = py+tempx*sin(psi)+tempy*cos(psi);
        tmp.color.data = "b";
        coneInMap.cone_blue.push_back(tmp);
    }
    return;

}


} //end of ns_boundarydetection