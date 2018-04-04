#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include "Path.h"
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>



struct TriangleStruct {

	ConstrainedTriangulation::Face_handle currFace;

	TriangleStruct t;

	bool first=true;

};



using namespace std;

bool InsideTriangle (Point_2 p,Point_2 t1,Point_2 t2,Point_2 t3) {
	auto Area1 = CGAL::area(t1,p,t2); auto Area2 = CGAL::area(t2,p,t3); auto Area3 = CGAL::area(t3,p,t1);

	auto Area = CGAL::area(t1,t2,t3);

	if (Area==Area1+Area2+Area3) {
		return true;
	}

	return false;
}



Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}

vector<Point_2> findPath(const Point_2 &start, const Point_2 &end, const Polygon_2 &robot, vector<Polygon_2> &obstacles) {
    //todo: implement this function.

    // minus robot
    Polygon_2 minus_robot;
    for (Polygon_2::Vertex_iterator vi = robot.vertices_begin(); vi != robot.vertices_end(); ++vi) {
        Kernel::FT x = vi->x(), y = vi->y();
        Point_2 minus_vertex(-x, -y);
        minus_robot.push_back(minus_vertex);
    }

    int obstacles_size = obstacles.size();

    // minkowsky sum = calc c-obstacles arrangement
    //Meanwhile , build a constrained triangulation - c-obstacles are the constraints
    ConstrainedTriangulation CT;
 //   Arrangement_2 arr;
    for(int i = 0; i < obstacles_size; i++) {
        Polygon_with_holes_2  c_obstacle_with_hole  = minkowski_sum_2(obstacles[i], minus_robot);
        CGAL_assertion (c_obstacle_with_hole.number_of_holes() == 0);
        Polygon_2 c_obstacle = c_obstacle_with_hole.outer_boundary();

  //      int obs_size = c_obstacle.size();
 //       Segment_2 edges[obs_size];
        int index = 0;
        for (Polygon_2::Edge_const_iterator ei = c_obstacle.edges_begin(); ei != c_obstacle.edges_end(); ++ei) {

            CT.insert_constraint(ei->source(),ei->target());
        }
    }

    CT.insert(start);
    CT.insert(end);

    ConstrainedTriangulation::Face_handle f;

    //find triangle which contains the start point

    for (auto i = CT.finite_faces_begin(); i!=CT.finite_faces_end(); i++) {
    	Triangle_2 tri = Triangle_2(i->vertex(0)->point(),i->vertex(1)->point(),i->vertex(2)->point());
    	if (!tri.oriented_side(start)==CGAL::ON_NEGATIVE_SIDE) {
    		f=i;

    		break;
    	}

    }
    
    //use BFS to get all connected triangles (do not cross triangles through constraint edge).
    //use a struct which holds the triangle and a pointer to previous triangle.
    //In case of arriving at triangle which contains the end point, use pointer to previous triangles to create the path.

    std::queue<TriangleStruct> TriQueue;

    TriangleStruct temp;

    temp.currFace=f;

    f->info() = "visited";

    TriQueue.push(temp);

    bool foundPath = false;

    while (!TriQueue.empty()) {
    	temp = TriQueue.front();
    	TriQueue.pop();

    	temp.currFace->info() = "visited";

        TriangleStruct temp2;

    	//check neighboring triangles: do not cross if: 1. edge is constraint 2. edge leads to infinite face 3. face is marked as "visited"
    	if (!(temp.currFace->is_constrained(0)) && !(temp.currFace ->neighbor(0)->info()=="visited")) {

    		temp2.currFace = temp.currFace->neighbor(0)->neighbor(0);
    		temp2.t = temp;
    		temp2.first = false;
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(0)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(2)->point());
        	if (!tri.oriented_side(start)==CGAL::ON_NEGATIVE_SIDE) {
        		foundPath=true;
        		break;
        	}

    	if (!(temp.currFace->is_constrained(1)) && !(temp.currFace ->neighbor(1)->info()=="visited")) {

    		temp2.currFace = temp.currFace->neighbor(1)->neighbor(1);
    		temp2.t = temp;
    		temp2.first = false;
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(0)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(2)->point());
        	if (!tri.oriented_side(start)==CGAL::ON_NEGATIVE_SIDE) {
        		foundPath=true;
        		break;
        	}


    	}
    	if (!(temp.currFace->is_constrained(2)) && !(temp.currFace ->neighbor(2)->info()=="visited")) {

    		temp2.currFace = temp.currFace->neighbor(2)->neighbor(2);
    		temp2.t = temp;
    		temp2.first = false;
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(0)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(2)->point());
        	if (!tri.oriented_side(start)==CGAL::ON_NEGATIVE_SIDE) {
        		foundPath=true;
        		break;
        	}

    	}

    }



    }

    //make path

    if (foundPath) {

    	auto temp3 = TriQueue.front();


    }


   //Djikstra?

    return vector<Point_2>({start,{1.71,5.57},{23.84,5.94},{21.21,29.17}, end});
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        cerr << "[USAGE]: inputRobot inputObstacles outputFile" << endl;
        return 1;
    }

    ifstream inputRobotFile(argv[1]), inputObstaclesFile(argv[2]);
    if (!inputRobotFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!inputRobotFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        return -1;
    }

    auto startPoint = loadPoint_2(inputRobotFile);
    auto endPoint = loadPoint_2(inputRobotFile);
    if (startPoint == endPoint) {
        cout << "Can't have the same startPoint endPoint" << endl;
        return -1;
    };
    auto robot = loadPolygon(inputRobotFile);
    inputRobotFile.close();

    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();
    boost::timer timer;
    auto result = Path(findPath(startPoint, endPoint, robot, obstacles));
    auto secs = timer.elapsed();
    cout << "Path created:      " << secs << " secs" << endl;
    cout << "Path validation:   " << ((result.verify(startPoint, endPoint, robot, obstacles)) ? "Success!" : "Faulure")
         << endl;

    ofstream outputFile;
    outputFile.open(argv[3]);
    if (!outputFile.is_open()) {
        cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
        return -1;
    }
    outputFile << result << endl;
    outputFile.close();
    return 0;
}
