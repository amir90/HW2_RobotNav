#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include "Path.h"

using namespace std;


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
    Polygon_set_2 polygon_set; 
    for(int i = 0; i < obstacles_size; i++) {
        Polygon_with_holes_2  c_obstacle_with_hole  = minkowski_sum_2(obstacles[i], minus_robot);
        CGAL_assertion (c_obstacle_with_hole.number_of_holes() == 0);
        Polygon_2 c_obstacle = c_obstacle_with_hole.outer_boundary();
        
        polygon_set.join(c_obstacle);
        // int obs_size = c_obstacle.size();
        // Segment_2 edges[obs_size];
        // int index = 0;
        // for (Polygon_2::Edge_const_iterator ei = c_obstacle.edges_begin(); ei != c_obstacle.edges_end(); ++ei) {
        //     edges[index++] = *ei;
        // }

        // CGAL::insert(arr, &edges[0], &edges[obs_size]);
    }

    polygon_set.complement();
    Arrangement_2 free_space_arrangement = polygon_set.arrangement();

    //trapezodial decomposition
    pair<Arrangement_2::Vertex_const_handle, pair<CGAL::Object, CGAL::Object> > io;
    CGAL::decompose(free_space_arrangement, io);
    
    //create graph from decomposition
    
    // find path (bfs or Digstra for bonus) from start to end

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