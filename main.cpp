#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include "Path.h"
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <iostream>
#include <fstream>


struct TriangleStruct {

	ConstrainedTriangulation::Face_handle currFace;

	TriangleStruct* t = nullptr;

	Point_2 midPoint;

	bool first=true;

};


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


    Polygon_2 minus_robot;
    Polygon_2 tempRobot;
    auto delta = CGAL::ORIGIN - Point_2(robot.vertices_begin()->x(), robot.vertices_begin()->y());
    Transformation translate(CGAL::TRANSLATION,delta);
    tempRobot = transform(translate, robot);

    for (Polygon_2::Vertex_iterator vi = tempRobot.vertices_begin(); vi != tempRobot.vertices_end(); ++vi) {
        Kernel::FT x = vi->x(), y = vi->y();
        Point_2 minus_vertex(-x, -y);
        minus_robot.push_back(minus_vertex);
    }

    minus_robot.reverse_orientation();

    int obstacles_size = obstacles.size();


    // minkowsky sum = calc c-obstacles arrangement
    //Meanwhile , build a constrained triangulation - c-obstacles are the constraints
    ConstrainedTriangulation CT;
    for(int i = 0; i < obstacles_size; i++) {
    	obstacles[i].reverse_orientation();
        Polygon_with_holes_2  c_obstacle_with_hole  = minkowski_sum_2(minus_robot,obstacles[i]);
    //    CGAL_assertion (c_obstacle_with_hole.number_of_holes() == 0);
        Polygon_2 c_obstacle = c_obstacle_with_hole.outer_boundary();
        for (Polygon_2::Edge_const_iterator ei = c_obstacle.edges_begin(); ei != c_obstacle.edges_end(); ++ei) {
        	Point_2 src = Point_2(ei->source().x(),ei->source().y());
        	Point_2 trgt = Point_2(ei->target().x(),ei->target().y());
        	CT.insert_constraint(src,trgt);
        	}
    }



    CGAL_assertion (CT.is_valid());


    auto vs = CT.insert(start);
    auto ve = CT.insert(end);


    //add bounding box as constraint
    auto Xmax = CT.finite_vertices_begin()->point().x(); auto Xmin = CT.finite_vertices_begin()->point().x();
    auto Ymax = CT.finite_vertices_begin()->point().y(); auto Ymin = CT.finite_vertices_begin()->point().y();
    for (auto i = CT.finite_vertices_begin(); i!=CT.finite_vertices_end(); i++) {

    	if (Xmax<i->point().x()) {

    		Xmax = i->point().x();

    	}
    	if (Xmin>i->point().x()) {

    		Xmin = i->point().x();

    	}
    	if (Ymax<i->point().y()) {

    		Ymax = i->point().y();

    	}
    	if (Ymin>i->point().y()) {

    		Ymin = i->point().y();

    	}

    }

	Point_2 maxPoint = Point_2(Xmax*1.1,Ymax*1.1);

	auto outerVertex = CT.insert(maxPoint);

    CT.insert_constraint(Point_2(Xmax*1.2,Ymax*1.2),Point_2(Xmin*1.2,Ymax*1.2));
    CT.insert_constraint(Point_2(Xmax*1.2,Ymin*1.2),Point_2(Xmin*1.2,Ymin*1.2));
    CT.insert_constraint(Point_2(Xmax*1.2,Ymin*1.2),Point_2(Xmax*1.2,Ymax*1.2));
    CT.insert_constraint(Point_2(Xmin*1.2,Ymin*1.2),Point_2(Xmax*1.2,Ymin*1.2));




    ConstrainedTriangulation::Face_handle f;

    //use BFS to get all connected triangles (do not cross triangles through constraint edge).
    //use a struct which holds the triangle and a pointer to previous triangle.
    //In case of arriving at triangle which contains the end point, use pointer to previous triangles to create the path.

    std::queue<TriangleStruct *> TriQueue;


    //mark all ooutside faces as "outside"

  auto incidentFaces = outerVertex->incident_faces();

  auto firstFace = incidentFaces;

       do {
    	   if (!CT.is_infinite(incidentFaces)) {
    		    TriangleStruct * first = new TriangleStruct;
    		    first->currFace=incidentFaces;
    		    TriQueue.push(first);
    	   }
       } while (++incidentFaces!=firstFace);


        while (!TriQueue.empty()) {

            TriangleStruct * temp;
        	temp = TriQueue.front();
        	TriQueue.pop();
        	temp->currFace->info() = "outside";


        	if ( !(CT.is_infinite(temp->currFace->neighbor(0))) && !(temp->currFace->is_constrained(0)) && !(temp->currFace->neighbor(0)->info()=="outside")) {

        		TriangleStruct* temp2 = new TriangleStruct;
        		temp2->currFace = temp->currFace->neighbor(0);

    			TriQueue.push(temp2);
        	}


        	if (!(CT.is_infinite(temp->currFace->neighbor(1))) && !(temp->currFace->is_constrained(1)) && !(temp->currFace->neighbor(1)->info()=="outside")) {
                TriangleStruct* temp2 = new TriangleStruct;
        		temp2->currFace = temp->currFace->neighbor(1);
    			TriQueue.push(temp2);
        	}
        	if (!(temp->currFace->is_constrained(2)) && !(temp->currFace ->neighbor(2)->info()=="outside") && !(CT.is_infinite(temp->currFace->neighbor(2)))) {
                TriangleStruct* temp2 = new TriangleStruct;
        		temp2->currFace = temp->currFace->neighbor(2);
    			TriQueue.push(temp2);
    		}
    		delete(temp);
    	}


        //begin building "graph" of traversable triangles for BFS

        incidentFaces = vs->incident_faces();

        firstFace = incidentFaces;


   do {

	   if (!CT.is_infinite(incidentFaces) && incidentFaces->info()=="outside") {

		    TriangleStruct * first = new TriangleStruct;

		    first->currFace=incidentFaces;

		    first->first = true;

		    TriQueue.push(first);

	   }

   } while (++incidentFaces!=firstFace);

    bool foundPath = false;


    while (!TriQueue.empty()) {

        TriangleStruct * temp;

    	temp = TriQueue.front();
    	TriQueue.pop();

    	temp->currFace->info() = "visited";


    	if ( !(CT.is_infinite(temp->currFace->neighbor(0))) && !(temp->currFace->is_constrained(0)) && !(temp->currFace ->neighbor(0)->info()=="visited")) {

    		TriangleStruct* temp2 = new TriangleStruct;
    		temp2->currFace = temp->currFace->neighbor(0);
    		temp2->t = temp;
    		temp2->first = false;
    		temp2->midPoint = CGAL::midpoint(temp->currFace->vertex(1)->point(),temp->currFace->vertex(2)->point());
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2->currFace->vertex(2)->point(),temp2->currFace->vertex(1)->point(),temp2->currFace->vertex(0)->point());
        	if (temp2->currFace->has_vertex(ve)) {
        		foundPath=true;
        		break;
        	}

    	}

    	if (!(CT.is_infinite(temp->currFace->neighbor(1))) && !(temp->currFace->is_constrained(1)) && !(temp->currFace ->neighbor(1)->info()=="visited")) {

            TriangleStruct* temp2 = new TriangleStruct;

    		temp2->currFace = temp->currFace->neighbor(1);
    		temp2->t = temp;
    		temp2->first = false;
    		temp2->midPoint = CGAL::midpoint(temp->currFace->vertex(0)->point(),temp->currFace->vertex(2)->point());
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2->currFace->vertex(2)->point(),temp2->currFace->vertex(1)->point(),temp2->currFace->vertex(0)->point());
        	if (temp2->currFace->has_vertex(ve)) {
        		foundPath=true;
        		break;
        	}


    	}
    	if (!(temp->currFace->is_constrained(2)) && !(temp->currFace ->neighbor(2)->info()=="visited") && !(CT.is_infinite(temp->currFace->neighbor(2)))) {

            TriangleStruct* temp2 = new TriangleStruct;

    		temp2->currFace = temp->currFace->neighbor(2);
    		temp2->t = temp;
    		temp2->first = false;
    		temp2->midPoint = CGAL::midpoint(temp->currFace->vertex(0)->point(),temp->currFace->vertex(1)->point());
    		TriQueue.push(temp2);

        	Triangle_2 tri = Triangle_2(temp2->currFace->vertex(0)->point(),temp2->currFace->vertex(1)->point(),temp2->currFace->vertex(2)->point());
        	if (temp2->currFace->has_vertex(ve)) {
        		foundPath=true;
        		break;
        	}

    	}

    }

    //make path

    if (foundPath) {

    	std::list<Point_2> path;

    	auto temp3 = TriQueue.back();

    	path.push_front(end);

    	while (temp3->first!=true) {

    		auto p1 = temp3->currFace->vertex(0)->point();
    		auto p2 = temp3->currFace->vertex(1)->point();
    		auto p3 = temp3->currFace->vertex(2)->point();
    		Point_2 p4 = CGAL::midpoint(p1,CGAL::midpoint(p2,p3));
    		path.push_front(p4);
    		path.push_front(temp3->midPoint);
    		temp3 = temp3->t;
    	}

		auto p1 = temp3->currFace->vertex(0)->point();
		auto p2 = temp3->currFace->vertex(1)->point();
		auto p3 = temp3->currFace->vertex(2)->point();
		Point_2 p4 = CGAL::midpoint(p1,CGAL::midpoint(p2,p3));
		path.push_front(p4);

		path.push_front(start);

		return vector<Point_2>{ std::make_move_iterator(std::begin(path)),
		                  std::make_move_iterator(std::end(path)) };



    } else {

    	std::cout<<"no path found\n";

    }
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
