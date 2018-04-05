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

	TriangleStruct* t = NULL;

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



	ConstrainedTriangulation cdt;
	  std::cout << "Inserting a grid of 5x5 constraints " << std::endl;
	  for (int i = 1; i < 6; ++i)
	    cdt.insert_constraint( Point_2(0,i*100), Point_2(6*100,i*100));
	  for (int j = 1; j < 6; ++j)
	    cdt.insert_constraint( Point_2(j*100,0), Point_2(j*100,6*100));
	  assert(cdt.is_valid());
	  int count1 = 0;
	  for (ConstrainedTriangulation::Finite_edges_iterator eit = cdt.finite_edges_begin();
	       eit != cdt.finite_edges_end();
	       ++eit)
	    if (cdt.is_constrained(*eit)) ++count1;
	  std::cout << "The number of resulting constrained edges is  ";
	  std::cout <<  count1 << std::endl;


    // minus robot
	//need to bring first point of robot to start point?
    Polygon_2 minus_robot;
    Polygon_2 tempRobot;
    minus_robot = robot;
    auto delta = start - Point_2(minus_robot.vertices_begin()->x(), minus_robot.vertices_begin()->y());
    Transformation translate(CGAL::TRANSLATION,delta);
    tempRobot = transform(translate, robot);

    cout<<"done translation";

    for (Polygon_2::Vertex_iterator vi = tempRobot.vertices_begin(); vi != tempRobot.vertices_end(); ++vi) {
        Kernel::FT x = vi->x(), y = vi->y();
        Point_2 minus_vertex(-x, -y);
        minus_robot.push_back(minus_vertex);
    }

    int obstacles_size = obstacles.size();

    // minkowsky sum = calc c-obstacles arrangement
    //Meanwhile , build a constrained triangulation - c-obstacles are the constraints
    ConstrainedTriangulation CT;
    for(int i = 0; i < obstacles_size; i++) {
        Polygon_with_holes_2  c_obstacle_with_hole  = minkowski_sum_2(obstacles[i], minus_robot);
        CGAL_assertion (c_obstacle_with_hole.number_of_holes() == 0);
        Polygon_2 c_obstacle = c_obstacle_with_hole.outer_boundary();
        std::cout<<"Calculated Minkowski sum of "<<i<<" obstacle" <<endl;
   //     for (Polygon_2::Edge_const_iterator ei = c_obstacle.edges_begin(); ei != c_obstacle.edges_end(); ++ei) {
        	for (Polygon_2::Edge_const_iterator ei = obstacles[i].edges_begin(); ei != obstacles[i].edges_end(); ++ei) {
        	if (i==1) {
        	std::cout<<ei->source()<<" ; "<<ei->target()<<endl;
        	CT.insert_constraint(ei->source(),ei->target());
        	}
        	}
        std::cout<<"Added constrains of "<<i<<" obstacle" <<endl;
    }



    //output mesh structure using ipe
    std::ofstream myFile;
    std::ifstream Template;
	 std::string line;

    Template.open("ipe2.xml");
    myFile.open("Ipe.xml");


    while (std::getline(Template,line)) {
    	myFile <<line<<"\n";
    }

    myFile << "<page>\n";

    for (auto i=CT.finite_vertices_begin(); i!=CT.finite_vertices_end(); i++) {
    myFile << "<use name=\"mark/disk(sx)\" " << "pos= \"" << i->point().x() << " " << i->point().y() << "\" size=\"normal\" stroke=\"black\"/>\n";
    }

    for (auto i = CT.finite_faces_begin(); i!=CT.finite_faces_end(); i++) {

    Point_2 p1 = i->vertex(0)->point();

    Point_2 p2 = i->vertex(1)->point();

    myFile << "<path stroke = \"black\"> \n"  << p1 <<" m \n" << p2 << " l \n" << "</path> \n";

     p1 = i->vertex(1)->point();

     p2 = i->vertex(2)->point();

    myFile << "<path stroke = \"black\"> \n"  << p1 <<" m \n" << p2 << " l \n" << "</path> \n";

     p1 = i->vertex(2)->point();

     p2 = i->vertex(0)->point();

    myFile << "<path stroke = \"black\"> \n"  << p1 <<" m \n" << p2 << " l \n" << "</path> \n";


    }
    myFile << "</page>\n";
    myFile << "</ipe>\n";
    myFile.close();


    CGAL_assertion (CT.is_valid());


    auto vs = CT.insert(start);
    CT.insert(end);

    std::cout<<"created triangulation\n";

    ConstrainedTriangulation::Face_handle f;

    //find triangle which contains the start point
/*
    for (auto i = CT.finite_faces_begin(); i!=CT.finite_faces_end(); i++) {
    	Triangle_2 tri = Triangle_2(i->vertex(0)->point(),i->vertex(1)->point(),i->vertex(2)->point());
    	if (!tri.oriented_side(start)==CGAL::ON_NEGATIVE_SIDE) {
    		f=i;

    		break;
    	}

    }
   */
    //use BFS to get all connected triangles (do not cross triangles through constraint edge).
    //use a struct which holds the triangle and a pointer to previous triangle.
    //In case of arriving at triangle which contains the end point, use pointer to previous triangles to create the path.

    std::queue<TriangleStruct> TriQueue;

    TriangleStruct first;

    first.currFace=f;

   auto incidentFaces = vs->incident_faces();

   auto firstFace = incidentFaces;

   int count=0;

   do {

	   if (!CT.is_infinite(incidentFaces)) {

		    TriangleStruct first;

		    first.currFace=incidentFaces;

		    first.first = true;

		    incidentFaces->info() = "visited";

		    TriQueue.push(first);

		    count++;

	   }

   } while (++incidentFaces!=firstFace);

   std::cout<<"added first faces!"<<endl;

    bool foundPath = false;

    while (!TriQueue.empty()) {

        TriangleStruct temp;

    	temp = TriQueue.front();
    	TriQueue.pop();

    	temp.currFace->info() = "visited";

    	//check neighboring triangles: do not cross if: 1. edge is constraint 2. edge leads to infinite face 3. face is marked as "visited"
    	if ( !(CT.is_infinite(temp.currFace->neighbor(0)))/* && !(temp.currFace->is_constrained(0)) */ && !(temp.currFace ->neighbor(0)->info()=="visited")) {

    	cout<<"checking face 1"<<endl;

            TriangleStruct temp2;

    		temp2.currFace = temp.currFace->neighbor(0);
    		temp2.t = &temp;
    		temp2.first = false;
    		TriQueue.push(temp2);
    		count++;

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(2)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(0)->point());
        	if (tri.oriented_side(end)==CGAL::ON_ORIENTED_BOUNDARY) {
        		foundPath=true;
        		break;
        	}

    	if (!(CT.is_infinite(temp.currFace->neighbor(1)))/* && !(temp.currFace->is_constrained(1))*/ && !(temp.currFace ->neighbor(1)->info()=="visited")) {

    		cout<<"checking face 2"<<endl;

            TriangleStruct temp2;

    		temp2.currFace = temp.currFace->neighbor(1);
    		temp2.t = &temp;
    		temp2.first = false;
    		TriQueue.push(temp2);
    		count++;

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(2)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(0)->point());
        	if (tri.oriented_side(end)==CGAL::ON_ORIENTED_BOUNDARY) {
        		foundPath=true;
        		break;
        	}


    	}
    	if (/*!(temp.currFace->is_constrained(2)) && */!(temp.currFace ->neighbor(2)->info()=="visited") && !(CT.is_infinite(temp.currFace->neighbor(2)))) {

    		cout<<"checking face 3"<<endl;

            TriangleStruct temp2;

    		temp2.currFace = temp.currFace->neighbor(2);
    		temp2.t = &temp;
    		temp2.first = false;
    		TriQueue.push(temp2);
    		count++;

        	Triangle_2 tri = Triangle_2(temp2.currFace->vertex(0)->point(),temp2.currFace->vertex(1)->point(),temp2.currFace->vertex(2)->point());
        	if (tri.oriented_side(end)==CGAL::ON_ORIENTED_BOUNDARY) {
        		foundPath=true;
        		break;
        	}

    	}

    }

    }

    std::cout<<"Done building face graph"<<endl;
    std::cout<<"Added: "<<count<<" Triangles\n";

    for (auto i=CT.finite_vertices_begin(); i!=CT.finite_vertices_end(); i++) {
    	std::cout<<i->point()<<endl;
    }



    //make path

    if (foundPath) {

    	std::list<Point_2> path;

    	auto temp3 = TriQueue.front();

    	path.push_front(end);

    	while (temp3.first!=true) {

    		auto p1 = temp3.currFace->vertex(0)->point();
    		auto p2 = temp3.currFace->vertex(1)->point();
    		auto p3 = temp3.currFace->vertex(2)->point();
    		Point_2 p4 = CGAL::centroid(p1,p2,p3);
    		path.push_front(p4);

    		temp3 = *(temp3.t);
    	}

		auto p1 = temp3.currFace->vertex(0)->point();
		auto p2 = temp3.currFace->vertex(1)->point();
		auto p3 = temp3.currFace->vertex(2)->point();
		Point_2 p4 = CGAL::centroid(p1,p2,p3);
		path.push_front(p4);

		path.push_front(start);

		return vector<Point_2>{ std::make_move_iterator(std::begin(path)),
		                  std::make_move_iterator(std::end(path)) };



    } else {

    	std::cout<<"no path found\n";
  //  	exit (0);

    }




   //Djikstra?

//    return vector<Point_2>({start,{1.71,5.57},{23.84,5.94},{21.21,29.17}, end});
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
