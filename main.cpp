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
#include <unordered_map>

struct TriangleStruct {

	ConstrainedTriangulation::Face_handle currFace;

	TriangleStruct* t = nullptr;

	bool first=true;

};

struct Edge {
	int v1, v2;
	double dist;
};

using namespace std;

// methods for Dijstra
string getKey(Point_2 * p) {
	ostringstream stream;
	stream << p->x().to_double() << "/" << p->y().to_double();
	return stream.str();
}

int pushVertex(vector<Point_2> &vertices, unordered_map<string, int> &vertices_to_index, Point_2 * p) {
	vertices.push_back(*p);
	int size = vertices.size();
	vertices_to_index.insert(make_pair(getKey(p), size-1));
}

void pushEdge(vector<Edge> &edges, int i1, int i2, Point_2 * p1, Point_2 * p2) {
	Edge e;
	e.v1=i1;
	e.v2=i2;

	double x1 = p1->x().to_double();
	double y1 = p1->y().to_double();
	double x2 = p2->x().to_double();
	double y2 = p2->y().to_double();

	e.dist=sqrt( pow(x1-x2,2) + pow(y1-y2,2));

	edges.push_back(e);   
}

void pushNeighbors(std::queue<TriangleStruct *> q, TriangleStruct * triangle, ConstrainedTriangulation &CT) {
	ConstrainedTriangulation::Face_handle face = triangle->currFace;
	for(int i = 0; i<3; i++) {
		if (!(face->is_constrained(i)) && !(face->neighbor(i)->info()=="visited") && !(CT.is_infinite(face->neighbor(i)))) {
			TriangleStruct* temp2 = new TriangleStruct;

    		temp2->currFace = face->neighbor(i);
    		temp2->t = triangle;
    		temp2->first = false;
    		q.push(temp2);
		}
	}
}

void pushUnhandeledVerticesAndEdges(TriangleStruct * triangle, vector<Point_2> &vertices, unordered_map<string, int> &vertices_to_index, vector<Edge> &edges) {
	int vertices_index[3];
	Point_2 vertices_points[3];

	for(int i=0; i<3; i++) {
		Point_2 p = triangle->currFace->vertex(i)->point();
		vertices_points[i] = p;
		unordered_map<string,int>::const_iterator indexIter = vertices_to_index.find(getKey(&p));
		int index;
		if(indexIter == vertices_to_index.end()) {
			pushVertex(vertices, vertices_to_index, &p);
			index = vertices.size() - 1;
		}
		else {
			index = indexIter->second;
		}

		vertices_index[i] = index;
	}

	pushEdge(edges, vertices_index[0], vertices_index[1], &vertices_points[0], &vertices_points[1]);
	pushEdge(edges, vertices_index[0], vertices_index[2], &vertices_points[0], &vertices_points[2]);
	pushEdge(edges, vertices_index[2], vertices_index[1], &vertices_points[2], &vertices_points[1]);
}

char* dijsktra(double** cost, int size, int source,int target)
{
    int i,m,min,start,d,j;
	double* dist = (double*)calloc(size, sizeof(double));
	int* prev = (int*) calloc(size, sizeof(int));
    int* selected = (int*) calloc(size, sizeof(int));
	char* path = (char*) calloc(size, sizeof(char));
	int N = size;
	double IN = numeric_limits<double>::max();

    for(i=1;i< N;i++)
    {
        dist[i] = IN;
        prev[i] = -1;
    }
    start = source;
    selected[start]=1;
    dist[start] = 0;
    while(selected[target] ==0)
    {
        min = IN;
        m = 0;
        for(i=1;i< N;i++)
        {
            d = dist[start] +cost[start][i];
            if(d< dist[i]&&selected[i]==0)
            {
                dist[i] = d;
                prev[i] = start;
            }
            if(min>dist[i] && selected[i]==0)
            {
                min = dist[i];
                m = i;
            }
        }
        start = m;
        selected[start] = 1;
    }
    start = target;
    j = 0;
    while(start != -1)
    {
        path[j++] = start+65;
        start = prev[start];
    }
    path[j]='\0';
	cout << "PATHHHH " << path << endl;

	free(dist);
	free(prev);
	free(selected);

    return path;
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


/*
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
*/

    // minus robot
	//need to bring first point of robot to start point?
    Polygon_2 minus_robot;
    Polygon_2 tempRobot;
    auto delta = CGAL::ORIGIN - Point_2(robot.vertices_begin()->x(), robot.vertices_begin()->y());
    Transformation translate(CGAL::TRANSLATION,delta);
    tempRobot = transform(translate, robot);

    cout<<"done translation"<<endl;

    for (Polygon_2::Vertex_iterator vi = tempRobot.vertices_begin(); vi != tempRobot.vertices_end(); ++vi) {
        Kernel::FT x = vi->x(), y = vi->y();
        Point_2 minus_vertex(-x, -y);
        minus_robot.push_back(minus_vertex);
    }

    cout<<"Minus_robot: "<<endl;

    minus_robot.reverse_orientation();

    for (auto i=minus_robot.vertices_begin(); i!=minus_robot.vertices_end(); i++) {

    	 cout<<*i<<endl;

    }
    cout<<"Minus_robot: "<<endl;

    int obstacles_size = obstacles.size();

    if (obstacles[0].orientation()==minus_robot.orientation()) {
    	if (minus_robot.orientation()==CGAL::COUNTERCLOCKWISE) {
    	std::cout<<"Polygons have same orientation (CounterClockWise) "<<endl;
    	} else if (minus_robot.orientation()==CGAL::CLOCKWISE) {
    	std::cout<<"Polygons have same orientation (ClockWise) "<<endl;
    	}
    }

    // minkowsky sum = calc c-obstacles arrangement
    //Meanwhile , build a constrained triangulation - c-obstacles are the constraints
    ConstrainedTriangulation CT;
    for(int i = 0; i < obstacles_size; i++) {
    	obstacles[i].reverse_orientation();
        Polygon_with_holes_2  c_obstacle_with_hole  = minkowski_sum_2(minus_robot,obstacles[i]);
    //    CGAL_assertion (c_obstacle_with_hole.number_of_holes() == 0);
    	cout<<"calculating minkowski sum"<<endl;
        Polygon_2 c_obstacle = c_obstacle_with_hole.outer_boundary();
        std::cout<<"Calculated Minkowski sum of "<<i<<" obstacle" <<endl;
        for (Polygon_2::Edge_const_iterator ei = c_obstacle.edges_begin(); ei != c_obstacle.edges_end(); ++ei) {
        //	for (Polygon_2::Edge_const_iterator ei = obstacles[i].edges_begin(); ei != obstacles[i].edges_end(); ++ei) {
        	std::cout<<ei->source()<<" ; "<<ei->target()<<endl;
        	Point_2 src = Point_2(ei->source().x(),ei->source().y());
        	Point_2 trgt = Point_2(ei->target().x(),ei->target().y());
        	CT.insert_constraint(src,trgt);
        	}
        std::cout<<"Added constrains of "<<i<<" obstacle" <<endl;
    }



    CGAL_assertion (CT.is_valid());


    auto vs = CT.insert(start);
    CT.insert(end);




    //wrap

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
    myFile << "<use name=\"mark/disk(sx)\" " << "pos= \"" << i->point().x().to_double() << " " << i->point().y().to_double() << "\" size=\"normal\" stroke=\"black\"/>\n";
    }

    for (auto i = CT.finite_faces_begin(); i!=CT.finite_faces_end(); i++) {

    double p1x = i->vertex(0)->point().x().to_double(); double p1y = i->vertex(0)->point().y().to_double();

    double p2x = i->vertex(1)->point().x().to_double(); double p2y = i->vertex(1)->point().y().to_double();

    myFile << "<path stroke = \"black\"> \n"  << p1x <<" "<<p1y<<" m \n" << p2x <<" "<<p2y<< " l \n" << "</path> \n";

     p1x = i->vertex(1)->point().x().to_double();  p1y = i->vertex(1)->point().y().to_double();

     p2x = i->vertex(2)->point().x().to_double();  p2y = i->vertex(2)->point().y().to_double();

     myFile << "<path stroke = \"black\"> \n"  << p1x <<" "<<p1y<<" m \n" << p2x <<" "<<p2y<< " l \n" << "</path> \n";

     p1x = i->vertex(2)->point().x().to_double();  p1y = i->vertex(2)->point().y().to_double();

     p2x = i->vertex(0)->point().x().to_double();  p2y = i->vertex(0)->point().y().to_double();

     myFile << "<path stroke = \"black\"> \n"  << p1x <<" "<<p1y<<" m \n" << p2x <<" "<<p2y<< " l \n" << "</path> \n";

    }
    myFile << "</page>\n";
    myFile << "</ipe>\n";
    myFile.close();




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

	// Dijkstra : build graph
   vector<Point_2> vertices;
   unordered_map<string, int> vertices_to_index = {};
   vector<Edge> edges;
   

std::queue<TriangleStruct *> TriQueue;

   auto incidentFaces = vs->incident_faces();

   auto firstFace = incidentFaces;

   int count=0;

   do {

	   if (!CT.is_infinite(incidentFaces)) {

		    TriangleStruct * first = new TriangleStruct;

		    first->currFace=incidentFaces;

		    first->first = true;

		    TriQueue.push(first);

		    count++;

	   }

   } while (++incidentFaces!=firstFace);

   std::cout<<"added first face!"<<endl;

    while (!TriQueue.empty()) {

        TriangleStruct * temp;

    	temp = TriQueue.front();
    	TriQueue.pop();

    	temp->currFace->info() = "visited";

		pushUnhandeledVerticesAndEdges(temp, vertices, vertices_to_index, edges);

		pushNeighbors(TriQueue, temp, CT);
    }

	Point_2 startPoint(start.x(), start.y());
	Point_2 endPoint(end.x(), end.y());
	cout << "start key" << getKey(&endPoint) << endl;	
	unordered_map<string,int>::const_iterator sourceIt = vertices_to_index.find(getKey(&startPoint));
	unordered_map<string,int>::const_iterator targetIt = vertices_to_index.find(getKey(&endPoint));


	int source = sourceIt->second;
	int target = targetIt -> second;
	cout << " source" << source << endl;
	cout << " target" << source << endl;

	// construct graph array repressentation
	int vertices_count = vertices.size();
	double** graph = (double **) calloc(vertices_count, sizeof(double *));
	for(int i=0; i<vertices_count;i++) {
		graph[i] = (double*) calloc(vertices_count, sizeof(double));
		for(int j = 0; j < vertices_count; j++)
			graph[i][j] = numeric_limits<double>::max();
	}

	for(int i=0; i<edges.size(); i++) {
		Edge e = edges[i];
		graph[e.v1][e.v2] = e.dist;
		graph[e.v2][e.v1] = e.dist;
	}

	cout << "vertices size " << vertices_count << endl;
	cout << "edges size " << edges.size() << endl;
	for ( auto it = vertices_to_index.begin(); it != vertices_to_index.end(); ++it )
		std::cout << " " << it->first << ":" << it->second;
	std::cout << std::endl;

	char* path = dijsktra(graph, vertices_count,source,target);

	// go through the path and create a vector of points.
	// translate indexes in path using the vertices vector
	//TODO  

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
