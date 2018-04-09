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

	Point_2 midPoint;

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

// vector<int> dijsktra(double** cost, int size, int source,int target)
// {
//     int i,m,min,start,d;
// 	double* dist = (double*)calloc(size, sizeof(double));
// 	int* prev = (int*) calloc(size, sizeof(int));
//     int* selected = (int*) calloc(size, sizeof(int));
// 	vector<int> path;
// 	int N = size;
// 	double IN = numeric_limits<double>::max();

//     for(i=0;i< N;i++)
//     {
//         dist[i] = IN;
//         prev[i] = -1;
//     }
//     start = source;
//     selected[start]=1;
//     dist[start] = 0;
//     while(selected[target] ==0)
//     {
// 		cout << " dijkstra iteration " << endl;
//         min = IN;
//         m = 0;
//         for(i=0;i< N;i++)
//         {
//             d = dist[start] +cost[start][i];
//             if(d< dist[i]&&selected[i]==0)
//             {
//                 dist[i] = d;
// 				cout << "prev of " << i << " is " << start << endl;
//                 prev[i] = start;
//             }
//             if(min>dist[i] && selected[i]==0)
//             {
//                 min = dist[i];
//                 m = i;
//             }
//         }
//         start = m;
//         selected[start] = 1;
//     }

// 	cout << " dijkstra search ended " << endl;

//     start = target;

// 	cout << "start " << start << endl;
// 	cout << "prev[start] " << prev[start] << endl;
// 	// retrieve path
//     while(start != -1)
//     {
//         path.push_back(start);
//         start = prev[start];
//     }

// 	free(dist);
// 	free(prev);
// 	free(selected);

//     return path;
// }

int minDistance(double* dist, bool* sptSet, int V)
{
   // Initialize min value
   double min = numeric_limits<double>::max();
   int min_index;

   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
}

// A utility function to print the constructed distance array
// int printSolution(double* dist, int n)
// {
//    printf("Vertex   Distance from Source\n");
//    for (int i = 0; i < V; i++)
//       printf("%d tt %d\n", i, dist[i]);
// }

// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
vector<int> dijkstra(double** graph, int V, int src, int target)
{
	vector<int> path;
	double MAX = numeric_limits<double>::max();
    double* dist = (double*) calloc(V, sizeof(double));     // The output array.  dist[i] will hold the shortest
                      // distance from src to i

    bool* sptSet = (bool*) calloc(V, sizeof(bool)); // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized

	int* prev = (int*) calloc(V, sizeof(int));

     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = MAX, sptSet[i] = false, prev[i]=-1;

     // Distance of source vertex from itself is always 0
     dist[src] = 0;

     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet, V);

       // Mark the picked vertex as processed
       sptSet[u] = true;

       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)

         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != MAX
                                       && dist[u]+graph[u][v] < dist[v]) {
            dist[v] = dist[u] + graph[u][v];
			prev[v] = u;
		}
     }

	int vertex = target;
	while(vertex != -1) {
		path.push_back(vertex);
		vertex = prev[vertex];
	}

	free(dist);
	free(sptSet);
	free(prev);

	return path;

     // print the constructed distance array
    //  printSolution(dist, V);
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
        //	for (Polygon_2::Edge_const_iterator ei = obstacles[i].edges_begin(); ei != obstacles[i].edges_end(); ++ei) {
        	std::cout<<ei->source()<<" ; "<<ei->target()<<endl;
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

    CT.insert_constraint(Point_2(Xmax*1.2,Ymax*1.2),Point_2(Xmin*1.2,Ymax*1.2));
    CT.insert_constraint(Point_2(Xmax*1.2,Ymin*1.2),Point_2(Xmin*1.2,Ymin*1.2));
    CT.insert_constraint(Point_2(Xmax*1.2,Ymin*1.2),Point_2(Xmax*1.2,Ymax*1.2));
    CT.insert_constraint(Point_2(Xmin*1.2,Ymin*1.2),Point_2(Xmax*1.2,Ymin*1.2));

	Point_2 maxPoint = Point_2(Xmax*1.1,Ymax*1.1);

	auto outerVertex = CT.insert(maxPoint);

	// Dijkstra : build graph
   vector<Point_2> vertices;
   unordered_map<string, int> vertices_to_index = {};
   vector<Edge> edges;


std::queue<TriangleStruct *> TriQueue;

	//go through all triangles reachable by edge of bounding box, and mark them as "outside"

	   auto incidentFaces = outerVertex->incident_faces();

   	   auto firstFace = incidentFaces;

   do {
	   if (!CT.is_infinite(incidentFaces)) {
		    TriangleStruct * first = new TriangleStruct;
		    first->currFace = incidentFaces;
		    TriQueue.push(first);
	   }
   } while (++incidentFaces!=firstFace);
    while (!TriQueue.empty()) {
        TriangleStruct * temp;
    	temp = TriQueue.front();
    	TriQueue.pop();
    	temp->currFace->info() = "outside";
    	if ( !(CT.is_infinite(temp->currFace->neighbor(0))) && !(temp->currFace->is_constrained(0)) && !(temp->currFace ->neighbor(0)->info()=="outside")) {
    		TriangleStruct* temp2 = new TriangleStruct;
    		temp2->currFace = temp->currFace->neighbor(0);
    		temp2->t = temp;
			TriQueue.push(temp2);
    	}
    	if (!(CT.is_infinite(temp->currFace->neighbor(1))) && !(temp->currFace->is_constrained(1)) && !(temp->currFace ->neighbor(1)->info()=="outside")) {
            TriangleStruct* temp2 = new TriangleStruct;
    		temp2->currFace = temp->currFace->neighbor(1);
    		temp2->t = temp;
			TriQueue.push(temp2);
    	}
    	if (!(temp->currFace->is_constrained(2)) && !(temp->currFace ->neighbor(2)->info()=="outside") && !(CT.is_infinite(temp->currFace->neighbor(2)))) {
            TriangleStruct* temp2 = new TriangleStruct;
    		temp2->currFace = temp->currFace->neighbor(2);
    		temp2->t = temp;
			TriQueue.push(temp2);
		}

		delete(temp);
	}

    //Beginning from starting vertex - traverse all triangles legal triangles, which will form the path graph


    incidentFaces = vs->incident_faces();
    firstFace = incidentFaces;

   do {

	   if (!CT.is_infinite(incidentFaces) && (incidentFaces->info()=="outside")) {

		    TriangleStruct * first = new TriangleStruct;

		    first->currFace=incidentFaces;

		    first->first = true;

		    TriQueue.push(first);


	   }

   } while (++incidentFaces!=firstFace);

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

    	}

    	if (!(CT.is_infinite(temp->currFace->neighbor(1))) && !(temp->currFace->is_constrained(1)) && !(temp->currFace ->neighbor(1)->info()=="visited")) {

            TriangleStruct* temp2 = new TriangleStruct;

    		temp2->currFace = temp->currFace->neighbor(1);
    		temp2->t = temp;
    		temp2->first = false;
    		temp2->midPoint = CGAL::midpoint(temp->currFace->vertex(0)->point(),temp->currFace->vertex(2)->point());
    		TriQueue.push(temp2);

    	}
    	if (!(temp->currFace->is_constrained(2)) && !(temp->currFace ->neighbor(2)->info()=="visited") && !(CT.is_infinite(temp->currFace->neighbor(2)))) {

            TriangleStruct* temp2 = new TriangleStruct;

    		temp2->currFace = temp->currFace->neighbor(2);
    		temp2->t = temp;
    		temp2->first = false;
    		temp2->midPoint = CGAL::midpoint(temp->currFace->vertex(0)->point(),temp->currFace->vertex(1)->point());
    		TriQueue.push(temp2);

    	}


		pushUnhandeledVerticesAndEdges(temp, vertices, vertices_to_index, edges);

		// pushNeighbors(TriQueue, temp, CT);
    }

	Point_2 startPoint(start.x(), start.y());
	Point_2 endPoint(end.x(), end.y());
	cout << "start key : " << getKey(&startPoint) << endl;
	cout << "end key : " << getKey(&endPoint) << endl;
	unordered_map<string,int>::const_iterator sourceIt = vertices_to_index.find(getKey(&startPoint));
	unordered_map<string,int>::const_iterator targetIt = vertices_to_index.find(getKey(&endPoint));


	int source = sourceIt->second;
	int target = targetIt -> second;
	cout << " source index : " << source << endl;
	cout << " target index : " << target << endl;

	// construct graph array repressentation
	int vertices_count = vertices.size();
	double** graph = (double **) calloc(vertices_count, sizeof(double *));
	for(int i=0; i<vertices_count;i++) {
		graph[i] = (double*) calloc(vertices_count, sizeof(double));
		for(int j = 0; j < vertices_count; j++)
			if(i == j)
				graph[i][j] = 0;
			else {
				graph[i][j] = numeric_limits<double>::max();
			}
	}

	for(int i=0; i<edges.size(); i++) {
		Edge e = edges[i];
		graph[e.v1][e.v2] = e.dist;
		graph[e.v2][e.v1] = e.dist;
	}

	for(int i=0; i<vertices_count; i++) {
		for(int j=i+1; j < vertices_count; j++)
			if(graph[i][j] != numeric_limits<double>::max())
				cout << " graph[" << i << " , " << j << "] : " << graph[i][j] << " | ";

		cout << endl;
	}

	cout << "vertices size " << vertices_count << endl;
	cout << "edges size " << edges.size() << endl;
	// for ( auto it = vertices_to_index.begin(); it != vertices_to_index.end(); ++it )
	// 	std::cout << " " << it->first << ":" << it->second << endl;
	// std::cout << std::endl;

	cout << "START DIJSTRA... " << endl;
	vector<int> path = dijkstra(graph, vertices_count,source,target);
	cout << "PATH LENGTH " << path.size() << endl;

	// go through the path and create a vector of points.
	// translate indexes in path using the vertices vector
	vector<Point_2> ret;

	cout << "PATH: " << endl;
	for(int i=path.size()-1; i>=0 ; i--) {
		int vertex_index = path[i];
		Point_2 p = vertices[vertex_index];
		ret.push_back(p);
		cout << getKey(&p) << endl;
	}

	return ret;
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
