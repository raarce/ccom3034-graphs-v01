/*--- Digraph.h ------------------------------------------------------------
 Header file for Digraph Class Template - L. Nyhoff

 R. Arce-Nazario - Added the OutputGraphviz function, added a few comments,
   and sanitized a few variable/class names. 
 -------------------------------------------------------------------------*/

#include <list>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include "ParsedFile.h"
using namespace std;

template <typename DataType>
class Digraph {
public:
    /*------------------------------------------------------------------------
      operator [] is overloaded to allow access to the data of a vertex.
      Example: operator[3] returns the data of vertex 3.
    ------------------------------------------------------------------------*/
    DataType operator[](int index);

    /*------------------------------------------------------------------------
      resize: Resizes the vectices vector.
    ------------------------------------------------------------------------*/
    void resize(int n) { Vertices.resize(n);}

    void readEdgeFile(string fName);


    /*------------------------------------------------------------------------
      read(): given an ifstream, populates a graph with vertices and edges.

      Precondition:  ifstream inStream is open.  The lines in the file to
        which it is connected are organized so that the data item in a
        vertex is on one line and on the next line is the number of 
        vertices adjacent to it followed by a list of these vertices.
      Postcondition: The adjacency list representation of this digraph 
         has been stored in Vertices.

      Example of input file. Each vertex requires two lines: A line with
        the vertex name followed by a line with the output degree and numbers
        of the adjacent nodes.

        San Juan
        2 2 3
        Carolina
        1 3
        Guaynabo
        1 1

      Symbolizes the graph:
        San Juan ---> Carolina
          |  ^           |
          \/ |           |
        Guaynabo  <------+ 
    ------------------------------------------------------------------------*/
    void read(ifstream & inStream);

    /*-----------------------------------------------------------------------
      display(): given an output stream, outputs each vertex and its
        adjacency list.

      Precondition:  ostream out is open.
      Postcondition: Each vertex and its adjacency list have
        been output to out.
    -----------------------------------------------------------------------*/

    void display(ostream & out);

    /*-----------------------------------------------------------------------
      Depth first search of digraph via depthFirstSearchAux(), starting
      at vertex start.

      Precondition:  start is a vertex.
      Postcondition: Digraph has been depth-first searched from start.
    -----------------------------------------------------------------------*/

    void depthFirstSearch(int start);

    /*-----------------------------------------------------------------------
      shortestPath(): find a shortest path in the digraph from vertex 
      start to vertex destination.

      Precondition:  start and destination are vertices.
      Postcondition: A vector of vertices along the shortest path from
        start to destination is returned.
    -----------------------------------------------------------------------*/
    vector<int> shortestPath(int start, int destination);
	
	  void InsertNode(DataType);
    void InsertNodeAt(DataType, int position);
	  
    void AddEdge(int OriginNodeID, int DestNodeID);


    /*-----------------------------------------------------------------------
      OutputGraphviz(): Write the graph to a text file in the 
      Graphviz (DOT) format.
   
      Precondition:  The file name is a valid file name.
      Postcondition: The file is created and filled with the graph information
       in Graphiviz format. The graphviz software may be used to visualize
       the graph.
    -----------------------------------------------------------------------*/	
    void OutputGraphviz(string fname);
	
private:
    /***** The class for each vertex *****/
    class Vertex {
    public: 
        DataType data;
        list<int> adjacencyList;
    }; // end of Vertex class

    /***** A Digraph is a container of Vertices *****/
    vector<Vertex> Vertices;


    /*-----------------------------------------------------------------------
      depthFirstSearchAux(): Helper function for the depthFirstSearch(), 
      recursively called.

      Precondition:  start is a vertex;  unvisited[i] is true if vertex i has
        not yet been visited, and is false otherwise.
      Postcondition: Vector unvisited has been updated.
    -----------------------------------------------------------------------*/
    void depthFirstSearchAux(int start, vector<bool> & unvisited);

}; // end of Digraph class template declaration


// Create a new Vertex entry and insert it to the vector of adjacency lists
template <typename DataType>
void Digraph<DataType>::InsertNode(DataType d) {
    typename Digraph<DataType>::Vertex myVInfo;
	  myVInfo.data = d;
	  Vertices.push_back(myVInfo);
}

// Create a new Vertex entry and insert it to the vector of adjacency lists
template <typename DataType>
void Digraph<DataType>::InsertNodeAt(DataType d, int position) {
    typename Digraph<DataType>::Vertex myVInfo;
    myVInfo.data = d;
    Vertices[position] = myVInfo;
}

// Add the DestNodeID to the adjacency list of the OriginNodeID
template <typename DataType>
void Digraph<DataType>::AddEdge(int OriginNodeID, int DestNodeID) {
    Vertices[OriginNodeID].adjacencyList.push_back(DestNodeID);
}

template <typename DataType>
void Digraph<DataType>::OutputGraphviz(string fname) {
    ofstream outfile (fname.c_str());
    outfile << "digraph G {\n" ;
    
    // for each vertex v
	  for (typename vector<Vertex>::iterator 
		    vector_it = Vertices.begin();
		    vector_it != Vertices.end(); vector_it++) {
		    // for each vertex u in the adjacency list of v 
		    for (list<int>::iterator it = vector_it->adjacencyList.begin();
			       it != vector_it->adjacencyList.end(); it++) {        
            // output v's name -> u's name 
            outfile << "\"" << vector_it->data  << "\"" <<  " -> " << "\"" <<  Vertices[*it].data << "\"" <<  " ;" << endl;
        }
    }

	outfile << "}" ;
	outfile.close();
}


//--- Implementation of operator[]
template <typename DataType>
inline DataType Digraph<DataType>::operator[](int index) {
    return Vertices[index].data;
}

//--- Implementation of read(), which reads a graph from a file.
template <typename DataType>
void Digraph<DataType>::read(ifstream & inStream) {
    typename Digraph<DataType>::Vertex vi;
    int n;           // number of vertices adjacent to some vertex
    int vertex;      // the ID number of a vertex

    // Put a garbage 0-th value so indices start with 1, as is customary
    Vertices.push_back(vi); 

    // Read from file to construct adjacency list representation
    for (;;) {
        // read the node label
        inStream >> vi.data;
        if (inStream.eof()) break;

        // read the number of nodes
        inStream >> n;
        list<int> adjList;      // construct empty list
        for (int i = 1; i <= n; i++) {
            // read the vertex ID
            inStream >> vertex;
            adjList.push_back(vertex);
        }
        vi.adjacencyList = adjList;
        Vertices.push_back(vi);
    }
}

//--- Definition of display()
template <typename DataType>
void Digraph<DataType>::display(ostream & out)
{
    out << "Adjacency-List Representation: \n";
    for (int i = 0; i < Vertices.size(); i++) {
        out << i << ": " <<  Vertices[i].data << "->";
        for (list<int>::iterator 
          it = Vertices[i].adjacencyList.begin();
          it != Vertices[i].adjacencyList.end(); it++)
              out << *it << "  ";
        out << endl;
    }
}
  
//-- Definitions of depthFirstSearch() and depthFirstSearchAux()
template <typename DataType>
inline void Digraph<DataType>::depthFirstSearch(int start) {
    vector<bool> unvisited(Vertices.size(), true);
    depthFirstSearchAux(start, unvisited);
}

template <typename DataType>
void Digraph<DataType>::depthFirstSearchAux(
                             int start, vector<bool> & unvisited) {

    // Add statements here to process Vertices[start].data
    cout << "\t" << Vertices[start].data << endl;

    unvisited[start] = false;

    // Traverse its adjacency list, performing depth-first 
    // searches from each unvisited vertex in it.
    for (list<int>::iterator 
         it = Vertices[start].adjacencyList.begin();
         it != Vertices[start].adjacencyList.end(); it++)
        // check if current vertex has been visited
        if (unvisited[*it])
            // start DFS from new node
            depthFirstSearchAux(*it, unvisited); 
}

//--- Definition of shortestPath()
template<typename DataType>
vector<int> Digraph<DataType>::shortestPath(int start, int destination) {
    int n = Vertices.size(); // number of vertices (#ed from 1)
    vector<int> distLabel(n,-1);     // distance labels for vertices, all
                                   // marked as unvisited (-1)
    vector<int>  predLabel(n);        // predecessor labels for vertices
  
    // Perform breadth first search from start to find destination,
    // labeling vertices with distances from start as we go.
    distLabel[start] = 0;
    int distance = 0;                // distance from start vertex
    int vertex;                      // a vertex
  
    queue<int> vertexQueue;          // queue of vertices
    vertexQueue.push(start);
    while (distLabel[destination] < 0 && !vertexQueue.empty()) {
        vertex = vertexQueue.front();
        vertexQueue.pop();
        if (distLabel[vertex] > distance)
            distance++;
        for (list<int>::iterator it = Vertices[vertex].adjacencyList.begin();
            it != Vertices[vertex].adjacencyList.end(); it++) {
          
            if (distLabel[*it] < 0) {
                distLabel[*it] = distance + 1;
                predLabel[*it] = vertex;
                vertexQueue.push(*it);
            }
        }
    }
    distance++;

    // Now reconstruct the shortest path if there is one
    vector<int> path(distance+1);
    if (distLabel[destination] < 0)
        cout << "Destination not reachable from start vertex\n";
    else {
        path[distance] = destination;
        for (int k = distance - 1; k >= 0; k--)
            path[k] = predLabel[path[k+1]];
    }

    return path;
}


template <typename DataType>
void Digraph<DataType>::readEdgeFile(string fName) {

    vector<string> files = vector<string>();

    ParsedFile P(fName);
    vector<string> tokens = P.readAndTokenize(";");
    map<string, int> M;

    //for_each (tokens.begin(), tokens.end(), print);

    int ctr = 0;
    cout << "Size:" << tokens.size() << endl;
    for (int i=0; i<(tokens.size()/3)*3;i=i+3) {
      if (M.find(tokens[i])   == M.end()) M[tokens[i]] = ctr++;
      if (M.find(tokens[i+2]) == M.end()) M[tokens[i+2]] = ctr++;
    }

    cout << "Distinct actors:" << M.size() << endl;

    // create the nodes of our graph

    // add the edges
    //for_each(M.begin(),M.end(),printPair);
    resize(M.size());
    map<string, int>::iterator it;
    for(it=M.begin();it!=M.end();it++) 
      InsertNodeAt(it->first,it->second);
    
    for (int i=0; i<(tokens.size()/3)*3;i=i+3) {
      AddEdge(M[tokens[i]],M[tokens[i+2]]);
      AddEdge(M[tokens[i+2]],M[tokens[i]]);
    } 

}


