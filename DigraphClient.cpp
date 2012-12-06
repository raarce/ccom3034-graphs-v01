/*
 *  DigraphClient.cpp
 *  
 *
 *  Created by Rafael Arce on 11/27/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 *
 *  This code creates a simple
 */

#include <iostream>
#include "Digraph.h"
using namespace std;

int main() {
	Digraph<string> G;
	/*
        G.InsertNode("Buho");    //0
        G.InsertNode("Halcon");	 //1
        G.InsertNode("Zorra");	 //2
        G.InsertNode("Arbol");  //3
        G.InsertNode("Pajaro"); //4
        G.InsertNode("Mosquito");//5
        G.InsertNode("Conejo"); //6
        G.InsertNode("Grama");      //7
        G.InsertNode("Ciervo");  //8

        //int tmp;
        //cout << "Enter anything:";
        //cin >>  tmp;

        G.AddEdge(3,4);
        G.AddEdge(4,0);
        G.AddEdge(3,8);
        G.AddEdge(6,0);
        G.AddEdge(6,1);
        G.AddEdge(8,5);
        G.AddEdge(7,8);
        G.AddEdge(6,2);
        G.AddEdge(6,1);
*/
    ifstream inFileStream("flights.txt");
    G.read(inFileStream);
    cout << endl << "creating dot...";
    G.OutputGraphviz("flights.dot");
    cout << "done!, see the file flights.dot" << endl << endl;

    cout << "Doing a DFS starting at: " << G[3] << endl; 
    G.depthFirstSearch(3);
    cout << ".. and that ends the DFS " << endl << endl; 

    vector<int> SP = G.shortestPath(3,5);
    cout << "Shortest path from " << G[3] << " to " << G[5] << " is:" << endl; 
    for (int i=0; i<SP.size(); i++) cout << G[SP[i]]<< " " ;
    cout << endl << "A path of length: " << SP.size()-1 << endl;
    return 0;
}
 
