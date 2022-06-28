/*
Creator: Giles Turnbull
date: 22/06/2022
github link: https://github.com/Giles-Turnbull

credits to OneLoneCoder.com - PathFinding A*
"No more getting lost..." - @Javidx9
github link: https://github.com/OneLoneCoder/videos/blob/master/OneLoneCoder_PathFinding_AStar.cpp
The basis for this code was taken and changed from this developer's github. Parts of this code were taken
from this developer's github and modified to fit my needs.

--ALL PARTS OF THE CODING CHALLENGE ARE COMPLETE--

This is a C++ implementation of the A* pathfinding algorithm used to travse a 2D grid.
The start and end points are defined by the user.
The algorithm is very simple and is not optimized for speed.
Obstacles are defined by the user, both amount and placement.
The grid can be traversed in 8 directions. The grid is fully expandable by the programmer.

I changed @Javidx9's code as I realised that fLocalGoal was being used as
a bias for the algorithm. Instead of avoiding obstacles, the algorithm
now just increases a bias number for a path if an obstacle is in that path.
This means that the algorithm will prefer paths that are less obstructed.
When the path is obstructed, the algorithm will still prefer paths that have
a lower fLocalGoal value, meaning that it will choose the path with the fewest
obstacles.
*/

#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <algorithm>
using namespace std;

//function to create a node for a graph
struct node
{
    bool obstacle;
    bool visited;
    float fGlobalGoal;
    float fLocalGoal;
    int x;
    int y;
    vector<node*> vecNeighbours;
    node* parent;
};

node *nodes = nullptr;
int mapWidth = 10;
int mapHeight = 10;

node *nodeStart = nullptr;
node *nodeEnd = nullptr;


bool ASTAR(){

    //set all nodes to default values
	for (int x = 0; x < mapWidth; x++)
		for (int y = 0; y < mapHeight; y++)
		{
			nodes[y*mapWidth + x].visited = false;
			nodes[y*mapWidth + x].fGlobalGoal = INFINITY;
			nodes[y*mapWidth + x].fLocalGoal = INFINITY;
			nodes[y*mapWidth + x].parent = nullptr;
		}
    
    //define distance between nodes and heuristic value
	auto distance = [](node* a, node* b)
	{
		return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
	};
	auto heuristic = [distance](node* a, node* b)
	{
		return distance(a, b);
	};

    //create a current node and set it to the start node
	node *nodeCurrent = nodeStart;
	nodeStart->fLocalGoal = 0.0f;
	nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);
	
    //create a list of nodes to be visited and add the start node to it
	list<node*> listNotTestedNodes;
	listNotTestedNodes.push_back(nodeStart);

	//loop until the end node is found or the list is empty
	while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
	{
			// Sort Untested nodes by global goal, so lowest is first
		listNotTestedNodes.sort([](const node* lhs, const node* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );
			
			// Front of listNotTestedNodes is potentially the lowest distance node. Our
			// list may also contain nodes that have been visited, so ditch these...
		while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->visited)
			listNotTestedNodes.pop_front();

			// ...or abort because there are no valid nodes left to test
		if (listNotTestedNodes.empty())
			break;

		nodeCurrent = listNotTestedNodes.front();
		nodeCurrent->visited = true; // We only explore a node once
			
					
			// Check each of this node's neighbours...
		for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
		{
				// all nodes that are unvisited are added to the list
			listNotTestedNodes.push_back(nodeNeighbour);

				// Calculate the neighbours potential lowest parent distance
				// if an obstacle is found then it isnt skipped but the bias is increased
			float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);
			if (nodeNeighbour->obstacle == 1){
				fPossiblyLowerGoal += 1000;
			}

				// If choosing to path through this node is a lower distance than what 
				// the neighbour currently has set, update the neighbour to use this node
				// as the path source, and set its distance scores as necessary
			if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
			{
				nodeNeighbour->parent = nodeCurrent;
				nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
					// The best path length to the neighbour being tested has changed, so
					// update the neighbour's score. The heuristic is used to globally bias
					// the path algorithm, so it knows if its getting better or worse. At some
					// point the algo will realise this path is worse and abandon it, and then go
					// and search along the next best path.
				nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
			}
		}	
	}

	return true;
}

//function to print path and steps of the algorithm
void printPath(node* current, node* nodeStart){
	int steps = 0;
	cout << "[";
	while (current->parent != nullptr){
            cout << "(" << current->x << "," << current->y << "), ";
            current = current->parent;
            steps++;
        }
	cout << "(" << nodeStart->x << "," << nodeStart->y << ")]" << endl;
	cout << "number of steps: " << steps << endl;
	
}


int main()
{
    //defining the nodes
    nodes = new node[mapWidth * mapHeight];
    for(int x=0; x<mapWidth; x++)
    {
        for(int y=0; y<mapHeight; y++)
        {
            nodes[x*mapHeight + y].x = x;
            nodes[x*mapHeight + y].y = y;
            nodes[x*mapHeight + y].obstacle = false;
            nodes[x*mapHeight + y].parent = nullptr;
            nodes[x*mapHeight + y].visited = false;
            nodes[x*mapHeight + y].fGlobalGoal = 1000;
            nodes[x*mapHeight + y].fLocalGoal = 1000;
        }
    }

    //creating the node connections
    for(int x=0; x<mapWidth; x++){
        for(int y=0; y<mapHeight; y++){
            if (y > 0){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y-1) * mapWidth + (x+0)]);
            }
            if (y<mapHeight - 1){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y+1) * mapWidth + (x+0)]);
            }
            if (x > 0){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y+0) * mapWidth + (x-1)]);
            }
            if (x<mapWidth - 1){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y+0) * mapWidth + (x+1)]);
            }
            //node connections for diagonals
            if (y > 0 && x > 0){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y-1) * mapWidth + (x-1)]);
            }
            if (y < mapHeight - 1 && x < mapWidth - 1){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y+1) * mapWidth + (x+1)]);
            }
            if (y > 0 && x < mapWidth - 1){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y-1) * mapWidth + (x+1)]);
            }
            if (y < mapHeight - 1 && x > 0){
                nodes[y*mapWidth + x].vecNeighbours.push_back(&nodes[(y+1) * mapWidth + (x-1)]);
            } 
        }
    }

    // user input start and end nodes
	// they are revesed to the user as the path prints backwards
    cout << "Enter the end point x and y values: ";
    int startX, startY;
    cin >> startX >> startY;
    cout << "Enter the start point x and y values: ";
    int endX, endY;
    cin >> endX >> endY;
    nodeStart = &nodes[startX * mapHeight + startY];
    nodeEnd = &nodes[endX * mapHeight + endY];

    //add obstacles
    cout << "Enter the number of obstacles: ";
    int numObstacles;
    cin >> numObstacles;
    for(int i=0; i<numObstacles; i++){
        cout << "Enter the obstacle x and y value: ";
        int obstacleX, obstacleY;
        cin >> obstacleX >> obstacleY;
        //if the node is already an obstacle
        bool loop = true;
        while (loop == true){
            if (nodes[obstacleX * mapHeight + obstacleY].obstacle == true){
                cout << "This node is already an obstacle, please try again" << endl;
                cout << "Enter the obstacle x and y value: ";
                cin >> obstacleX >> obstacleY;
            }
			//if the node is the start or end node
			else if (nodes[obstacleX * mapHeight + obstacleY].x == startX && nodes[obstacleX * mapHeight + obstacleY].y == startY){
				cout << "This node is the start node, please try again" << endl;
				cout << "Enter the obstacle x and y value: ";
				cin >> obstacleX >> obstacleY;
			}
            else{
                nodes[obstacleX * mapHeight + obstacleY].obstacle = true;
                loop = false;
            }
        }
    }

    //run the algorithm
    ASTAR();

    //print the path
    node* current = nodeEnd;
    int steps = 0;
	// if the path isn't fully blocked by obstacles
    if (nodeEnd->fLocalGoal < 1000){
        printPath(current, nodeStart);
    }
    else{
        cout << "Unable to reach delivery point" << endl;
		printPath(current, nodeStart);
		int obstacleCount = 0;
		node* current = nodeEnd;
		while (current->parent != nullptr){
            if (current->obstacle == true){
				cout << "(" << current->x << "," << current->y << ") is an ostacle so it is removed" << endl;
				obstacleCount++;
			}
			current = current->parent;
        }
		cout << "number of obstacles removed: " << obstacleCount << endl;
    }  
    

    //print all the nodes as a map
    for(int x=0; x<mapWidth; x++){
        for(int y=0; y<mapHeight; y++){
            //if the node is the start node
            if(nodes[y*mapWidth + x].x == startX && nodes[y*mapWidth + x].y == startY){
                cout << "E" << " ";
            }
            //if the node is the end node
            else if(nodes[y*mapWidth + x].x == endX && nodes[y*mapWidth + x].y == endY){
                cout << "R" << " ";
            }
            //if the node is an obstacle
            else if(nodes[y*mapWidth + x].obstacle){
                cout << "X" << " ";
            }
            //if the node is not an obstacle
            else{
                cout << "#" << " ";
            }
        }
        cout << endl;
    }
}