#include "PathSolver.h"
#include <iostream>

PathSolver::PathSolver(){
	nodesExplored = new NodeList(); 
	envRows = 0;
	envCols = 0;
}

PathSolver::~PathSolver(){
	delete nodesExplored;
	nodesExplored = nullptr;
}

void PathSolver::forwardSearch(Env env){
	int envRows = getEnvRows();
	int envCols = getEnvCols();

	// Make a map with same size of env and mark the explored position
	// 0 -> unvisited, 1 -> visited
	int visit[envRows][envCols];
	for(int i = 0; i < envRows; ++i){
		for(int j = 0; j < envCols; j++){
			visit[i][j] = 0;
		}
	}

	// Position of current node's UP, DOWN, LEFT, RIGHT
	int position_y[4] = {-1,1,0,0}, position_x[4] = {0,0,-1,1};
	Node* start = nullptr, * goal = nullptr;

	// Make open list to put all the possible nodes
	NodeList* openList = new NodeList();
	// Initialize open list with size of env
	openList->initialization(envRows * envCols);

	// Initialize nodesExplored nodelist and this is close list
	this->nodesExplored->initialization(envRows * envCols);

	// Check position of S and G, and then make S(start) and G(goal) node
	for(int i = 0; i < envRows; ++i){
		for(int j = 0; j < envCols; ++j){
			if(env[i][j] == SYMBOL_START){
				start = new Node(i, j, 0);
			}
			if(env[i][j] == SYMBOL_GOAL){
				goal = new Node(i, j, 0); 
			}
		}
	}

	// Mark S position
	visit[start->getRow()][start->getCol()] = 1; 
	// Put S in open_list
	openList->addElement(start);

	// flag will turn to true when current Node reaches at G and then break the loop
	bool flag = false; 
	
	// Take one node from open list until it gets G
	for(int i = 0; !flag && i < openList->getLength(); i++){
		// Take one node from open list and make it current node
		Node* currentNode = new Node(openList->getNode(i)->getRow(), openList->getNode(i)->getCol(), openList->getNode(i)->getDistanceTraveled());
		int current_y = currentNode->getRow();
		int current_x = currentNode->getCol();
		
		// Put current node in close list which is nodesExplored
		this->nodesExplored->addElement(currentNode);

		// if current position is G, then make flag true and stop searching
		if(current_y == goal->getRow() && current_x == goal->getCol()){
			flag = true;
		}
		else{
			for(int j = 0; j < 4; j++){
				// Search 4 nodes in order of UP, DOWN, LEFT and RIGHT
				int next_y = current_y + position_y[j]; 
				int next_x = current_x + position_x[j];

				// if any of UP, DOWN, LEFT or RIGHT position is inside of maze, not wall and not explored, then put it in open list 
				if(next_y >= 0 && next_x >= 0 && next_y < envRows && next_x < envCols && env[next_y][next_x] != SYMBOL_WALL && visit[next_y][next_x] != 1){
					// Mark this position as explored
					visit[next_y][next_x] = 1; 
					// Using this position, make next node and then put it in open list
					Node* nextNode = new Node(next_y, next_x, currentNode->getDistanceTraveled() + 1);
					openList->addElement(nextNode);

					// delete next node
					delete nextNode;
					nextNode = nullptr;
				}
			}
		}
		// delete current node
		delete currentNode;
		currentNode = nullptr;
	}
	// delete start, goal and open list
	delete start;
	start = nullptr;
	delete goal;
	goal = nullptr;
	delete openList;
	openList = nullptr;
}

NodeList* PathSolver::getNodesExplored(){
	// Deep copy
	return new NodeList(*(this->nodesExplored));
}

NodeList* PathSolver::getPath(Env env){
	int envRows = getEnvRows();
	int envCols = getEnvCols();
	
	// Make answer node list and then put the shortest nodes from G to S
	NodeList* answer = new NodeList();
	// Initialize answer node list with same size of env
	answer->initialization(envRows * envCols);
	// Last node of close list is G
	int lastIndex = (this->nodesExplored->getLength() - 1);
	// flag will turn to true when current node reaches at S and then break the loop
	bool flag = false;

	while(!flag){
		// Take one node from close list and make it current node
		Node* current_node = new Node(this->nodesExplored->getNode(lastIndex)->getRow(), this->nodesExplored->getNode(lastIndex)->getCol(), this->nodesExplored->getNode(lastIndex)->getDistanceTraveled());
		// Put current node in answer
		answer->addElement(current_node);
		// if current position's getDistanceTraveled is 0 which is S position, then make flag true and stop searching
		if(current_node->getDistanceTraveled() == 0){
			flag = true; 
		}
		else{
			// flag will turn to true when find the previous node
			bool findPrevNode = false; 

			for(int j = lastIndex -1; !findPrevNode && j > -1 ; j--){
				// Take one node from close list and make it previous node and compare with current node
				Node* PrevNode = new Node(this->nodesExplored->getNode(j)->getRow(), this->nodesExplored->getNode(j)->getCol(), this->nodesExplored->getNode(j)->getDistanceTraveled());
				// If any node's manhattan distance plus distance traveled is current node's distance traveled, that is the right previous node 
				if(PrevNode->getEstimatedDist2Goal(current_node) + PrevNode->getDistanceTraveled() == current_node->getDistanceTraveled()){
					lastIndex = j;
					findPrevNode = true;
				}
				// delete previous node
				delete PrevNode;
				PrevNode = nullptr;
			}
		}
	
		// delete current node
		delete current_node;
		current_node = nullptr;
	}
	// To prevent memory leak, delete original close list and put deep copy of answer node list in nodesExplored
	delete nodesExplored;
	nodesExplored = nullptr;
	nodesExplored = new NodeList(*answer);
	// delete answer
	delete answer;
	answer = nullptr;
	// return deep copy of nodesExplored which is shortest path here
	return new NodeList(*nodesExplored);
	
}

void PathSolver::setEnvRows(int envRows){
	this->envRows = envRows;
}

void PathSolver::setEnvCols(int envCols){
	this->envCols = envCols;
}

int PathSolver::getEnvRows(){
	return this->envRows;
}

int PathSolver::getEnvCols(){
	return this->envCols;
}
//-----------------------------