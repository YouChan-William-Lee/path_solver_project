#include "Node.h"
#include <iostream>


Node::Node(int row, int col, int dist_traveled){
	this->row = row;
	this->col = col;
	this->dist_traveled = dist_traveled;
}

Node::~Node(){
}

int Node::getRow(){
	return this->row;
}

int Node::getCol(){
	return this->col;
}

int Node::getDistanceTraveled(){
	return this->dist_traveled;
}

void Node::setDistanceTraveled(int dist_traveled){
	this->dist_traveled = dist_traveled;
}

int Node::getEstimatedDist2Goal(Node* goal){
	return abs(goal->col - this->col) + abs(goal->row - this->row);
}
    
//--------------------------------         
