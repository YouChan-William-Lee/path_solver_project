#include "NodeList.h"
#include <iostream>

NodeList::NodeList(){
	this->nodes = nullptr;
	this->length = 0;
	this->envSize = 0;
}

NodeList::~NodeList(){
	for (int i = 0; i < this->getLength(); ++i){
		delete this->nodes[i];
		this->nodes[i] = nullptr;
	}
	delete[] this->nodes;
	this->nodes = nullptr;
}

NodeList::NodeList(NodeList& other){
	// Itinialization with other node lists's env size
	this->initialization(other.getEnvSize());
	this->length = 0;

	// Deep copy of other node list' nodes
	for (int i = 0; i < other.length; ++i){	
		this->addElement(other.getNode(i));
	}
}

int NodeList::getLength(){
	return this->length;
}

void NodeList::addElement(Node* newPos){
	this->nodes[this->length] = new Node(newPos->getRow(), newPos->getCol(), newPos->getDistanceTraveled());
	this->length += 1;
}

Node* NodeList::getNode(int i){
	return this->nodes[i];
}

void NodeList::initialization(int size){
	this->nodes = new Node*[size];
	this->envSize = size;
}

int NodeList::getEnvSize(){
	return this->envSize;
}