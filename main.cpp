/* 
 *Basically, I've followed all the instructions written in assignment 1 pdf file step by step.
 *Firstly, I've made test files which show the order of priority which direction the robot should go first.
 *Secondly, I've implemented Forward Search with visit 2D array to check whether the position is explored or not with the algorithm wirtten in assignment 1 pdf file. 
 *I've put all the reachable nodes from S to G into open list and compare one by one using Manhattan distance, and then I've put all explored nodes into close list.
 *Thirdly, I've implemented Backtracking algorithm using close list. I've taken a node from end of close list and compare one by one using Manhattan distance. 
 *Lastly, I've implemented milestone4 which is how to deal with double pointer without row and col size. 
 *I've figured out the size of env which is row and col size and then initialized env and all node lists.
 *I've encounterd many memory leaks when I implemented PathSolver but ended up deleting all the created heaps. 
 *I've also found mismatched free() / delete / delete [] error as milestone4.h has 'delete' instead of 'delete[]'.
 */

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>

#include "Types.h"
#include "Node.h"
#include "NodeList.h"
#include "PathSolver.h"
#include "milestone4.h"

// Helper test functions
// void testNode();
// void testNodeList();

// Read a environment from standard input.
void readEnvStdin(Env* env, int* envRows, int* envCols);

// Print out a Environment to standard output with path.
// To be implemented for Milestone 3
void printEnvStdout(Env* env, NodeList* solution, int row, int col);


int main(int argc, char** argv){
    // THESE ARE SOME EXAMPLE FUNCTIONS TO HELP TEST YOUR CODE
    // AS YOU WORK ON MILESTONE 2. YOU CAN UPDATE THEM YOURSELF
    // AS YOU GO ALONG.
    // COMMENT THESE OUT BEFORE YOU SUBMIT!!!
    // std::cout << "TESTING - COMMENT THE OUT TESTING BEFORE YOU SUBMIT!!!" << std::endl;
    // testNode();
    // testNodeList();
    // std::cout << "DONE TESTING" << std::endl << std::endl;

    // Load Environment 
    Env env;
	env = nullptr;
	int envRows = 0;
	int envCols = 0;
	// Call by reference to save changed env, env's row and col size
    readEnvStdin(&env , &envRows, &envCols);
    
    // Solve using forwardSearch
    // THIS WILL ONLY WORK IF YOU'VE FINISHED MILESTONE 2
    PathSolver* pathSolver = new PathSolver();
	// Set env's row size
	pathSolver->setEnvRows(envRows);
	// Set env's col size
	pathSolver->setEnvCols(envCols);
    pathSolver->forwardSearch(env);

    NodeList* exploredPositions = nullptr;
    exploredPositions = pathSolver->getNodesExplored();

    // Get the path
    // THIS WILL ONLY WORK IF YOU'VE FINISHED MILESTONE 3
    NodeList* solution = pathSolver->getPath(env);
	// Call by reference to save changed env, and pass solution node, env's row and col size
    printEnvStdout(&env, solution, envRows, envCols);

	// Delete pathSolver, exploredPositions and solution
    delete pathSolver;
	pathSolver = nullptr;
    delete exploredPositions;
	exploredPositions = nullptr;
    delete solution;
	solution = nullptr;
}

void readEnvStdin(Env* env, int* envRows, int* envCols){
	// row and col variables for finding env's size
	int row = 0, col = 0;
	// Take a character from input file to c
    char c;
	// Put all characters into one string
	std::string mazeInOneLine ="";
	// When it reaches end of file, while loop stops
    bool stop = false;

	// Read test env file by one character and find number of row and col 
    while(!stop){
		// Keep reading until input is a new line character
        while(std::cin.get(c) && c != '\n'){
            col += 1;
			mazeInOneLine += c;
        }
		row += 1;
		// Stop when it reaches the end of test env file
        if(std::cin.eof()){
            stop = true;
        }
        else{
            col = 0;
        }
    }
	// Make env by dynamically allocating memory 
	*env = make_env(row, col);

	// Input a chracter one by one from mazeInOneLine to env
	int index = 0;
	for(int i = 0; i < row; ++i){
		for(int j = 0; j < col; ++j){
			(*env)[i][j] = mazeInOneLine[index];
			index += 1;
		}
	}
	// Pass env's row and col size to envRows and envCols
	*envRows = row;
	*envCols = col;
}

void printEnvStdout(Env* env, NodeList* solution, int envRows, int envCols){

	// Position of current node's UP, DOWN, LEFT, RIGHT by x and y position
	int position_y[4] = {-1,1,0,0}, position_x[4] = {0,0,-1,1};
	// Take last index of solution node list 
	int lastIndex = solution->getLength() - 1;

	for(int i = lastIndex - 1; i > 0; --i){
		// Take one node which is the last element of solution node list 
		Node* current_node = new Node(solution->getNode(i)->getRow(), solution->getNode(i)->getCol(), solution->getNode(i)->getDistanceTraveled());
		
		// Take one node which is just one before the last element of solution node list
		Node* next_node = new Node(solution->getNode(i - 1)->getRow(), solution->getNode(i - 1)->getCol(), solution->getNode(i - 1)->getDistanceTraveled());

		// Take current node and next node's Row and Col
		int cur_y = current_node->getRow();
		int cur_x = current_node->getCol();
		int nxt_y = next_node->getRow();
		int nxt_x = next_node->getCol();

		// Compare the x, y position of current node and next node, then change env with right symbol
		for(int j = 0; j < 4; j++){
			if((cur_y + position_y[j] == nxt_y) && (cur_x + position_x[j] == nxt_x)){
				if(j == 0){
					// y position -1, x position 0
					(*env)[cur_y][cur_x] = MOVE_UP;		
				}
				else if(j == 1){
					// y position +1, x position 0
					(*env)[cur_y][cur_x] = MOVE_DOWN;
				}
				else if(j == 2){
					// y position 0, x position -1
					(*env)[cur_y][cur_x] = MOVE_LEFT;
				}
				else if(j == 3){
					// y position 0, x position +1
					(*env)[cur_y][cur_x] = MOVE_RIGHT;
				}
			}
		}

		// Delete current node and next node
		delete current_node;
		current_node = nullptr;
		delete next_node;
		next_node = nullptr;
	}
	
	// Print the result of backtracking

	for(int i = 0; i < envRows; ++i){
		for(int j = 0; j < envCols; ++j){
			std::cout << (*env)[i][j];
		}
		std::cout << std::endl;
	}

	// Delete env
	delete_env(*env, envRows, envCols);
}

// void testNode() {
//     std::cout << "TESTING Node" << std::endl;

//     // Make a Node and print out the contents
//     Node* node = new Node(1, 1, 2);
//     std::cout << node->getRow() << ",";
//     std::cout << node->getCol() << ",";
//     std::cout << node->getDistanceTraveled() << std::endl;
//     delete node;

//     // Change Node and print again
//     node = new Node(4, 2, 3);
//     std::cout << node->getRow() << ",";
//     std::cout << node->getCol() << ",";
//     std::cout << node->getDistanceTraveled() << std::endl;
//     delete node;
// }

// void testNodeList() {
//     std::cout << "TESTING NodeList" << std::endl;

//     // Make a simple NodeList, should be empty size
//     NodeList* nodeList = new NodeList();
//     std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

//     // Add a Node to the NodeList, print size
//     Node* b1 = new Node(1, 1, 1);
//     nodeList->addElement(b1);
//     std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

//     // Add second Nodetest
//     Node* b2 = new Node(0, 0, 1);
//     nodeList->addElement(b2);
//     std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

//     // Test Get-ith - should be 0,0,1
//     Node* getB = nodeList->getNode(1);
//     std::cout << getB->getRow() << ",";
//     std::cout << getB->getCol() << ",";
//     std::cout << getB->getDistanceTraveled() << std::endl;

//     // Print out the NodeList
//     std::cout << "PRINTING OUT A NODELIST IS AN EXERCISE FOR YOU TO DO" << std::endl;
// }