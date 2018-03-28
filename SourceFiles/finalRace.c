#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <time.h>
#include <math.h>

#include "abdrive.h"
#include "simpletext.h"
#include "simpletools.h"
#include "ping.h"
#include "helpFuncs2.h"

#ifdef BUILDING_IN_SIMULATOR
#include "simulator.h"
#endif

#define V 16

// TODO: Make drive_speed functional instead of using drive_goto. (DO IT IN ANOTHER FILE DONT SCREW THIS ONE UP)

// global constants
const int SIDEWALLDETECTDIST = 22; // used to be 20 alongside dacVal of ++7
const int FRONTWALLDETECTDIST = 26;

// global variables
int numOfLTurns = 1;
int numOfRTurns = 1;
int adjMatrix[16][16];
int currDir = 0;
int currNode;
int xPos = 0;
int yPos = -1;

int DO = 22, CLK = 23, DI = 24, CS = 25;      // SD card pins on Propeller BOE

void printAdjMat(){ // used for debugging
  for(int i = 0; i < 16; i++){
    print("|");
    for(int j = 0; j < 16; j++){
      int val = adjMatrix[i][j];
      printf("%d ", val);
    }
    print("|\n");
  }
}

int findNodes(int tempDir){
  int tempY; // uses temp values to not change the actual values of x and y
  int tempX;
  int connNode; // possible node to be connected with in the adjacency matrix
  switch(tempDir){
    case 0:
      tempY = yPos + 1;
      tempX = xPos;
      break;
    case 1:
      tempX = xPos + 1;
      tempY = yPos;
      break;
    case 2:
      tempY = yPos - 1;
      tempX = xPos;
      break;
    default:
      tempX = xPos - 1;
      tempY = yPos;
  }
  connNode = tempX + 4 * tempY;
  return connNode;
}

void analyzeAndStoreData(int irLeft, int irRight, int uvFront)
{
  int tempDir;
  int connectedNode;
  if(yPos != -1)
  {
    print("reaches here\n");
    printf("irLeft: %d   irRight: %d    uvFront: %d\n", irLeft, irRight, uvFront);
    if(irLeft >= SIDEWALLDETECTDIST){ // if no wall exists on the left (hence can connect to left node) then store in matrix
      tempDir = (currDir - 1 + 4) % 4;
      printf("temp direction (left): %d ", tempDir);
      connectedNode = findNodes(tempDir); // calculates the node value of the node on the left
      printf("connected node: %d ", connectedNode);
      adjMatrix[currNode][connectedNode] = 1; // makes a connection between nodes (represented by matix index with a edge value of 1 since the distance between two points is always 1 in this case)
      adjMatrix[connectedNode][currNode] = 1;
    }
    if(irRight >= SIDEWALLDETECTDIST){ // same process done for each direction
      tempDir = (currDir + 1 + 4) % 4;
      printf("temp direction (right): %d ", tempDir);
      connectedNode = findNodes(tempDir);
      printf("connected node: %d ", connectedNode);
      adjMatrix[currNode][connectedNode] = 1;
      adjMatrix[connectedNode][currNode] = 1;
    }
    if(uvFront >= FRONTWALLDETECTDIST){
      printf("uvFront: %d ", uvFront);
      printf("temp direction (front): %d ", tempDir);
      connectedNode = findNodes(currDir);
      printf("connected node: %d \n", connectedNode);
      adjMatrix[currNode][connectedNode] = 1;
      adjMatrix[connectedNode][currNode] = 1;
    }
    print("\n");
  }
}

void adjustAngleError(){
  /* This function makes up for the errors made by the rotation of the bot since the values are
  truncated. So after a consecutive use of the same rotation it adjusts the angle further */

  if(numOfLTurns % 3 == 0){ // change to % 3
    drive_goto(0, 1); // change to drive_goto(0, 1)
    print("ADJUSTED LEFT\n");
  }
  else{
    if(numOfRTurns % 3 == 0){ // change to % 3
      drive_goto(1, 0); // change to drive_goto(1, 0)
      print("ADJUSTED RIGHT\n");
    }
  }
}

void locate(){
  /* locates the robots position for each movement ready to store info in mapData */

  if(currDir == 0)
    yPos++;
  else if(currDir == 1)
    xPos++;
  else if(currDir == 2)
    yPos--;
  else
    xPos--;
}

void wallFollower(int irLeft, int irRight, int movingBack){
  /* Implements left wall follower algorithm */

  if(irLeft < SIDEWALLDETECTDIST && ping_cm(8) < FRONTWALLDETECTDIST && irRight < SIDEWALLDETECTDIST){
      // if left, right and front wall exists
      movingBack = 1;
      // turnRight();
      // turnRight();
      halfTurnC();
      numOfLTurns = 1; // resets values (checking for consecutive turns)
      numOfRTurns += 2;
      currDir = (currDir + 2 + 4) % 4;
      //adjustAngleError();
    }
    else{ // else if atleast one side hasn't got a wall
      if(irLeft >= SIDEWALLDETECTDIST){ // if left side free...
        turnLeft();
        numOfLTurns++;
        numOfRTurns = 1;
        //adjustAngleError();
        currDir = (currDir - 1 + 4) % 4; // +4 is to handle negative numbers
      }
      else{
        if(ping_cm(8) >= FRONTWALLDETECTDIST){ // if front free...
          
          // empty because doesn't have to turn when wall in front is free
        }
        else{
          //print("There is a wall in front. dist = %d\n", ping_cm(8));
          if(irRight >= SIDEWALLDETECTDIST){ // if right free...
            turnRight();
            numOfLTurns = 1;
            numOfRTurns++;
//            adjustAngleError();
            currDir = (currDir + 1 + 4) % 4;
          }
        }
      }
    }

    if(ping_cm(8) <= 5){
        drive_goto(-10, -10);
        print("ITS WORKING!\n");
    }

    if(!movingBack){ // it should only move foward if it isn't turning away from a dead end
      drive_goto(126, 126);
      locate();
      currNode = xPos + 4 * yPos;
    }

    movingBack = 0;
    pause(50);
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], int sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
  
   for (int v = 0; v < V; v++)
     if (sptSet[v] == 0 && dist[v] <= min)
         min = dist[v], min_index = v;
  
   return min_index;
}

void findPath(int path[], int parent[]){
  for(int i = 0; i < V; i++)
    path[i] = 999;

  int node = 15;
  int count = 1;
  path[0] = 15;

  while(node != 0){ // finds path by looking at the parent node of each node starting from node 15 all the way to node 0
    path[count] = parent[node];
    count++;
    node = parent[node];
  }
}
  
// A utility function to print the constructed distance array
void printSolution(int dist[], int n)
{
  //  printf("Vertex   Distance from Source\n");
  //  for (int i = 0; i < V; i++)
  //     printf("%d tt %d\n", i, dist[i]);
}

// A utility function to print the constructed parent array
void printParent(int parent[]){
  // print("[");
  // for(int i = 0; i < V; i++)
  //   print("%d, ", parent[i]);
  // print("]\n");
}

void printPath(int path[]){
  // print("[");
  // for(int i = 15; i >= 0; i--)
  //   if(path[i] != 999)
  //     print("%d, ", path[i]);
  // print("]");
}

// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(int graph[V][V], int src, int path[])
{
     //int path[V];
     int parent[V];   // the array that stores which nodes are connected to each other in the shortest path set
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
  
     int sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
  
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = 0;
  
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
     parent[src] = -1;
  
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);
  
       // Mark the picked vertex as processed
       sptSet[u] = 1;
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
  
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX 
                                       && dist[u]+graph[u][v] < dist[v]){
            dist[v] = dist[u] + graph[u][v];
            parent[v] = u;
         }   
     }
     
     // print the constructed distance array
    //  printSolution(dist, V);
    //  printParent(parent);
     findPath(path, parent); // finds a reversed version of the path due to nature of the way it finds the path
}

void controlTurning(int dir, int justTurned){ // justTurned
    if(dir == 1){ // Left
        if(justTurned == 1){
            left_consec();
        }
        else{
            d_left();
        }
    }
    else{         // Right
        if(justTurned == 2){
            right_consec();
        }
        else{
            d_right();
        }
    }
}

int driveRelative(int relDir, int dirToGo, int speed, int value)
{ // justTurned
  /* Controls the direction of the robot as it follows the shortest path */
  int justTurned;  
  if(dirToGo == 0){
    if(relDir == 0){ // FORWARD MOTION
      normal_forward();
      justTurned = 0;
    }
    else if(relDir == 1){ 
        controlTurning(1,value); // 1 is left direction
        forward_after_turn();
        justTurned = 1; // 1 means it just turned left, 2 means just turned right
    }
    // it will never be the case the relDir will be 2 for dirGoTo = 0 because the bot would never go up and then go back down for shortest path
    else{ 
      controlTurning(0,value); // 0 is right direction
      forward_after_turn();
      justTurned = 2;
    }
  }
  else if(dirToGo == 1){
    if(relDir == 0){
      controlTurning(0,value);
      forward_after_turn();
      justTurned = 2;
    }
    else if(relDir == 1){ // FORWARD MOTION
      normal_forward();
      justTurned = 0;
    }
    else{
      controlTurning(1,value);
      forward_after_turn();
      justTurned = 1;
    }
  }
  else if(dirToGo == 2){
    if(relDir == 1){
      controlTurning(0,value);
      forward_after_turn();
      justTurned = 2;
    }
    else if(relDir == 2){ // FORWARD MOTION
      normal_forward();
      justTurned = 0;
    }
    else{
      controlTurning(1,value);
      forward_after_turn();
      justTurned = 1;
    }
  }
  else{
    if(relDir == 0){
      controlTurning(1,value);
      forward_after_turn();
      justTurned = 1;
    }
    else if(relDir == 2){
      controlTurning(0,value);
      forward_after_turn();
      justTurned = 2;
    }
    else{                // FORWARD MOTION
      normal_forward();
      justTurned = 0;
    }
  }

//   printf(" \nthe value ot T is : %d\n",justTurned);
  return justTurned;
}

void goToFinish(){
  //int justTurned = 0; // set to false initially (used for the speed control later on)
  int speed = 100;
  int relativeDirection = 0; // where 0 is facing up, 1 right, 2 down and 3 left
  int directionToGo; // The direction the robot is trying to go to in that moment in time.
  int revPath[V]; // reversed shortest path route
  dijkstra(adjMatrix, 0, revPath);
  printPath(revPath);

  int prevNode = -4;
  int value = 0;
  int new_value = 0;
  for(int i = 15; i >= 0; i--)
  {
    print("loop no: %d\n", i);
    if(revPath[i] != 999){
      print("-----------------------\n");
      print("previous node: %d    node to go: %d\n", prevNode, revPath[i]);
      if(revPath[i] == 0){
          directionToGo = 0;
          starting(); // always has a starting drive
          relativeDirection = 0;
          // justTurned = 0;
      }
      else{
        if(revPath[i] - prevNode == 4){
            directionToGo = 0;
            new_value = driveRelative(relativeDirection, directionToGo, speed,value);
            relativeDirection = 0;
        }
        else if(revPath[i] - prevNode == 1){
            directionToGo = 1;
            new_value = driveRelative(relativeDirection, directionToGo, speed,value);
            relativeDirection = 1;
        }
        else if(revPath[i] - prevNode == -1){
            directionToGo = 3;
            new_value = driveRelative(relativeDirection, directionToGo, speed,value);
            relativeDirection = 3;
        }
        else if(revPath[i] - prevNode == -4){
            directionToGo = 2;
            new_value = driveRelative(relativeDirection, directionToGo, speed,value);
            relativeDirection = 2;
        }
      }
      prevNode = revPath[i];
    }
    value = new_value;
  }
}

void run(){
  int flagOn = 1; // flag variables
  int movingBack = 0;
  sd_mount(DO, CLK, DI, CS);                  // Mount SD card

  FILE* fp = fopen("test.txt", "w");          // Open a file for writing

  while(flagOn){
    // Read the left and right sensors
    int irLeft = 0;
    for(int dacVal = 0; dacVal <= 160; dacVal += 7)  // <- add // 8
      {                                               // <- add
        dac_ctr(26, 0, dacVal);                       // <- add
        freqout(11, 1, 38000);                        // <- add
        irLeft += input(10);
      }

    int irRight = 0;
    for(int dacVal = 0; dacVal <= 160; dacVal += 7)  // <- add // 8
      {                                               // <- add
        dac_ctr(27, 1, dacVal);                       // <- add
        freqout(1, 1, 38000);                        // <- add
        irRight += input(2);
      }
    fflush(fp);

    printf("current direction: %d    x: %d       y: %d     node: %d\n", currDir, xPos, yPos, currNode);
    int front = ping_cm(8);
    analyzeAndStoreData(irLeft, irRight, front);
    printAdjMat();
    wallFollower(irLeft, irRight, movingBack);

    if(xPos == 0 && yPos == -1){
      print("Back to start \n");
      flagOn = 0;
    }
  }
  // turnRight(); // rotate 180 degrees to prepare for phase 2
  // turnRight(); 
  halfTurn(); 
  pause(1000); // wait a few seconds for TA to prepare timing
  goToFinish(); // starts phase 2
}


int main(){
    #ifdef BUILDING_IN_SIMULATOR // activates drawing mode
    simulator_startNewSmokeTrail();
    #endif

    for(int i = 0; i < 16; i++){ // assigns placeholder values for each edge connection
      for(int j = 0; j < 16; j++){
        adjMatrix[i][j] = 0;
      }
    }

    drive_goto(20, 20); // 20 ticks is 3 tiny boxes (fix bot at the beginning position)
    run();
    
    //drive_goto(126, 126);
    return 0;
}
