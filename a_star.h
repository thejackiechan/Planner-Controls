#ifndef A_STAR_H
#define A_STAR_H

#include<iostream>
#include<limits>
#include<cmath>
#include<vector>
#include<queue>
#include "mainwindow.h"


using namespace std;

class Node
{
    public:
       int x;
       int y;
       float cumCost;
       Node* prevNode;

       Node(int inX, int inY, float inCumCost, Node* inPrevNode = 0);
};

struct compareNodes
{
   bool operator()( const Node* node1, const Node* node2 ) const
   {
    return node1->cumCost > node2->cumCost;
   }
};

vector<Node> getMotionModel();

vector<vector<int>> buildObsMap(vector<int> &obsX, vector<int> &obsY,
                                const int minX, const int maxX,
                                const int minY, const int maxY,
                                float resolution, float vehicleSize,
                                QCustomPlot *customPlot);

float computeHeuristic(Node* node1, Node* node2);

bool isValidNode(Node* node, vector<vector<int>> &obsMap,
                 int minX, int maxX, int minY, int maxY);

void getFinalPath(Node* goalNode, vector<float> &xCoords, vector<float> &yCoords,
                  float resolution, QCustomPlot *customPlot, int &pathLength);

vector<int> getBoundsAndObs(int &minVal, int &maxVal, vector<float> &inObs, float resolution);

void runAStar(float startX, float startY, float goalX, float goalY,
              vector<float> &inObsX, vector<float> inObsY,
              float resolution, float vehicleSize, QCustomPlot *customPlot,
              int &pathLength);

void drawRect(int x, int y, QCustomPlot* customPlot, QColor color);

#endif // A_STAR_H
