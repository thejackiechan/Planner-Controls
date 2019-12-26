#include<iostream>
#include<limits>
#include<cmath>
#include<vector>
#include<queue>
#include "a_star.h"

using namespace std;

Node::Node(int inX, int inY, float inCumCost, Node* inPrevNode)
{
    x = inX;
    y = inY;
    cumCost = inCumCost;
    prevNode = inPrevNode;
}

vector<Node> getMotionModel()
{
    return { Node(1, 0, 1),
             Node(0, 1, 1),
             Node(-1, 0, 1),
             Node(0, -1, 1),
             Node(-1, -1, sqrt(2)),
             Node(-1, 1, sqrt(2)),
             Node(1, -1, sqrt(2)),
             Node(1, 1, sqrt(2)) };
}

vector<vector<int>> buildObsMap(vector<int> &obsX, vector<int> &obsY,
                                const int minX, const int maxX,
                                const int minY, const int maxY,
                                float resolution, float vehicleSize,
                                QCustomPlot *customPlot)
{
    int width = maxX - minX;
    int height = maxY - minY;

    vector<vector<int>> obsMap(height, vector<int> (width, 0));

    for(int i = 0; i < width; i++)
    {
        int x = minX + i;
        for(int j = 0; j < height; j++)
        {
            int y = minY + j;
            for(size_t k = 0; k < obsX.size(); k++)
            {
                float dist = sqrt(pow((obsX[k] - x), 2) + pow((obsY[k] - y), 2));
                if(dist <= vehicleSize / resolution)
                {
                    // there is an obstacle here
                    obsMap[i][j] = 1;
                    drawRect(i, j, customPlot, QColor(Qt::black));
                    break;
                }
            }
        }
    }
    return obsMap;
}

// Returns the Euclidean distance between two nodes
float computeHeuristic(Node* node1, Node* node2)
{
    return sqrt(pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2));
}

bool isValidNode(Node* node, vector<vector<int>> &obsMap,
                 int minX, int maxX, int minY, int maxY)
{
    // check bounds
    if(node->x < minX || node->x > maxX || node->y < minY || node->y > maxY)
    {
        return false;
    }
    // check for obstacles
    else if(obsMap[node->x - minX][node->y - minY])
    {
        return false;
    }
    else
    {
        return true;
    }
}

void getFinalPath(Node* goalNode, vector<float> &xCoords, vector<float> &yCoords,
                  float resolution, QCustomPlot *customPlot, int &pathLength)
{
    Node* node = goalNode;

    while(node->prevNode != 0)
    {
        node = node->prevNode;
        xCoords.push_back(resolution * node->x);
        yCoords.push_back(resolution * node->y);
        drawRect(node->x, node->y, customPlot, QColor(Qt::blue));
        pathLength++;
    }
}

vector<int> getBoundsAndObs(int &minVal, int &maxVal, vector<float> &inObs, float resolution)
{
    vector<int> obs;

    for(float i:inObs)
    {
        int val = (int) round(i / resolution);
        obs.push_back(val);

        minVal = min(val, minVal);
        maxVal = max(val, maxVal);
    }
    return obs;
}

void runAStar(float startX, float startY, float goalX, float goalY,
              vector<float> &inObsX, vector<float> inObsY,
              float resolution, float vehicleSize, QCustomPlot *customPlot,
              int &pathLength)
{
    // Define start and goal nodes (integers)
    Node* startNode = new Node((int) round(startX / resolution),
                               (int) round(startY / resolution), 0.0);

    Node* goalNode = new Node((int) round(goalX / resolution),
                              (int) round(goalY / resolution), 0.0);

    // Initialize bounds
    int minX = numeric_limits<int>::max();
    int maxX = numeric_limits<int>::min();
    int minY = numeric_limits<int>::max();
    int maxY = numeric_limits<int>::min();

    // Obtain new obstacles (integers) and bounds
    vector<int> obsX = getBoundsAndObs(minX, maxX, inObsX, resolution);
    vector<int> obsY = getBoundsAndObs(minY, maxY, inObsY, resolution);

    int width = maxX - minX;
    int height = maxY - minY;

    vector<vector<int>> visitedMap(height, vector<int>(width, 0)); // (1 = visited, 0 else)
    vector<vector<float>> costMap(height, vector<float>(width, numeric_limits<float>::max()));
    vector<vector<int>> obsMap = buildObsMap(obsX, obsY, minX, maxX, minY, maxY,
                                             resolution, vehicleSize, customPlot);
    vector<Node> motions = getMotionModel();

    costMap[startNode->x][startNode->y] = 0;

    // Initialize A*
    priority_queue<Node*, vector<Node*>, compareNodes> pq;

    pq.push(startNode);

    while(true)
    {
        Node* node = pq.top();
        pq.pop();

        // check if node has been visited
        if(visitedMap[node->x - minX][node->y - minY]) // returns bool
        {
            delete node;
            continue;
        }
        else
        {
            visitedMap[node->x - minX][node->y - minY] = 1;
        }

        // check if reached goal
        if(node->x == goalNode->x && node->y == goalNode->y)
        {
            goalNode->cumCost = node->cumCost;
            goalNode->prevNode = node;
            break;
        }

        // explore neighboring nodes
        for(size_t i = 0; i < motions.size(); i++)
        {
            Node* newNode = new Node(node->x + motions[i].x, node->y + motions[i].y,
                                     costMap[node->x][node->y] + motions[i].cumCost +
                                     computeHeuristic(node, goalNode), node);

            if(!isValidNode(newNode, obsMap, minX, maxX, minY, maxY))
            {
                delete newNode;
                continue;
            }

            if(visitedMap[newNode->x - minX][newNode->y - minY])
            {
                delete newNode;
                continue;
            }

            drawRect(newNode->x, newNode->y, customPlot, QColor(Qt::yellow));

            // update costMap
            if(costMap[node->x][node->y] + motions[i].cumCost < costMap[newNode->x][newNode->y])
            {
                costMap[newNode->x][newNode->y] = costMap[node->x][node->y] + motions[i].cumCost;
                pq.push(newNode); // add only if the cost decreases
            }
        }
    }

    // get final path
    vector<float> xCoords;
    vector<float> yCoords;
    getFinalPath(goalNode, xCoords, yCoords, resolution, customPlot, pathLength);

    // Visualize start and goal
    drawRect(startNode->x, startNode->y, customPlot, QColor(Qt::red));
    drawRect(goalNode->x, goalNode->y, customPlot, QColor(Qt::green));
    delete startNode;
    delete goalNode;
}

void drawRect(int x, int y, QCustomPlot* customPlot, QColor color)
{
    QCPItemRect* rect = new QCPItemRect(customPlot);
    rect->setVisible(true);
    rect->topLeft->setCoords(x, y);
    rect->bottomRight->setCoords(x + 1, y + 1);
    rect->setBrush(QBrush(color));
}
