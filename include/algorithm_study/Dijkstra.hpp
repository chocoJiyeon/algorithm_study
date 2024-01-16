#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <math.h>
#include <queue>


class Dijkstra
{
private:
    
    struct Node
    {
        int index;
        std::pair<int,int> point;       //좌표값(x,y)

        bool have_visited = false;  // 해당 노드의 계산이 끝났는지 여부
        int prev = -1;              // 최단 거리의 이전 노드 인덱스 : -1로 초기화
        float dist = INFINITY;      // 해당 노드까지의 최단거리 : INF로 초기화

        bool operator< (Node A) const 
        {
            return this->dist > A.dist;     // dist가 작은게 우선 --> priority queue에서 사용
        }
    };

    struct Graph
    {  
        std::vector<Node> vertex;
        std::vector<std::vector<int>> edges;
    };

    float calculate_dist(Node u, Node v)     // 두 노드 사이의 거리
    {
        float res;
        res = sqrtf(std::pow((u.point.first - v.point.first),2) + std::pow((u.point.second - v.point.second),2));
        return res;
    }

public:
    Dijkstra();
    ~Dijkstra();

    void Dijkstra_algorithm(Dijkstra::Graph* graph, int source, int destination);
};