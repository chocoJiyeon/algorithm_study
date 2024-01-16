#include "algorithm_study/Dijkstra.hpp"

Dijkstra::Dijkstra()
{
    Graph graph;
    int grid_size = 6, start_node = 0, end_node = 17;

    for(int i=0; i< grid_size; i++)
    {
        for(int j=0; j< grid_size; j++)
        {
            // 노드 만들기
            Node node;
            node.index = i*grid_size + j;
            node.point = std::make_pair(i,j);
            graph.vertex.push_back(node);

            // 엣지
            std::vector<int> now_node;
            for(int a = i-1; a <= i+1; a++)
            {
                if(a < 0 || a >= grid_size) continue;
                for(int b = j-1; b <= j+1; b++)
                {
                    if(b < 0 || b >= grid_size || (a==i && b==j)) continue;
                    now_node.push_back(a*grid_size + b);
                }
            }
            graph.edges.push_back(now_node);
        }
    }

    // [TEST] edge 확인
    // for(auto it : graph.edges)
    // {
    //     std::cout<<"edges : ";
    //     for(auto it2 : it) std::cout<< it2 <<" ";
    //     std::cout<<" + "<<std::endl;
    // }

    Dijkstra_algorithm(&graph, start_node, end_node);
    std::cout<< "목적지까지의 거리 : " << graph.vertex[end_node].dist << std::endl;
}//Dijkstra

Dijkstra::~Dijkstra()
{

}


void Dijkstra::Dijkstra_algorithm(Dijkstra::Graph* graph, int source, int destination)
{
    int graph_size = graph->vertex.size();

    // 우선순위 큐 : dist가 작은 순서대로
    std::priority_queue<Node> pq;

    graph->vertex[source].dist = 0;     // 시작점은 거리가 0 
    pq.push(graph->vertex[source]);     // 시작점 큐에 넣기

    while(!pq.empty())
    {
        Node u = pq.top();  // 제일 가까운 노드부터 계산
        pq.pop();
        graph->vertex[u.index].have_visited = true;     // 방문한 노드로 기록

        for(auto it : graph->edges[u.index])
        {
            Node v = graph->vertex[it];
            if(v.have_visited) continue;    // 이미 계산이 완료된 노드이면 넘어가기

            float d = u.dist + calculate_dist(u, v);
            if(d < v.dist)
            {
                // 최단 거리와 이전노드 값 graph에 업데이트
                graph->vertex[it].dist = d;
                graph->vertex[it].prev = u.index;
                pq.push(graph->vertex[it]);
            }
        }//for
    }//while
}//Dijkstra_algorithm

