#include "algorithm_study/Dijkstra.hpp"

Dijkstra::Dijkstra()
{
    Graph graph;
    int grid_size = 20, start_node = 22, end_node = 213;

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

    // 랜덤 장애물 생성
    std::vector<int> obstacle_node;
    std::srand((unsigned int)time(NULL));
    for(int num_of_ob = 100; num_of_ob > 0; num_of_ob--)
    {
        std::cout<<"obstacle : ";
        int ob = rand() % (grid_size*grid_size);
        if(ob != start_node && ob !=end_node) 
        {
            obstacle_node.push_back(ob);
            std::cout<<ob<<" ";
        }
        std::cout<<"\n";
    }
    for(auto it : obstacle_node)
    {
        graph.vertex[it].have_visited = true;   // 장애물 있으면 안가도록 함
    }

    Dijkstra_algorithm(&graph, start_node, end_node);
    std::cout<< "목적지까지의 거리 : " << graph.vertex[end_node].dist << std::endl;

    draw_image(&graph, obstacle_node, start_node, end_node, grid_size);
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

        if(graph->vertex[destination].have_visited == true) return;  //목적지까지 계산이 완료되면 종료
    }//while
}//Dijkstra_algorithm

void Dijkstra::draw_image(Dijkstra::Graph* graph, std::vector<int> obstacle_node, int source, int destination, int grid_size)
{
    int image_size = 1000, center_of_image = image_size/2;
    int side_length = (image_size - 100)/(grid_size-1);
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // 노드 표시
    for(auto it : graph->vertex)
    {
        auto color = CV_RGB(100, 100, 100);
        int thick = 1;
        if(it.index == source) {color = CV_RGB(255, 50, 50); thick = 3;}
        else if(it.index == destination) {color = CV_RGB(50, 50, 255); thick = 3;}
        
        cv::circle(image, cvPoint(50 + it.point.second*side_length, 50 + it.point.first*side_length), 4, color, thick);
    }
    
    // 장애물 표시
    for(auto it : obstacle_node)   
    {
        auto p1 = cvPoint(50 - side_length/2 + graph->vertex[it].point.second * side_length, 50 - side_length/2 + graph->vertex[it].point.first * side_length);
        auto p2 = cvPoint(50 + side_length/2 + graph->vertex[it].point.second * side_length, 50 + side_length/2 + graph->vertex[it].point.first * side_length);
        cv::rectangle(image, cv::Rect(p1, p2), cv::Scalar(0, 0, 0), cv::FILLED, 1);

    }
    
    bool is_start_node = false;
    Node now = graph->vertex[destination], pre = graph->vertex[now.prev];
    // 경로 표시
    while(now.prev != -1)
    {
        auto p1 = cvPoint(50 + now.point.second*side_length, 50 + now.point.first*side_length);
        auto p2 = cvPoint(50 + pre.point.second*side_length, 50 + pre.point.first*side_length);
        cv::line(image, p1, p2, CV_RGB(50, 255, 0), 3, cv::LINE_AA);
        now = graph->vertex[now.prev];
        pre = graph->vertex[now.prev];
    }

    cv::imshow("Dijkstra", image);
    cv::waitKey(0);
}