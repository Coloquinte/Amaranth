
#include "detailed/incremental_flow.hpp"

#include <cassert>
#include <queue>
#include <limits>

namespace{
    int const max_int = std::numeric_limits<int>::max();
}

struct node_elt{
    int cost;
    int incoming_edge;

    node_elt(int c, int i) : cost(c), incoming_edge(i) {}
};

struct queue_elt : node_elt{
    int destination_node;

    queue_elt(int d, int c, int i) : node_elt(c, i), destination_node(d) {}
};

void MCF_graph::add_edge(int esource, int edestination, int ecost){
    assert(esource != edestination and esource < node_count() and edestination < node_count());
    int sent_flow=0;

    bool maybe_cycle = bounded;
    while(maybe_cycle){
        // Perform Bellman-Ford algorithm
        // Find a path from the edge's *destination* to its *source* to make a cycle
        std::vector<node_elt> accessibles(node_count(), node_elt(max_int, -1));
        std::queue<queue_elt> queue;
        queue.emplace(edestination, 0, -1);
        while(not queue.empty()){
            queue_elt const cur = queue.front();
            int cdest = cur.destination_node;
            int ccost = cur.cost;
            if(ccost < accessibles[cdest].cost){
                accessibles[cdest] = cur;
                for(int e : adjacent_edges[cdest]){ // Now push every edge to the queue
                    assert(edges[e].source == cdest or edges[e].dest == cdest);
                    if( (edges[e].source == cdest or edges[e].flow > 0) // There is an edge in the residual graph
                       and accessibles[edges[e].dest].cost == max_int // and the destination wasn't already visited
                       ){
                        int edge_source = cdest;
                        int edge_destination = edges[e].source ^ edges[e].dest ^ edge_source;
                        int dest_cost = ccost + (edges[e].source == cdest ? edges[e].cost : -edges[e].cost);
                        queue.emplace(dest_cost, edge_destination, e);
                    }
                }
            }
        }
        // If the other node was reached with negative non-reduced cost, send flow along this cycle
        int cycle_cost = accessibles[esource].cost;
        if(cycle_cost < max_int){ // Reachable
            cycle_cost += ecost;
            if(cycle_cost < 0){
                // Find the maximum flow along the cycle
                int max_flow = max_int;
                std::vector<int> pos_edges, neg_edges; // Edges where the flow is sent in the same direction or not
                int cur_node=esource;
                while(cur_node != edestination){
                    int e = accessibles[cur_node].incoming_edge;
                    assert(e >= 0);
                    if(cur_node == edges[e].source){
                        cur_node = edges[e].dest;
                        neg_edges.push_back(e);
                        max_flow = std::min(max_flow, edges[e].flow);
                    }else{
                        assert(cur_node == edges[e].dest);
                        cur_node = edges[e].source;
                        pos_edges.push_back(e);
                    }
                }
                // Send it
                if(max_flow >= max_int){
                    bounded = false;
                    break;
                }
                cost -= (max_flow * cycle_cost);
                sent_flow += max_flow;
                for(int e : pos_edges)
                    edges[e].flow += max_flow;
                for(int e : neg_edges)
                    edges[e].flow -= max_flow;
            }
        }else{ // Ok, no more cycle, optimal solution, we can just exit
            maybe_cycle = false;
        }
    }
 
    int ind = edges.size();
    edges.push_back(edge(esource, edestination, ecost, sent_flow));
    adjacent_edges[esource     ].push_back(ind);
    adjacent_edges[edestination].push_back(ind);
}

MCF_graph::MCF_graph(int node_cnt, std::vector<MCF_graph::edge> edge_list) : edges(edge_list), adjacent_edges(node_cnt), bounded(true){
    for(int e=0; e<edges.size(); ++e){
        edge cur = edges[e];
        adjacent_edges[cur.source].push_back(e);
        adjacent_edges[cur.dest  ].push_back(e);
    }
}


