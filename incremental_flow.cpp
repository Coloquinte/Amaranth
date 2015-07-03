
#include "detailed/incremental_flow.hpp"

#include <cassert>
#include <queue>
#include <limits>
#include <iostream>

namespace{
    int const max_int = std::numeric_limits<int>::max();
}

std::vector<MCF_graph::node_elt> MCF_graph::get_Bellman_Ford(int source_node) const{
    std::vector<node_elt> accessibles(node_count(), node_elt(max_int, -1));
    accessibles[source_node] = node_elt(0, -1);
    for(int i=0; i<=node_count(); ++i){
        bool found_relaxation = false;
        for(int e=0; e<edges.size(); ++e){
            edge const E = edges[e];
            if(accessibles[E.source].cost < max_int){
                int forward_cost = accessibles[E.source].cost + E.cost;
                if(forward_cost < accessibles[E.dest].cost){
                    accessibles[E.dest] = node_elt(forward_cost, e);
                    found_relaxation = true;
                }
            }
            if(E.flow > 0 and accessibles[E.dest].cost < max_int){
                int backward_cost = accessibles[E.dest].cost - E.cost;
                if(backward_cost < accessibles[E.source].cost){
                    accessibles[E.source] = node_elt(backward_cost, e);
                    found_relaxation = true;
                }
            }
        }
        if(not found_relaxation) break;
        if(found_relaxation and i == node_count()){
            accessibles[source_node] = node_elt(-1, -1);
            // std::cout << "Negative cost cycle in Bellman-Ford!!!" << std::endl;
        }
    }
    return accessibles;
}

std::vector<int> MCF_graph::get_potentials() const{
    if(node_count() == 0) return std::vector<int>();
    std::vector<node_elt> accessibles = get_Bellman_Ford(0);
    std::vector<int> ret;
    for(node_elt const N : accessibles) ret.push_back(N.cost);
    return ret;
}

void MCF_graph::add_edge(int esource, int edestination, int ecost){
    assert(esource != edestination and esource < node_count() and edestination < node_count() and esource >= 0 and edestination >= 0);
    int sent_flow=0;

    //std::cout << "Creating an edge (" << esource << ", " << edestination << "), cost " << ecost << std::endl;
    for(auto it = edges.begin(); it != edges.end(); ){
        if(it->source == esource and it->dest == edestination){
            if(it->cost > ecost){
                sent_flow += it->flow;
                cost -= it->flow * (it->cost - ecost);
                it = edges.erase(it);
            }
            else{
                assert(sent_flow == 0);
                return;
            }
        }
        else{
            ++it;
        }
    }

    bool maybe_cycle = bounded;
    while(maybe_cycle){
        // Perform Bellman-Ford algorithm
        // Find a path from the edge's *destination* to its *source* to make a cycle
        std::vector<node_elt> accessibles = get_Bellman_Ford(edestination);
        assert(accessibles[edestination].cost == 0);// std::cout << "Found a negative-cost cycle in the residual graph!" << std::endl;

        // If the cycle has negative cost, send flow along this cycle
        int path_cost = accessibles[esource].cost;
        if(path_cost < max_int and path_cost + ecost < 0){ // Reachable and negative cost cycle
            int cycle_cost = path_cost + ecost;
            // std::cout << "Found an improving cycle" << std::endl;
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
                //std::cout << "Found a negative-cost cycle with unbounded flow!" << std::endl;
                break;
            }
            cost -= (max_flow * cycle_cost);
            sent_flow += max_flow;
            for(int e : pos_edges)
                edges[e].flow += max_flow;
            for(int e : neg_edges)
                edges[e].flow -= max_flow;
        }
        else{ // Ok, no more cycle, optimal solution, we can just exit
            maybe_cycle = false;
        }
    }

 
    edges.emplace_back(esource, edestination, ecost, sent_flow);
}

MCF_graph::MCF_graph(int node_cnt, std::vector<MCF_graph::edge> edge_list) : edges(edge_list), nb_nodes(node_cnt), bounded(true), cost(0){
    for(int e=0; e<edges.size(); ++e){
        edge cur = edges[e];
        assert(cur.source < node_count() and cur.dest < node_count());
    }
}

void MCF_graph::print() const{
    std::cout << "Printing a flow problem" << std::endl;
    for(edge const E : edges){
        std::cout << E.source << " " << E.dest << ": " << E.cost << " ; flow " << E.flow << std::endl;
    }
}

