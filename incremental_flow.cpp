
#include "detailed/incremental_flow.hpp"

#include <cassert>
#include <queue>
#include <limits>
#include <iostream>

namespace{
    int const max_int = std::numeric_limits<int>::max()/2; // Avoid overflows: half the maximum
}

std::vector<MCF_graph::node_elt> MCF_graph::get_Bellman_Ford(int source_node) const{
    assert(source_node < node_count());
    std::vector<node_elt> accessibles(node_count(), node_elt(max_int, -1));
    accessibles[source_node] = node_elt(0, -1);
    for(int i=0; i<=node_count(); ++i){
        bool found_relaxation = false;
        for(int e=0; e<edges.size(); ++e){
            edge const E = edges[e];
            int forward_cost = accessibles[E.source].cost + E.cost;
            if(forward_cost < accessibles[E.dest].cost){
                accessibles[E.dest] = node_elt(forward_cost, e, accessibles[E.source].max_flow);
                found_relaxation = true;
            }
            int backward_cost = accessibles[E.dest].cost - E.cost;
            if(backward_cost < accessibles[E.source].cost and E.flow > 0){
                accessibles[E.source] = node_elt(backward_cost, e, std::min(accessibles[E.dest].max_flow, E.flow));
                found_relaxation = true;
            }
        }
        if(not found_relaxation) break;
        else if(i == node_count() or accessibles[source_node].cost < 0){
            assert(false); // Negative cost cycle, unexpected
            accessibles[source_node] = node_elt(-1, -1);
            break;
        }
    }
    return accessibles;
}

std::vector<MCF_graph::node_elt> MCF_graph::get_reduced_Dijkstra(int source_node) const{
    assert(source_node < node_count());
    std::vector<node_elt> accessibles(node_count(), node_elt(max_int, -1));

    std::priority_queue<queue_elt> pqueue;
    pqueue.emplace(source_node, -potentials[source_node], -1, max_int);

    std::cerr << "Potentials (" << potentials.size() << ", " << node_count() << " nodes): ";
    for(int p : potentials)
        std::cerr << p << " ";
    std::cerr << std::endl;

    // Use the cost - the potential: enforces non-negative edge costs
    while(not pqueue.empty()){
        auto cur = pqueue.top(); pqueue.pop();
        int real_cost = cur.cost + potentials[cur.destination_node];
        if(real_cost < accessibles[cur.destination_node].cost){
            assert(accessibles[cur.destination_node].cost >= max_int);
            accessibles[cur.destination_node] = node_elt(real_cost, cur.incoming_edge, cur.max_flow);
            for(int e : adjacent_edges[cur.destination_node]){
                edge const E = edges[e];
                assert(potentials[E.dest] - potentials[E.source] >= E.cost);
                if(E.source == cur.destination_node){
                    int this_cost = real_cost + E.cost;
                    int reduced_cost = this_cost - potentials[E.dest];
                    if(this_cost < accessibles[E.dest].cost){
                        pqueue.emplace(E.dest, reduced_cost, e, cur.max_flow);
                    }
                }
                else if(E.flow > 0){
                    assert(potentials[E.dest] - potentials[E.source] == E.cost);
                    assert(E.dest == cur.destination_node);
                    int this_cost = real_cost - E.cost;
                    int reduced_cost = this_cost - potentials[E.source];
                    if(this_cost < accessibles[E.source].cost){
                        pqueue.emplace(E.source, reduced_cost, e, std::min(cur.max_flow, E.flow));
                    }
                }
            }
        }
    }

    return accessibles;
}

std::vector<MCF_graph::node_elt> MCF_graph::update_Bellman_Ford(int source_node){
    auto accessibles = get_Bellman_Ford(source_node);
    for(int i=0; i<node_count(); ++i)
        potentials[i]=accessibles[i].cost;
    assert(check_optimal());
    return accessibles;
}

std::vector<MCF_graph::node_elt> MCF_graph::update_reduced_Dijkstra(int source_node){
    auto accessibles = get_reduced_Dijkstra(source_node);
    for(int i=0; i<node_count(); ++i)
        potentials[i]=accessibles[i].cost;
    assert(check_optimal());
    return accessibles;
}
std::vector<int> const MCF_graph::get_potentials() const{ return potentials; }

bool MCF_graph::check_optimal() const{
    std::vector<int> potentials = get_potentials();
    for(edge const E : edges){
        if(E.cost < potentials[E.dest] - potentials[E.source] and potentials[E.dest] < max_int and potentials[E.source] < max_int) return false;
        if(E.flow > 0 and E.cost > potentials[E.dest] - potentials[E.source] and potentials[E.dest] < max_int and potentials[E.source] < max_int) return false;
    }
    return true;
}

std::pair<bool, int> MCF_graph::try_edge(int esource, int edestination, int ecost) const{
    std::vector<node_elt> accessibles = get_Bellman_Ford(edestination);
    int path_cost = accessibles[esource].cost + ecost;
    int max_flow = accessibles[esource].max_flow;
    if(path_cost < 0){
        if(max_flow >= max_int) // Infeasible
            return std::pair<bool, int>(false, get_cost());
        else
            return std::pair<bool, int>(true, get_cost() - path_cost * max_flow);
    }
    else{
        return std::pair<bool, int>(true, get_cost());
    }
}

void MCF_graph::add_edge(int esource, int edestination, int ecost){
    assert(esource != edestination and esource < node_count() and edestination < node_count() and esource >= 0 and edestination >= 0);
    int sent_flow=0;

    // Handling of redundant edges
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
        std::vector<node_elt> accessibles = update_Bellman_Ford(edestination);
        //std::vector<node_elt> accessibles = update_reduced_Dijkstra(edestination);
        assert(accessibles[edestination].cost == 0); // If we found a negative cost cycle, then our previous solution was not optimal

        // If the cycle has negative cost, send flow along this cycle
        int path_cost = accessibles[esource].cost;
        if(path_cost < max_int and path_cost + ecost < 0){ // Reachable and negative cost cycle
            int cycle_cost = path_cost + ecost;
            // std::cout << "Found an improving cycle" << std::endl;
            // Find the maximum flow along the cycle
            int max_flow = accessibles[esource].max_flow;
            if(max_flow >= max_int){
                bounded = false;
                break;
            }
            int cur_node=esource;
            // TODO: detect negative cost cycles with a single edge in the opposite direction; this edge may be safely removed
            while(cur_node != edestination){
                int e = accessibles[cur_node].incoming_edge;
                assert(e >= 0);
                if(cur_node == edges[e].source){
                    edges[e].flow -= max_flow;
                    cur_node = edges[e].dest;
                }else{
                    assert(cur_node == edges[e].dest);
                    edges[e].flow += max_flow;
                    cur_node = edges[e].source;
                }
            }
            cost -= (max_flow * cycle_cost);
            sent_flow += max_flow;
        }
        else{ // Ok, no more cycle, optimal solution, we can just exit
            maybe_cycle = false;
        }
    }
    adjacent_edges[esource     ].push_back(edges.size()); 
    adjacent_edges[edestination].push_back(edges.size()); 
    edges.emplace_back(esource, edestination, ecost, sent_flow);
    assert(not bounded or check_optimal());
}

MCF_graph::MCF_graph(int node_cnt, std::vector<MCF_graph::edge> edge_list)
: edges(edge_list)
, adjacent_edges(node_cnt)
, potentials(node_cnt, 0)
, nb_nodes(node_cnt)
, cost(0)
, bounded(true){
    for(int e=0; e<edges.size(); ++e){
        edge cur = edges[e];
        adjacent_edges[cur.source].push_back(e);
        adjacent_edges[cur.dest  ].push_back(e);
        assert(cur.source < node_count() and cur.dest < node_count());
    }
    if(node_count() > 0)
        update_Bellman_Ford(0); // Obtain the potentials
}

void MCF_graph::print() const{
    std::cout << "Printing a flow problem" << std::endl;
    for(edge const E : edges){
        std::cout << E.source << " " << E.dest << ": " << E.cost << " ; flow " << E.flow << std::endl;
    }
}

