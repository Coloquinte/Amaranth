
#include <vector>

class MCF_graph{
    public:
    struct edge{
        int source, dest;
        int cost;
        int flow;

        edge(int s, int d, int c, int f=0) : source(s), dest(d), cost(c), flow(f) {}
    };

    public:
    std::vector<edge> edges;

    int nb_nodes;
    int cost;
    bool bounded;

    private:
    struct node_elt{
        int cost;
        int incoming_edge;
    
        node_elt(int c, int i) : cost(c), incoming_edge(i) {}
    };
    
    struct queue_elt : node_elt{
        int destination_node;
    
        queue_elt(int d, int c, int i) : node_elt(c, i), destination_node(d) {}
    };

    bool check_optimal() const;
    std::vector<node_elt> get_Bellman_Ford(int source_node) const;

    public:
    // Create a graph from an OPTIMAL flow (later maybe add cycle cancelling)
    MCF_graph(int node_cnt=0, std::vector<edge> edges=std::vector<edge>());

    // Add a new edge and get a new optimal flow and potentials for the node; uses Dijkstra instead of Bellman-Ford
    void add_edge(int source, int dest, int cost);

    std::vector<int> get_potentials() const;
    int get_cost() const{ return cost; }
    bool is_bounded() const{ return bounded; }
    int node_count() const{ return nb_nodes; }

    void print() const;
};

