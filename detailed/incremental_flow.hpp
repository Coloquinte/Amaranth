
#include <vector>
#include <limits>

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
        int max_flow;
    
        node_elt(int c, int i, int mf=std::numeric_limits<int>::max()/2) : cost(c), incoming_edge(i), max_flow(mf) {}
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

    // Just check the cost of the first cycle
    std::pair<bool, int> try_edge(int source, int dest, int cost) const;
    // Add a new edge and get a new optimal flow and potentials for the node; uses Dijkstra instead of Bellman-Ford
    void add_edge(int source, int dest, int cost);

    std::vector<int> const get_potentials() const;
    int get_cost() const{ return cost; }
    bool is_bounded() const{ return bounded; }
    int node_count() const{ return nb_nodes; }

    void print() const;
};

