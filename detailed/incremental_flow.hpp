
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
    std::vector<std::vector<int> > adjacent_edges;

    int cost;
    bool bounded;

    private:
    // Get a path to send flow; uses Dijkstra instead of Bellman-Ford (careful use of potentials)
    void update_potential();

    public:
    // Create a graph from an OPTIMAL flow (later maybe add cycle cancelling)
    MCF_graph(int node_cnt, std::vector<edge> edges);

    // Add a new edge and get a new optimal flow and potentials for the node; uses Dijkstra instead of Bellman-Ford
    void add_edge(int source, int dest, int cost);

    std::vector<int> get_potentials() const;
    int get_cost() const{ return cost; }
    bool is_bounded() const{ return bounded; }
    int node_count() const{ return adjacent_edges.size(); }
};

