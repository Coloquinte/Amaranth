
#include "incremental_flow.hpp"

struct point{
    int x, y;
    point(int xi, int yi) : x(xi), y(yi) {}
};

struct cell{
    int width, height;
    int x_pitch, y_pitch
    dim(int w, int r) : widths(w), rows(r) {}
};
struct rect{
    int xmin, ymin, xmax, ymax;
    dim(int xmn, int ymn, int xmx, int ymx) : xmin(xmn), ymin(ymn), xmax(xmx), ymax(ymx) {}
};

struct pin : rect{
    int ind;
    pin(int i, rect r) : rect(r), ind(i) {}
};

class placement_problem{
    MCF_graph x_flow, y_flow; // Flows with 1 fixed node, cell_count() cell nodes and 2*net_count() net nodes, in that order

    std::vector<cell> cells;
    std::vector<std::vector<pin> > nets;

    std::vector<rect> position_constraints;
    //std::vector<std::vector<biconstraint> > cell_constraints;
    //std::vector<std::pair<int, int> > row_limits;

    bool feasible;
    bool correct;

    public:
    int cell_count() const{ return cells.size(); }
    int net_count() const{ return nets.size(); }

    bool maybe_feasible() const;
    bool feasible() const;
    int get_cost() const;

    void tighten();
    std::vector<point> get_positions() const;
    std::vector<placement_problem> branch() const;

    placement_problem(std::vector<cell> icells, std::vector<net> inets);
};


