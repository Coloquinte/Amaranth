
#include "incremental_flow.hpp"

struct point{
    int x, y;
    point(int xi, int yi) : x(xi), y(yi) {}
};

struct cell{
    int width, height;
    int x_pitch, y_pitch;
    cell(int w, int h, int xp, int yp) : width(w), height(h), x_pitch(xp), y_pitch(yp) {}
};
struct rect{
    int xmin, ymin, xmax, ymax;
    rect() {}
    rect(int xmn, int ymn, int xmx, int ymx) : xmin(xmn), ymin(ymn), xmax(xmx), ymax(ymx) {}

    static rect intersection(rect a, rect b){ return rect(std::max(a.xmin, b.xmin), std::max(a.ymin, b.ymin), std::min(a.xmax, b.xmax), std::min(a.ymax, b.ymax)); }
    int get_area() const { return std::max(0, xmax-xmin) * std::max(0, ymax-ymin); }
};

struct pin : rect{
    int ind;
    pin(int i, rect r) : rect(r), ind(i) {}
};

enum branching_rule{
    AREA,
    // Rules based on the minimum displacement necessary on x and y
    LMIN,
    LMAX,
    LAVG,
    // Rules based on the dimensions of the cells
    WMIN,
    WMAX,
    WAVG,
    // Smart rules
    FIRST_CYCLE,
    STRONG
};

class placement_problem{
    public:
    struct relative_constraint{
        int fc, sc; // The indexes for the two cells
        int min_dist; // Left corner to left corner
        relative_constraint(int fst, int snd, int dist) : fc(fst), sc(snd), min_dist(dist) {}
    };
    struct generic_constraint : relative_constraint{
        bool direction;
        generic_constraint(bool dir, int fst, int snd, int dist) : relative_constraint(fst, snd, dist), direction(dir) {}
    };

    private:
    MCF_graph x_flow, y_flow; // Flows with 1 fixed node, cell_count() cell nodes and 2*net_count() net nodes, in that order

    std::vector<cell> cells;
    std::vector<std::vector<pin> > nets;
    std::vector<rect> fixed_elts;

    std::vector<rect> position_constraints;
    std::vector<relative_constraint> x_constraints, y_constraints;

    // To use
    //bool feasible;
    //bool correct;

    void add_x_constraint(int fc, int sc, int min_dist);
    void add_y_constraint(int fc, int sc, int min_dist);
    void apply_constraint(generic_constraint constraint);

    // Branch with given added constraints, without or with added opposite constraints
    std::vector<placement_problem> branch_on_constraints(std::vector<generic_constraint> constraints) const;

    std::vector<placement_problem> branch_overlap_removal(int c1, int c2) const;
    std::vector<placement_problem> branch_overlap_removal(int c1, rect fixed_elt) const;
    std::vector<placement_problem> branch_pitch(int c) const;

    int evaluate_branch(int c1, int c2) const;
    int evaluate_branch(int c1, int c2, std::vector<point> const & pos, branching_rule rule = AREA) const;
    int evaluate_branch(int c1, rect fixed, std::vector<point> const & pos, branching_rule rule = AREA) const;
    int evaluate_branch(int c) const;

    void tighten();

    public:
    int cell_count() const{ return cells.size(); }
    int net_count() const{ return nets.size(); }
    int fixed_count() const{ return fixed_elts.size(); }

    bool operator<(placement_problem const & o) const;

    bool is_feasible() const;
    bool is_correct() const;
    int get_cost() const;

    int get_cost_from_primal() const;

    std::vector<point> get_positions() const;
    std::vector<placement_problem> branch(branching_rule rule = AREA) const;

    placement_problem(rect bounding_box, std::vector<cell> icells, std::vector<std::vector<pin> > inets, std::vector<rect> fixed=std::vector<rect>());
};


