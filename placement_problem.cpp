

#include "detailed/placement_problem.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>

int eval_overlap(rect r1, rect r2, branching_rule rule){
    int dist_x = std::min(r1.xmax-r2.xmin, r2.xmax-r1.xmin);
    int dist_y = std::min(r1.ymax-r2.ymin, r2.ymax-r1.ymin);
    assert(dist_x > 0 and dist_y > 0);
    int height = r1.get_height() + r2.get_height();
    int width  = r1.get_width()  + r2.get_width();
    switch(rule){
      case AREA:
        return rect::intersection(r1, r2).get_area();
      case LMIN:
        return std::min(dist_x, dist_y);
      case LMAX:
        return std::max(dist_x, dist_y);
      case LAVG:
        return dist_x+dist_y;
      case WMIN:
        return std::min(height, width);
      case WMAX:
        return std::max(height, width);
      case WAVG:
        return height + width;
      default:
        abort();
    }
}

std::vector<int> placement_problem::get_expected_branch(int c1, int c2, int w1, int w2, int h1, int h2) const{
    auto rgt = x_flow.try_edge(c2+1, c1+1, -w1);
    auto lft = x_flow.try_edge(c1+1, c2+1, -w2);
    auto upp = y_flow.try_edge(c2+1, c1+1, -h1);
    auto dow = y_flow.try_edge(c1+1, c2+1, -h2);
    std::vector<int> ret;
    int x_cost = x_flow.get_cost(), y_cost = y_flow.get_cost();
    if(rgt.first){ ret.push_back(rgt.second + y_cost); }
    if(lft.first){ ret.push_back(lft.second + y_cost); }
    if(upp.first){ ret.push_back(upp.second + x_cost); }
    if(dow.first){ ret.push_back(dow.second + x_cost); }
    return ret;
}

std::vector<int> placement_problem::get_strong_branch(int c1, int c2, int w1, int w2, int h1, int h2) const{
    MCF_graph rgt_x_flow = x_flow, lft_x_flow = x_flow;
    MCF_graph upp_y_flow = y_flow, dow_y_flow = y_flow;
    rgt_x_flow.add_edge(c2+1, c1+1, -w1);
    lft_x_flow.add_edge(c1+1, c2+1, -w2);
    upp_y_flow.add_edge(c2+1, c1+1, -h1);
    dow_y_flow.add_edge(c1+1, c2+1, -h2);
    std::vector<int> ret;
    int x_cost = x_flow.get_cost(), y_cost = y_flow.get_cost();
    if(rgt_x_flow.is_bounded()){ ret.push_back(rgt_x_flow.get_cost() + y_cost); }
    if(lft_x_flow.is_bounded()){ ret.push_back(lft_x_flow.get_cost() + y_cost); }
    if(upp_y_flow.is_bounded()){ ret.push_back(upp_y_flow.get_cost() + x_cost); }
    if(dow_y_flow.is_bounded()){ ret.push_back(dow_y_flow.get_cost() + x_cost); }
    return ret;
} 

int placement_problem::evaluate_branch(int c1, int c2, std::vector<point> const & pos, branching_rule rule) const{

    rect fc(pos[c1].x, pos[c1].y, pos[c1].x+cells[c1].width, pos[c1].y+cells[c1].height),
         sc(pos[c2].x, pos[c2].y, pos[c2].x+cells[c2].width, pos[c2].y+cells[c2].height);
    int area = rect::intersection(fc, sc).get_area();
    if(area <= 0) return -1;

    if(rule == CMIN or rule == CAVG
    or rule == SMIN or rule == SAVG){
        std::vector<int> res = 
            (rule == CMIN or rule == CAVG) ?
            get_expected_branch (c1, c2, cells[c1].width, cells[c2].width, cells[c1].height, cells[c2].height)
          : get_strong_branch   (c1, c2, cells[c1].width, cells[c2].width, cells[c1].height, cells[c2].height);
        if(res.size() <= 1) return std::numeric_limits<int>::max();
        if(rule == SMIN or rule == CMIN) return *std::min_element(res.begin(), res.end());
        else{
            int tot=0;
            for(int t : res) tot += t;
            return tot/res.size();
        }
    }
    else{
        return eval_overlap(fc, sc, rule);
    }
}

int placement_problem::evaluate_branch(int c1, rect fixed, std::vector<point> const & pos, branching_rule rule) const{
    rect crect(pos[c1].x, pos[c1].y, pos[c1].x+cells[c1].width, pos[c1].y+cells[c1].height);
    int area = rect::intersection(fixed, crect).get_area();
    if(area <= 0) return -1;

    if(rule == CMIN or rule == CAVG
    or rule == SMIN or rule == SAVG){
        std::vector<int> res = 
            (rule == CMIN or rule == CAVG) ?
            get_expected_branch (c1, -1, cells[c1].width-fixed.xmax, fixed.xmin, cells[c1].height-fixed.ymax, fixed.ymin)
          : get_strong_branch   (c1, -1, cells[c1].width-fixed.xmax, fixed.xmin, cells[c1].height-fixed.ymax, fixed.ymin);
        if(res.size() <= 1) return std::numeric_limits<int>::max();
        if(rule == SMIN or rule == CMIN) return *std::min_element(res.begin(), res.end());
        else{
            int tot=0;
            for(int t : res) tot += t;
            return tot/res.size();
        }
    }
    else{
        return eval_overlap(fixed, crect, rule);
    }
}

bool placement_problem::operator<(placement_problem const & o) const{
    if(is_feasible() and o.is_feasible()) return get_cost() < o.get_cost();
    else return (not is_feasible()) and o.is_feasible(); // Unfeasible first
}

void placement_problem::apply_constraint(generic_constraint constraint){
    assert(constraint.fc < cell_count() and constraint.sc < cell_count());
    assert(constraint.fc >= -1 and constraint.sc >= -1);
    if(constraint.direction){
        //if(constraint.fc >= 0 and constraint.sc >= 0)
        y_constraints.push_back(constraint);
        y_flow.add_edge(constraint.sc+1, constraint.fc+1, -constraint.min_dist);
    }
    else{
        //if(constraint.fc >= 0 and constraint.sc >= 0)
        x_constraints.push_back(constraint);
        x_flow.add_edge(constraint.sc+1, constraint.fc+1, -constraint.min_dist);
    }
}

std::vector<placement_problem> placement_problem::branch_on_constraints(std::vector<generic_constraint> constraints) const{
    typedef std::pair<placement_problem::generic_constraint, placement_problem::generic_constraint> cpair;
    typedef std::pair<placement_problem, placement_problem::generic_constraint> ppair;

    std::vector<ppair> probs;
    for(generic_constraint cur : constraints){
        generic_constraint opposite(cur.direction, cur.sc, cur.fc, -cur.min_dist+1);
        probs.push_back(ppair(*this, opposite));
        probs.back().first.apply_constraint(cur);
    }
    std::sort(probs.begin(), probs.end(), [](ppair const & a, ppair const & b) { return a.first < b.first; });
    for(int i=0; i+1<probs.size(); ++i){
        for(int j=i+1; j<probs.size(); ++j){
            probs[j].first.apply_constraint(probs[i].second);
        }
    }
    std::vector<placement_problem> ret;
    for(auto const & cur : probs){
        if(cur.first.is_feasible())
            ret.push_back(cur.first);
    }
    return ret;
}


std::vector<placement_problem> placement_problem::branch_overlap_removal(int c1, int c2) const{

    // A constraint satisfied in the branch, and another one to constrain the other sides further
    std::vector<generic_constraint> constraints({
        generic_constraint(false, c1, c2, cells[c1].width ),
        generic_constraint(false, c2, c1, cells[c2].width ),
        generic_constraint(true , c1, c2, cells[c1].height),
        generic_constraint(true , c2, c1, cells[c2].height)
    });
    return branch_on_constraints(constraints);
}

std::vector<placement_problem> placement_problem::branch_overlap_removal(int c1, rect fixed) const{

    // A constraint satisfied in the branch, and another one to constrain the other sides further
    std::vector<generic_constraint> constraints({
        generic_constraint(false, c1, -1, cells[c1].width  - fixed.xmax),
        generic_constraint(false, -1, c1, fixed.xmin                   ),
        generic_constraint(true , c1, -1, cells[c1].height - fixed.ymax),
        generic_constraint(true , -1, c1, fixed.ymin                   )
    });
    return branch_on_constraints(constraints);
}

bool placement_problem::is_feasible() const{
    return x_flow.is_bounded() and y_flow.is_bounded();
}

// Verify that the pitches for the cells are respected and that the cells do not overlap
bool placement_problem::is_correct() const{
    return is_solution_correct(get_positions());
}

bool placement_problem::is_solution_correct(std::vector<point> const pos) const{
    /*
    for(int i=0; i<cells.size(); ++i){
        if(pos[i].x % cells[i].x_pitch != 0) return false;
        if(pos[i].y % cells[i].y_pitch != 0) return false;
    }
    */
    /*for(int i=0; i<cells.size(); ++i){
        rect const & cur = position_constraints[i];
        if(cur.xmin > cur.xmax or cur.xmin > pos[i].x or cur.xmax < pos[i].x) return false;
        if(cur.ymin > cur.ymax or cur.ymin > pos[i].y or cur.ymax < pos[i].y) return false;
    }*/
    if(not is_feasible()) return false;

    for(rect const R : fixed_elts){
        for(int i=0; i<cells.size(); ++i){
            if(pos[i].x + cells[i].width  > R.xmin
           and pos[i].y + cells[i].height > R.ymin
           and R.xmax  > pos[i].x
           and R.ymax  > pos[i].y) return false;
        }
    }
    
    // TODO: Should use a line sweep to verify that there is no overlap
    for(int i=0; i+1<cells.size(); ++i){
        for(int j=i+1; j<cells.size(); ++j){
            if(pos[i].x + cells[i].width  > pos[j].x
           and pos[j].x + cells[j].width  > pos[i].x
           and pos[i].y + cells[i].height > pos[j].y
           and pos[j].y + cells[j].height > pos[i].y) return false;
        }
    }
    
    return true;
}

int placement_problem::get_cost() const{
    int ret = x_flow.get_cost() + y_flow.get_cost();
    if(is_feasible())
        assert(get_solution_cost(get_positions()) == ret);
    return ret;
}

int placement_problem::get_solution_cost(std::vector<point> const pos) const{
    int tot_cost=0;
    for(auto const & n : nets){
        if(n.empty()) continue;
        int xmin=std::numeric_limits<int>::max(),
            ymin=std::numeric_limits<int>::max(),
            xmax=std::numeric_limits<int>::min(),
            ymax=std::numeric_limits<int>::min();
        for(auto const p : n){
            if(p.ind == -1){
                xmin = std::min(xmin, p.xmin);
                xmax = std::max(xmax, p.xmax);
                ymin = std::min(ymin, p.ymin);
                ymax = std::max(ymax, p.ymax);
            }
            else{
                xmin = std::min(xmin, pos[p.ind].x + p.xmin);
                xmax = std::max(xmax, pos[p.ind].x + p.xmax);
                ymin = std::min(ymin, pos[p.ind].y + p.ymin);
                ymax = std::max(ymax, pos[p.ind].y + p.ymax);
            }
        }
        assert(xmax >= xmin and ymax >= ymin);
        tot_cost += ((xmax-xmin) + (ymax-ymin));
    }
    return tot_cost;
}

std::vector<point> placement_problem::get_positions() const{
    std::vector<int> x_pos = x_flow.get_potentials(), y_pos = y_flow.get_potentials();
    for(int x : x_pos) assert(x < std::numeric_limits<int>::max() / 2);
    for(int y : y_pos) assert(y < std::numeric_limits<int>::max() / 2);

    // Use the potentials of the cells - the potential of the fixed node
    std::vector<point> ret;
    for(int i=0; i<cell_count(); ++i){
        ret.push_back(point(x_pos[i+1]-x_pos[0], y_pos[i+1]-y_pos[0]));
    }
    return ret;
}

std::vector<placement_problem> placement_problem::branch(branching_rule rule) const{
    // Chose a good branch based simply on the positions of the cells
    std::vector<point> pos = get_positions();

    int best_fc, best_sc;
    int best_cell_measure=-1;
    bool found_cell_overlap=false;

    // Branch to avoid overlaps between cells
    for(int i=0; i+1<cells.size(); ++i){
        for(int j=i+1; j<cells.size(); ++j){
            int measure = evaluate_branch(i, j, pos, rule);
            if(measure >= 0 and measure > best_cell_measure){
                found_cell_overlap=true;
                best_cell_measure = measure;
                best_fc = i; best_sc = j;
            }
        }
    }

    rect best_fixed;
    int best_fixed_c;
    int best_fixed_measure=-1;
    bool found_fixed_overlap=false;

    for(rect const R : fixed_elts){
        for(int i=0; i<cells.size(); ++i){
            int measure = evaluate_branch(i, R, pos, rule);
            if(measure >= 0 and measure > best_fixed_measure){
                found_fixed_overlap=true;
                best_fixed_measure = measure;
                best_fixed = R; best_fixed_c = i;
            }
        }
    }

    if(found_cell_overlap and (not found_fixed_overlap or best_cell_measure >= best_fixed_measure) ){
        return branch_overlap_removal(best_fc, best_sc);
    }
    else if(found_fixed_overlap){
        return branch_overlap_removal(best_fixed_c, best_fixed);
    }
    else{
        assert(is_correct());
        return std::vector<placement_problem>();
    }
}



placement_problem::placement_problem(rect bounding_box, std::vector<cell> icells, std::vector<std::vector<pin> > inets, std::vector<rect> fixed)
:
    cells(icells),
    nets(inets)
{
    for(cell const c : cells){
        position_constraints.emplace_back(bounding_box.xmin, bounding_box.ymin, bounding_box.xmax - c.width, bounding_box.ymax - c.height);
    }
    for(rect const R : fixed){
        if(rect::intersection(R, bounding_box).get_area() > 0)
            fixed_elts.emplace_back(rect::intersection(R, bounding_box));
    }

    // The simplest edges: the constraints that a net's upper bound is bigger than a net's lower bound
    std::vector<MCF_graph::edge> basic_x_edges, basic_y_edges;
    for(int i=0; i<net_count(); ++i){
        int UB_ind = cell_count() + 1 + 2*i;
        int LB_ind = UB_ind + 1;
        basic_x_edges.emplace_back(UB_ind, LB_ind, 0, 1);
        basic_y_edges.emplace_back(UB_ind, LB_ind, 0, 1);
    }

    // Edges for the placement constraints
    for(int i=0; i<cell_count(); ++i){
        basic_x_edges.emplace_back(i+1, 0, -bounding_box.xmin); // Edge to the fixed node: left limit of the region
        basic_x_edges.emplace_back(0, i+1, bounding_box.xmax - cells[i].width); // Edge from the fixed node: right limit of the region
        basic_y_edges.emplace_back(i+1, 0, -bounding_box.ymin); // Edge to the fixed node: lower limit of the region
        basic_y_edges.emplace_back(0, i+1, bounding_box.ymax - cells[i].height); // Edge from the fixed node: upper limit of the region
    }

    x_flow = MCF_graph(cell_count() + 2*net_count() + 1, basic_x_edges);
    y_flow = MCF_graph(cell_count() + 2*net_count() + 1, basic_y_edges);
    //x_flow.print();
    //y_flow.print();

    //std::cout << "Net edges" << std::endl;
    // Edges for the nets
    for(int i=0; i<net_count(); ++i){
        assert(not nets[i].empty());
        int UB_ind = cell_count() + 1 + 2*i;
        int LB_ind = UB_ind + 1;
        for(pin const cur_pin : nets[i]){
            assert(cur_pin.ind >= -1 and cur_pin.ind < cell_count());
            // cur_pin.ind == -1 ==> Fixed pin case
            x_flow.add_edge(UB_ind, cur_pin.ind+1, -cur_pin.xmax);
            y_flow.add_edge(UB_ind, cur_pin.ind+1, -cur_pin.ymax);
            x_flow.add_edge(cur_pin.ind+1, LB_ind,  cur_pin.xmin);
            y_flow.add_edge(cur_pin.ind+1, LB_ind,  cur_pin.ymin);
        }
    }

    for(point p : get_positions()){
        assert(p.x != std::numeric_limits<int>::max());
        assert(p.y != std::numeric_limits<int>::max());
    }

    //x_flow.print();
    //y_flow.print();
}

void placement_problem::print() const{
    std::cout << "X constraints" << std::endl;
    for(auto co : x_constraints){
        std::cerr << co.fc << " + " << co.min_dist << " <= " << co.sc << std::endl;
    }
    std::cout << "Y constraints" << std::endl;
    for(auto co : y_constraints){
        std::cerr << co.fc << " + " << co.min_dist << " <= " << co.sc << std::endl;
    }
}

