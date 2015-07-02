

#include "detailed/placement_problem.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>

void placement_problem::add_x_constraint(int fc, int sc, int min_dist){
    assert(fc >= 0 and fc < cell_count()); assert(sc >= 0 and sc < cell_count());
    relative_constraint constraint(fc, sc, min_dist);
    x_constraints.push_back(constraint);
    x_flow.add_edge(sc+1, fc+1, -min_dist);
}
void placement_problem::add_y_constraint(int fc, int sc, int min_dist){
    relative_constraint constraint(fc, sc, min_dist);
    y_constraints.push_back(constraint);
    y_flow.add_edge(sc+1, fc+1, -min_dist);
}

std::vector<placement_problem> placement_problem::branch_overlap_removal(int c1, int c2) const{
    auto rgt_problem = *this; rgt_problem.add_x_constraint(c1, c2, cells[c1].width);
    auto lft_problem = *this; lft_problem.add_x_constraint(c2, c1, cells[c2].width);
    auto upp_problem = *this; upp_problem.add_y_constraint(c1, c2, cells[c1].height);
    auto dow_problem = *this; dow_problem.add_y_constraint(c2, c1, cells[c2].height);
    //upp_problem.x_flow.add_edge(best_fc+1, best_sc+1, cells[best_sc].width+1);
    //dow_problem.x_flow.add_edge(best_fc+1, best_sc+1, cells[best_sc].width+1);
    //upp_problem.x_flow.add_edge(best_sc+1, best_fc+1, cells[best_fc].width+1);
    //dow_problem.x_flow.add_edge(best_sc+1, best_fc+1, cells[best_fc].width+1);
    auto ret = std::vector<placement_problem>({rgt_problem, lft_problem, upp_problem, dow_problem});
    std::sort(ret.begin(), ret.end(), [](placement_problem const a, placement_problem const b){ return a.get_cost() < b.get_cost(); });
    return ret;
}

bool placement_problem::is_feasible() const{
    bool maybe = x_flow.is_bounded() and y_flow.is_bounded();
    for(rect const R : position_constraints){
        maybe = maybe and (R.xmin <= R.xmax) and (R.ymin <= R.ymax);
    }
    return maybe;
}

// Verify that the pitches for the cells are respected and that the cells do not overlap
bool placement_problem::is_correct() const{
    std::vector<point> pos = get_positions();
    /*
    for(int i=0; i<cells.size(); ++i){
        if(pos[i].x % cells[i].x_pitch != 0) return false;
        if(pos[i].y % cells[i].y_pitch != 0) return false;
    }
    */
    for(int i=0; i<cells.size(); ++i){
        rect const & cur = position_constraints[i];
        if(cur.xmin > cur.xmax or cur.xmin > pos[i].x or cur.xmax < pos[i].x) return false;
        if(cur.ymin > cur.ymax or cur.ymin > pos[i].y or cur.ymax < pos[i].y) return false;
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
    return x_flow.get_cost() + y_flow.get_cost();
}

std::vector<point> placement_problem::get_positions() const{
    std::vector<int> x_pos = x_flow.get_potentials(), y_pos = y_flow.get_potentials();
    // Use the potentials of the cells - the potential of the fixed node
    std::vector<point> ret;
    for(int i=0; i<cell_count(); ++i){
        ret.push_back(point(x_pos[i+1]-x_pos[0], y_pos[i+1]-y_pos[0]));
    }
    return ret;
}

std::vector<placement_problem> placement_problem::branch() const{
    // Chose a good branch based simply on the positions of the cells
    std::vector<point> pos = get_positions();
    /*
    std::cout << "X positions: ";
    for(point p : pos)
        std::cout << p.x << " ";
    std::cout << std::endl;
    std::cout << "Y positions: ";
    for(point p : pos)
        std::cout << p.y << " ";
    std::cout << std::endl;
    */

    int best_fc, best_sc;
    int best_overlap=0;
    bool found_overlap = false;
    //int best_max_cost=std::numeric_limits<int>::max();
    //int best_diff = 0; // std::numeric_limits<int>::max();

    // Branch to avoid overlaps
    for(int i=0; i+1<cells.size(); ++i){
        for(int j=i+1; j<cells.size(); ++j){
            rect fc(pos[i].x, pos[i].y, pos[i].x+cells[i].width, pos[i].y+cells[i].height),
                 sc(pos[j].x, pos[j].y, pos[j].x+cells[j].width, pos[j].y+cells[j].height);
            // Now how much area is shared
            
            if(rect::intersection(fc, sc).get_area() > best_overlap){
                found_overlap = true;
                best_fc = i; best_sc = j;
                best_overlap = rect::intersection(fc, sc).get_area();
            }
            
            /*
            if(rect::intersection(fc, sc).get_area() > 0){
                auto rgt_flow = x_flow; rgt_flow.add_edge(i+1, j+1, -cells[j].width);
                auto lft_flow = x_flow; lft_flow.add_edge(j+1, i+1, -cells[i].width);
                auto upp_flow = y_flow; upp_flow.add_edge(i+1, j+1, -cells[j].height);
                auto dow_flow = y_flow; dow_flow.add_edge(j+1, i+1, -cells[i].height);
                int rgt_diff = rgt_flow.get_cost() - x_flow.get_cost(),
                    lft_diff = lft_flow.get_cost() - x_flow.get_cost(),
                    upp_diff = upp_flow.get_cost() - y_flow.get_cost(),
                    dow_diff = dow_flow.get_cost() - y_flow.get_cost();
                int max_diff = rgt_diff;
                max_diff = std::max(max_diff, lft_diff);
                max_diff = std::max(max_diff, upp_diff);
                max_diff = std::max(max_diff, dow_diff);
                int min_diff = rgt_diff;
                min_diff = std::min(min_diff, lft_diff);
                min_diff = std::min(min_diff, upp_diff);
                min_diff = std::min(min_diff, dow_diff);

                int cur_diff = std::sqrt(rect::intersection(fc, sc).get_area());

                //if(max_cost < best_max_cost){
                if(not found_overlap or cur_diff > best_diff){
                    //std::cout << "Found better overlap: " << best_overlap_cost << std::endl;
                    best_fc = i; best_sc = j;
                    //best_max_cost = max_cost;
                    best_diff = cur_diff;
                }
                found_overlap = true;
            }
            */
        }
    }

    if(not found_overlap){
        //std::cout << "The solution was correct!" << std::endl;
        assert(is_correct());
        return std::vector<placement_problem>();
    }
    else{
        return branch_overlap_removal(best_fc, best_sc);
    }
    

    /*    
    bool found_unpitched, best_pitch_dir;
    int best_pitch_var, best_pitched_cell;

    // Branch to respect the pitch
    for(int i=0; i+1<cells.size(); ++i){
        
    }
    */
}



placement_problem::placement_problem(rect bounding_box, std::vector<cell> icells, std::vector<std::vector<pin> > inets, std::vector<rect> fixed)
:
    cells(icells),
    nets(inets)
{
    for(cell const c : cells){
        position_constraints.emplace_back(bounding_box.xmin, bounding_box.ymin, bounding_box.xmax - c.width, bounding_box.ymax - c.height);
    }

    // The simplest edges: the constraints that a net's upper bound is bigger than a net's lower bound
    std::vector<MCF_graph::edge> basic_x_edges, basic_y_edges;
    for(int i=0; i<net_count(); ++i){
        int UB_ind = cell_count() + 1 + 2*i;
        int LB_ind = UB_ind + 1;
        basic_x_edges.emplace_back(UB_ind, LB_ind, 0, 1);
        basic_y_edges.emplace_back(UB_ind, LB_ind, 0, 1);
    }

    x_flow = MCF_graph(cell_count() + 2*net_count() + 1, basic_x_edges);
    y_flow = MCF_graph(cell_count() + 2*net_count() + 1, basic_y_edges);
    //x_flow.print();
    //y_flow.print();

    //std::cout << "Net edges" << std::endl;
    // Edges for the nets
    for(int i=0; i<net_count(); ++i){
        int UB_ind = cell_count() + 1 + 2*i;
        int LB_ind = UB_ind + 1;
        for(pin const cur_pin : nets[i]){
            assert(cur_pin.ind >= -1 and cur_pin.ind < cell_count());
            // cur_pin.ind == -1 ==> Fixed pin case
            x_flow.add_edge(UB_ind, cur_pin.ind+1, -cur_pin.xmax);
            y_flow.add_edge(UB_ind, cur_pin.ind+1, -cur_pin.ymax);
            x_flow.add_edge(cur_pin.ind+1, LB_ind, cur_pin.xmin);
            y_flow.add_edge(cur_pin.ind+1, LB_ind, cur_pin.ymin);
        }
    }

    //std::cout << "Fixed edges" << std::endl;
    // Edges for the placement constraints
    for(int i=0; i<cell_count(); ++i){
        x_flow.add_edge(i+1, 0, -bounding_box.xmin); // Edge to the fixed node: left limit of the region
        x_flow.add_edge(0, i+1, bounding_box.xmax - cells[i].width); // Edge from the fixed node: right limit of the region
        y_flow.add_edge(i+1, 0, -bounding_box.ymin); // Edge to the fixed node: lower limit of the region
        y_flow.add_edge(0, i+1, bounding_box.ymax - cells[i].height); // Edge from the fixed node: upper limit of the region
    }

    //x_flow.print();
    //y_flow.print();
}

