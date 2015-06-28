

#include "detailed/placement_problem.hpp"


bool is_maybe_feasible() const{
    bool maybe = x_flow.is_bounded() and y_flow<.is_bounded();
    for(rect const R : position_constraints){
        maybe = maybe and (R.x_min <= R.x_max) and (R.y_min <= R.y_max);
    }
    return maybe;
}

// Verify that the pitches for the cells are respected and that the cells do not overlap
bool is_correct_solution() const{
    std::vector<point> pos = get_positions();
    for(int i=0; i<cells.size(); ++i){
        if(pos[i].x % cells[i].x_pitch != 0) return false;
        if(pos[i].y % cells[i].y_pitch != 0) return false;
    }
    for(int i=0; i<cells.size(); ++i){
        rect & cur = position_constraints[i];
        if(cur.x_min > cur.x_max or cur.x_min > pos[i].x or cur.x_max < pos[i].x) return false;
    }

    // Should use a line sweep to verify that there is no overlap
    

    return true;
}

int get_cost(){
    return x_flow.get_cost() + y_flow.get_cost();
}

std::vector<point> get_positions() const{
    std::vector<int> x_pos = x_flow.get_potentials(), y_pos = y_flow.get_potentials();
    // Use the potentials of the cells - the potential of the fixed node
    std::vector<point> ret;
    for(int i=0; i<cell_count(); ++i){
        ret.push_back(point(x_pos[i+1]-x_pos[0], y_pos[i+1]-y_pos[0]));
    }
    return ret;
}


