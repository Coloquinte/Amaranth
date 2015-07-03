

#include "detailed/placement_problem.hpp"

// Use of modulos: careful with negative numbers
int round_lower(int pos, int pitch){
    int modulo = pos % pitch;
    if(modulo < 0) modulo += pitch; // Obtain the mathematical modulo result
    return pos - modulo;
}
int round_upper(int pos, int pitch){
    int modulo = pos % pitch;
    if(modulo <= 0) modulo += pitch; // Obtain the mathematical modulo result, or pitch if it is 0
    return pos + pitch - modulo;
}

void placement_problem::tighten(){
    // Tighten based on fixed position: doesn't improve on the minimum-cost-flow relaxation in itself
    for(relative_constraint constraint : x_constraints){
        position_constraints[constraint.sc].xmin = std::max(position_constraints[constraint.sc].xmin, position_constraints[constraint.fc].xmin + constraint.min_dist);
        position_constraints[constraint.fc].xmax = std::min(position_constraints[constraint.fc].xmax, position_constraints[constraint.sc].xmax - constraint.min_dist);
    }
    for(relative_constraint constraint : y_constraints){
        position_constraints[constraint.sc].ymin = std::max(position_constraints[constraint.sc].ymin, position_constraints[constraint.fc].ymin + constraint.min_dist);
        position_constraints[constraint.fc].ymax = std::min(position_constraints[constraint.fc].ymax, position_constraints[constraint.sc].ymax - constraint.min_dist);
    }

    // Tighten based on the pitches for the cells
    for(int i=0; i<cells.size(); ++i){
        rect & cur = position_constraints[i];
        cur.xmin = round_upper(cur.xmin, cells[i].x_pitch);
        cur.xmax = round_lower(cur.xmax, cells[i].x_pitch);
        cur.ymin = round_upper(cur.ymin, cells[i].y_pitch);
        cur.ymax = round_lower(cur.ymax, cells[i].y_pitch);
    }

    // Tighten based on the other cells
    // Simple transitivity of the order is handled by the MCF relaxation: we need something better
    // A simple approximation is area-based: limited by the area taken by cells constrained to stay on each side
    
    // Tighten based on feasible regions
}




