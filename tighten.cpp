

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
    // Tighten based on the pitches for the cells
    for(int i=0; i<cells.size(); ++i){
        rect & cur = position_constraints[i];
        cur.xmin = round_upper(cur.xmin, cells[i].x_pitch);
        cur.xmax = round_lower(cur.xmax, cells[i].x_pitch);
        cur.ymin = round_upper(cur.ymin, cells[i].y_pitch);
        cur.ymax = round_lower(cur.ymax, cells[i].y_pitch);
    }

    // Tighten based on the other cells
    // A simple approximation is area-based: limited by the area taken by cells constrained to stay on each side
    
    
}




