

#include "detailed/placement_problem.cpp"

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
        cur.x_min = round_upper(cur.x_min, cells[i].x_pitch);
        cur.x_max = round_lower(cur.x_max, cells[i].x_pitch);
        cur.y_min = round_upper(cur.y_min, cells[i].y_pitch);
        cur.y_max = round_lower(cur.y_max, cells[i].y_pitch);
    }

    // Tighten based on the other cells
    // A simple approximation is area-based: limited by the area taken by cells constrained to stay on each side
    
    
}




