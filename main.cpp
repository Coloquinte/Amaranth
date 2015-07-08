
#include "detailed/placement_problem.hpp"


#include <iostream>
#include <stack>
#include <limits>
#include <chrono>

placement_problem input_placement_problem(){
    int bxmn, bymn, bxmx, bymx;
    std::cin >> bxmn >> bymn >> bxmx >> bymx;
    rect bounding_box(bxmn, bymn, bxmx, bymx);

    int nb_cells, nb_nets, nb_fixeds;
    std::cin >> nb_cells;
    std::vector<cell> cells;
    for(int i=0; i<nb_cells; ++i){
        int width, height, x_pitch, y_pitch;
        std::cin >> width >> height >> x_pitch >> y_pitch;
        cells.emplace_back(width, height, x_pitch, y_pitch);
    }

    std::cin >> nb_fixeds;
    std::vector<rect> fixeds;
    for(int i=0; i<nb_fixeds; ++i){
        int xmn, ymn, xmx, ymx, ind;
        std::cin >> xmn >> ymn >> xmx >> ymx;
        fixeds.emplace_back(xmn, ymn, xmx, ymx);
    }

    std::cin >> nb_nets;
    std::vector<std::vector<pin> > pins;
    for(int i=0; i<nb_nets; ++i){
        pins.emplace_back();
        int nb_pins;
        std::cin >> nb_pins;
        for(int j=0; j<nb_pins; ++j){
            int xmn, ymn, xmx, ymx, ind;
            std::cin >> ind >> xmn >> ymn >> xmx >> ymx;
            pins.back().emplace_back(ind, rect(xmn, ymn, xmx, ymx));
        }
    }
    return placement_problem(bounding_box, cells, pins, fixeds);
}

int main(){
    placement_problem first_pl = input_placement_problem();

    //std::cout << "Problem with " << first_pl.cell_count() << " cells and " << first_pl.net_count() << " nets " << std::endl;
    std::stack<placement_problem> to_evaluate;
    to_evaluate.push(first_pl);

    const int max_time_ms = 40000;
    std::chrono::time_point<std::chrono::system_clock> start, end, last_sol;
    start = std::chrono::system_clock::now();
    last_sol = std::chrono::system_clock::now();

    std::vector<std::pair<int, int> > sols;

    int best_correct_solution = std::numeric_limits<int>::max();
    long long nb_evaluated_nodes = 0, nb_bound_pruned = 0, nb_feasibility_pruned = 0;
    while(not to_evaluate.empty()){
        if(nb_evaluated_nodes % 1000 == 0){
            std::chrono::time_point<std::chrono::system_clock> cur_time = std::chrono::system_clock::now();
            int time_to_last_sol = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time-last_sol).count();
            if(time_to_last_sol > max_time_ms) break;
        }
        ++ nb_evaluated_nodes;
        //if(nb_evaluated_nodes % 1000 == 0) std::cout << "Evaluated " << nb_evaluated_nodes << " nodes" << std::endl;

        placement_problem cur = to_evaluate.top(); to_evaluate.pop();
        int cur_cost = cur.get_cost();
        if(cur_cost < best_correct_solution){
            if(cur.is_correct()){
                best_correct_solution = cur_cost;
                // std::cout << "Found new correct solution, of cost " << cur_cost << std::endl;
                std::chrono::time_point<std::chrono::system_clock> cur_time = std::chrono::system_clock::now();
                int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time-start).count();
                sols.emplace_back(cur_cost, elapsed_ms);
                last_sol = std::chrono::system_clock::now();
            }
            else if(cur.is_feasible()){
                std::vector<placement_problem> nexts = cur.branch();
                for(auto it = nexts.crbegin(); it != nexts.crend(); ++it)
                    to_evaluate.push(*it);
            }
            else{
                ++nb_feasibility_pruned;
            }
        }
        else{
            ++nb_bound_pruned;
        }
    }

    end = std::chrono::system_clock::now();

    int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << first_pl.cell_count() << "\t" << first_pl.net_count() << "\t";
    if(not sols.empty()){
        //std::cout << "Found: " << sols.back().second << " ms";
        if(to_evaluate.empty()) std::cout << "O";
        else std::cout << "U";
    }
    else{
        if(to_evaluate.empty()) std::cout << "I";
        else std::cout << "F";
    }
    std::cout << "\t" << elapsed_ms << "\t" << nb_evaluated_nodes << "\t" << nb_bound_pruned << "\t" << nb_feasibility_pruned << std::endl;
    //std::cout << "Finished, in " << elapsed_ms << " ms, evaluated " << nb_evaluated_nodes << " nodes" << std::endl;
    //std::cout << nb_bound_pruned + nb_feasibility_pruned << " were pruned, " << nb_bound_pruned << " for being suboptimal and " << nb_feasibility_pruned << " for being infeasible" << std::endl;

    //std::cout << "Columns:\tTime\tNodes" << std::endl;
    //std::cout << "Results:\t" <<  elapsed_ms << "\t" << nb_evaluated_nodes << std::endl;
    return 0;
}


