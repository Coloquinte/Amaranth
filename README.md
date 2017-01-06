# Amaranth

Amaranth is an algorithm to optimize 2D rectangle placement for electronic design.
It is a branch-and-bound algorithm based on a minimum-cost-flow relaxation, intended to solve the leaf case for the Coriolis/Coloquinte placer.

## Principle

The wirelength minimization problem can be expressed as the dual of a minimum-cost-flow problem, where new constraints are equivalent to added new edges to the graph.
Amaranth solves the problem like a branch-and-bound ILP solver with dual simplex: at each node, it branches on the placement constraint between two rectangles (left/right/below/above) by adding an edge to the graph and updating the min-cost-flow solution.

## Experiments

At the time, I compared it against several ILP formulations for standard cell placement.
I used CBC and GLPK; although it did better than all of them, it was worse than published results with Gurobi/CPLEX.
However, it can handle arbitrary rectangles, while those algorithms need to use a grid with rectangles of small integer sizes.

## License

The code and the ideas are placed in the public domain - although I'd be interested to know if someone uses either!
