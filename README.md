# hector_waypoint_order

This package implements functionality to compute the optimal waypoint order e.g. by solving the TSP, and computing the costs required for the waypoint order computation.

<br>

## Cost computer

For computing the waypoint order costs between the waypoints are required. Therefore, a base class
called `CostComputerBase` is implemented. Subclasses provide different ways of computing costs and are implemented as
plugins.

But costs can also be provided by external software, since the `WaypointOrderComputerBase` class takes a `CostMap` as
argument.

### Cost computer plugins

The following cost computer plugins are implemented:

| Plugin              | Description                                                                                                                                                                                                                                                                                                                                      |
|---------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `PathCostComputer`  | Computes paths between each pair of waypoints. Therefore, different path planners can be used which must be implemented as subclass of the class `PathPlannerBase`. The path planner to use is specified in the parameter `path_planner_plugin`. Here, either the path planner itself provides costs or the length of the path is used as costs. |


<br>

## Waypoint order computer

The waypoint order computer is also implemented as a plugin system. The base class is called `WaypointOrderComputerBase`.

One way of computing the waypoint order is by solving the Travelling Salesman Problem (TSP). Here, the goal is to find
the shortest path that visits each waypoint once and returns to the start point.

But also other ways of computing the waypoint order can be implemented as plugins, e.g. if points should be visited multiple
times or a start and end point is given and only the order for the intermediate points should be computed.

The waypoint order computer base class provides a parameter called `publish_path`. If it is true, the computed paths or
intermediate results are published on the topic `waypoint_order_computer_path` (depending on the used waypoint order computer plugin!).

### Waypoint order computer plugins: TSP solvers

| Plugin                                 | Description                                                                                                                                                                                                                                                         |
|----------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `BruteForceTspSolver`                  | Tests each possible permutation and stores best. Only usable for small problems with few waypoints, since there are (n-1)! permutations for n waypoints.                                                                                                            |
| `GreedyTspSolver`                      | Starting with a randomly selected point, a list with all unvisited viewpoints is held and always the one with the lowest costs to the current waypoint is chosen as next waypoint.                                                                                  |
| `MstTspSolver`                         | Solves the TSP by generating a minimum spanning tree (MST) and traversing it. Each time, a node in the tree is visited first, it is added to the path. Uses the [Boost Graph Library](https://www.boost.org/doc/libs/1_71_0/libs/graph/doc/metric_tsp_approx.html). |
| `SimulatedAnnealingTspSolver`          | Uses the Simulated annealing (SA) algorithm to solve the TSP. A random solution is used as initial solution, which can also be overwritten. More details about the parameters are [here](#SAparams).                                                                |
| `GreedyAndSimulatedAnnealingTspSolver` | Solves the TSP using the `SimulatedAnnealingTspSolver` with the solution from the `GeedyTspSolver` as initial solution.                                                                                                                                             |
| `MstAndSimulatedAnnealingTspSolver`    | Solves the TSP using the `SimulatedAnnealingTspSolver` with the solution from the `MstTspSolver` as initial solution.                                                                                                                                               |


#### <a name="SAparams"></a> Simulated annealing parameters

| Name                  | Description                                                                                                            | Default value |
|-----------------------|------------------------------------------------------------------------------------------------------------------------|---------------|
| `initial_temperature` | Initial temperature for SA algorithm. Needs to be > 0.                                                                 | 5             |
| `cooling_rate`        | In range (0,1). Temperature is adapted in each iteration by the factor given in the cooling_rate                       | 0.99          |
| `use_best_mutator`    | Always use the mutator which currently generates the best neighbor. If false, the mutator is chosen probabilistically. | true          |