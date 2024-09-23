// Copyright © 2021 Giorgio Audrito. All Rights Reserved.

/**
 * @file exercises.cpp
 * @brief Quick-start aggregate computing exercises.
 */

// [INTRODUCTION]
//! Importing the FCPP library.
#include "lib/fcpp.hpp"

/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

//! @brief Dummy ordering between positions (allows positions to be used as secondary keys in ordered tuples).
template <size_t n>
bool operator<(vec<n> const&, vec<n> const&) {
    return false;
}

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {

//! @brief Tags used in the node storage.
namespace tags {
    //! @brief Color of the current node.
    struct node_color {};
    //! @brief Size of the current node.
    struct node_size {};
    //! @brief Shape of the current node.
    struct node_shape {};
    //! @brief Number of neighbours of the current node.
    struct node_nbr {};
    //! @brief Maximum number of neighbours of the current node.
    struct node_max_nbr {};
    //! @brief Maximum number of neighbours of all nodes.
    struct node_max_nbr_abs {};

    // ... add more as needed, here and in the tuple_store<...> option below
}

//! @brief The maximum communication range between nodes.
constexpr size_t communication_range = 100;

// [AGGREGATE PROGRAM]

/**
 * BASE EXERCISES:
 *
 * Expand the MAIN function below to compute the following:
 *
 * 1)    The number of neighbour devices.
 *
 * 2)    The maximum number of neighbour devices ever witnessed by the current device.
 *
 * 3)    The maximum number of neighbour devices ever witnessed by any device in the network.
 *
 * 4)    Move towards the neighbour with the lowest number of neighbours.
 *
 * Every exercise above is designed to help solving the following one.
 *
 *
 * SIMULATION PHYSICS:
 *
 * 5)    Move away from the neighbour with the highest number of neighbours.
 *
 * 6)    Move as if the device was attracted by the neighbour with the lowest number of neighbours,
 *       and repulsed by the neighbour with the highest number of neighbours.
 *
 * 7)    Move as if the device was repulsed by every neighbour, and by the four walls of the
 *       rectangular box between points [0,0] and [500,500].
 *
 *
 * COMBINING SPATIAL COMPUTING BLOCKS:
 *
 * 8)   Select a node called "source", chosen by finding the node with minimum uid 
 *      in the network, assuming that the diameter of the network is no more than 10 hops.
 *
 * 9)   Compute the distances between any node and the "source" using the adaptive bellman-ford algorithm.
 *
 * 10)  Calculate in the source an estimate of the true diameter of the network
 *      (the maximum distance of a device in the network).
 *
 * 11)  Broadcast the diameter to every node in the network.
 *
 *
 * RUNTIME MONITORING:
 * 
 * Given that:
 * - the node(s) identified as "source" in exercise (8) are Internet Gateways (gateway),
 * - a node is at risk of disconnection (disrisk) iff it has less than three neighbours,
 * monitor the following properties:
 * 
 * 12)  You (the current device) have never been at disrisk.
 * 
 * 13)  In the network, there exists a node that has never been at disrisk.
 * 
 * 14)  You (the current device) can always reach a gateway through nodes that are not at disrisk.
 * 
 * 15)  You (the current device) can always reach a gateway through nodes that have never been at disrisk.
 * 
 * In order to check whether what you computed is correct, you may display the computed
 * quantities as node qualities through tags `node_color`, `node_size` and `node_shape`.
 * You can also save your computed quantities in additional specific node attributes:
 * towards this end, you should both add a tag in namespace tags above, then list it
 * (together with the corresponding data type) in the `tuple_store` option below.
 *
 * HINTS:
 *
 * -    In the first few exercises, start by reasoning on when/where to use `nbr` (collecting from
 *      neighbours) and `old` (collecting from the past).
 *
 * -    In order to move a device, you need to set a velocity vector through something like
 *      `node.velocity() = make_vec(0,0)`.
 *
 * -    Coordinates are available through `node.position()`. Coordinates can be composed as physical
 *      vectors: `[1,3] + [2,-1] == [3,2]`, `[2,4] * 0.5 == [1,2]`.
 *
 * -    In the simulation physics exercises, you can model attraction/repulsion using the classical inverse square law.
 *      More precisely, if `v` is the vector between two objects, the resulting force is `v / |v|^3` where
 *      `|v| = sqrt(v_x^2 + v_y^2)`. In FCPP, `norm(v)` is available for computing `|v|`.
 *
 * -    FCPP provides some built-in APIs, like "diameter_election" and "abf_distance". 
 *      Refer to the documentation: https://fcpp-doc.surge.sh
 */

/*
 * @brief example function for checking a property. 
 * Sample property: you (the current device) have not been at disrisk for a couple of rounds.
 */
FUN bool recent_dis_monitor(ARGS, bool disrisk) { CODE
    using namespace logic;
    bool prev_disrisk = Y(CALL, disrisk);
    return !disrisk & !prev_disrisk;
}
FUN_EXPORT monitor_t = export_list<past_ctl_t, slcs_t>;


// @brief Main function.
MAIN() {
    // import tag names in the local scope.

    using namespace tags;

    // 1. compute the number of neighbours
    int num_neighbours = fold_hood(CALL, [&](int a, int b) { return a + b; }, nbr(CALL, 1));

    // 2. compute the maximum number of neighbours
    int max_neighbours_local = old(CALL, 0, [&](int prev_max) {
        return prev_max < num_neighbours ? num_neighbours : prev_max;
    });

    // 3. compute the maximum number of neighbours of any node
    int max_neighbours_global = nbr(CALL, max_neighbours_local, [&](field<int> x) {
        return max(max_hood(CALL, x), max_neighbours_local);
    });


    // 4. move to the node with less neighbours
    tuple<int, vec<2>> curr_tuple = make_tuple(num_neighbours, node.position());
    field<tuple<int, vec<2>>> nbr_tuples = nbr(CALL, curr_tuple);

    vec<2> min_nbr_pos = get<1>(min_hood(CALL, nbr_tuples));

    node.velocity() = (min_nbr_pos - node.position()) / 10;


    /*
    // 5. move away from the neighbour with the highest number of neighbours.
    tuple<int, vec<2>> curr_tuple = make_tuple(num_neighbours, node.position());
    field<tuple<int, vec<2>>> nbr_tuples = nbr(CALL, curr_tuple);

    vec<2> max_nbr_pos = get<1>(max_hood(CALL, nbr_tuples));

    node.velocity() = (node.position() - max_nbr_pos) / 10;
    */

    /*
    // 6. move as if the device was attracted by the neighbour with the lowest number of neighbours,
    // and repulsed by the neighbour with the highest number of neighbours.

    tuple<int, vec<2>> curr_tuple = make_tuple(num_neighbours, node.position());
    field<tuple<int, vec<2>>> nbr_tuples = nbr(CALL, curr_tuple);

    vec<2> min_nbr_pos = get<1>(min_hood(CALL, nbr_tuples));
    vec<2> max_nbr_pos = get<1>(max_hood(CALL, nbr_tuples));

    vec<2> attr = (min_nbr_pos - node.position());
    vec<2> rep = (node.position() - max_nbr_pos);

    vec<2> n_attr = norm(attr) != 0 ? attr / pow(norm(attr), 3) : vec<2>{0, 0};

    vec<2> n_rep = norm(rep) != 0 ? rep / pow(norm(rep), 3) : vec<2>{0, 0};

    vec<2> tot = n_attr + n_rep;

    node.propulsion() = tot;
    */

    /*
    // 6. with velocity instead of propulsion

    tuple<int, vec<2>> curr_tuple = make_tuple(num_neighbours, node.position());
    field<tuple<int, vec<2>>> nbr_tuples = nbr(CALL, curr_tuple);

    vec<2> min_nbr_pos = get<1>(min_hood(CALL, nbr_tuples));
    vec<2> max_nbr_pos = get<1>(max_hood(CALL, nbr_tuples));

    vec<2> attr = (min_nbr_pos - node.position());
    vec<2> rep = (max_nbr_pos - node.position());

    int x_tot = attr[0] - rep[0];
    int y_tot = attr[1] - rep[1];

    vec<2> tot = make_vec(x_tot, y_tot); 

    node.velocity() = tot / 10;
    */

    /*
    // 7. move as if the device was repulsed by every neighbour, and by the four walls of the
    // rectangular box between points [0,0] and [500,500].
    
    vec<2> curr_pos = node.position();
    vec<2> nbr_force = sum_hood(CALL, map_hood([](vec<2> nbr_dist) {
        return norm(nbr_dist) != 0 ? nbr_dist / pow(norm(nbr_dist), 3) : make_vec(0,0);
    }, node.nbr_vec()), vec<2>{});

    vec<2> wall_force = make_vec(0, 0);

    vec<2> down_wall = make_vec(curr_pos[0], 0);
    vec<2> up_wall = make_vec(curr_pos[0], 500);
    vec<2> left_wall = make_vec(0, curr_pos[1]);
    vec<2> right_wall = make_vec(500, curr_pos[1]);

    vec<2> down_arrow = curr_pos - down_wall;
    vec<2> up_arrow = curr_pos - up_wall;
    vec<2> left_arrow = curr_pos - left_wall;
    vec<2> right_arrow = curr_pos - right_wall;

    wall_force += (down_arrow / pow(norm(down_arrow), 3));
    wall_force += (up_arrow / pow(norm(up_arrow), 3));
    wall_force += (left_arrow / pow(norm(left_arrow), 3));
    wall_force += (right_arrow / pow(norm(right_arrow), 3));

    node.propulsion() = nbr_force + wall_force;
    */

    // usage of node storage
    node.storage(node_size{}) = 10;
    node.storage(node_color{}) = color(PURPLE);
    node.storage(node_shape{}) = shape::sphere;
    node.storage(node_nbr{}) = num_neighbours;
    node.storage(node_max_nbr{}) = max_neighbours_local;
    node.storage(node_max_nbr_abs{}) = max_neighbours_global;

}
//! @brief Export types used by the main function (update it when expanding the program).
FUN_EXPORT main_t = export_list<double, int, fcpp::vec<2> ,fcpp::tuple<int, fcpp::vec<2>>, monitor_t>;

} // namespace coordination

// [SYSTEM SETUP]

//! @brief Namespace for component options.
namespace option {

//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;

//! @brief Number of people in the area.
constexpr int node_num = 100;
//! @brief Dimensionality of the space.
constexpr size_t dim = 2;

//! @brief Description of the round schedule.
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1>,    // uniform time in the [0,1] interval for start
    distribution::weibull_n<times_t, 10, 1, 10> // weibull-distributed time for interval (10/10=1 mean, 1/10=0.1 deviation)
>;
//! @brief The sequence of network snapshots (one every simulated second).
using log_s = sequence::periodic_n<1, 0, 1>;
//! @brief The sequence of node generation events (node_num devices all generated at time 0).
using spawn_s = sequence::multiple_n<node_num, 0>;
//! @brief The distribution of initial node positions (random in a 500x500 square).
using rectangle_d = distribution::rect_n<1, 0, 0, 500, 500>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    node_color,                 color,
    node_size,                  double,
    node_shape,                 shape,
    node_nbr,                   int,
    node_max_nbr,               int,
    node_max_nbr_abs,           int
>;
//! @brief The tags and corresponding aggregators to be logged (change as needed).
using aggregator_t = aggregators<
    node_size,                  aggregator::mean<double>
>;

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<true>,      // multithreading enabled on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN above)
    exports<coordination::main_t>, // export type list (types used in messages)
    retain<metric::retain<2,1>>,   // messages are kept for 2 seconds before expiring
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<log_s>,     // the sequence generator for log events on the network
    spawn_schedule<spawn_s>, // the sequence generator of node creation events on the network
    store_t,       // the contents of the node storage
    aggregator_t,  // the tags and corresponding aggregators to be logged
    init<
        x,      rectangle_d // initialise position randomly in a rectangle for new nodes
    >,
    dimension<dim>, // dimensionality of the space
    connector<connect::fixed<100, 1, dim>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size  of a node is read from this tag in the store
    color_tag<node_color>  // the color of a node is read from this tag in the store
);

} // namespace option

} // namespace fcpp


//! @brief The main function.
int main() {
    using namespace fcpp;

    //! @brief The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;
    //! @brief The initialisation values (simulation name).
    auto init_v = common::make_tagged_tuple<option::name>("Exercises");
    //! @brief Construct the network object.
    net_t network{init_v};
    //! @brief Run the simulation until exit.
    network.run();
    return 0;
}
