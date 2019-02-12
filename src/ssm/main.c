#include <dirent.h>
#include <signal.h>

#include <lcm/lcm.h>
#include "lcmtypes/global_world_state_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/grid_map_t.h"

#include "common/doubles.h"
#include "common/floats.h"
#include "common/time_util.h"
#include "common/http_advertiser.h"
#include "common/config.h"
#include "common/rand_util.h"
#include "common/config_util.h"
#include "common/getopt.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "common/global_map.h"
#include "common/stype_lcm.h"
#include "common/rand_util.h"
#include "common/zhash.h"
#include "common/workerpool.h"

#include "vx/vx.h"
#include "vx/webvx.h"

#include "april_graph/april_graph.h"
#include "april_graph/dijkstra_xyt.h"

#include "kdtree.h"


typedef struct state state_t;
struct state {
    april_graph_t *graph;
    kdtree_t **kdtrees;
    zarray_t **points;
    zarray_t *selected_nodes;
};


static zarray_t *laser_to_floats(const laser_t *laser, double xyt[3])
{
    zarray_t *points = zarray_create(sizeof(double[2]));

    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + i * laser->radstep;
        double r = laser->ranges[i];
        if (r < 0 || r > 40)
            continue;

        double xy[2] = { r*cos(theta), r*sin(theta) };
        double res[2];
        doubles_xyt_transform_xy(xyt, xy, res);
        zarray_add(points, res);
    }

    return points;
}

// cal observation prob p(z | x, s) = p(q_1|x,s) * ... p(q_n|x,s)
double cal_log_p(state_t *state, int k, int i)
{
    kdtree_t *kd = state->kdtrees[k];
    double dist = 0;
    for(int pIdx = 0; pIdx < zarray_size(state->points[i]); pIdx++) {
        double xy[2];
        zarray_get(state->points[i], pIdx, xy);
        kdres_t *set = kd_nearest(kd, xy);
        //printf( "found %d results:\n", kd_res_size(set) );
        double pos[2];
        while(!kd_res_end(set)) {
            kd_res_item(set, pos);
            //printf("%d,%f,%f\n",*id, pos[0], pos[1]);
            kd_res_next(set);
        }
        dist += doubles_distance(pos, xy, 2);
        kd_res_free(set);
    }
    return dist;
}

double cal_log_likelihood(state_t *state, int sIdx)
{
    april_graph_node_t *na, *nb;
    double total_log_p = 0;
    for(int i = 0; i < state->graph->nodes->size; i++) {
        int selectedNodeIdx = -1;
        double dMin = DBL_MAX;
        zarray_get(state->graph->nodes, i, &na);
        // so called approximate sensor model. Just find the closest one. Haha
        for(int j = 0; j < state->selected_nodes->size; j++) {
            int id;
            zarray_get(state->selected_nodes, j, &id);
            zarray_get(state->graph->nodes, id, &nb);
            double dist = doubles_distance(na->state, nb->state, 2);
            //printf("dist+id:%f,%d\n",dist, id);
            if(dist < dMin) {
                dMin = dist;
                selectedNodeIdx = id;
            }
        }
        {
            zarray_get(state->graph->nodes, sIdx, &nb);
            double dist = doubles_distance(na->state, nb->state, 2);
            if(dist < dMin) {
                dMin = dist;
                selectedNodeIdx = sIdx;
            }
        }
        // p_i = \eta * e^(-log_p^2 / sigma); P = \prod_i{p_i}
        //double log_p = cal_log_p(state, selectedNodeIdx, i);
        //total_log_p += log_p;
        //printf("selectedNodeIdx:%d->%d,%f\n", i,selectedNodeIdx,log_p);
        total_log_p += cal_log_p(state, selectedNodeIdx, i);
    }
    return total_log_p;
}
void maxlikelihood_search(state_t *state, int max_N)
{
    int N = state->graph->nodes->size;

    FILE *f = fopen("/tmp/maxlikelihood.list", "w");
    if(f == NULL) {
        printf("Wrong bootstrap file \n");
        exit(-1);
    }
    while(state->selected_nodes->size < max_N) {
        double P_max = DBL_MAX;
        int selectId = -1;
        for(int i = 0; i < N; i++) {
            if(zarray_contains(state->selected_nodes, &i)) {
                continue;
            }
            printf(" Test : %d \n", i);
            double p = cal_log_likelihood(state, i);
            //printf("Total log p:%d->%f \n", i, p);
            if(p < P_max) {
                P_max = p;
                selectId = i;
            }
        }
        printf("SelectIdx: %d\n",selectId);
        fprintf(f, "%d ", selectId);
        fflush(f);
        zarray_add(state->selected_nodes, &selectId);
    }
    fprintf(f, "\n=\n");
    fclose(f);
}
int main(int argc, char *argv[])
{

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, 'g', "graph-file", "", "Specify the graph file to load");
    getopt_add_int(gopt, 'i', "maxnodes", "100", "maxnodes");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }

    state_t *state = calloc(1, sizeof(state_t));
    setlinebuf(stdout);
    setlinebuf(stderr);
    // Register types for serialization;
    // If you want to store information into graph, you need to register the type
    stype_register_basic_types();
    april_graph_stype_init();
    stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
    stype_register(STYPE_FROM_LCM("laser_t", laser_t));

    //Load graph
    state->graph = april_graph_create_from_file(getopt_get_string(gopt, "graph-file"));
    double a[2] = {0,0};
    double b[2] = {1,1};
    double c[2] = {2,2};
    int ia = 0;
    int ib = 1;
    int ic = 2;
    if(0) {
        kdtree_t *kdtree = kd_create(2);
        kd_insert(kdtree, a , &ia);
        kd_insert(kdtree, b , &ib);
        kd_insert(kdtree, c , &ic);
        double tp[2] = {1.3,1.3};
        kdres_t *set = kd_nearest(kdtree, tp);
        //kdres_t *set = kd_nearest_range(kdtree, tp, 10);
        printf( "found %d results:\n", kd_res_size(set) );
        while(!kd_res_end(set)) {
            double pos[2];
            int *id = (int*)kd_res_item(set, pos);
            printf("%d,%f,%f\n",*id, pos[0], pos[1]);
            kd_res_next(set);
        }
        kd_res_free(set);
    }
    if(state->graph == NULL) {
        printf("no graph loaded\n");
        exit(-1);
    }

    april_graph_t *g = state->graph;
    // HACK:
    //g->nodes->size = 2;
    state->points = calloc(zarray_size(g->nodes), sizeof(zarray_t*));
    state->kdtrees = calloc(zarray_size(g->nodes), sizeof(kdtree_t*));
    state->selected_nodes = zarray_create(sizeof(int));
    for (int i = 0; i < zarray_size(g->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(g->nodes, i, &node);
        //doubles_print(node->state, 3, "%f,");
        laser_t *laser = april_graph_node_attr_get(node, "laser");
        if (laser == NULL) {
            continue;
        } else {
            state->kdtrees[i] = kd_create(2);
            state->points[i] = laser_to_floats(laser, node->state);
            for(int k = 0; k < zarray_size(state->points[i]); k++) {
                double pos[2];
                zarray_get(state->points[i], k, pos);
                kd_insert(state->kdtrees[i], pos, NULL);
            }
        }
    }
    int MAX_NUM_OF_NODES = getopt_get_int(gopt, "maxnodes");
    MAX_NUM_OF_NODES = iclamp(MAX_NUM_OF_NODES, 0, zarray_size(state->graph->nodes));
    maxlikelihood_search(state, MAX_NUM_OF_NODES);
    for (int i = 0; i < zarray_size(state->graph->nodes); i++) {
        zarray_destroy(state->points[i]);
        kd_free(state->kdtrees[i]);
    }
    free(state->points);
    april_graph_destroy(state->graph);
    /* while(1) { */
    /*     usleep(100E3); */
    /* } */
    return 0;
}
