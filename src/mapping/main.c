/* Copyright (C) 2013-2019, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

/*$LICENSE*/

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

#include "vx/vx.h"
#include "vx/webvx.h"

#include "april_graph/april_graph.h"
#include "april_graph/dijkstra_xyt.h"

#include "scanmatch/scanmatch.h"

#include "loopval.h"

#define METER_PER_PIXEL 0.05
#define GRID_MAP_SIZE 768

#define MIN_DIST_CHANGE 0.2
#define MIN_HEADING_CHANGE 15

#define SENSOR_RANGE_CHI_M 4
#define FIND_NODE_THRESH 1.0  // In meters

// used to display maps most of the time. First index is the GRID_VAL_* above.
static float grid_val_rgba[4][4] = { { 0.0, 0.0, 0.0, 0.0 },
                                     { 0.0, 0.0, 0.0, 0.0 },
                                     { 0.0, 0.5, 0.5, 1.0 },
                                     { 0.0, 0.0, 0.0, 0.0 } };
static float grid_val_hover_rgba[4][4] = { { 0.0, 0.0, 0.0, 0.3 },
                                           { 1.0, 1.0, 0.0, 0.3 },
                                           { 1.0, 1.0, 0.0, 1.0 },
                                           { 1.0, 1.0, 0.0, 0.0 }};
static float grid_val_ref_rgba[4][4] = { { 0.0, 0.0, 0.0, 0.3 },
                                         { 0.0, 0.8, 1.0, 0.3 },
                                         { 0.0, 0.8, 1.0, 1.0 },
                                         { 0.0, 0.8, 1.0, 0.0 } };



typedef struct state state_t;
struct state {
    lcm_t *lcm;
    lcm_eventlog_t *eventlog_read;

    //VX
    vx_world_t *vw;
	webvx_t *webvx;
    vx_console_t *vx_console;

    pthread_mutex_t mutex;

    grid_map_t *local_gm;
    double mpp;

    global_map_t *compositor;

    april_graph_t *graph;
    loopval_t *loopval;

    int auto_close_next;
    int32_t ref_idx, query_idx;

    double last_local_xyt[3];
    bool first;
};

typedef struct composite_record composite_record_t;
struct composite_record
{
    const grid_map_t *gm;
    double state[3];
};

int find_node(state_t *state, double xy[2], double max_distance)
{
    int a = -1;
    double bestdist2 = max_distance*max_distance;
    for (int i = 0; i < zarray_size(state->graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(state->graph->nodes, i, &node);
        double dist2 = pow(xy[0] - node->state[0], 2) + pow(xy[1] - node->state[1], 2);
        if (dist2 < bestdist2) {
            bestdist2 = dist2;
            a = i;
        }
    }
    return a;
}


void read_laser_event(lcm_eventlog_event_t *event, laser_t *laser)
{
    assert(0 < laser_t_decode(event->data, 0, event->datalen, laser));
}

void read_pose_event(lcm_eventlog_event_t *event, pose_t *pose)
{
    assert(0 < pose_t_decode(event->data, 0, event->datalen, pose));
}

static void redraw_selected_node(april_graph_node_t *node,
                          int idx,
                          float rgbas[4][4],
                          vx_buffer_t *vb)
{
    vx_resource_t *tex = april_graph_node_attr_get(node, "maptex");
    grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");

    // Render observations from query node over map
    if (tex) {
        vx_object_t *vxo = vxo_image_tile(tex,
                                          rgbas[0], rgbas[1],
                                          rgbas[2], rgbas[3]);

        vx_buffer_add_back(vb,
                           vxo_matrix_xyt(node->state),
                           vxo_depth_test(0,
                                          vxo_matrix_translate(gm->x0, gm->y0, 0),
                                          vxo_matrix_scale(gm->meters_per_pixel),
                                          vxo,
                                          NULL),
                           NULL);
    }
    if (node->type == APRIL_GRAPH_NODE_XYT_TYPE) {
            vx_buffer_add_back(vb,
                               vxo_matrix_xyt(node->state),
                               vxo_depth_test(0,
                                              vxo_robot_solid(rgbas[GRID_VAL_SLAMMABLE]),
                                              vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                                       "<<#000000, sansserif-0.25>> %d", idx),
                                              NULL),
                               NULL);
    }
}

static void redraw_manual_poses(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "selected nodes");

    if (state->query_idx >= 0 && state->query_idx < zarray_size(state->graph->nodes)) {
        april_graph_node_t *node;
        zarray_get(state->graph->nodes, state->query_idx, &node);
        redraw_selected_node(node, state->query_idx, grid_val_hover_rgba, vb);
    }

    if (state->ref_idx >= 0 && state->ref_idx < zarray_size(state->graph->nodes)) {
        april_graph_node_t *node;
        zarray_get(state->graph->nodes, state->ref_idx, &node);
        redraw_selected_node(node, state->ref_idx, grid_val_ref_rgba, vb);
    }

    vx_buffer_swap(vb);
}

static int key_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;
    char key = ev->u.key.key_code;
    if(ev->type == VX_EVENT_KEY_DOWN) {
        pthread_mutex_lock(&state->mutex);
        switch (key) {
            case 37: //Left
                break;
            case 38: //Up
                break;
            case 39: //Right
                break;
            case 40: //Down
                break;
            case ' ':
                break;
            case 27: //ESC
                break;
            default:
                break;
        }
        pthread_mutex_unlock(&state->mutex);
    }
    if(ev->type == VX_EVENT_KEY_PRESSED) {
        if (key == 'j') {
            state->query_idx--;
            state->query_idx = iclamp(state->query_idx, 0, zarray_size(state->graph->nodes)-1);
            redraw_manual_poses(state);
        }
        else if(key == 'k') {
            state->query_idx++;
            state->query_idx = iclamp(state->query_idx, 0, zarray_size(state->graph->nodes)-1);
            redraw_manual_poses(state);
        } else if(key == 'J') {
            state->ref_idx--;
            state->ref_idx = iclamp(state->ref_idx, 0, zarray_size(state->graph->nodes)-1);
            redraw_manual_poses(state);
        } else if(key == 'K') {
            state->ref_idx++;
            state->ref_idx = iclamp(state->ref_idx, 0, zarray_size(state->graph->nodes)-1);
            redraw_manual_poses(state);
        }

    }

    return 0;
}

static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
	state_t *state = impl;
    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);
    double plane_xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, plane_xyz);
    int idx = find_node(state, plane_xyz, FIND_NODE_THRESH);

    // API MOUSE click sets the selected pose for scanmatching.
    if (ev->flags == 0) {
        if (idx != state->query_idx) {
            state->query_idx = idx;
            redraw_manual_poses(state);
        }
        return 1;
    }

    // API MOUSE shift-click sets the reference pose for scan-matching.
    if (ev->flags == VX_EVENT_FLAGS_SHIFT) {
        if (idx != state->ref_idx) {
            state->ref_idx = idx;
            redraw_manual_poses(state);
        }
        return 1;
    }

    return 0;
}

void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;

    printf("console: %s\n", cmd);
    zarray_t *toks = str_split(cmd, " ");

    if (zarray_size(toks) == 0)
        goto cleanup;

    char *tok;
    zarray_get(toks, 0, &tok);

    if (!strcmp(tok, "clear")) {
        vx_console_clear(vc);
        goto cleanup;
    }
    if (!strcmp(tok, "save-graph")) {
        pthread_mutex_lock(&state->mutex);
        april_graph_save(state->graph, "/tmp/saved.graph");
        pthread_mutex_unlock(&state->mutex);
        goto cleanup;
    }

    vx_console_printf(vc, "unknown command '%s'", cmd);

    goto cleanup;

  cleanup:
    zarray_vmap(toks, free);
    zarray_destroy(toks);

}

zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    //state_t *state = user;
    zarray_t *completions = zarray_create(sizeof(char*));

    const char *commands[] = { "clear",
                               "save-graph",
                               NULL };
    for (int idx = 0; commands[idx]; idx++) {
        if (str_starts_with(commands[idx], cmd)) {
            char *s = strdup(commands[idx]);
            zarray_add(completions, &s);
        }
    }

    return completions;
}

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
	state_t *state = user;
    if(ev->type == VX_EVENT_KEY_PRESSED || ev->type == VX_EVENT_KEY_DOWN) {
        return key_down(vl, ev, state);
    }
    if(ev->type == VX_EVENT_MOUSE_DOWN) {
        return mouse_down(vl, ev, state);
    }
	return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
	state_t *state = impl;
	vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
	vx_layer_set_world(vl, state->vw);
	vx_layer_add_event_handler(vl, on_event, 0, state);
    vx_console_setup_layer(state->vx_console, vl);
    int order = 0;
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "console"), ++order);

}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
	printf("ON DESTROY CANVAS\n");
}

/*
static zarray_t *laser_to_floats(state_t *state, const laser_t *laser)
{
    zarray_t *points = zarray_create(sizeof(float[2]));

    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + i * laser->radstep;
        double r = laser->ranges[i];
        if (r < 0 || r > 40)
            continue;

        float xy[2] = { r*cos(theta), r*sin(theta) };
        zarray_add(points, xy);
    }

    return points;
}
*/

static void generate_local_grid_map(grid_map_t *gm, laser_t *laser)
{
    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + i * laser->radstep;
        double r = laser->ranges[i];
        if (r < 0)
            continue;

        float xy[2] = { r*cos(theta), r*sin(theta) };
        // Skip points that are out of the map
        int ix = gridmap_get_index_x(gm, xy[0]);
        int iy = gridmap_get_index_y(gm, xy[1]);
        if (ix < 0 || iy < 0) {
            continue;
        }
        gm->data[iy*gm->width + ix] = (uint8_t)GRID_VAL_SLAMMABLE;
    }
}

static void reset_grid_map(grid_map_t *gm)
{
    gm->x0 = -gm->width *  gm->meters_per_pixel / 2.0;
    gm->y0 = -gm->height * gm->meters_per_pixel / 2.0;
    memset(gm->data, GRID_VAL_UNKNOWN, gm->datalen);
}

static zarray_t *tile_grid_map(const grid_map_t *ogm, uint32_t tile_size, uint8_t default_fill)
{
    double MPP = ogm->meters_per_pixel;

    zarray_t *gms = zarray_create(sizeof(grid_map_t*));

    for (uint32_t y = 0; y < ogm->height; y+=tile_size) {
        for (uint32_t x = 0; x < ogm->width; x+=tile_size) {
            double x0 = ogm->x0 + x*MPP;
            double y0 = ogm->y0 + y*MPP;
            grid_map_t *gm = gridmap_make_pixels(x0, y0,
                                                 tile_size, tile_size,
                                                 MPP,
                                                 default_fill,
                                                 0);

            uint32_t copy_width = min(tile_size, ogm->width - x);
            uint32_t copy_height = min(tile_size, ogm->height - y);

            // Copy to new tile.
            for (int iy = 0; iy < copy_height; iy++) {
                int idx0 = iy*gm->width;
                int idx1 = (y+iy)*ogm->width + x;
                memcpy(&gm->data[idx0],
                       &ogm->data[idx1],
                       copy_width);
            }

            zarray_add(gms, &gm);
        }
    }

    return gms;
}

static void redraw_global_map(state_t *state)
{
    global_map_crop(state->compositor);
    const grid_map_t *ogm = global_map_get_map(state->compositor);

    if (ogm->width && ogm->height) {
        zarray_t *gms = tile_grid_map(ogm, 1024, GRID_VAL_UNKNOWN);
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "composite");
        for (int i = 0; i < zarray_size(gms); i++) {
            grid_map_t *gm = NULL;
            zarray_get(gms, i, &gm);

            image_u8_t im = { .width = gm->width,
                              .height = gm->height,
                              .stride = gm->width,
                              .buf = gm->data
            };

            vx_buffer_add_back(vb,
                               vxo_matrix_translate(gm->x0, gm->y0, 0),
                               vxo_matrix_scale(gm->meters_per_pixel),
                               vxo_depth_test(0,
                                              vxo_image_tile(vx_resource_make_texture_u8_copy(&im, 0),
                                                             grid_val_rgba[0], grid_val_rgba[1],
                                                             grid_val_rgba[2], grid_val_rgba[3]),
                                              NULL),
                               NULL);
        }
        zarray_vmap(gms, grid_map_t_destroy);
        zarray_destroy(gms);
        vx_buffer_swap(vb);
    }
}

static void redraw_graph(state_t *state)
{
    if (1) {
        april_graph_t *g = state->graph;

        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "graph");

        for (int i = 0; i < zarray_size(g->nodes); i++) {
            april_graph_node_t *node;
            zarray_get(g->nodes, i, &node);
            vx_buffer_add_back(vb,
                               vxo_matrix_xyt(node->state),
                               vxo_robot_solid(vx_cyan),
                               NULL);
        }
        vx_buffer_swap(vb);
    }
}

void optimize_cholesky(state_t *state)
{
    struct april_graph_cholesky_param chol_param;
    april_graph_cholesky_param_init(&chol_param);
    chol_param.tikhanov = 1.0E-6;
    april_graph_cholesky(state->graph, &chol_param);
}

void add_model_data_attr(april_graph_node_t *node)
{
    if (!april_graph_node_attr_get(node, "model_data")) {
        // convert grid_map SLAMMABLEs into a binary grid
        grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
        image_u8_t *im = image_u8_create(gm->width, gm->height);
        for (int y = 0; y < gm->height; y++) {
            for (int x = 0; x < gm->width; x++) {
                uint8_t v = gm->data[y*gm->width + x];
                if (v == GRID_VAL_SLAMMABLE)
                    v = 255;
                else
                    v = 0;
                im->buf[y*im->stride + x] = v;
            }
        }

        // blur it (optional)
        int ksz = 7;
        double sigma = 1.5;
        if (0) {
            uint8_t k[255] = {0};
            for (int i = 0; i < ksz; i++) {
                int x = -ksz/2 + i;
                k[i] = (double) 255.*exp(-.5*sq((double) x / sigma));
            }
            image_u8_convolve_2D(im, k, ksz);
            for (int i = 0; i < ksz; i++)
                printf("%d %5d\n", i, k[i]);
        }
        if (1) {
            image_u8_gaussian_blur(im, sigma, ksz);
        }

        sm_model_data_t *model_data = sm_model_data_create(im,
                                                           gm->x0 / gm->meters_per_pixel,
                                                           gm->y0 / gm->meters_per_pixel,
                                                           gm->meters_per_pixel,
                                                           12);

        april_graph_node_attr_put(node, NULL, "model_data", model_data);
    }
}

void add_points_data_attr(april_graph_node_t *node)
{
    if (!april_graph_node_attr_get(node, "points_data")) {
        // convert the query scan into points (as observed from the robot location)
        zarray_t *pts = zarray_create(sizeof(float[2]));
        if (1) {
            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
            assert (gm);

            for (int y = 1; y + 1 < gm->height; y++) {
                for (int x = 1; x + 1 < gm->width; x++) {
                    int acc = 0;
                    if (0) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                int yy = y + dy;
                                int xx = x + dx;
                                if (gm->data[yy*gm->width + xx] == GRID_VAL_SLAMMABLE)
                                    acc ++;
                            }
                        }
                        if (acc >= 1) {
                            zarray_add(pts, (float[]) {
                                    gm->x0 + (x + 0.5)*gm->meters_per_pixel,
                                        gm->y0 + (y + 0.5)*gm->meters_per_pixel });
                        }
                    } else {
                        if (gm->data[y*gm->width + x] == GRID_VAL_SLAMMABLE)
                            zarray_add(pts, (float[]) {
                                    gm->x0 + (x + 0.5)*gm->meters_per_pixel,
                                        gm->y0 + (y + 0.5)*gm->meters_per_pixel });

                    }
                }
            }
        }

        sm_points_data_t *points_data = sm_points_data_create(pts);
        april_graph_node_attr_put(node, NULL, "points_data", points_data);
    }

}

static double sensorRangeChi(const double xyt[3], const double cov[9], double r)
{
    double R = pow(r, 2);

    // construct covariance ellipse
    double C00 = cov[0] + R, C01 = cov[1], C11 = cov[4] + R;

    // compute inverse
    double det = C00*C11 - C01*C01;
    double D00 = C11 / det, D01 = -C01 / det, D11 = C00 / det;

    double chi2 = D00*xyt[0]*xyt[0] + D01*xyt[0]*xyt[1] + D11*xyt[1]*xyt[1];

    return chi2;
}

void add_scanmatch(state_t *state, int ref_idx, int query_idx,
                   float fpred_inv[9],
                   double trange, double radrange, double radstep,
                   sm_search_t * sm)
{

    april_graph_node_t *refnode, *querynode;
    zarray_get(state->graph->nodes, ref_idx, &refnode);
    zarray_get(state->graph->nodes, query_idx, &querynode);

    // XXX We may revisit how these attrs are used. Hot cache?
    sm_points_data_t *points_data = april_graph_node_attr_get(querynode, "points_data");
    if (!points_data)
        return;
    sm_model_data_t *model_data = april_graph_node_attr_get(refnode, "model_data");
    if (!model_data)
        return;

    model_data->uid = ref_idx;
    points_data->uid = query_idx;

    double pred_xyt[3];
    doubles_xyt_inv_mul(refnode->state, querynode->state, pred_xyt);

    //double pdistance = sqrt(sq(pred_xyt[0]) + sq(pred_xyt[1]));
    //// if outside plausible sensor range, then we're probably completely lost.
    //if (pdistance > 15) {
    //    displayf(state, "manual match - null prior (dist %f)", pdistance);
    //    memset(pred_xyt, 0, sizeof(pred_xyt));
    //}

    double scale = 1.0 / zarray_size(points_data->points);
    double impp = 1.0 / model_data->meters_per_pixel;
    float minscore_before_penalty = 20;

    float fpred_xyt[3];
    for (int i = 0; i < 3; i++)
        fpred_xyt[i] = pred_xyt[i];

    sm_search_add(sm, points_data, model_data,
                  (fpred_xyt[0] - trange)*impp,  (fpred_xyt[0] + trange)*impp,
                  (fpred_xyt[1] - trange)*impp,  (fpred_xyt[1] + trange)*impp,
                  fpred_xyt[2] - radrange, fpred_xyt[2] + radrange, radstep,
                  scale, fpred_xyt, fpred_inv, minscore_before_penalty);
}

/** Construct a laser odometry factor based on a scanmatch search. Includes
 *  a hillclimbing step to further optimize the results from the search, if
 *  possible.
 *
 *  Returns NULL if no match is found.
 **/
april_graph_factor_t *extract_match(state_t * state, sm_search_t *sm)
{
    sm_result_t *sres = sm_search_run(sm);

    if (!sres)
        return NULL;

    sm_hillclimb_params_t hcparams = { .maxiters = 1000,
                                       .initial_step_sizes = { sres->handle->model_data->meters_per_pixel / 2,
                                                               sres->handle->model_data->meters_per_pixel / 2,
                                                               0.5 * M_PI / 180 },
                                       .step_size_shrink_factor = 0.5, // was .5
                                       .max_step_size_shrinks = 8 }; // was 8

    double scale = 1.0 / zarray_size(sres->handle->points_data->points);

    sm_hillclimb_result_t *hres = sm_hillclimb(sres->handle->points_data,
                                               sres->handle->model_data, sres->xyt, &hcparams,
                                               scale, sres->handle->mean3, sres->handle->inf33);


    // Laser odometry information weights
    matd_t *W = matd_identity(3);
    MATD_EL(W, 0, 0) = 1.0 / pow(0.01, 2);
    MATD_EL(W, 1, 1) = 1.0 / pow(0.01, 2);
    MATD_EL(W, 2, 2) = 1.0 / pow(0.01*M_PI / 180.0, 2);

    //sm_search_remove(sm, sres->handle);

    april_graph_factor_t * factor = april_graph_factor_xyt_create(sres->handle->model_data->uid,
                                                                  sres->handle->points_data->uid,
                                                                  hres->xyt,
                                                                  NULL,
                                                                  W);
    matd_destroy(W);
    sm_hillclimb_result_destroy(hres);

    return factor;
}

void close_loop(state_t *state, int query_idx)
{
    april_graph_node_t *querynode; // "a"
    zarray_get(state->graph->nodes, query_idx, &querynode);

    sm_points_data_t *points_data = april_graph_node_attr_get(querynode, "points_data");
    if (!points_data)
        return;
    assert (points_data);
    assert (april_graph_node_attr_get(querynode, "grid_map"));

    dijkstra_xyt_t *dijkstra = dijkstra_xyt_create(state->graph, query_idx);

    zarray_t *candidaterefs = zarray_create(sizeof(int));
    for (int ref_idx = 0; ref_idx < query_idx; ref_idx++) {
        dijkstra_xyt_edge_t *pred = dijkstra->edges[ref_idx];
        if (!pred)
            continue;

        if (sensorRangeChi(pred->xyt, pred->cov, SENSOR_RANGE_CHI_M) < 1) {
            zarray_add(candidaterefs, &ref_idx);
        }
    }
    dijkstra_xyt_destroy(dijkstra);

    // XXX I have some qualms about what we're doing, here. I think we're trying
    // to avoid paying too much for scanmatching costs. It seems preferable to
    // speed this up some other way that doesn't throw away data. In particular
    // this is effectively a random discarding of data, with no taking into account
    // which matches might be important.
    // Here, we only match against a subset of the possible number of matches.
    sm_search_t *sm = sm_search_create();
    int candidaterefs_idx_increment = iclamp(zarray_size(candidaterefs) / 5, 2, INT32_MAX);
    if (zarray_size(candidaterefs) <= 3)
        candidaterefs_idx_increment = 1;

    int num_added = 0;
    for (int candidaterefs_idx = 0; candidaterefs_idx < zarray_size(candidaterefs); candidaterefs_idx += candidaterefs_idx_increment) {
        int ref_idx;
        zarray_get(candidaterefs, candidaterefs_idx, &ref_idx);

        float fpred_inv[9] = { 20, 0, 0,
                                0, 20, 0,
                                0, 0, 5 };

        add_scanmatch(state, ref_idx, query_idx,
                      fpred_inv, 5.0, // translation range (m)
                      20 * M_PI / 180.0, // rotation range(rad)
                      2 * M_PI / 180.0, // rotation step size (rad)
                      sm);

        num_added++;
    }

    int added = 0;
    april_graph_factor_t * factor1;
    double perc_kept = 0.5;
    while (added < max(perc_kept*num_added, 3) && (factor1 = extract_match(state, sm))) {
        added++;
        matd_t *W2 = matd_op("0.01*M", factor1->u.common.W);
        april_graph_factor_t *factor2 = april_graph_factor_xyt_create(factor1->nodes[0], factor1->nodes[1], factor1->u.common.z, NULL, W2);

        april_graph_factor_t *maxfactor = april_graph_factor_max_create(
                                                                        (april_graph_factor_t*[]) { factor1, factor2 },
                                                                        (double[]) { 1, 5 },
                                                                        2);

        april_graph_factor_attr_put(maxfactor, stype_get("string"), "type", strdup("auto"));
        matd_destroy(W2);

        loopval_add(state->loopval, factor1->nodes[0], factor1->nodes[1], factor1->u.common.z, maxfactor, 0);

        april_graph_factor_t *f;
        while ((f = loopval_get_graduate(state->loopval)) != NULL) {
            zarray_add(state->graph->factors, &f);
        }
    }
    sm_points_data_clear_cache(points_data);
    sm_search_destroy(sm);
    optimize_cholesky(state);
}

void update_compositor(state_t *state)
{
    const double comp_thresh_xy = 0.1;
    const double comp_thresh_theta = to_radians(1); // XXX Can make this depend on an range

    zarray_t *records = zarray_create(sizeof(composite_record_t));
    int sz = zarray_size(state->graph->nodes);
    composite_record_t rec;
    for (int i = 0; i < sz; i++) {
        april_graph_node_t *node = NULL;
        zarray_get(state->graph->nodes, i, &node);
        if (node) {
            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
            if (gm) {
                rec.gm = gm;
                memcpy(rec.state, node->state, 3*sizeof(double));
                zarray_add(records, &rec);
            }
        }
    }

    for (int i = 0; i < zarray_size(records); i++) {
        zarray_get(records, i, &rec);
        double prev_xyt[3];
        global_map_get_xyt(state->compositor, rec.gm, prev_xyt);

        double dxy = sqrt(sq(prev_xyt[0]-rec.state[0]) + sq(prev_xyt[1]-rec.state[1]));
        double dtheta = fabs(prev_xyt[2]-rec.state[2]);
        if (!(dxy < comp_thresh_xy && dtheta < comp_thresh_theta)) {
            global_map_move_gridmap(state->compositor, rec.gm, rec.state);
        }
    }
}

void add_maptex(april_graph_node_t *node)
{
    if (!april_graph_node_attr_get(node, "maptex")) {
        grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
        image_u8_t *im = image_u8_create(gm->width, gm->height);
        for (int y = 0; y < gm->height; y++)
            memcpy(&im->buf[y*im->stride], &gm->data[y*gm->width], gm->width);
        vx_resource_t *tex = vx_resource_make_texture_u8_copy(im, 0);
        tex->incref(tex);
        april_graph_node_attr_put(node, NULL, "maptex", tex);
        image_u8_destroy(im);
    }
}

bool process_map(state_t *state, laser_t *laser, pose_t *pose)
{
    double local_xyt[3];
    doubles_quat_xyz_to_xyt(pose->orientation, pose->pos, local_xyt);
    april_graph_t *g = state->graph;
    if(!state->first) {
        if(doubles_distance(state->last_local_xyt, local_xyt, 2) < MIN_DIST_CHANGE && fabs(mod2pi(state->last_local_xyt[2] - local_xyt[2])) < to_radians(MIN_HEADING_CHANGE)) {
            return false;
        } else {
            memcpy(state->last_local_xyt, local_xyt, sizeof(double)*3);
        }
    } else {
        memcpy(state->last_local_xyt, local_xyt, sizeof(double)*3);
        state->first = false;
    }
    //zarray_t *points = laser_to_floats(state, laser);
    //zarray_destroy(points);
    grid_map_t *local_gm = state->local_gm;
    //TODO: Observe free space
    generate_local_grid_map(local_gm, laser);
    grid_map_t *local_gm_copy = grid_map_t_copy(local_gm);
    if(zarray_size(g->nodes) == 0) {
        // Add first node and a pin factor to its initial location
        april_graph_node_t *node = april_graph_node_xyt_create(local_xyt, local_xyt, NULL);
        april_graph_node_attr_put(node, stype_get("laser_t"), "laser", laser_t_copy(laser));
        april_graph_node_attr_put(node, stype_get("grid_map_t"), "grid_map", local_gm_copy);
        add_maptex(node);
        //Add Model data
        add_model_data_attr(node);
        //Add points data
        add_points_data_attr(node);
        node->UID = zarray_size(g->nodes);
        zarray_add(g->nodes, &node);
        matd_t * W = matd_create_data(3,3,
                                      (double []) {10000, 0, 0,
                                                   0, 10000, 0,
                                                   0, 0, 1000});
        april_graph_factor_t *factor = april_graph_factor_xytpos_create(node->UID,
                                                                        node->state,
                                                                        NULL,
                                                                        W);
        april_graph_factor_attr_put(factor, stype_get("string"), "type", strdup("geopin"));
        zarray_add(state->graph->factors, &factor);
        matd_destroy(W);
    } else {
        //If not the first node
        april_graph_node_t *na, *nb;
        zarray_get(g->nodes, zarray_size(g->nodes)-1, &na);
        if(doubles_distance(na->init, local_xyt, 2) > MIN_DIST_CHANGE || fabs(mod2pi(na->init[2] - local_xyt[2])) > to_radians(MIN_HEADING_CHANGE)) {
            //Move enough far
            nb = april_graph_node_xyt_create(local_xyt, local_xyt, NULL);
            april_graph_node_attr_put(nb, stype_get("laser_t"), "laser", laser_t_copy(laser));
            april_graph_node_attr_put(nb, stype_get("grid_map_t"), "grid_map", local_gm_copy);
            add_maptex(nb);
            //Add Model data
            add_model_data_attr(nb);
            //Add points data
            add_points_data_attr(nb);
            nb->UID = zarray_size(g->nodes);
            zarray_add(g->nodes, &nb);
            april_graph_node_attr_put(nb, stype_get("laser_t"), "laser", laser_t_copy(laser));
            // Add odom edge
            double z[3];
            doubles_xyt_inv_mul(na->init, nb->init, z);
            matd_t *W = matd_create(3,3);
            MATD_EL(W, 0, 0) = 1.0 / pow(0.1, 2);
            MATD_EL(W, 1, 1) = 1.0 / pow(0.1, 2);
            MATD_EL(W, 2, 2) = 1.0 / pow(to_radians(1), 2);
            doubles_xyt_mul(na->state, z, nb->state);
            april_graph_factor_t *factor = april_graph_factor_xyt_create(na->UID, nb->UID, z, NULL, W);
            zarray_add(g->factors, &factor);
            matd_destroy(W);
        }
    } //END ELSE
    //Optimize graph
    optimize_cholesky(state);
    if(state->auto_close_next < zarray_size(g->nodes)) {
        //close_loop(state, state->auto_close_next);
        state->auto_close_next++;
        printf("Auto closing to ID:%d\n", zarray_size(g->nodes)-1);
    }
    redraw_graph(state);
    global_map_add_gridmap(state->compositor, local_gm_copy, local_xyt);
    update_compositor(state);
    reset_grid_map(local_gm);
    return true;;
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, 'f', "log-file", "", "Specify the log file to read");
    getopt_add_int(gopt, 'p', "port", "8889", "webvx port");
    printf("Port: %d\n", getopt_get_int(gopt,"port"));

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }
    state_t *state = calloc(1, sizeof(state_t));

    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    state->vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                          on_console_command,
                                          on_console_tab,
                                          state);
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"), NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    if (1) {
		vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
		vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
		vx_buffer_swap(vb);
	}

    pthread_mutex_init(&state->mutex, NULL);
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "MOSS-mapping", "MOSS mapping tool");

    setlinebuf(stdout);
    setlinebuf(stderr);
    // Register types for serialization;
    // If you want to store information into graph, you need to register the type
    stype_register_basic_types();
    april_graph_stype_init();
    stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
    stype_register(STYPE_FROM_LCM("laser_t", laser_t));
    stype_register(STYPE_FROM_LCM("global_world_state_t", global_world_state_t));

    state->auto_close_next = 0;
    state->graph = april_graph_create();
    state->loopval = loopval_create(4 /* max depth */,
                                    8 /*vote thresh */,
                                    0.10 /* 0.05  trans thresh */,
                                    1.0 * M_PI / 180 /* radians thresh */);

    state->mpp = METER_PER_PIXEL;
    state->local_gm = gridmap_make_pixels(0, 0, GRID_MAP_SIZE, GRID_MAP_SIZE, state->mpp, (uint8_t)GRID_VAL_UNKNOWN, 0);
    reset_grid_map(state->local_gm);

    state->compositor = global_map_create(state->local_gm->meters_per_pixel);
    state->first = true;

    if(strlen(getopt_get_string(gopt, "log-file"))){
        const char *log_path = getopt_get_string(gopt, "log-file");
        const char *pose_channel = "POSE";
        const char *laser_channel = "LASER";
        //Test reading
        lcm_eventlog_t *eventlog_read = lcm_eventlog_create(log_path, "r");
        lcm_eventlog_event_t *event = NULL;
        pose_t pose = {0};
        laser_t laser = {0};
        while ((event = lcm_eventlog_read_next_event(eventlog_read)) != NULL) {
            if(strcmp(event->channel, pose_channel) == 0) {
                read_pose_event(event, &pose);
            }
            if(strcmp(event->channel, laser_channel) == 0) {
                read_laser_event(event, &laser);
            }
            if(laser.utime && pose.utime && laser.utime == pose.utime) {
                pthread_mutex_lock(&state->mutex);
                if(process_map(state, &laser, &pose)) {
                    //redraw_global_map(state);
                }
                pthread_mutex_unlock(&state->mutex);
            }
            lcm_eventlog_free_event(event);
        }
        lcm_eventlog_destroy(eventlog_read);
        redraw_global_map(state);
    }
    while(1) {
        usleep(100E3);
    }
    return 0;
}
