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

#include "scanmatch/scanmatch.h"


/** TODO:

*/
//./moss-localization -f /var/tmp/testlog-map-odom.lcmlog -g /var/tmp/saved.graph

#define VX 0

#define METER_PER_PIXEL 0.05
#define MAX_ITERS 500
#define ITER_PER_SAVE 10
#define SEARCH_RANGE 2
#define PERCENTAGE 1

#define MIN_DIST_CHANGE 0.2
#define MIN_HEADING_CHANGE 5

#define FIND_NODE_THRESH 1.0  // In meters

#define ONLY_POINT_MATCH 0

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

typedef struct error_rec error_rec_t;
struct error_rec {
    // compute covariances and expectations
    //Cxx = mXX/nPoints - pow(mX/nPoints,2);
    //Ex  = mX/nPoints;
    //mX -= x;
    //mXX -= x^2;
    double total;
    double mX, mXX;
    int npoint;
    bool isbad;
    int selected_cnt;
};

typedef struct state state_t;
struct state {
    lcm_t *lcm;
    lcm_eventlog_t *eventlog_read;

    //VX
    vx_world_t *vw;
	webvx_t *webvx;
    vx_console_t *vx_console;

    pthread_mutex_t mutex;
    double mpp;
    global_map_t *compositor;
    april_graph_t *graph;

    bool first;
    double prior_xyt[3];
    double prior_l2g[3];
    double last_local_xyt_truth[3];
    double last_local_xyt[3];

    int event_id;

    //error statistics
    error_rec_t last_dist_error_rec;
    error_rec_t dist_err_rec;
    error_rec_t heading_err_rec;

    int *labels;
    int switch_picked[2];
    zarray_t *switch_picked_array;
    bool improved;
    int switch_id;

    int step;
    bool pause;

    int32_t ref_idx, query_idx;

    FILE *f;

    zarray_t *log_files_path;
    bool manual_switch;
    zarray_t *nonSelectIdx;
    zarray_t *selectIdx;
};

typedef struct  {
    state_t *state;
    zarray_t *heading_errors;
    zarray_t *distance_errors;
    int id;
    int nthreads;
} wp_data_t;

int MAX_NUM_OF_NODES; //XXX: Smaller than number of nodes in graph
state_t *global_state;


static void optimize_cholesky(april_graph_t *loc_graph)
{
    struct april_graph_cholesky_param chol_param;
    april_graph_cholesky_param_init(&chol_param);
    chol_param.tikhanov = 1.0E-6;
    april_graph_cholesky(loc_graph, &chol_param);
}

void update_err_rec(error_rec_t *rec, double x)
{
    rec->npoint++;
    rec->mX += x;
    rec->mXX += sq(x);
    rec->total += fabs(x);
}

double extract_mu(error_rec_t *rec)
{
    return rec->mX/rec->npoint;
}

double extract_var(error_rec_t *rec)
{
    return rec->mXX / rec->npoint - pow(rec->mX/rec->npoint, 2);
}

double extract_err_total(error_rec_t *rec)
{
    return rec->total;
}

static zarray_t *laser_to_floats(const laser_t *laser)
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

void render_truth_node(state_t *state, double xyt[3])
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "truth node");
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_xyt(xyt),
                                      vxo_robot_solid(vx_red),
                                      NULL),
                       NULL);
    vx_buffer_swap(vb);

}

void render_prior_node(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "prior node");
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_xyt(state->prior_xyt),
                                      vxo_robot_solid(vx_blue),
                                      NULL),
                       NULL);
    vx_buffer_swap(vb);
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

void render_proposed_match(state_t *state, double res_xyt[3],
                           sm_model_data_t *sm, sm_points_data_t *pd)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "Scan matching");
    sm_model_t *sm0;
    zarray_get(sm->models, 0, &sm0);
    if(!sm0)
        return;
    image_u8_t *im = sm0->im;
    if(!im)
        return;
    float rgba[4] = { 1.0, 0.0, 0.0, 0.8 };
    vx_resource_t *points_resc = NULL;
    if (pd) {
        points_resc = vx_resource_make_attr_f32_copy((float*) pd->points->data,
                                                     zarray_size(pd->points)*2,
                                                     2);
    }

    double screen_size =  0.6 / (im->width > im->height ? im->width : im->height);
    vx_object_t *im_obj = vxo_image_u8(im, 0);
    vx_object_t *overlay_obj = NULL;
    if (points_resc) {
        // Render with points of proposed match overlaid
        overlay_obj = vxo_chain(vxo_pixcoords(VXO_PIXCOORDS_TOP_RIGHT,
                                              VXO_PIXCOORDS_SCALE_MODE_ONE,
                                              NULL),
                                vxo_pixcoords(VXO_PIXCOORDS_TOP_RIGHT,
                                              VXO_PIXCOORDS_SCALE_MODE_MIN,
                                              vxo_matrix_scale(screen_size),
                                              vxo_matrix_translate(-im->width, -im->height, 0),
                                              vxo_chain(
                                                        vxo_matrix_translate(-sm0->x0, -sm0->y0, 0),
                                                        vxo_matrix_scale(1.0 / sm->meters_per_pixel),
                                                        vxo_matrix_xyt(res_xyt),
                                                        vxo_points(points_resc, rgba, 2),
                                                        vxo_robot_solid(rgba),
                                                        NULL),
                                              im_obj,
                                              NULL),
                                NULL);
        vx_buffer_add_back(vb, vxo_depth_test(1, overlay_obj, NULL), NULL);
    }
    vx_buffer_swap(vb);
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

static void redraw_selected_node(april_graph_node_t *node,
                                 int idx,
                                 float rgbas[4][4],
                                 vx_buffer_t *vb)
{
    vx_resource_t *tex = april_graph_node_attr_get(node, "maptex");
    grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");

    if(tex == NULL) {
        add_maptex(node);
    }
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


void read_laser_event(lcm_eventlog_event_t *event, laser_t *laser)
{
    assert(0 < laser_t_decode(event->data, 0, event->datalen, laser));
}

void read_pose_event(lcm_eventlog_event_t *event, pose_t *pose)
{
    assert(0 < pose_t_decode(event->data, 0, event->datalen, pose));
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
        if (key == 'n') {
            state->event_id++;
        } else if(key == 'm') {
            state->event_id +=5;
        }
        else if(key == 'N') {
            state->event_id += 50;
        } else if(key == 'r') {
            state->event_id = -1;
        } else if(key == 'p') {
            state->pause = !state->pause;
            puts("toggle pause");
            if(state->pause) {
                puts("Pause");
            } else {
                puts("Press n to resume");
            }
        }

    }

    return 0;
}

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
    if (!strcmp(tok, "set-event-id")) {
        if (zarray_size(toks) != 2) {
            vx_console_printf(vc, "Usage: set-event-id ID");
        } else {
            char *t1;
            zarray_get(toks, 1, &t1);
            state->event_id = atoi(t1);
            vx_console_printf(vc, "event_id set to %d\n", state->event_id);
        }
        goto cleanup;
    }
    if (!strcmp(tok, "add-manual-switch")) {
        int selectIdx = -1;
        int nonSelectIdx = -1;
        for(int i = 0; i < MAX_NUM_OF_NODES; i++) {
            if(state->labels[i] == state->query_idx) {
                nonSelectIdx = i;
            }
        }
        for(int i = MAX_NUM_OF_NODES + 1; i < zarray_size(state->graph->nodes); i++) {
            if(state->labels[i] == state->ref_idx) {
                selectIdx = i;
            }
        }
        if(selectIdx != -1 && nonSelectIdx != -1) {
            zarray_add(state->nonSelectIdx, &nonSelectIdx);
            zarray_add(state->selectIdx, &selectIdx);
            puts("manual switch succeed");
        }
        goto cleanup;
    }
    if (!strcmp(tok, "set-manual-switch")) {
        if(state->nonSelectIdx->size != 0) {
            state->manual_switch = true;
            puts("set manual switch");
        }
        goto cleanup;
    }
    vx_console_printf(vc, "unknown command '%s'", cmd);

 cleanup:
    zarray_vmap(toks, free);
    zarray_destroy(toks);

}

zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    //state_t *state = user;
    zarray_t *completions = zarray_create(sizeof(char*));

    const char *commands[] = { "clear",
                               "set-event-id",
                               "add-manual-switch",
                               "set-manual-switch",
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

sm_model_data_t *create_model(laser_t *laser, double meters_per_pixel, int nlevels)
{
    zarray_t *points = laser_to_floats(laser);

    double xmin = HUGE, xmax = -HUGE;
    double ymin = HUGE, ymax = -HUGE;

    // how far away (in meters) does it take for our cost model to drop to 0?
    double MODEL_COST_RANGE = 0.05;

    for (int i = 0; i < zarray_size(points); i++) {
        float xy[2];
        zarray_get(points, i, xy);
        double x = xy[0];
        double y = xy[1];

        xmin = fmin(x, xmin);
        xmax = fmax(x, xmax);
        ymin = fmin(y, ymin);
        ymax = fmax(y, ymax);
    }

    // Render the model
    double pad = 1;
    image_u8_t *im = image_u8_create((xmax-xmin+2*pad) / meters_per_pixel,
                                     (ymax-ymin+2*pad) / meters_per_pixel);
    double x0 = xmin - pad; // position of pixel 0 (in meters);
    double y0 = ymin - pad;

    image_u8_lut_t lut;
    if (1) {
        memset(&lut, 0, sizeof(image_u8_lut_t));
        lut.scale = 1;
        float max_dist2 = MODEL_COST_RANGE * MODEL_COST_RANGE / (meters_per_pixel * meters_per_pixel);

        lut.nvalues = (int) (max_dist2 / lut.scale) + 1;
        lut.values = calloc(lut.nvalues, sizeof(uint8_t));
        for (int i = 0; i < lut.nvalues; i++) {
            int penalty = 255.0*i*i / ((lut.nvalues-1)*(lut.nvalues-1));
            if (penalty > 255)
                penalty = 255;
            lut.values[i] = 255 - penalty;
        }
    }

    float last_xy[2];

    for (int i = 0; i < zarray_size(points); i++) {
        float xy[2];
        zarray_get(points, i, xy);
        double x = xy[0];
        double y = xy[1];

        // convert to pixel coordinates
        float model_xy[] = { (x - x0) / meters_per_pixel,
                             (y - y0) / meters_per_pixel };

        image_u8_fill_line_max(im, &lut, model_xy, model_xy);

        if (i > 0) {
            double dist2 = sq(x - last_xy[0]) + sq(y - last_xy[1]);
            float xy[2] = { x, y };
            if (dist2 < 1) {
                image_u8_fill_line_max(im, &lut,
                                       (float[]) { (last_xy[0] - x0) / meters_per_pixel,
                                               (last_xy[1] - y0) / meters_per_pixel },
                                       (float[]) { (xy[0] - x0) / meters_per_pixel,
                                               (xy[1] - y0) / meters_per_pixel });
            }
        }
        last_xy[0] = x;
        last_xy[1] = y;
    }

    zarray_destroy(points);

    return sm_model_data_create(im, x0 / meters_per_pixel, y0 / meters_per_pixel, meters_per_pixel, nlevels);
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
            //for (int i = 0; i < ksz; i++)
            //printf("%d %5d\n", i, k[i]);
        }
        if (0) {
            image_u8_gaussian_blur(im, sigma, ksz);
        }

        /* sm_model_data_t *model_data = sm_model_data_create(im, */
        /*                                                    gm->x0 / gm->meters_per_pixel, */
        /*                                                    gm->y0 / gm->meters_per_pixel, */
        /*                                                    gm->meters_per_pixel, */
        /*                                                    12); */

        laser_t *laser = april_graph_node_attr_get(node, "laser");
        sm_model_data_t *model_data = create_model(laser, METER_PER_PIXEL, 12);


        april_graph_node_attr_put(node, NULL, "model_data", model_data);
    }
}

static inline double randNormal(struct drand48_data *randBuffer)
{
    float s = 0.0f;
    float u, v, R2;
    double x1, x2;
    while (s == 0.0f || s >= 1.0f)
    {
        drand48_r(randBuffer, &x1);
        drand48_r(randBuffer, &x2);
        // Sample two values uniformly from (-1, +1)
        u = x1 * 2 - 1.0f;
        v = x2 * 2 - 1.0f;

        R2 = u*u + v*v;
        s = R2;
    }

    float factor = sqrt( (-2*log(s)) / (s) );

    // both are normally distributed
    float z0 = u * factor;
    //float z1 = v * factor;

    // we only want to return one, for convenience
    return z0;
}

static double gaussian_sample(double mu, double variance, struct drand48_data *randBuffer)
{
    return mu + sqrt(variance) * randNormal(randBuffer);
}

static void add_pose_noise(double last_xyt[3], double xyt[3], double delta_xyt[3], struct drand48_data *randBuffer)
{
    double z[3] = { gaussian_sample(delta_xyt[0], fabs(delta_xyt[0])/10, randBuffer),
                    gaussian_sample(delta_xyt[1], fabs(delta_xyt[1])/10, randBuffer),
                    mod2pi(gaussian_sample(delta_xyt[2], fabs(mod2pi(to_radians(to_degrees(delta_xyt[2]) / 100.0))), randBuffer)) };
    //double z[3];
    //memcpy(z, delta_xyt, sizeof(double)*3);
    doubles_xyt_mul(last_xyt, z, xyt);
}

static int selected_contains(state_t *state, int nidx)
{
    int ret = 0;
    if(MAX_NUM_OF_NODES == 0) {
        //no selected node; //odometry only
        return 0;
    }
    for(int i = 0; i < zarray_size(state->switch_picked_array); i++) {
        zarray_get(state->switch_picked_array, i, state->switch_picked);
        if(MAX_NUM_OF_NODES != zarray_size(state->graph->nodes) && state->switch_picked[0] != -1 && state->switch_picked[1] !=-1) {
            if(nidx == state->labels[state->switch_picked[0]])
                return 0;
            if(nidx == state->labels[state->switch_picked[1]])
                return 1;
        }
    }
    for(int i =0; i<MAX_NUM_OF_NODES; i++) {
        if(state->labels[i] == nidx)
            return 1;
    }
    return ret;
}

void process_map(wp_data_t *wp_data, const laser_t *laser, const pose_t *pose, struct drand48_data *randBuffer, april_graph_t *loc_graph)
{
    state_t *state = wp_data->state;
    double dist_err;
    double heading_err;
    double local_xyt_truth[3]; // our current position in local coordinates
    doubles_quat_xyz_to_xyt(pose->orientation, pose->pos, local_xyt_truth);
    double local_xyt[3];
    april_graph_node_t *loc_node;
    if(state->first) {
        memcpy(state->prior_xyt, local_xyt_truth, sizeof(double)*3);
        memcpy(local_xyt, local_xyt_truth, sizeof(double)*3);
        double nstate[3] = { 0,0,0};
        double ninit[3] = { local_xyt[0], local_xyt[1], local_xyt[2] };
        double ntruth[3] = { 0, 0, 0 };

        loc_node = april_graph_node_xyt_create(nstate, ninit, ntruth);
        zarray_add(loc_graph->nodes, &loc_node);
        state->first = false;

        matd_t *W1 = matd_identity(3);
        MATD_EL(W1, 0, 0) = 1.0 / pow(0.001, 2);
        MATD_EL(W1, 1, 1) = 1.0 / pow(0.001, 2);
        MATD_EL(W1, 2, 2) = 1.0 / pow(0.001*M_PI/180.0, 2);
        april_graph_factor_t *factor1 = april_graph_factor_xytpos_create(loc_graph->nodes->size-1, local_xyt, NULL, W1);
        zarray_add(loc_graph->factors, &factor1);
        matd_destroy(W1);

    } else {
        double delta_xyt[3];
        doubles_xyt_inv_mul(state->last_local_xyt_truth, local_xyt_truth, delta_xyt);
        if(doubles_distance(state->last_local_xyt_truth, local_xyt_truth, 2) < MIN_DIST_CHANGE &&
           fabs(mod2pi(state->last_local_xyt_truth[2] - local_xyt_truth[2])) < to_radians(MIN_HEADING_CHANGE)) {
            return;
        }
        //TODO: Better odometry error model
        add_pose_noise(state->last_local_xyt, local_xyt, delta_xyt, randBuffer);
        double delta_xyt_noise[3];
        doubles_xyt_inv_mul(state->last_local_xyt, local_xyt, delta_xyt_noise);
        //memcpy(local_xyt, local_xyt_truth, sizeof(double)*3);
        double nstate[3] = { 0, 0, 0 };
        double ninit[3] = { local_xyt_truth[0], local_xyt_truth[1], local_xyt_truth[2] };
        doubles_xyt_mul(state->prior_xyt, delta_xyt_noise, ninit);

        double ntruth[3] = { 0, 0, 0 };

        loc_node = april_graph_node_xyt_create(nstate, ninit, ntruth);
        int last_node_idx = zarray_size(loc_graph->nodes) - 1;
        zarray_add(loc_graph->nodes, &loc_node);

        int32_t aidx = last_node_idx;
        int32_t bidx = zarray_size(loc_graph->nodes) - 1;

        april_graph_node_t *na, *nb;
        zarray_get(loc_graph->nodes, aidx, &na);
        zarray_get(loc_graph->nodes, bidx, &nb);
        double z[3];

        doubles_xyt_inv_mul(na->init, nb->init, z);
        matd_t *W = matd_create(3, 3);
        MATD_EL(W, 0, 0) = 1.0 / pow(0.1,2);
        MATD_EL(W, 1, 1) = 1.0 / pow(0.1,2);
        MATD_EL(W, 2, 2) = 1.0 / pow(1*M_PI/180.0, 2);

        assert (nb == loc_node);
        // A = ZB
        // inv(Z)*A = B
        // Z = Ainv(B)
        doubles_xyt_mul(na->state, z, nb->state);

        april_graph_factor_t *factor = april_graph_factor_xyt_create(aidx, bidx, z, NULL, W);
        zarray_add(loc_graph->factors, &factor);
        matd_destroy(W);

        //doubles_xyt_mul(state->prior_l2g, local_xyt, state->prior_xyt);
        memcpy(state->prior_xyt, loc_node->init, sizeof(double)*3);

        if(VX) {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "predicted-prior");
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_xyt(state->prior_xyt),
                                              vxo_robot_solid(vx_black),
                                              NULL),
                               NULL);
            vx_buffer_swap(vb);
        }
    }
    memcpy(state->last_local_xyt_truth, local_xyt_truth, sizeof(double)*3);
    memcpy(state->last_local_xyt, local_xyt, sizeof(double)*3);


    //Scan matching here:
    zarray_t *neighbors = zarray_create(sizeof(int));
    april_graph_t *g = state->graph;
    int best_idx = -1;
    double best_dist = DBL_MAX;
    vx_buffer_t *vb_selected;
    vx_buffer_t *vb;
    if(VX) {
        vb_selected = vx_world_get_buffer(state->vw, "selected-nodes");
    }
    for (int nidx = 0; nidx < zarray_size(g->nodes); nidx++) {

        if(!selected_contains(state, nidx)) {
            continue;
        }

        april_graph_node_t *node;
        zarray_get(g->nodes, nidx, &node);


        if(VX) {
            vx_buffer_add_back(vb_selected,
                               vxo_matrix_xyt(node->state),
                               vxo_robot_solid(vx_green),
                               NULL);

        }

        double this_dist = doubles_distance(state->prior_xyt, node->state, 2);
        if (this_dist < SEARCH_RANGE)
            zarray_add(neighbors, &nidx);

        if (this_dist < best_dist) {
            best_dist = this_dist;
            best_idx = nidx;
        }
    }
    if(VX) {
        vx_buffer_swap(vb_selected);
    }
    if(VX) {
        vb = vx_world_get_buffer(state->vw, "live");
    }

    if(zarray_size(neighbors) == 0) {
        goto laser_process_cleanup;
    }
    int maxneighbors = 1;
    // if we got TOO many, decimate it. (never remove the closest one)
    while (zarray_size(neighbors) > maxneighbors) {
        int ridx = randi_uniform(0, zarray_size(neighbors));
        int victim;
        zarray_get(neighbors, ridx, &victim);
        if (victim != best_idx)
            zarray_remove_index(neighbors, ridx, 0);
    }

    zarray_t *points = laser_to_floats(laser);
    sm_points_data_t *points_data = sm_points_data_create(points);

    for (int nn = 0; nn < zarray_size(neighbors); nn++) {
        int nidx;
        zarray_get(neighbors, nn, &nidx);

        april_graph_node_t *node;
        zarray_get(g->nodes, nidx, &node);

        sm_model_data_t *model_data = april_graph_node_attr_get(node, "model_data");
        assert (model_data);

        double pred_xyt[3];
        // use current state estimate for mean (instead of dijkstra)
        // Tnode = state->prior_xyt
        doubles_xyt_inv_mul(node->state, state->prior_xyt, pred_xyt);

        float fpred_xyt[3];
        for (int i = 0; i < 3; i++)
            fpred_xyt[i] = pred_xyt[i];


        sm_search_t *search = sm_search_create();

        float fpred_inf[9];
        memset(fpred_inf, 0, 9*sizeof(float));
        //for (int i = 0; i < 3; i++) {
        //    fpred_inf[3*i+i] = 0.0001;
        //}
        fpred_inf[0] = fpred_inf[4] = 10;
        fpred_inf[8] = 50;

        int trange = SEARCH_RANGE; // in meters
        float radrange = 15 * M_PI / 180, radstep = 1.0 * M_PI / 180;
        double scale = 1.0 / zarray_size(points_data->points);
        double impp = 1.0 / state->mpp;
        double minscore = -1;
        if(!ONLY_POINT_MATCH) {
            sm_search_add(search, points_data, model_data,
                          (fpred_xyt[0] - trange)*impp,  (fpred_xyt[0] + trange)*impp,
                          (fpred_xyt[1] - trange)*impp,  (fpred_xyt[1] + trange)*impp,
                          fpred_xyt[2] - radrange, fpred_xyt[2] + radrange, radstep,
                          scale, fpred_xyt, fpred_inf, minscore);
        } else {
            sm_search_add(search, points_data, model_data,
                          (fpred_xyt[0] - trange)*impp,  (fpred_xyt[0] + trange)*impp,
                          (fpred_xyt[1] - trange)*impp,  (fpred_xyt[1] + trange)*impp,
                          fpred_xyt[2] - radrange, fpred_xyt[2] + radrange, radstep,
                          scale, NULL, NULL, minscore);
        }
        sm_result_t *sres = sm_search_run(search);

        if (sres) {
            double xyt[3];
            memcpy(xyt, sres->xyt, 3 * sizeof(double));
            //printf("scode: %f: \n", sres->score);
            //doubles_print(xyt, 3, "%f,");
            if (1) {
                sm_hillclimb_params_t hcparams = { .maxiters = 100,
                                                   .initial_step_sizes = { state->mpp / 2,
                                                                           state->mpp / 2,
                                                                           0.5 * M_PI / 180 },
                                                   .step_size_shrink_factor = 0.5, // was .5
                                                   .max_step_size_shrinks = 8 }; // was 8
                sm_hillclimb_result_t *hres;
                if(!ONLY_POINT_MATCH) {
                    hres = sm_hillclimb(points_data, model_data, sres->xyt, &hcparams,
                                        scale, fpred_xyt, fpred_inf);
                } else {
                    hres = sm_hillclimb(points_data, model_data, sres->xyt, &hcparams,
                                        scale, NULL, NULL);
                }
                memcpy(xyt, hres->xyt, 3 * sizeof(double));
                //printf("score: %f: \n", hres->score);
                //doubles_print(xyt, 3, "%f,");
                sm_hillclimb_result_destroy(hres);
            }

            double gxyt[3]; // our current position in global coordinates
            doubles_xyt_mul(node->state, xyt, gxyt);
            double lxytinv[3];
            doubles_xyt_inv(local_xyt, lxytinv);
            doubles_xyt_mul(gxyt, lxytinv, state->prior_l2g);
            if(VX) {
                vx_resource_t *points_resc = vx_resource_make_attr_f32_copy((float*) points->data, 2*zarray_size(points), 2);
                vx_buffer_add_back(vb,
                                   vxo_matrix_xyt(node->state),
                                   vxo_matrix_xyt(xyt),
                                   vxo_points(points_resc, vx_yellow, 3),
                                   NULL);
                vx_buffer_add_back(vb,
                                   vxo_matrix_xyt(node->state),
                                   vxo_robot_solid(vx_cyan),
                                   NULL);
                render_proposed_match(state, xyt,
                                      model_data, points_data);

            }

            //add scan matching factor
            matd_t *W1 = matd_identity(3);
            MATD_EL(W1, 0, 0) = 1.0 / pow(0.01, 2);
            MATD_EL(W1, 1, 1) = 1.0 / pow(0.01, 2);
            MATD_EL(W1, 2, 2) = 1.0 / pow(0.5*M_PI/180.0, 2);

            april_graph_factor_t *factor1 = april_graph_factor_xytpos_create(loc_graph->nodes->size-1, gxyt, NULL, W1);
            zarray_add(loc_graph->factors, &factor1);
            matd_destroy(W1);
            optimize_cholesky(loc_graph);

            // Update pose for next scan match
            {
                int32_t bidx = zarray_size(loc_graph->nodes) - 1;
                april_graph_node_t *nb;
                zarray_get(loc_graph->nodes, bidx, &nb);
                memcpy(state->prior_xyt, nb->state, sizeof(double)*3);
            }

            sm_result_destroy(sres);

        }
        sm_search_destroy(search);
    }
    sm_points_data_destroy(points_data);

 laser_process_cleanup:
    zarray_destroy(neighbors);
    if(VX) {
        vx_buffer_swap(vb);
        render_truth_node(state, local_xyt_truth);
        render_prior_node(state);
    }
    dist_err = doubles_distance(state->prior_xyt, local_xyt_truth, 2);
    heading_err = mod2pi(local_xyt_truth[2]-state->prior_xyt[2]);
    zarray_add(wp_data->distance_errors, &dist_err);
    zarray_add(wp_data->heading_errors, &heading_err);
}

void initialize_selection(state_t *state, int max_num_of_nodes)
{
    april_graph_t *g = state->graph;
    int N = zarray_size(g->nodes);
    state->labels = calloc(N+1, sizeof(int));
    int *label = state->labels;
    for(int i = 0; i<N; i++) {
        label[i] = i;
    }
    label[N] = -1;
    if(1) {
        int small = 0;
        for(int i=0; i < max_num_of_nodes; i++) {
            int picked_id = randi_uniform(small, N);
            int tmp = label[small];
            label[small] = label[picked_id];
            label[picked_id] = tmp;
            small++;
        }
    }
}

void initialize_selection_equal_space(state_t *state, int max_num_of_nodes)
{
    april_graph_t *g = state->graph;
    int N = zarray_size(g->nodes);
    state->labels = calloc(N+1, sizeof(int));
    int *label = state->labels;
    for(int i = 0; i<N; i++) {
        label[i] = i;
    }
    label[N] = -1;
    // We have N nodes and we would like select max_num_of_nodes from them
    int space = floor((float)(N) / (max_num_of_nodes - 2));
    int interval = N / (max_num_of_nodes + 1);
    printf("%d, %d\n",N, max_num_of_nodes);
    int selectIdx = 0;
    int nonSelectIdx = selectIdx + 1;
    for(int i = 0; i < max_num_of_nodes; i++) {
        label[i] = (i+1) * interval;
        printf("%d,",label[i]);
        selectIdx += space;
        if(selectIdx >= N) {
            selectIdx = N-1;
        }
    }
    puts("\n");
    if(label[max_num_of_nodes-1] == N-1 && label[max_num_of_nodes-1] == label[max_num_of_nodes-2]) {
        label[max_num_of_nodes-2] = label[max_num_of_nodes-1]-1;
    }
    selectIdx = 1;
    for(int i = max_num_of_nodes; i < N; i++) {
        if(nonSelectIdx < label[selectIdx]) {
            label[i] = nonSelectIdx;
            printf("%d,",nonSelectIdx);
            nonSelectIdx++;
        } else {
            nonSelectIdx++;
            label[i] = nonSelectIdx;
            printf("%d,",nonSelectIdx);
            if(selectIdx < max_num_of_nodes - 1){
                selectIdx++;
                nonSelectIdx++;
            }
        }
    }
    puts("\n");
}

void initialize_selection_from_bootstrap(state_t *state, const char* path)
{
    FILE *f = fopen(path, "r");
    if(f == NULL) {
        printf("Wrong bootstrap file \n");
        exit(-1);
    }
    april_graph_t *g = state->graph;
    int N = zarray_size(g->nodes);
    state->labels = calloc(N+1, sizeof(int));
    int *label = state->labels;
    for(int i = 0; i<N; i++) {
        label[i] = i;
    }
    label[N] = -1;
    if(1) {
        for(int i=0; i < MAX_NUM_OF_NODES; i++) {
            if(!fscanf(f, "%d ", &label[i])) {
                printf("Loaded graph is not corresponding to bootstrapping file\n");
                assert(0);
            }
        }
    }
    fclose(f);
}

void initialize_selection_from_resume_file(state_t *state, const char* path)
{
    FILE *f = fopen(path, "r");
    if(f == NULL) {
        printf("Wrong resume file \n");
        exit(-1);
    }
    april_graph_t *g = state->graph;
    int N = zarray_size(g->nodes);
    state->labels = calloc(N+1, sizeof(int));
    int *label = state->labels;
    for(int i = 0; i<N; i++) {
        label[i] = i;
    }
    label[N] = -1;
    char *line = NULL;
    size_t n = 0;
    bool found = false;
    while(getline(&line, &n, f) != -1) {
        if(strchr(line, '=')) {
            found = true;
            break;
        }
    }
    assert(found);
    char c = '\0';
    float error;
    int new_N;
    fscanf(f, "%c%f,%d,%d", &c, &error, &MAX_NUM_OF_NODES, &new_N);
    assert(N == new_N);
    printf("selected: %d, Total: %d \n", MAX_NUM_OF_NODES, N);
    while(c != '+') {
        fscanf(f, "%c", &c);
    }
    printf("%c\n",c);
    for(int i=0; i < MAX_NUM_OF_NODES; i++) {
        if(!fscanf(f, "%d,", &(state->labels[i]))) {
            printf("Loaded graph is not corresponding to bootstrapping file\n");
            assert(0);
        }
    }
    while(c != '-') {
        fscanf(f, "%c", &c);
    }
    printf("%c\n",c);
    for(int i=MAX_NUM_OF_NODES; i < N; i++) {
        if(!fscanf(f, "%d,", &(state->labels[i]))) {
            printf("Loaded graph is not corresponding to bootstrapping file\n");
            assert(0);
        }
    }

    free(line);
    fclose(f);
}


static void hill_climbing_switch(state_t *state)
{
    for(int i = 0; i < zarray_size(state->switch_picked_array); i++) {
        zarray_get(state->switch_picked_array, i, state->switch_picked);
        if(state->switch_picked[0] == -1 || state->switch_picked[1] ==-1 ||
            state->labels[state->switch_picked[1]] == -1 || state->labels[state->switch_picked[0]] == -1)
            return;
        int tmp = state->labels[state->switch_picked[0]];
        state->labels[state->switch_picked[0]] = state->labels[state->switch_picked[1]];
        state->labels[state->switch_picked[1]] = tmp;
    }
}

void save_result(state_t *state)
{
    fprintf(state->f, ":%.2f,%d,%d\n",
            extract_err_total(&state->last_dist_error_rec) / state->last_dist_error_rec.npoint, MAX_NUM_OF_NODES, zarray_size(state->graph->nodes));
    fprintf(state->f, "+");
    for(int i = 0; i < MAX_NUM_OF_NODES; i++) {
        fprintf(state->f, "%d,", state->labels[i]);
    }
    fprintf(state->f, "\n-");
    for(int i = MAX_NUM_OF_NODES; i < zarray_size(state->graph->nodes); i++) {
        fprintf(state->f, "%d,", state->labels[i]);
    }
    fprintf(state->f, "\n");
    fflush(state->f);
}

void method_1(state_t *state);
void method_2(state_t *state);
static void hill_climbing_pick(state_t *state)
{
    srand(utime_now());
    zarray_clear(state->switch_picked_array);
    if(state->manual_switch) {
        for(int i = 0; i < zarray_size(state->selectIdx); i++) {
            zarray_get(state->nonSelectIdx, i, &state->switch_picked[0]);
            zarray_get(state->selectIdx, i, &state->switch_picked[1]);
            zarray_add(state->switch_picked_array, state->switch_picked);
            printf("manual switch: %d,%d \n", state->labels[state->switch_picked[0]], state->labels[state->switch_picked[1]]);
        }
    } else {
        int N = zarray_size(state->graph->nodes);
        const int method = 0;
        if(method == 0) {
            state->switch_picked[0] = randi_uniform(0, MAX_NUM_OF_NODES);
            state->switch_picked[1] = randi_uniform(MAX_NUM_OF_NODES, N);
        } else if(method == 1) {
            method_1(state);
        } else if(method == 2) {
            method_2(state);
        }
        zarray_add(state->switch_picked_array, state->switch_picked);
    }
    if(VX) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "picked switch nodes");
        for (int i = 0; i < zarray_size(state->switch_picked_array); i++) {
          zarray_get(state->switch_picked_array, i, state->switch_picked);
          if(state->labels[state->switch_picked[1]] != -1) {
              april_graph_node_t *node;
              zarray_get(state->graph->nodes, state->labels[state->switch_picked[0]], &node);
              vx_buffer_add_back(vb,
                  vxo_matrix_translate(0, 0, 0.01),
                  vxo_matrix_xyt(node->state),
                  vxo_matrix_scale(0.5),
                  vxo_robot_solid(vx_black),
                  NULL);

              zarray_get(state->graph->nodes, state->labels[state->switch_picked[1]], &node);
              vx_buffer_add_back(vb,
                  vxo_matrix_translate(0, 0, 0.01),
                  vxo_matrix_xyt(node->state),
                  vxo_matrix_scale(0.5),
                  vxo_robot_solid(vx_white),
                  NULL);
          }
        }
        vx_buffer_swap(vb);
    }
    zarray_clear(state->nonSelectIdx);
    zarray_clear(state->selectIdx);
    state->manual_switch = false;
}

void method_1(state_t *state)
{
    int N = zarray_size(state->graph->nodes);
    if(state->improved || state->switch_id == -1) {
        state->step = 16;
        state->improved = false;
        //if improved, random pick one
        state->switch_picked[0] = randi_uniform(0, MAX_NUM_OF_NODES);
        state->switch_id = MAX_NUM_OF_NODES;
    }
    if(state->switch_id >= N) {
        if(state->step == 1) {
            state->switch_id = -1;
        } else {
            state->switch_id = MAX_NUM_OF_NODES;
            //TODO: This could be smarter to avoid duplicated computing.
            state->step /= 2;
            assert(state->step > 0);
        }
    } else {
        state->switch_picked[1] = state->switch_id;
        state->switch_id += state->step;
    }
    //printf("Step size:%d \n", state->step);
}

void method_2(state_t *state)
{
    assert(0);
}
static void signal_handler(int signum)
{
    switch (signum) {
    case SIGINT:
        printf("SIGINT\n");
        fprintf(global_state->f, "=\n");
        save_result(global_state);
        exit(-1);
    default:
        break;
    }
}

void wp_func(void *_wp_data)
{
    wp_data_t *wp_data = (wp_data_t*)_wp_data;
    state_t *state = wp_data->state;
    char *log_path;
    const char *pose_channel = "POSE";
    //const char *laser_channel = "LASER";
    const char *laser_channel = "LASER_NOISE";
    //Test reading
    lcm_eventlog_event_t *event = NULL;
    struct drand48_data randBuffer;
    for(int trip_i = wp_data->id; trip_i < zarray_size(state->log_files_path); trip_i += wp_data->nthreads) {
        //printf("trip_i %d on %d: \n", trip_i, wp_data->id);
        zarray_get(state->log_files_path, trip_i, &log_path);
        state->event_id = 0;
        int running_event_id = 0;
        state->first = true;
        memset(state->prior_l2g, 0, sizeof(double)*3);
        lcm_eventlog_t *eventlog_read = lcm_eventlog_create(log_path, "r");
        pose_t pose = {0};
        laser_t laser = {0};
        april_graph_t *loc_graph = april_graph_create();
        while ((event = lcm_eventlog_read_next_event(eventlog_read)) != NULL) {
            if(strcmp(event->channel, pose_channel) == 0) {
                read_pose_event(event, &pose);
            }
            if(strcmp(event->channel, laser_channel) == 0) {
                read_laser_event(event, &laser);
            }
            if(laser.utime && pose.utime && laser.utime == pose.utime) {
                srand48_r(trip_i + running_event_id, &randBuffer);
                //srand48_r(running_event_id, &randBuffer);
                process_map(wp_data, &laser, &pose, &randBuffer, loc_graph);
                running_event_id++;
                free(laser.ranges);
                free(laser.intensities);
            }
            lcm_eventlog_free_event(event);
        }
        april_graph_destroy(loc_graph);
        lcm_eventlog_destroy(eventlog_read);
    }
}

void test_gaussian()
{
    struct drand48_data randBuffer;
    srand48_r(0, &randBuffer);
    FILE *f = fopen("/tmp/test-gaussian", "w");

    for(int i = 0; i < 10000; i++) {
        fprintf(f, "%f,", gaussian_sample(0, 1, &randBuffer));
    }
    fprintf(f, "\n");
    fclose(f);
    exit(-1);
}

int str_cmp(const void *_a, const void *_b)
{
    char **a = (char**)_a;
    char **b = (char**)_b;
    return strcmp(*a, *b);
}

int main(int argc, char *argv[])
{

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'd', "debug", 0, "Debug mode");
    getopt_add_string(gopt, 'f', "log-file-folder", "", "Specify the folder containing log files to read");
    getopt_add_string(gopt, 'w', "out-file", "", "Specify the file to write");
    getopt_add_string(gopt, 'g', "graph-file", "", "Specify the graph file to load");
    getopt_add_string(gopt, '\0', "bootstrapping-file", "", "Specify the bootstrapping result file to load");
    getopt_add_string(gopt, '\0', "resume-file", "", "Specify the saved results file to load");
    getopt_add_int(gopt, 'p', "port", "8890", "webvx port");
    getopt_add_int(gopt, 'i', "nnodes", "21", "nnodes");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }

    state_t *state = calloc(1, sizeof(state_t));
    global_state = state;
    state->event_id = 0;
    state->ref_idx = -1;
    state->query_idx = -1;

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
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "MOSS-localization", "MOSS localization tool");

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
    if(state->graph == NULL) {
        printf("no graph loaded");
        exit(-1);
    }
    // INIT SET
    MAX_NUM_OF_NODES = getopt_get_int(gopt, "nnodes"); //ceil(zarray_size(state->graph->nodes) * PERCENTAGE);
    MAX_NUM_OF_NODES = iclamp(MAX_NUM_OF_NODES, 0, zarray_size(state->graph->nodes));
    //printf("MAX_NUM_OF_NODES: %d \n", MAX_NUM_OF_NODES);
    state->mpp = METER_PER_PIXEL;
    state->compositor = global_map_create(state->mpp);
    if (1) {
        april_graph_t *g = state->graph;
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "graph-node");
        for (int i = 0; i < zarray_size(g->nodes); i++) {
            april_graph_node_t *node;
            zarray_get(g->nodes, i, &node);
            //printf("Reading node: %d/%d \r", i, zarray_size(g->nodes));
            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");

            if (gm == NULL) {
                continue;
            }
            add_model_data_attr(node);
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_xyt(node->state),
                                              vxo_robot_solid((float[]) { 0, .5, .5, 1 }),
                                              NULL),
                               NULL);
            global_map_add_gridmap(state->compositor, gm, node->state);
        }
        vx_buffer_swap(vb);
        redraw_global_map(state);
        //printf("\nFinish loading graph.\n");
    }

    if(strlen(getopt_get_string(gopt, "log-file-folder"))){
        const char *log_folder_path = getopt_get_string(gopt, "log-file-folder");

        struct dirent *de;  // Pointer for directory entry
        DIR *dr = opendir(log_folder_path);
        if (dr == NULL)  // opendir returns NULL if couldn't open directory
            {
                printf("Could not open current directory" );
                exit(-1);
            }

        state->log_files_path = zarray_create(sizeof(char*));
        while ((de = readdir(dr)) != NULL) {
            if(!strcmp(de->d_name, ".") || !strcmp(de->d_name, ".."))
                continue;
            char *path = sprintf_alloc("%s/%s", getopt_get_string(gopt, "log-file-folder"), de->d_name);
            zarray_add(state->log_files_path, &path);
            zarray_sort(state->log_files_path, str_cmp);
        }
        /* for(int i = 0; i < zarray_size(state->log_files_path); i++) { */
        /*     char *path; */
        /*     zarray_get(state->log_files_path, i, &path); */
        /*     printf("%s\n", path); */
        /* } */
    }

    // Training
    {
        //TODO: Localization graph node selection initialization.
        srand(utime_now());
        if(strlen(getopt_get_string(gopt, "resume-file"))) {
            initialize_selection_from_resume_file(state, getopt_get_string(gopt, "resume-file"));
            if(strlen(getopt_get_string(gopt, "out-file")) != 0)
                state->f = fopen(getopt_get_string(gopt, "out-file"), "w");
            else
                state->f = fopen(sprintf_alloc("%s.resume-result", getopt_get_string(gopt, "resume-file")), "w");
            if(!state->f) {
                exit(-1);
            }

        } else if(strlen(getopt_get_string(gopt, "bootstrapping-file"))) {
            initialize_selection_from_bootstrap(state, getopt_get_string(gopt, "bootstrapping-file"));
            if(strlen(getopt_get_string(gopt, "out-file")) != 0)
                state->f = fopen(getopt_get_string(gopt, "out-file"), "w");
            else
                state->f = fopen(sprintf_alloc("%s.bootstrap-result", getopt_get_string(gopt, "bootstrapping-file")), "w");
            if(!state->f) {
                exit(-1);
            }
        } else {
            initialize_selection(state, MAX_NUM_OF_NODES);
            //initialize_selection_equal_space(state, MAX_NUM_OF_NODES);
            if(strlen(getopt_get_string(gopt, "out-file")) != 0)
                state->f = fopen(getopt_get_string(gopt, "out-file"), "w");
            else
                state->f = fopen(sprintf_alloc("/tmp/%d.result", getopt_get_int(gopt, "port")), "w");
            if(!state->f) {
                exit(-1);
            }
        }

        signal(SIGINT, signal_handler);
        state->switch_picked_array = zarray_create(sizeof(int[2]));
        state->switch_picked[0] = -1; //not pick this
        state->switch_picked[1] = -1; //pick this
        state->switch_id = -1;
        state->nonSelectIdx = zarray_create(sizeof(int));
        state->selectIdx = zarray_create(sizeof(int));
        int nthreads = workerpool_get_nprocs() - 1;
        //printf("Running on %d threads \n", nthreads);
        if(zarray_size(state->log_files_path) <= nthreads) {
            nthreads = zarray_size(state->log_files_path);
        }
        workerpool_t *wp = workerpool_create(nthreads);
        wp_data_t *wp_data = calloc(nthreads, sizeof(wp_data_t));
        for(int i = 0; i < nthreads; i++) {
            wp_data[i].state = calloc(1,sizeof(state_t));
            memcpy(wp_data[i].state, state, sizeof(state_t));
            wp_data[i].heading_errors = zarray_create(sizeof(double));
            wp_data[i].distance_errors = zarray_create(sizeof(double));
            wp_data[i].id = i;
            wp_data[i].nthreads = nthreads;
        }
        for( int iter=0; iter<MAX_ITERS; iter++) {
            //puts("===============================");
            //printf("Iteration :%d/%d \n", iter, MAX_ITERS);
            for(int i = 0; i < nthreads; i++) {
                workerpool_add_task(wp, wp_func, &wp_data[i]);
            }
            workerpool_run(wp);
            for(int i = 0; i < nthreads; i++) {
                for (int j = 0; j < zarray_size(wp_data[i].heading_errors); j++) {
                    double heading_err = ((double*)wp_data[i].heading_errors->data)[j];
                    double dist_err = ((double*)wp_data[i].distance_errors->data)[j];
                    update_err_rec(&state->dist_err_rec, dist_err);
                    update_err_rec(&state->heading_err_rec, heading_err);
                }
                zarray_clear(wp_data[i].heading_errors);
                zarray_clear(wp_data[i].distance_errors);
            }

            if(iter==0 || extract_err_total(&state->dist_err_rec) < extract_err_total(&state->last_dist_error_rec)) {
                if(iter != 0) {
                    hill_climbing_switch(state);
                    state->improved = true;
                }
                memcpy(&state->last_dist_error_rec, &state->dist_err_rec, sizeof(error_rec_t));
            }
            hill_climbing_pick(state);


            //printf("Last distance error: %.2f\n", extract_err_total(&state->last_dist_error_rec) / state->last_dist_error_rec.npoint);

            //printf("Distance error: mu :%.2f, var: %.2f, total: %.2f \n",
            //extract_mu(&state->dist_err_rec), extract_var(&state->dist_err_rec), extract_err_total(&state->dist_err_rec));


            if(iter % ITER_PER_SAVE == 0)
                save_result(state);

            memset(&state->dist_err_rec, 0, sizeof(error_rec_t));
        }
        fprintf(state->f, "=\n");
        save_result(state);
        /* printf("Distance error: mu :%.2f, var: %.2f, total: %.2f \n", */
        /*        extract_mu(&state->dist_err_rec), extract_var(&state->dist_err_rec), extract_err_total(&state->dist_err_rec)); */

        /* printf("Heading error: mu :%.2f, var: %.2f \n", extract_mu(&state->heading_err_rec), extract_var(&state->heading_err_rec)); */
    }

    /* while(1) { */
    /*     usleep(100E3); */
    /* } */
    printf("Last distance error: %.2f\n", extract_err_total(&state->last_dist_error_rec) / state->last_dist_error_rec.npoint);
    return 0;
}
