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

#include "sim/sim.h"
#include "sim/sim_agent.h"

#include "apriltag/apriltag.h"
#include "apriltag/tag36artoolkit.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag36h11.h"

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

#include "lcm/lcm.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/grid_map_t.h"

#include "scanmatch/scanmatch.h"

#define MAX_ITER 100
#define SEARCH_RANGE 2
#define CAR_WIDTH 0.5
#define CAR_LENGTH 0.6
#define MAX_RANGE 4
#define METER_PER_PIXEL 0.05

/**
   TODO:
   1. Generate laser noise based roughness of the object specfied in configure. (Done)
   2. If robot didn't move, we don't write any events into the file.
   3. RRT for planning and automaticly generate path

 */

typedef struct error_rec error_rec_t;
struct error_rec {
    // compute covariances and expectations
    //Cxx = mXX/nPoints - pow(mX/nPoints,2);
    //Ex  = mX/nPoints;
    //mX -= x;
    //mXX -= x^2;
    int id;
    double total;
    double mX, mXX;
    int npoint;
};

typedef struct state state_t;
struct state {
    lcm_t *lcm;

    //VX
    vx_world_t *vw;
	webvx_t *webvx;
    vx_console_t *vx_console;

    //Simworld
    sim_world_t *simworld;
    vx_object_t *vxo_robot;
    sim_agent_t * agent;

    pthread_mutex_t mutex;
    double xyt[3];

    laser_t *laser;

    zarray_t *objects; //double[8]->4 corner points of objects;

    april_graph_t *graph;
    zarray_t *graph_heading_err;
    zarray_t *graph_dist_err;
};

typedef struct hokuyo_inner hokuyo_inner_t;
struct hokuyo_inner {
    double lidar_M[16];
    int nodding;
    double nod_high, nod_low;
    double nod_period;
};

bool SAVE;
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

void sense_nodding(state_t *state)
{
    sim_agent_t * agent = state->agent;
    hokuyo_inner_t *inner = agent->sense_state;
    if(1) {
        laser_t msg;
        memset(&msg, 0, sizeof(laser_t));
        msg.utime = agent->time;
        msg.rad0 = -M_PI + to_radians(0);
        msg.nranges = 1441;
        msg.radstep = to_radians(360) / (msg.nranges - 1);
        msg.ranges = calloc(msg.nranges, sizeof(float));
        msg.nintensities = msg.nranges;
        msg.intensities = calloc(msg.nranges, sizeof(float));

        double pose_M[16];
        doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, pose_M);

        double M[16];
        doubles_mat_AB(pose_M, 4, 4, inner->lidar_M, 4, 4, M, 4, 4);

        laser_t msg_n = msg;
        msg_n.ranges = calloc(msg.nranges, sizeof(float));
        msg_n.intensities = calloc(msg.nranges, sizeof(float));


        if (inner->nodding) {
            assert(0);
        }

        double lidar_pos[3] = { M[4*0+3], M[4*1+3], M[4*2+3] };

        for (int i = 0; i < msg.nranges; i++) {
            double rad = msg.rad0 + msg.radstep * i;
            double _dir[3] = { cos(rad), sin(rad), 0 };
            double dir[3];
            doubles_mat44_rotate_vector(M, _dir, dir);

            double max_range = MAX_RANGE;
            sim_object_t *out_object;
            double r = sim_world_ray_cast(state->simworld, lidar_pos, dir, max_range,
                                          (sim_object_t*[]) { agent->so_self }, 1, &out_object);
            //printf("roughness: %f\n", out_object->u.box.roughness);
            msg.ranges[i] = r < max_range ? r : -1;
            msg.intensities[i] = 1;
            if(msg.ranges[i] != -1 && out_object) {
                double err = randf_normal() * (fabs(1-out_object->u.box.roughness)/10);
                msg_n.ranges[i] = msg.ranges[i] + err;
            } else {
                msg_n.ranges[i] = -1;
            }
            msg_n.intensities[i] = 1;
        }
        if(state->laser) {
            laser_t_destroy(state->laser);
        }
        state->laser = laser_t_copy(&msg);
        free(msg.intensities);
        free(msg_n.intensities);
        free(msg.ranges);
        free(msg_n.ranges);
    }
}

void render_robot(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "Robot");
    vx_buffer_add_back(vb,
                       vxo_matrix_quat_xyz(state->agent->pose.orientation,
                                           state->agent->pose.pos),
                       state->vxo_robot,
                       NULL);
    vx_buffer_swap(vb);
    if (state->laser) {
        laser_t *msg = state->laser;
        sim_agent_t * agent = state->agent;
        struct hokuyo_inner *sensor_state = agent->sense_state;
        double pose_M[16];
        doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, pose_M);
        double M[16];
        doubles_mat_AB(pose_M, 4, 4, sensor_state->lidar_M, 4, 4, M, 4, 4);

        zarray_t *xyzs = zarray_create(sizeof(float[3]));
        for (int i = 0; i < msg->nranges; i++) {
            double rad = msg->rad0 + msg->radstep * i;
            if (msg->ranges[i] < 0)
                continue;
            float p[3] = { msg->ranges[i] * cos(rad),
                           msg->ranges[i] * sin(rad),
                           0 };
            zarray_add(xyzs, p);
        }
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "hokuyo");
        vx_resource_t *resc = vx_resource_make_attr_f32_copy((float*) xyzs->data, 3*zarray_size(xyzs), 3);
        vx_buffer_add_back(vb,
                           vxo_matrix(M),
                           vxo_points(resc, (float[]) { 1, 1, 0, 1 }, 2),
                           NULL);
        vx_buffer_swap(vb);
        zarray_destroy(xyzs);
    }

    /* double current_xyt[3]; */
    /* doubles_quat_xyz_to_xyt(state->pose.orientation, state->pose.pos, current_xyt); */
    /* double eye[3] = {current_xyt[0], current_xyt[1], 50.0}; */
    /* double lookat[3] = {current_xyt[0], current_xyt[1], 0.0}; */
    /* double up[3] = {0.0, 1.0, 0.0}; */
    /* vx_world_set_elu(state->vw, eye, lookat, up, 0); */
}

void laser_at_xyt(state_t *state, double xyt[3])
{
    sim_agent_t *agent = state->agent;
    memcpy(agent->pose.pos, xyt, sizeof(double)*2);
    double rpy[3] = {0, 0, xyt[2]};
    doubles_rpy_to_quat(rpy, agent->pose.orientation);
    doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, agent->so_self->T);
}

//NOTE: inplace update prior_xyt
void evaluate_at_xyt(state_t *state, april_graph_node_t*node, double prior_xyt[3])
{
    sim_agent_t *agent = state->agent;
    memcpy(agent->pose.pos, prior_xyt, sizeof(double)*2);
    double rpy[3] = {0, 0, prior_xyt[2]};
    doubles_rpy_to_quat(rpy, agent->pose.orientation);
    doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, agent->so_self->T);

    sense_nodding(state);

    zarray_t *points = laser_to_floats(state->laser);
    sm_points_data_t *points_data = sm_points_data_create(points);
    sm_model_data_t *model_data = april_graph_node_attr_get(node, "model_data");
    assert (model_data);

    double pred_xyt[3];
    // use current state estimate for mean (instead of dijkstra)
    // Tnode = state->prior_xyt
    doubles_xyt_inv_mul(node->state, prior_xyt, pred_xyt);

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
    double impp = 1.0 / METER_PER_PIXEL;
    double minscore = -1;
    const int ONLY_POINT_MATCH = 1;
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
                                               .initial_step_sizes = { METER_PER_PIXEL / 2,
                                                                       METER_PER_PIXEL / 2,
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

        if(1) {
            render_proposed_match(state, xyt,
                                  model_data, points_data);
        }

        // Update pose for next scan match
        doubles_xyt_mul(node->state, xyt, prior_xyt);
        sm_result_destroy(sres);
    }
    sm_search_destroy(search);
    sm_points_data_destroy(points_data);
}

static int key_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    //state_t *state = impl;
    char key = ev->u.key.key_code;
    if(ev->type == VX_EVENT_KEY_DOWN) {
        switch (key) {
            case 27: //ESC
                break;
            default:
                break;
        }
    }
    if(ev->type == VX_EVENT_KEY_PRESSED) {
        if (key == 'c') {
        }
        else if(key == 'f') {
        }
    }

    return 0;
}

april_graph_node_t* find_node(state_t *state, double xyt[3])
{
    april_graph_node_t *node;
    april_graph_node_t *ret;
    double dist = DBL_MAX;
    for(int i = 0; i<zarray_size(state->graph->nodes); i++) {
        zarray_get(state->graph->nodes, i, &node);
        if(doubles_distance(node->state, xyt, 2) < dist) {
            ret = node;
            dist = doubles_distance(node->state, xyt, 2);
        }
    }
    return ret;
}

static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
	state_t *state = impl;
    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);
    double plane_xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, plane_xyz);
    if(1) {
        if (ev->flags & VX_EVENT_FLAGS_SHIFT) {
            state->xyt[2] = atan2(plane_xyz[1] - state->xyt[1], plane_xyz[0] - state->xyt[0]);
            april_graph_node_t* node = find_node(state, state->xyt);
            double xyt[3];
            memcpy(xyt, state->xyt, 3*sizeof(double));
            evaluate_at_xyt(state, node, xyt);
            printf("err:%f,%f,%f \n", state->xyt[0]-xyt[0], state->xyt[1]-xyt[1], to_degrees(mod2pi(state->xyt[2]- xyt[2])));
            render_robot(state);
        } else {
            state->xyt[0] = plane_xyz[0];
            state->xyt[1] = plane_xyz[1];
            laser_at_xyt(state, state->xyt);
            sense_nodding(state);
            render_robot(state);
        }
    }
    return 0;
}

void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    //state_t *state = user;

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

static inline void sim_agent_noop(void *impl)
{
    return;
}

void create_robots(state_t *state, config_t *cf)
{
    int n_obj = 0;
    char key[32];
    sprintf(key, "robot%d.", n_obj);
    config_set_prefix(cf, key);
    while (config_has_key(cf, "id")) {
        sim_agent_t * agent = calloc(1, sizeof(sim_agent_t));
        agent->id = config_require_int(cf, "id");
        n_obj++;
        agent->impl = state;
        agent->manipulate = sim_agent_noop;
        agent->manipulate_state = NULL;
        agent->move = sim_agent_noop;
        agent->move_state = NULL;

        if(config_has_key(cf, "hokuyo.pos")) {
            agent->sense = sim_agent_noop;
            struct hokuyo_inner *sense_state = calloc(1,sizeof(struct hokuyo_inner));
            sense_state->nodding    = config_get_boolean(cf, "hokuyo.nodding",    0);
            sense_state->nod_period = config_get_double(cf,  "hokuyo.nod_period", 3);
            sense_state->nod_low    = config_get_double(cf,  "hokuyo.nod_low",    -15); //degrees
            sense_state->nod_high   = config_get_double(cf,  "hokuyo.nod_high",   15); //degrees
            agent->sense_state = sense_state;

            double pos[3];
            config_require_doubles_len(cf, "hokuyo.pos", pos, 3);
            doubles_mat44_identity(sense_state->lidar_M);
            sense_state->lidar_M[3] = pos[0];
            sense_state->lidar_M[7] = pos[1];
            sense_state->lidar_M[11] = pos[2];
        } else {
            agent->sense = sim_agent_noop;
            agent->sense_state = NULL;
        }

        double dims[3];
        config_require_doubles_len(cf, "dims", dims, 3);
        float M[16];
        floats_mat44_identity(M);
        M[0] = dims[0];
        M[5] = dims[1];
        M[10] = dims[2];

        config_require_doubles_len(cf, "pos", agent->pose.pos, 3);
        double rpy[3];
        config_require_doubles_len(cf, "rpy", rpy, 3);
        doubles_rpy_to_quat(rpy, agent->pose.orientation);
        float rgba[] = {0,0,0,0};
        double T[16];
        doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, T);

        //T is the pose; M is the size
        sim_object_t * so = sim_object_box_create(T, M, rgba, 1.0);
        so->world.valid = 1;
        so->world.mesh_last_version = so->mesh_version;

        agent->so_self = so;

        zarray_add(state->simworld->objects, &so);
        zarray_add(state->simworld->agents, &agent);

        sprintf(key, "robot%d.", n_obj);
        config_set_prefix(cf, key);
        printf("%d, %f,%f,%f\n", agent->id, agent->pose.pos[0], agent->pose.pos[1], agent->pose.pos[2]);
        state->agent = agent; //hack: assume only an agent.
    }
    config_set_prefix(cf, NULL);
    fprintf(stderr, "Made %d robots\n", n_obj);

    {
        int n_obj = 0;
        char key[32] = "sim_object0.";
        config_set_prefix(cf, key);
        while(config_has_key(cf, "pos"))
        {

            double dims[3];
            config_require_doubles_len(cf, "dims", dims, 3);
            double xyzrpy[6];
            config_require_doubles_len(cf, "pos", xyzrpy, 3);
            double cx = xyzrpy[0]; double cy = xyzrpy[1];
            double half_x = dims[0] / 2; double half_y = dims[1] / 2;
            double rect[8] = {cx-half_x, cy-half_y, cx+half_x, cy-half_y, cx+half_x, cy+half_y, cx-half_x, cy+half_y };
            if(dims[2] > 1)
                zarray_add(state->objects, rect);
            n_obj++;
            sprintf(key, "sim_object%d.", n_obj);
            config_set_prefix(cf, key);
        }
    }
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

static double gaussian_sample(double mu, double variance)
{
    return mu + sqrt(variance) * randf_normal();
}

int err_compar(const void *_a, const void *_b)
{
    error_rec_t *a = *((error_rec_t**)_a);
    error_rec_t *b = *((error_rec_t**)_b);
    //TODO: Based on variance and mu, but not total
    if(extract_err_total(a) < extract_err_total(b)) {
        return -1;
    } else if(extract_err_total(a) == extract_err_total(b)) {
        //XXX: double compare equality
        return 0;
    } else {
        return 1;
    }
}
void bootstrap_selected_node(state_t *state)
{
    april_graph_t *g = state->graph;
    april_graph_node_t *node;
    error_rec_t *dist_err;
    error_rec_t *heading_err;
    srand(utime_now());
    for(int i = 0; i < zarray_size(g->nodes); i++) {
        zarray_get(g->nodes, i, &node);
        zarray_get(state->graph_dist_err , i, &dist_err);
        zarray_get(state->graph_heading_err , i, &heading_err);

        for(int iter = 0; iter < MAX_ITER; iter++) {
            double xyt_noise[3] = { node->state[0] + gaussian_sample(0, 0.5),
                                    node->state[1] + gaussian_sample(0, 0.5),
                                    mod2pi(node->state[2] + gaussian_sample(0, to_radians(10))) };

            //TODO: input should have noises as well.
            double xyt_input[3] = {
                xyt_noise[0],
                xyt_noise[1],
                xyt_noise[2]
            };
            evaluate_at_xyt(state, node, xyt_input);
            double d_err = doubles_distance(xyt_input, xyt_noise, 2);
            update_err_rec(dist_err, d_err);
            double t_err = fabs(mod2pi((xyt_input[2] - xyt_noise[2])));
            update_err_rec(heading_err, t_err);
            render_robot(state);
        }
       printf("Node_%d :Distance error: mu :%.2f, var: %.2f, total: %.2f \n",
              i, extract_mu(dist_err), extract_var(dist_err), extract_err_total(dist_err));
    }
    zarray_sort(state->graph_dist_err, err_compar);
    //save results
    if(SAVE) {
        FILE *f = fopen("/tmp/bootstrap.list", "w");
        if(f == NULL) {
            printf("Wrong bootstrap file \n");
            exit(-1);
        }
        for(int i = 0; i < zarray_size(g->nodes); i++) {
            zarray_get(state->graph_dist_err , i, &dist_err);
            /* printf("Node_%d :Distance error: mu :%.2f, var: %.2f, total: %.2f \n", */
            /*        dist_err->id, extract_mu(dist_err), extract_var(dist_err), extract_err_total(dist_err)); */
            fprintf(f, "%d ", dist_err->id);
        }
        fprintf(f, "\n=\n");
        for(int i = 0; i < zarray_size(g->nodes); i++) {
            zarray_get(state->graph_dist_err , i, &dist_err);
            fprintf(f, "%.2f, %.2f, %.2f \n",
                    extract_mu(dist_err), extract_var(dist_err), extract_err_total(dist_err));
        }
        fclose(f);
    }
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 's', "save", 0, "Save results");
    getopt_add_string(gopt, 'c', "config", "/home/april/magic-lite/config/simworld-bbb.config", "Specify the world configure file");
    getopt_add_string(gopt, 'g', "graph-file", "", "Specify the graph file to generate random path");
    getopt_add_int(gopt, 'p', "port", "8888", "webvx port");
    printf("Port: %d\n", getopt_get_int(gopt,"port"));

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }
    SAVE = getopt_get_bool(gopt, "save");
    state_t *state = calloc(1, sizeof(state_t));

    state->lcm = lcm_create(NULL);

    state->simworld = sim_world_create_from_file(getopt_get_string(gopt,"config"));
    if (!state->simworld) {
        printf("World file load failed\n");
        exit(1);
    }
    config_t *config = config_create_path(getopt_get_string(gopt, "config"));
    if (!config) {
        printf("Config file load failed\n");
        exit(1);
    }

    //Make a nice robot
    apriltag_family_t *tf = tag36h11_create();
    image_u8_t *tag_im = apriltag_to_image(tf, 2);
    state->vxo_robot = vxo_chain(vxo_matrix_translate(0,0,0),
                                 vxo_matrix_scale3(CAR_LENGTH, CAR_WIDTH, 1),
                                 vxo_chain(
                                     vxo_matrix_scale3(0.5,0.5,0.3),
                                     vxo_box_solid(vx_navy),
                                     vxo_chain(vxo_matrix_translate(0.5,0,0),
                                               vxo_matrix_scale3(0.5,0.5,0.3),
                                               vxo_sphere_solid(vx_blue),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(0.3,0.55,0),
                                               vxo_matrix_scale3(0.2,0.1,0.5),
                                               vxo_sphere_solid(vx_black),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(-0.3,0.55,0),
                                               vxo_matrix_scale3(0.2,0.1,0.5),
                                               vxo_sphere_solid(vx_black),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(0.3,-0.55,0),
                                               vxo_matrix_scale3(0.2,0.1,0.5),
                                               vxo_sphere_solid(vx_black),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(-0.3,-0.55,0),
                                               vxo_matrix_scale3(0.2,0.1,0.5),
                                               vxo_sphere_solid(vx_black),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(-0.5,-0.5,0.6),
                                               vxo_matrix_scale(0.1),
                                               vxo_image_u8(tag_im, 0),
                                               NULL),
                                     vxo_chain(vxo_matrix_translate(-0.15,0,0.3),
                                               vxo_matrix_scale3(1,2,1),
                                               vxo_robot_solid(vx_black),
                                               NULL),
                                     NULL),
                                 NULL);
    state->vxo_robot->incref(state->vxo_robot);
    state->objects = zarray_create(sizeof(double[8]));
    create_robots(state, config);

    state->vw = vx_world_create();
    state->vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                          on_console_command,
                                          on_console_tab,
                                          state);
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"), NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "model");
        for (int i = 0; i < zarray_size(state->simworld->objects); i++) {
            sim_object_t *so;
            zarray_get(state->simworld->objects, i, &so);
            if (!so->world.valid || so->world.mesh_last_version != so->mesh_version) {
                vx_object_t *vxo = so->world.mesh_last_vxo;

                if (so->world.valid)
                    vxo->decref(vxo);

                so->world.mesh_last_version = so->mesh_version;
                vxo = vxo_mesh_model_create(so->mesh);
                so->world.mesh_last_vxo = vxo;
                vxo->incref(vxo);
                so->world.valid = 1;
                //printf("created mesh %p\n", vxo);
            }
            vx_buffer_add_back(vb,
                               vxo_matrix(so->T),
                               so->world.mesh_last_vxo,
                               NULL);
        }
        vx_buffer_swap(vb);
    }

    {
        stype_register_basic_types();
        april_graph_stype_init();
        stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
        stype_register(STYPE_FROM_LCM("laser_t", laser_t));
        state->graph = april_graph_create_from_file(getopt_get_string(gopt, "graph-file"));
        if(state->graph == NULL) {
            printf("no graph loaded\n");
            exit(-1);
        }
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "graph-node");
        for (int i = 0; i < zarray_size(state->graph->nodes); i++) {
            april_graph_node_t *node;
            zarray_get(state->graph->nodes, i, &node);
            node->UID = i;
            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
            if (gm == NULL) {
                continue;
            }

            add_model_data_attr(node);
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_translate(0, 0, 1),
                                              vxo_matrix_xyt(node->state),
                                              vxo_robot_solid((float[]) { 0, .5, .5, 1 }),
                                              NULL),
                               NULL);
        }
        vx_buffer_swap(vb);

        state->graph_dist_err = zarray_create(sizeof(error_rec_t*));
        state->graph_heading_err = zarray_create(sizeof(error_rec_t*));
        for(int i = 0; i<zarray_size(state->graph->nodes); i++) {
            error_rec_t *dist_err = calloc(1, sizeof(error_rec_t));
            dist_err->id = i;
            zarray_add(state->graph_dist_err, &dist_err);
            error_rec_t *heading_err = calloc(1, sizeof(error_rec_t));
            heading_err->id = i;
            zarray_add(state->graph_heading_err, &heading_err);
        }

    }
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "MOSS-bootstrapping", "MOSS bootstrapping tool");
    bootstrap_selected_node(state);
    while (0) {
        lcm_handle(state->lcm);
    }
    return 0;
}
