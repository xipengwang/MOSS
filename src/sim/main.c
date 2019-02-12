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
#include "common/stype_lcm.h"

#include "vx/vx.h"
#include "vx/webvx.h"

#include "april_graph/april_graph.h"

#include "lcm/lcm.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/grid_map_t.h"

#define SIM_FREQ 1000
#define HUMAN 0
#define OBJECT 1
#define CAR_WIDTH 0.5
#define CAR_LENGTH 0.6
#define MAX_RANGE 4
#define RUNNING_BUDGET_S 20
#define SAFE_MARGIN 0.25

/**
   TODO:
   1. XXX: When robot agent doesn't move, the rendering doesn't update.
*/
typedef struct state state_t;
struct state {
    lcm_t *lcm;
    lcm_eventlog_t *eventlog_write;

    //VX
    vx_world_t *vw;
	webvx_t *webvx;
    vx_console_t *vx_console;

    //Simworld
    sim_world_t *simworld;
    vx_object_t *vxo_robot;
    sim_agent_t * agent;
    sim_agent_t * agent_human;

    pthread_mutex_t mutex;

    pose_t pose;
    laser_t *laser;

    diff_drive_t diff_drive_command;
    bool debug;

    bool moved;
    bool start_log;
    zarray_t *objects; //double[8]->4 corner points of objects;

    april_graph_t *graph;
    double rect[8];
    double start[3];
    double end[3];
};

typedef struct diff_drive_inner diff_drive_inner_t;
struct diff_drive_inner {
    diff_drive_t *diff_drive;
    pose_t pose_command;
};

typedef struct hokuyo_inner hokuyo_inner_t;
struct hokuyo_inner {
    double lidar_M[16];
    int nodding;
    double nod_high, nod_low;
    double nod_period;
};

static lcm_eventlog_event_t *make_event(int64_t timestamp,
                                        int64_t channellen,
                                        int32_t datalen,
                                        char *channel,
                                        void *data)
{
    lcm_eventlog_event_t *event = calloc(1, sizeof(lcm_eventlog_event_t));
    event->timestamp = timestamp;
    event->channellen = channellen;
    event->datalen = datalen;
    event->channel = strdup(channel);
    event->data = data;

    return event;
}

void write_pose_event(lcm_eventlog_t *eventlog_write, pose_t *pose, char *channel)
{
    int32_t datalen = pose_t_encoded_size(pose);
    void *data = malloc(datalen);
    pose_t_encode(data, 0, datalen, pose);
    lcm_eventlog_event_t *event = make_event(pose->utime,
                                             strlen(channel),
                                             datalen,
                                             channel,
                                             data);
    lcm_eventlog_write_event(eventlog_write, event);
    lcm_eventlog_free_event(event);
}

void write_laser_event(lcm_eventlog_t *eventlog_write, laser_t *laser, char *channel)
{
    int32_t datalen = laser_t_encoded_size(laser);
    void *data = malloc(datalen);
    laser_t_encode(data, 0, datalen, laser);
    lcm_eventlog_event_t *event = make_event(laser->utime,
                                             strlen(channel),
                                             datalen,
                                             channel,
                                             data);
    lcm_eventlog_write_event(eventlog_write, event);
    lcm_eventlog_free_event(event);
}

void read_pose_event(lcm_eventlog_event_t *event, pose_t *pose)
{
    assert(0 < pose_t_decode(event->data, 0, event->datalen, pose));
}

bool check_lineline_collision(double p1[2], double p2[2], double v1[2], double v2[2])
{
    double det = (p2[0] - p1[0])*(v2[1]-v1[1]) - (v2[0]-v1[0])*(p2[1]-p1[1]);
    if(det == 0)
        return false;

    //line 1: r = p1 + lambda * d1, where d1 = p2-p1
    double lambda = ((v2[0]-v1[0])*(p1[1]-v1[1]) - (v2[1]-v1[1])*(p1[0]-v1[0])) / det;
    double gamma = ((p2[0]-p1[0])*(p1[1]-v1[1]) - (p2[1]-p1[1])*(p1[0]-v1[0])) / det;
    return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
}

bool check_collision(double p1[2], double p2[2], state_t *state)
{
    zarray_t *objects = state->objects;
    //double ** object_data = objects->data;
    bool ret = false;
    for (int i = 0; i<zarray_size(objects); i++){
        double rect[8];
        zarray_get(objects, i, rect);
        //double *rect = object_data[i];
        for(int j = 0; j < 4; j++) {
            double v1[2] = {rect[(2*j)%8], rect[(2*j+1)%8]}; double v2[2] = {rect[(2*j+2)%8], rect[(2*j+3)%8]};
            //printf("(%f, %f), (%f, %f), (%f, %f), (%f, %f)", p1[0], p1[1], p2[0], p2[1], v1[0], v1[1], v2[0], v2[1]);
            if(check_lineline_collision(p1, p2, v1, v2)) {
                ret = true;
                break;
            }
        }
        //printf("----------------\n");
        if(ret)
            return ret;
    }
    if(HUMAN) {
        for(int j = 0; j < 4; j++) {
            double v1[2] = {state->rect[(2*j)%8], state->rect[(2*j+1)%8]}; double v2[2] = {state->rect[(2*j+2)%8], state->rect[(2*j+3)%8]};
            //printf("(%f, %f), (%f, %f), (%f, %f), (%f, %f)", p1[0], p1[1], p2[0], p2[1], v1[0], v1[1], v2[0], v2[1]);
            if(check_lineline_collision(p1, p2, v1, v2)) {
                ret = true;
                break;
            }
        }
    }
    return ret;
}

double point_to_line_dist(double p[2], double v1[2], double v2[2])
{
    //v(t) = v1 + t(v2-v1)
    //t = <p-s1, s2-s1> / |s2-s1|^2
    double t = ((p[0]-v1[0])*(v2[0]-v1[0]) + (p[1]-v1[1])*(v2[1]-v1[1])) / (sq(v2[1] - v1[1]) + sq(v2[0] - v1[0]));
    //printf("point:%.2f, %.2f, %.2f\n", t, v1[0]+t*(v2[0]-v1[0]), v1[1]+t*(v2[1]-v1[1]));
    if(t > 0 && t < 1 ) {
        double dist = sqrt(sq(v1[0]+t*(v2[0]-v1[0]) - p[0]) + sq(v1[1]+t*(v2[1]-v1[1]) - p[1]));
        //double dist_2 = fabs((v2[1] - v1[1])*p[0] - (v2[0]-v1[0])*p[1] + v2[0]*v1[1] - v2[1]*v1[0]) / sqrt(sq(v2[1] - v1[1]) + sq(v2[0] - v1[0]));
        return dist;
    } else {
        double dist_1 = sqrt(sq(v1[0] - p[0]) + sq(v1[1] - p[1]));
        double dist_2 = sqrt(sq(v2[0] - p[0]) + sq(v2[1] - p[1]));
        if(dist_1 < dist_2)
            return dist_1;
        else
            return dist_2;
    }
}

double calc_distance(state_t *state, double p[2])
{
    zarray_t *objects = state->objects;
    //double ** object_data = objects->data;
    double ret = DBL_MAX;
    for (int i = 0; i<zarray_size(objects); i++) {
        double rect[8];
        zarray_get(objects, i, rect);
        //double *rect = object_data[i];
        for(int j = 0; j < 4; j++) {
            double v1[2] = {rect[(2*j)%8], rect[(2*j+1)%8]}; double v2[2] = {rect[(2*j+2)%8], rect[(2*j+3)%8]};
            //printf("(%f, %f), (%f, %f), (%f, %f), (%f, %f)", p1[0], p1[1], p2[0], p2[1], v1[0], v1[1], v2[0], v2[1]);
            double dist = point_to_line_dist(p, v1, v2);
            if(dist < ret) {
                ret = dist;
            }
        }
    }
    assert(ret > 0);
    return ret;
}

static int key_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;
    char key = ev->u.key.key_code;
    if(ev->type == VX_EVENT_KEY_DOWN) {
        pthread_mutex_lock(&state->mutex);
        switch (key) {
        case 37: //Left
            state->diff_drive_command.right = 0.5;
            state->diff_drive_command.left = -0.5;
            break;
        case 38: //Up
            state->diff_drive_command.right = 0.7;
            state->diff_drive_command.left = 0.7;
            break;
        case 39: //Right
            state->diff_drive_command.right = -0.5;
            state->diff_drive_command.left = 0.5;
            break;
        case 40: //Down
            state->diff_drive_command.right = -0.7;
            state->diff_drive_command.left = -0.7;
            break;
        case ' ':
            state->diff_drive_command.right = 0;
            state->diff_drive_command.left = 0;
            break;
        case 27: //ESC
            break;
        default:
            break;
        }
        pthread_mutex_unlock(&state->mutex);
    }
    if(ev->type == VX_EVENT_KEY_PRESSED) {
        if (key == 'c') {
        }
        else if(key == 'f') {
        }
    }

    return 0;
}

void test_collision(state_t *state, double xyz[3])
{
    double p1[2] = {state->agent->pose.pos[0], state->agent->pose.pos[1]};
    double p2[2] = {xyz[0], xyz[1]};
    //plot a line;
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "Lines");
    float points[4] = {state->agent->pose.pos[0], state->agent->pose.pos[1], xyz[0], xyz[1]};
    vx_resource_t *points_resc = (vx_resource_t *)vx_resource_make_attr_f32_copy(points, 4, 2);
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_depth_test(0,
                                                     vxo_matrix_translate(0, 0, 0.05),
                                                     vxo_line_strip(points_resc, vx_green, 2),
                                                     NULL),
                                      NULL),
                       NULL);

    if(check_collision(p1, p2, state)) {
        vx_buffer_add_back(vb, vxo_matrix_translate(p1[0], p1[1], 0.05),
                           vxo_square_solid(vx_red), NULL);
        vx_buffer_add_back(vb, vxo_matrix_translate(p2[0], p2[1], 0.05),
                           vxo_square_solid(vx_red), NULL);
    } else {
        vx_buffer_add_back(vb, vxo_matrix_translate(p1[0], p1[1], 0.05),
                           vxo_square_solid(vx_yellow), NULL);
        vx_buffer_add_back(vb, vxo_matrix_translate(p2[0], p2[1], 0.05),
                           vxo_square_solid(vx_yellow), NULL);
    }
    printf("================\n");
    vx_buffer_swap(vb);
}

void generate_start_end_points(state_t *state, double start[3], double end[2], bool set);
static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
	state_t *state = impl;
    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);
    double plane_xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, plane_xyz);
    if (ev->flags == 0) {
        state->start[0] = plane_xyz[0];
        state->start[1] = plane_xyz[1];
    }
    if (ev->flags == VX_EVENT_FLAGS_CTRL) {
        state->start[2] = atan2(plane_xyz[1] - state->start[1], plane_xyz[0] - state->start[0]);
    }
    if (ev->flags == VX_EVENT_FLAGS_SHIFT) {
        state->end[0] = plane_xyz[0];
        state->end[1] = plane_xyz[1];
        generate_start_end_points(state, state->start, state->end, false);
        doubles_quat_xyz_to_xyt(state->agent->pose.orientation, state->agent->pose.pos, state->start);

    }
    return 0;
}


void* generate_path(void *user){
    state_t *state = user;
    double start[3], end[2];
    generate_start_end_points(state, start, end, true);
    return NULL;
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
    if (!strcmp(tok, "random")) {
        pthread_t path_pt;
        pthread_create(&path_pt, NULL, generate_path, state);
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
                               "random",
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
    //if (state->simworld->paused)
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
     double eye[3] = {5,0,30};
     double lookat[3] = {5,5,0};
     double up[3] = {0 ,1, 0};
     vx_world_set_elu(state->vw, eye, lookat, up, 0);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
	printf("ON DESTROY CANVAS\n");
}

static inline void sim_agent_noop(void *impl)
{
    return;
}

void sense_nodding(state_t *state, sim_agent_t * agent)
{
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
            //TODO: Based on roughness to simulate different objects.
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
        if(state->moved) {
            if(state->eventlog_write)
                write_laser_event(state->eventlog_write, &msg, "LASER");
            if(state->eventlog_write)
                write_laser_event(state->eventlog_write, &msg_n, "LASER_NOISE");
        }
        free(msg.intensities);
        free(msg_n.intensities);
        free(msg.ranges);
        free(msg_n.ranges);
    }
}

void render_model(state_t *state);
void render_robot(void *user)
{
    state_t *state = user;
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "Robot");
    vx_buffer_add_back(vb,
                       vxo_matrix_quat_xyz(state->agent->pose.orientation,
                                           state->agent->pose.pos),
                       state->vxo_robot,
                       NULL);
    if(HUMAN) {
        vx_buffer_add_back(vb,
            vxo_matrix_quat_xyz(state->agent_human->pose.orientation,
                state->agent_human->pose.pos),
            state->vxo_robot,
            NULL);
        vx_buffer_add_back(vb,
            vxo_matrix_translate(0, 0, 0.5),
            vxo_matrix_quat_xyz(state->agent_human->pose.orientation,
                state->agent_human->pose.pos),
            vxo_matrix_scale(0.4),
            vxo_circle_line(vx_white,1),
            state->vxo_robot,
            NULL);

    }
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

void update_diff_drive(sim_agent_t * agent)
{
    //get the current moving state
    struct diff_drive_inner *inner = agent->move_state;
    //state_t * state = agent->impl;
    if (agent->prev_time == 0) {
        agent->prev_time = utime_now();
        return;
    }
    agent->time = utime_now();
    double dt = (agent->time - agent->prev_time) / 1.0E6;
    agent->prev_time = agent->time;

    // Commands
    double forward = 0, turn = 0;
    // add noise to commands to calculate robot real pose.
    double forward_n = 0, turn_n = 0;

    if(inner->diff_drive) {
        //generate noises
        double left_n = inner->diff_drive->left;// * randf_uniform(0.99, 1.01);
        double right_n = inner->diff_drive->right;// * randf_uniform(0.99, 1.01);

        forward = 1.0 * dt * (inner->diff_drive->left + inner->diff_drive->right);
        turn = 1.0 * dt * (inner->diff_drive->right - inner->diff_drive->left);

        forward_n = 1.0 * dt * (left_n + right_n);
        turn_n = 1.0 * dt * (right_n - left_n);
    }

    // Assumes at start position, we know exactly where the robot is
    if(inner->pose_command.utime == 0) {
        inner->pose_command = agent->pose;
    }

    double dpos_in[3] = { forward, 0, 0 };
    double dpos_in_n[3] = { forward_n, 0, 0 };
    double dpos[3];
    double dpos_n[3];

    doubles_quat_rotate(inner->pose_command.orientation, dpos_in, dpos);
    doubles_quat_rotate(agent->pose.orientation, dpos_in_n, dpos_n);

    for (int i = 0; i < 3; i++) {
        inner->pose_command.pos[i] += dpos[i];
        agent->pose.pos[i] += dpos_n[i];
    }

    if (1) {

        double aa[4] = { turn_n, 0, 0, 1 };
        double dq[4];
        doubles_angleaxis_to_quat(aa, dq);

        double tmp[4];
        doubles_quat_multiply(agent->pose.orientation, dq, tmp);
        memcpy(agent->pose.orientation, tmp, 4 * sizeof(double));
    }


    if (1) {
        double aa[4] = { turn, 0, 0, 1 };
        double dq[4];
        doubles_angleaxis_to_quat(aa, dq);

        double tmp[4];
        doubles_quat_multiply(inner->pose_command.orientation, dq, tmp);
        memcpy(inner->pose_command.orientation, tmp, 4 * sizeof(double));
    }

    doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, agent->so_self->T);
    doubles_quat_xyz_to_mat44(inner->pose_command.orientation, inner->pose_command.pos, agent->so_self->T);

    agent->pose.utime = agent->time;
    inner->pose_command.utime = agent->time;

    state_t *state = agent->impl;
    /* if(forward > 0.01) { */
    /*     state->moved = true; */
    /* } else { */
    /*     state->moved = false; */
    /* } */
    if(state->moved) {
        if(state->eventlog_write)
            write_pose_event(state->eventlog_write, &inner->pose_command, "POSE");

        /* if(state->eventlog_write) */
        /*     write_pose_event(state->eventlog_write, &agent->pose, "POSE_NOISE"); */
    }
}


void push_object(state_t *state, double xy[2]);

void pop_object(state_t *state);

void *moving(void * impl)
{
    state_t *state = impl;
    sim_agent_t * agent = state->agent_human;
    timeutil_rest_t *rt = timeutil_rest_create();
    render_robot(state);
    while(1) {
        timeutil_sleep_hz(rt, SIM_FREQ);
        if(!HUMAN)
            continue;
        pthread_mutex_lock(&state->mutex);
        diff_drive_inner_t * inner = agent->move_state;
        if (inner->diff_drive)
            diff_drive_t_destroy(inner->diff_drive);
        inner->diff_drive = diff_drive_t_copy(&state->diff_drive_command);
        update_diff_drive(agent);
        render_robot(state);
        {
            double dims[3] = {2, 2, 4};
            double xyt[3];
            doubles_quat_xyz_to_xyt(agent->pose.orientation, agent->pose.pos, xyt);
            double xyzrpy[6] = {xyt[0], xyt[1], 0, 0, 0, xyt[2]};
            const double safe_margin  = SAFE_MARGIN;
            double cx = xyzrpy[0]; double cy = xyzrpy[1];
            double half_x = dims[0] / 2; double half_y = dims[1] / 2;
            double tmp_rect[8] = {cx-half_x-safe_margin, cy-half_y-safe_margin,
                                   cx+half_x+safe_margin, cy-half_y-safe_margin,
                                   cx+half_x+safe_margin, cy+half_y+safe_margin,
                                   cx-half_x-safe_margin, cy+half_y+safe_margin};
            memcpy(state->rect, tmp_rect, sizeof(double)*8);
        }
        pthread_mutex_unlock(&state->mutex);
    }
}

sim_agent_t * create_human_agent(state_t* state, double xyt[3])
{
    sim_agent_t * agent = calloc(1, sizeof(sim_agent_t));
    agent->id = 110;
    agent->impl = state;
    agent->manipulate = sim_agent_noop;
    agent->manipulate_state = NULL;
    agent->move = sim_agent_noop;
    agent->move_state = calloc(1,sizeof(struct diff_drive_inner));
    agent->sense = sim_agent_noop;
    agent->sense_state = NULL;


    double dims[3] = {0.5, 0.5, 1};
    float M[16];
    floats_mat44_identity(M);
    M[0] = dims[0];
    M[5] = dims[1];
    M[10] = dims[2];

    agent->pose.pos[0] = xyt[0];
    agent->pose.pos[1] = xyt[1];
    double rpy[3] = {0,0,xyt[2]};
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
    return agent;
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
        agent->move_state = calloc(1,sizeof(struct diff_drive_inner));

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
        doubles_quat_xyz_to_xyt(state->agent->pose.orientation, state->agent->pose.pos, state->start);
    }
    config_set_prefix(cf, NULL);
    fprintf(stderr, "Made %d robots\n", n_obj);
    double human_agent_xyt[3] = {2, 0, 0};
    if(HUMAN) {
        state->agent_human = create_human_agent(state, human_agent_xyt);
    }
    {
        int n_obj = 0;
        char key[32] = "sim_object0.";
        config_set_prefix(cf, key);
        const double safe_margin  = SAFE_MARGIN;
        while(config_has_key(cf, "pos"))
        {

                double dims[3];
                config_require_doubles_len(cf, "dims", dims, 3);
                double xyzrpy[6];
                config_require_doubles_len(cf, "pos", xyzrpy, 3);
                double cx = xyzrpy[0]; double cy = xyzrpy[1];
                double half_x = dims[0] / 2; double half_y = dims[1] / 2;
                double rect[8] = {cx-half_x-safe_margin, cy-half_y-safe_margin,
                                  cx+half_x+safe_margin, cy-half_y-safe_margin,
                                  cx+half_x+safe_margin, cy+half_y+safe_margin,
                                  cx-half_x-safe_margin, cy+half_y+safe_margin};
                if(dims[2] > 1)
                    zarray_add(state->objects, rect);
                n_obj++;
                sprintf(key, "sim_object%d.", n_obj);
                config_set_prefix(cf, key);
        }
    }
}

typedef struct RRT_node RRT_node_t;
struct RRT_node {
    double xy[2];
    RRT_node_t *parent;
};

RRT_node_t *find_nearest_node(zarray_t *node_list, double xy[2])
{
    RRT_node_t *node;
    RRT_node_t *ret_node = NULL;
    double minDist  = DBL_MAX;
    for(int i = 0; i < zarray_size(node_list); i++) {
        zarray_get(node_list, i, &node);
        double dist = doubles_distance(node->xy, xy, 2);
        if(dist < minDist) {
            minDist = dist;
            ret_node = node;
        }
    }
    return ret_node;
}
// User need to free element in zarray
// Search path from the last node.
zarray_t * RRT_path_search(state_t* state, const double start[2], const double goal[2])
{
    const double minX = -2, maxX = 20;
    const double minY = -4, maxY = 4;
    const double expand_dist = 0.5;
    const int MaxIter = 1000;
    bool achieved = false;
    zarray_t *node_list = zarray_create(sizeof(RRT_node_t*));
    RRT_node_t *s_node = calloc(1, sizeof(RRT_node_t));
    memcpy(s_node->xy, start, 2*sizeof(double));
    RRT_node_t *g_node = calloc(1, sizeof(RRT_node_t));
    memcpy(g_node->xy, goal, 2*sizeof(double));

    zarray_add(node_list, &s_node);
    for(int iter = 0; iter < MaxIter; iter++) {
        double next_xy[2];
        if(randi_uniform(0, 100) < 5) {
            //sample goal point
            next_xy[0] = goal[0];
            next_xy[1] = goal[1];
        } else {
            //random sample
            next_xy[0] = randf_uniform(minX, maxX);
            next_xy[1] = randf_uniform(minY, maxY);
        }
        RRT_node_t* p_node =  find_nearest_node(node_list, next_xy);
        assert(p_node);
        if(check_collision(p_node->xy, next_xy, state)) {
            continue;
        }

        double theta = atan2(next_xy[1]-p_node->xy[1], next_xy[0]-p_node->xy[0]);
        RRT_node_t *new_node = calloc(1, sizeof(RRT_node_t));
        new_node->xy[0] = p_node->xy[0] + expand_dist * cos(theta);
        new_node->xy[1] = p_node->xy[1] + expand_dist * sin(theta);
        new_node->parent = p_node;
        zarray_add(node_list, &new_node);

        if(doubles_distance(new_node->xy, goal, 2) < expand_dist) {
            achieved = true;
            g_node->parent = new_node;
            zarray_add(node_list, &g_node);
            break;
        }
    }
    if(achieved) {
        return node_list;
    } else {
        for(int i = 0; i < zarray_size(node_list); i++) {
            RRT_node_t *node;
            zarray_get(node_list, i, &node);
            free(node);
        }
        zarray_destroy(node_list);
        return NULL;
    }
}

//X: (x y theta v yawrate)
static void motion(double x[5], double u[2], double dt)
{
    x[0] += u[0] * cos(x[2]) * dt;
    x[1] += u[0] * sin(x[2]) * dt;
    x[2] += u[1] * dt;
    x[3] = u[0];
    x[4] = u[1];
}

static zarray_t *calculate_traj(double x[5], double v, double yaw, double dt, double predict_time)
{
    zarray_t *ret_traj = zarray_create(sizeof(double[5]));
    zarray_add(ret_traj, x);
    //zarray_add(ret_traj, x);
    double time = 0;
    do{
        double u[2] = {v, yaw};
        motion(x, u, dt);
        zarray_add(ret_traj, x);
        time += dt;
    } while(time < predict_time);
    return ret_traj;
}

double calc_obstacle_cost(state_t *state, zarray_t *traj)
{
    double *x = (double*)traj->data;
    double min_dist = 100000000;
    for(int i = 0; i < zarray_size(traj)-1; i++) {
        if(check_collision(&x[i*5], &x[i*5+5], state)) {
            return -1;
        }
    }
    for(int i = 0; i < zarray_size(traj); i++) {
        double dist = calc_distance(state, &x[i*5]);
        if(dist < min_dist) {
            min_dist = dist;
        }
    }
    return 0;
    //return 1.0 / min_dist;
}

void plot_traj(state_t *state, zarray_t *traj, float color[4])
{
    zarray_t *plot_traj = zarray_create(sizeof(float[2]));
    double x[5];
    for(int i=0; i<zarray_size(traj); i++) {
        zarray_get(traj, i, x);
        float _x[2] = {x[0], x[1]};
        zarray_add(plot_traj, _x);
    }
    vx_resource_t *points = vx_resource_make_attr_f32_copy((float*)plot_traj->data, zarray_size(traj) * 2, 2);
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "Points");
    vx_buffer_add_back(vb,
                       vxo_matrix_translate(0 ,0, 0.05),
                       vxo_points(points, color, 1),
                       NULL);
    vx_buffer_swap(vb);
    zarray_destroy(plot_traj);
}

void move_robot(state_t *state, double xyt[3]);
zarray_t * dynamic_window_search(state_t *state, const double start[3], const double goal[2])
{
    //Configure robot parameter
    bool achieved = false;
    const double max_v = 0.5, min_v = 0;
    const double max_yawrate = to_radians(40);
    const double max_accel_v = 0.2, max_accel_yawrate = to_radians(90);
    const double v_reso = 0.01, yawrate_reso = to_radians(1);
    const double dt = 0.1;
    const double predict_window = 3;
    const double v_cost_gain = 1, to_goal_cost_gain = 1, ob_cost_gain = 0.1;
    const double robot_radius = 0.5;
    zarray_t *ret_traj = zarray_create(sizeof(double[5]));
    //initialize state: x,y,theta,v,yawrate
    double x[5]= {start[0], start[1], start[2], 0, 0};
    zarray_add(ret_traj, x);
    move_robot(state, x);
    //render_robot(state);
    //printf("state: %f,%f,%f,%f,%f \n", x[0],x[1],x[2],x[3],x[4]);

    //for(int iter = 0; iter < 1000; iter++) {
    int64_t now = utime_now();
    while(1){
        pthread_mutex_lock(&state->mutex);
        if((utime_now() - now) * 1E-6 > RUNNING_BUDGET_S)
            break;

        double windowT[4] = { max(x[3]-max_accel_v*dt, min_v), min(x[3]+max_accel_v*dt, max_v),
                              max(x[4]-max_accel_yawrate*dt, -max_yawrate),  min(x[4]+max_accel_yawrate*dt, max_yawrate) };
        //printf("%f, %f, %f, %f\n", windowT[0], windowT[1], windowT[2], windowT[3]);
        double min_cost = DBL_MAX;
        zarray_t *best_traj = zarray_create(sizeof(double[5]));
        double v_yaw[2];
        double v = windowT[0];
        do{
            double yaw = windowT[2];
            do {
                double tmp_x[5];
                memcpy(tmp_x, x, sizeof(double)*5);
                zarray_t *traj = calculate_traj(tmp_x, v, yaw, dt, predict_window);
                double to_goal_cost =  doubles_distance(goal, tmp_x, 2);
                double speed_cost =   (max_v - tmp_x[3]);
                double ob_cost =  calc_obstacle_cost(state, traj);
                double final_cost = DBL_MAX;
                if(ob_cost != -1) {
                    final_cost =  to_goal_cost_gain * to_goal_cost + v_cost_gain * speed_cost + ob_cost_gain * ob_cost;
                    if(min_cost > final_cost) {
                        min_cost = final_cost;
                        zarray_destroy(best_traj);
                        best_traj = traj;
                        v_yaw[0] = v;
                        v_yaw[1] = yaw;
                    } else {
                        zarray_destroy(traj);
                    }
                }
                //printf("%f,%f,%f,%f,%f\n", v, yaw, speed_cost, ob_cost, to_goal_cost);
                yaw += yawrate_reso;
            } while(yaw < windowT[3]);
            v += v_reso;
        } while(v < windowT[1]);
        // motion
        //printf("best: %f,%f \n", v_yaw[0], v_yaw[1]);
        motion(x, v_yaw, dt);
        //printf("state: %f,%f,%f,%f,%f \n", x[0],x[1],x[2],x[3],x[4]);
        if(doubles_distance(x, goal, 2) < robot_radius) {
            achieved = true;
            break;
        }
        //TODO: plot best_traj;
        zarray_add(ret_traj, x);
        move_robot(state, x);
        render_robot(state);
        plot_traj(state, best_traj, vx_green);
        zarray_destroy(best_traj);
        pthread_mutex_unlock(&state->mutex);
        usleep(1E3);
    }
    pthread_mutex_unlock(&state->mutex);
    plot_traj(state, ret_traj, vx_blue);
    //assert(calc_obstacle_cost(state, ret_traj) != -1);
    if(achieved) {
        return ret_traj;
    } else {
        zarray_destroy(ret_traj);
        return NULL;
    }
}

void move_robot(state_t *state, double xyt[3])
{
    //get the current moving state
    sim_agent_t *agent = state->agent;
    agent->time = utime_now();
    agent->prev_time = agent->time - 100;
    agent->pose.pos[0] = xyt[0];
    agent->pose.pos[1] = xyt[1];
    double rpy[3] = {0, 0, xyt[2]};
    doubles_rpy_to_quat(rpy, agent->pose.orientation);
    doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, agent->so_self->T);
    agent->pose.utime = agent->time;

    if(state->eventlog_write)
        write_pose_event(state->eventlog_write, &agent->pose, "POSE");
    state->moved = true;
    sense_nodding(state,state->agent);
}

void push_object(state_t *state, double xy[2])
{
    sim_world_t *sw = state->simworld;

    float rgbaf[4] = {1, 0, 1, 1};

    double xyzrpy[6] = {xy[0], xy[1], 0, 0, 0, 0};

    double dims[3] = {0.2, 0.2, 4};

    double shine = 0.5;

    double T[16];
    doubles_xyzrpy_to_mat44(xyzrpy, T);
    float M[16];
    floats_mat44_identity(M);
    M[0] = dims[0];
    M[5] = dims[1];
    M[10] = dims[2];

    sim_object_t * so = sim_object_box_create(T, M, rgbaf, shine);
    zarray_add(sw->objects, &so);

    const double safe_margin  = SAFE_MARGIN;
    double cx = xyzrpy[0]; double cy = xyzrpy[1];
    double half_x = dims[0] / 2; double half_y = dims[1] / 2;
    double rect[8] = {cx-half_x-safe_margin, cy-half_y-safe_margin,
                      cx+half_x+safe_margin, cy-half_y-safe_margin,
                      cx+half_x+safe_margin, cy+half_y+safe_margin,
                      cx-half_x-safe_margin, cy+half_y+safe_margin};
    if(dims[2] > 1)
        zarray_add(state->objects, rect);
}

void pop_object(state_t *state)
{
    sim_world_t *sw = state->simworld;
    zarray_remove_index(sw->objects, zarray_size(sw->objects)-1, 0);
    zarray_remove_index(state->objects, zarray_size(state->objects)-1, 0);
}

void render_model(state_t *state)
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

void generate_start_end_points(state_t *state, double start[3], double end[2], bool set)
{
    april_graph_t *g = state->graph;
    int N = zarray_size(g->nodes);
    int start_id = randi_uniform(0, N);
    int end_id = randi_uniform(0, N);
    april_graph_node_t *start_node;
    april_graph_node_t *end_node;
    zarray_get(g->nodes, start_id, &start_node);
    zarray_get(g->nodes, end_id, &end_node);
    srand(utime_now());
    while(set) {
        start[0] = start_node->state[0] + randf_normal() * 2;
        start[1] = start_node->state[1] + randf_normal() * 2;
        end[0] = end_node->state[0] + randf_normal() * 2;
        end[1] = end_node->state[1] + randf_normal() * 2;
        start[2] = mod2pi(randi_uniform(0, to_radians(360)));
        if(!check_collision(start, start_node->state, state) && !check_collision(end, end_node->state, state)) {
            break;
        }
    }
    //start[0] = -1.5; start[1]=0; start[2]=0; end[0] = 18; end[1]=0;
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "Start_End");

    vx_buffer_add_back(vb, vxo_matrix_translate(start[0], start[1], 0.05),
                       vxo_matrix_scale(0.2),
                       vxo_square_solid(vx_green), NULL);
    vx_buffer_add_back(vb, vxo_matrix_translate(end[0], end[1], 0.05),
                       vxo_matrix_scale(0.2),
                       vxo_square_solid(vx_red), NULL);
    vx_buffer_swap(vb);
    printf("start:(%f,%f), goal:(%f,%f)\n", start[0], start[1], end[0], end[1]);

    if(0) {
        zarray_t *node_list =  RRT_path_search(state, start, end);
        if(node_list) {
            vx_buffer_t* vb = vx_world_get_buffer(state->vw, "Path");
            RRT_node_t *node;
            zarray_get(node_list, zarray_size(node_list)-1, &node);
            while(node->parent) {
                vx_buffer_add_back(vb, vxo_matrix_translate(node->xy[0], node->xy[1], 0.05),
                                   vxo_matrix_scale(0.1),
                                   vxo_square_solid(vx_blue), NULL);
                node = node->parent;
            }
            vx_buffer_swap(vb);
            for(int i = 0; i<zarray_size(node_list); i++) {
                RRT_node_t *node;
                zarray_get(node_list, i, &node);
                free(node);
            }
            zarray_destroy(node_list);
        } else {
            puts("No Path!");
        }
    }
    if (1) {
        //double start_pose[3] = {start[0], start[1], atan2(end[1]-start[1], end[0]-start[0])};
        double xy[2] = { (end[0]+start[0])/2, (end[1]+start[1])/2 };
        if(OBJECT) {
            push_object(state, xy);
        }
        render_model(state);
        zarray_t *traj = dynamic_window_search(state, start, end);
        if(OBJECT) {
            pop_object(state);
        }
        if(traj) {
            zarray_destroy(traj);
        }
    }


    //Find closest graph point
    /* double s_dist = DBL_MAX; */
    /* double e_dist = DBL_MAX; */
    /* for(int i = 0; i<zarray_size(g->nodes); i++) { */
    /*     april_graph_node_t *node; */
    /*     zarray_get(g->nodes, start_id, &node); */
    /*     double start_dist = doubles_distance(start, node->state, 2); */
    /*     double end_dist = doubles_distance(end, node->state, 2); */
    /*     if(start_dist < s_dist) { */
    /*         s_dist = start_dist; */
    /*         start_id = i; */
    /*     } */
    /*     if(end_dist < e_dist) { */
    /*         e_dist = end_dist; */
    /*         end_id = i; */
    /*     } */
    /* } */
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'd', "debug", 1, "debug mode");
    getopt_add_string(gopt, 'c', "config", "/home/april/magic-lite/config/simworld-bbb.config", "Specify the world configure file");
    getopt_add_string(gopt, 'f', "log-file", "", "Specify the log file to write");
    getopt_add_string(gopt, 'g', "graph-file", "", "Specify the graph file to generate random path");
    getopt_add_int(gopt, 'p', "port", "8888", "webvx port");
    printf("Port: %d\n", getopt_get_int(gopt,"port"));

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }
    bool debug = getopt_get_bool(gopt, "debug");
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

    if (!debug) {
        printf("Debug is false!\n");
		vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
		vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
		vx_buffer_swap(vb);
	}

    if (debug) {
        printf("Debug mode \n");
        printf("-----------------------\n");
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
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_translate(0, 0, 1),
                                              vxo_matrix_xyt(node->state),
                                              vxo_robot_solid((float[]) { 0, .5, .5, 1 }),
                                              NULL),
                               NULL);
        }
        vx_buffer_swap(vb);

        //double start[3], end[2];
        //printf("Generate points!\n");
        //generate_start_end_points(state, start, end);
    }

    pthread_mutex_init(&state->mutex, NULL);
    state->laser = NULL;
    state->debug = debug;

    pthread_t moving_pt;
    pthread_create(&moving_pt, NULL, moving, state);
    state->eventlog_write = NULL;

    if(strlen(getopt_get_string(gopt, "log-file"))){
        const char *log_path = getopt_get_string(gopt, "log-file");
        state->eventlog_write = lcm_eventlog_create(log_path, "w");
    }
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "MOSS-sim", "MOSS sim tool");
    while (1) {
        lcm_handle(state->lcm);
    }
    return 0;
}
