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

#include "vx/vx.h"
#include "vx/webvx.h"

#include "lcm/lcm.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/laser_t.h"

#define SIM_FREQ 1000

#define CAR_WIDTH 0.5
#define CAR_LENGTH 0.6
#define MAX_RANGE 4
/**
   TODO:
   1. Generate laser noise based roughness of the object specfied in configure. (Done)
   2. If robot didn't move, we don't write any events into the file.
   3. RRT for planning and automaticly generate path

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

    pthread_mutex_t mutex;

    pose_t pose;
    laser_t *laser;

    diff_drive_t diff_drive_command;
    bool debug;

    bool moved;
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

static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
	//state_t *state = impl;
    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);
    double plane_xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, plane_xyz);
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
    if (state->simworld->paused) {
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

void sense_nodding(void *impl)
{
    state_t *state = impl;
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

void render_robot(void *user)
{
    state_t *state = user;
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
        double left_n = inner->diff_drive->left * randf_uniform(0.99, 1.01);
        double right_n = inner->diff_drive->right * randf_uniform(0.99, 1.01);

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
    //doubles_quat_rotate(agent->pose.orientation, dpos_in_n, dpos_n);
    doubles_quat_rotate(inner->pose_command.orientation, dpos_in_n, dpos_n);

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
        //doubles_quat_multiply(inner->pose_command.orientation, dq, tmp);
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

    //doubles_quat_xyz_to_mat44(agent->pose.orientation, agent->pose.pos, agent->so_self->T);
    doubles_quat_xyz_to_mat44(inner->pose_command.orientation, inner->pose_command.pos, agent->so_self->T);

    agent->pose.utime = agent->time;
    inner->pose_command.utime = agent->time;

    state_t *state = agent->impl;
    if(forward > 0.01) {
        state->moved = true;
    } else {
        state->moved = false;
    }
    if(state->moved) {
        if(state->eventlog_write)
            write_pose_event(state->eventlog_write, &inner->pose_command, "POSE");

        if(state->eventlog_write)
            write_pose_event(state->eventlog_write, &agent->pose, "POSE_NOISE");
    }
}

void *moving(void * impl)
{
    state_t *state = impl;
    sim_agent_t * agent = state->agent;
    timeutil_rest_t *rt = timeutil_rest_create();
    render_robot(state);
    while(1) {
        timeutil_sleep_hz(rt, SIM_FREQ);
        pthread_mutex_lock(&state->mutex);
        diff_drive_inner_t * inner = agent->move_state;
        if (inner->diff_drive)
            diff_drive_t_destroy(inner->diff_drive);
        inner->diff_drive = diff_drive_t_copy(&state->diff_drive_command);
        update_diff_drive(agent);
        sense_nodding(state);
        if(state->debug)
            render_robot(state);
        pthread_mutex_unlock(&state->mutex);
    }
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
    }
    config_set_prefix(cf, NULL);
    fprintf(stderr, "Made %d robots\n", n_obj);
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'd', "debug", 1, "debug mode");
    getopt_add_string(gopt, 'c', "config", "/home/april/magic-lite/config/simworld-bbb.config", "Specify the world configure file");
    getopt_add_string(gopt, 'f', "log-file", "", "Specify the log file to write");
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
