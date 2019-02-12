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

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "loopval.h"
#include "common/math_util.h"

#include "common/doubles.h"

loopval_t *loopval_create(int max_depth, int vote_thresh, double t_thresh, double r_thresh)
{
    loopval_t *loopval = calloc(1, sizeof(loopval_t));
    loopval->hyp_lists = zarray_create(sizeof(zarray_t*));
    loopval->graduates = zarray_create(sizeof(struct loopval_hypothesis*));
    loopval->all_hyps = zarray_create(sizeof(struct loopval_hypothesis*));
    loopval->vote_thresh = vote_thresh;
    loopval->max_depth = max_depth;
    loopval->t_thresh = t_thresh;
    loopval->r_thresh = r_thresh;

    return loopval;
}

// for node 'a', get all of the hypotheses that involve 'a'
static zarray_t *get_hypotheses(loopval_t *loopval, int a)
{
    // make sure we've allocated edge lists for node a (and any earlier nodes)
    while (a >= zarray_size(loopval->hyp_lists)) {
        zarray_t *list = zarray_create(sizeof(struct loopval_hypothesis*));
        zarray_add(loopval->hyp_lists, &list);
    }

    zarray_t *hypotheses;
    zarray_get(loopval->hyp_lists, a, &hypotheses);
    return hypotheses;
}

// startnode: initial node that we started from
// endnode: the last visited node (from which we will depart)
// xyt: the cumulative transform from startnode to endnode
// visited: a list of the hypotheses already visited (of size 'depth')
// depth: the number of nodes visited so far.

static void recurse(loopval_t *loopval, int startnode, int endnode, double xyt[3],
                    struct loopval_hypothesis **visited, int depth)
{

    if (startnode == endnode) {
        // we have a loop. is it any good?

        double dist = sqrt(xyt[0]*xyt[0] + xyt[1]*xyt[1]);
        double dtheta = fabs(mod2pi(xyt[2]));

        //printf("%d %d %f %f\n", depth, startnode, dist, dtheta);
        if (dist < loopval->t_thresh && dtheta < loopval->r_thresh) {
            // all of the edges in this loop get upvoted.
            for (int i = 0; i < depth; i++) {
                struct loopval_hypothesis *hypothesis = visited[i];
                hypothesis->nloops++;

                // does this hypothesis graduate?
                if (!hypothesis->graduated && hypothesis->nloops > loopval->vote_thresh) {
                    hypothesis->graduated = 1;
                    zarray_add(loopval->graduates, &hypothesis);
                    //printf("graduate %p (%d, %d) \n", hypothesis, hypothesis->a, hypothesis->b);
                }
            }
        }
        return;
    }

    if (depth == loopval->max_depth) {
        return;
    }

    // expand 'endnode'
    zarray_t *hypotheses = get_hypotheses(loopval, endnode);

    for (int i = 0; i < zarray_size(hypotheses); i++) {
        struct loopval_hypothesis *hypothesis;
        zarray_get(hypotheses, i, &hypothesis);

        // cannot be already visited
        int already_visited = 0;

        for (int j = 0; j < depth; j++) {
            if (hypothesis == visited[j]) {
                already_visited = 1;
                break;
            }
        }

        if (!already_visited) {

            if(hypothesis->a == endnode) {
                double xytr[3];
                doubles_xyt_mul(xyt, hypothesis->xyt, xytr);

                visited[depth] = hypothesis;
                recurse(loopval, startnode, hypothesis->b, xytr, visited, depth + 1);
            }else if(hypothesis->b == endnode) {
                double inv[3];
                doubles_xyt_inv(hypothesis->xyt,inv);

                double xytr[3];
                doubles_xyt_mul(xyt, inv, xytr);

                visited[depth] = hypothesis;
                recurse(loopval, startnode, hypothesis->a, xytr, visited, depth + 1);
            }else{
                assert(0);
            }
        }
    }
}

// graduated: if 1, we won't report it as graduated even if it passes.
// user: if we end up graduating the xyt, this is what we hand back user.

// XXX: only add LIDAR edges?
//
//
// XXX: don't add edges that don't correspond to some critical amount of motion. ("slipping" usually results in significant under-estimates of motion).

void loopval_add(loopval_t *loopval, int a, int b, const double xyt[3], april_graph_factor_t * factor,int graduated)
{
    // create the hypothesis
    struct loopval_hypothesis *hypothesis = calloc(1, sizeof(struct loopval_hypothesis));
    hypothesis->a = a;
    hypothesis->b = b;
    hypothesis->nloops = 0;
    hypothesis->factor = factor;
    hypothesis->graduated = graduated;
    memcpy(hypothesis->xyt, xyt, 3 * sizeof(double));

    zarray_add(loopval->all_hyps, &hypothesis);

    // and two edges, one in each direction..
    if (1) {
        zarray_t *hypotheses = get_hypotheses(loopval, a);
        zarray_add(hypotheses, &hypothesis);
    }

    if (1) {
        zarray_t *hypotheses = get_hypotheses(loopval, b);
        zarray_add(hypotheses, &hypothesis);
    }

    // with just this edge so far, the cumulative_xyt is just the edge's xyt.
    double cumulative_xyt[3];
    memcpy(cumulative_xyt, xyt, 3 * sizeof(double));

    // constructe the 'visited' list
    struct loopval_hypothesis *visited[loopval->max_depth];
    visited[0] = hypothesis;

    // and go!
    recurse(loopval, a, b, cumulative_xyt, visited, 1);

    double cumulative_xyt_inv[3];
    doubles_xyt_inv(cumulative_xyt, cumulative_xyt_inv);
    recurse(loopval, b, a, cumulative_xyt_inv, visited, 1);
}

void loopval_remove_node(loopval_t * loopval, const int idx){
    zarray_t * list;
    for(int i = 0; i < zarray_size(loopval->hyp_lists); i++) {
        if(i == idx) continue;

        list = get_hypotheses(loopval, i);
        for(int j = 0; j < zarray_size(list); j++) {
            struct loopval_hypothesis *  hypothesis;
            zarray_get(list, j, &hypothesis);
            if(hypothesis->a == idx || hypothesis->b == idx){
                zarray_remove_index(list, j, 1);
                j--;
            }
        }
    }

    list = loopval->graduates;
    for(int j = 0; j < zarray_size(list); j++) {
        struct loopval_hypothesis * hypothesis;
        zarray_get(list, j, &hypothesis);
        if(hypothesis->a == idx || hypothesis->b == idx) {
            zarray_remove_index(list, j, 1);
            j--;
        }
    }

    list = loopval->all_hyps;
    for(int j = 0; j < zarray_size(list); j++) {
        struct loopval_hypothesis * hypothesis;
        zarray_get(list, j, &hypothesis);
        if(hypothesis->a == idx || hypothesis->b == idx) {
            zarray_remove_index(list, j, 1);
            j--;
        }
    }

    //printf("REMOVE %d\n", idx);
    list = get_hypotheses(loopval, idx);
    for(int j = 0; j < zarray_size(list); j++) {
        struct loopval_hypothesis * hypothesis;
        zarray_get(list, j, &hypothesis);
        if(hypothesis->factor)
            hypothesis->factor->destroy(hypothesis->factor);

        free(hypothesis);
    }

    zarray_remove_index(loopval->hyp_lists, idx, 0);
    zarray_destroy(list);

    list = loopval->all_hyps;
    for(int j = 0; j < zarray_size(list); j++) {
        struct loopval_hypothesis * hypothesis;
        zarray_get(list, j, &hypothesis);
        assert(hypothesis->a != idx);
        assert(hypothesis->b != idx);
        if(hypothesis->a > idx)
            hypothesis->a--;
        if(hypothesis->b > idx)
            hypothesis->b--;
        if(hypothesis->factor){
            april_graph_factor_t * factor = hypothesis->factor;

            for(int n = 0; n < factor->nnodes; n++) {
                assert(factor->nodes[n] != idx);
                if(factor->nodes[n] > idx)
                    factor->nodes[n]--;
            }

            if(factor->type == APRIL_GRAPH_FACTOR_MAX_TYPE) {
                april_graph_factor_t * ifactor = NULL;
                for(int f = 0; f < factor->u.max.nfactors; f++) {
                    ifactor = factor->u.max.factors[f];
                    assert(ifactor->type != APRIL_GRAPH_FACTOR_MAX_TYPE);
                    for(int n = 0; n < ifactor->nnodes; n++) {
                        assert(ifactor->nodes[n] != idx);
                        if(ifactor->nodes[n] > idx)
                            ifactor->nodes[n]--;
                    }
                }
            }

        }
    }
}

// XXX need some criterion to remove hypotheses that are bad.
void loopval_remove_bad(loopval_t *loopval)
{
}

// retrieve any graduate / retrieve the 'user' value of any graduated edges since the last
// time this function was called, or NULL if there are no new ones.
april_graph_factor_t * loopval_get_graduate(loopval_t *loopval)
{
    if (zarray_size(loopval->graduates) == 0)
        return NULL;

    struct loopval_hypothesis *hypothesis;
    zarray_get(loopval->graduates, 0, &hypothesis);
    zarray_remove_index(loopval->graduates, 0, 1);

    april_graph_factor_t * ret = hypothesis->factor;
    hypothesis->factor = NULL;
    return ret;
}
