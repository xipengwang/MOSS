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

#ifndef _LOOP_VALIDATOR_H
#define _LOOP_VALIDATOR_H

#include "common/zarray.h"
#include "common/matd_coords.h"
#include "april_graph/april_graph.h"

// corresponds to a single hypothesis added to our system.
struct loopval_hypothesis {
    int a, b;

    // how many good loops was this hypothesis part of?
    int nloops;

    // data specified by user
    april_graph_factor_t * factor;

    // is this a "good" edge?
    int graduated;

    // transform that projects b into coordinate frame of a
    double xyt[3];
};

typedef struct loopval loopval_t;
struct loopval {
    // how many loops does each loop need to be part of in order to "graduate"?
    int vote_thresh;

    // how good do the loops need to be? (translation and radian thresholds)
    double t_thresh, r_thresh;

    // maximum number of edges in a loop.
    int max_depth;

    // indexed by node, a zarray_t of edges coming out of it.
    // each added hypothesis will have two edges, one in each direction.
    zarray_t *hyp_lists; // zarray_t*<struct loopval_hypothesis*>

    // all of the hypothesis that have graduated but not yet returned by get_graduate.
    zarray_t *graduates;
    zarray_t *all_hyps;
};

loopval_t *loopval_create(int max_depth, int votethresh, double t_thresh, double r_thresh);


void loopval_add(loopval_t *loopval, int a, int b, const double xyt[3], april_graph_factor_t * factor,int graduated);
void loopval_remove_node(loopval_t * loopval, int a);

// retrieve the 'user' value of any graduated edges since the last
// time this function was called, or NULL if there are no new ones.
april_graph_factor_t *loopval_get_graduate(loopval_t *loopval);


#endif
