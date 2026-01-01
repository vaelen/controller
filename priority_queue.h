/*
 * Pass Priority Queue for Satellite Tracking Controller
 *
 * Fixed-size array-based min-heap for scheduling satellite passes.
 * Prioritizes passes by start time for non-overlapping passes,
 * and by maximum elevation for overlapping passes.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <stdbool.h>

/* Maximum number of passes in the priority queue */
#define PASS_QUEUE_CAPACITY 64

/*
 * Pass information structure (simplified for queue storage).
 * Does not include pass plan steps - those are generated on-the-fly
 * by the pass executor during execution.
 */
typedef struct pass_info {
    int norad_id;               /* Satellite NORAD catalog ID */
    double aos_jd;              /* Acquisition of signal (Julian Date) */
    double los_jd;              /* Loss of signal (Julian Date) */
    double max_elevation_rad;   /* Maximum elevation angle (radians) */
    double aos_azimuth_rad;     /* Azimuth at AOS (radians) */
    double los_azimuth_rad;     /* Azimuth at LOS (radians) */
} pass_info_t;

/*
 * Priority queue for satellite passes.
 * Uses array-based min-heap for O(log n) insert/extract.
 */
typedef struct pass_priority_queue {
    pass_info_t passes[PASS_QUEUE_CAPACITY];
    int count;
} pass_priority_queue_t;

/*
 * Comparison result for pass priority ordering.
 */
typedef enum pass_compare_result {
    PASS_FIRST_HIGHER_PRIORITY = -1,
    PASS_EQUAL_PRIORITY = 0,
    PASS_SECOND_HIGHER_PRIORITY = 1
} pass_compare_result_t;

/*
 * Compare two passes for priority ordering.
 *
 * Priority rules:
 * 1. If passes do NOT overlap: earlier AOS has higher priority
 * 2. If passes overlap: higher max elevation has higher priority
 *    (if equal, earlier AOS breaks the tie)
 *
 * @param a     First pass to compare
 * @param b     Second pass to compare
 * @return      PASS_FIRST_HIGHER_PRIORITY if a should be executed before b
 */
pass_compare_result_t pass_compare(const pass_info_t *a, const pass_info_t *b);

/*
 * Check if two passes overlap in time.
 *
 * @param a     First pass
 * @param b     Second pass
 * @return      true if passes overlap, false otherwise
 */
bool pass_overlaps(const pass_info_t *a, const pass_info_t *b);

/*
 * Initialize priority queue to empty state.
 *
 * @param queue Queue to initialize
 */
void pass_queue_init(pass_priority_queue_t *queue);

/*
 * Insert a pass into the priority queue.
 *
 * @param queue Queue to insert into
 * @param pass  Pass to insert
 * @return      0 on success, -1 if queue is full
 */
int pass_queue_insert(pass_priority_queue_t *queue, const pass_info_t *pass);

/*
 * Get the highest priority pass without removing it.
 *
 * @param queue Queue to peek
 * @return      Pointer to highest priority pass, or NULL if queue is empty
 */
const pass_info_t *pass_queue_peek(const pass_priority_queue_t *queue);

/*
 * Remove and return the highest priority pass.
 *
 * @param queue Queue to extract from
 * @param out   Output buffer for extracted pass
 * @return      0 on success, -1 if queue is empty
 */
int pass_queue_extract(pass_priority_queue_t *queue, pass_info_t *out);

/*
 * Remove all passes that have ended (LOS < current_jd).
 *
 * @param queue      Queue to prune
 * @param current_jd Current time as Julian Date
 * @return           Number of passes removed
 */
int pass_queue_prune_expired(pass_priority_queue_t *queue, double current_jd);

/*
 * Remove a pass by NORAD ID (useful for updating passes).
 *
 * @param queue    Queue to remove from
 * @param norad_id NORAD ID of pass to remove
 * @return         0 if pass was removed, -1 if not found
 */
int pass_queue_remove_by_norad(pass_priority_queue_t *queue, int norad_id);

/*
 * Check if queue is empty.
 *
 * @param queue Queue to check
 * @return      true if empty, false otherwise
 */
bool pass_queue_is_empty(const pass_priority_queue_t *queue);

/*
 * Get number of passes in queue.
 *
 * @param queue Queue to check
 * @return      Number of passes in queue
 */
int pass_queue_count(const pass_priority_queue_t *queue);

#endif /* PRIORITY_QUEUE_H */
