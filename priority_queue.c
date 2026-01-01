/*
 * Pass Priority Queue Implementation
 *
 * Array-based min-heap for scheduling satellite passes.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "priority_queue.h"
#include <string.h>

/* Heap index calculations for 0-based array */
#define HEAP_PARENT(i)      (((i) - 1) / 2)
#define HEAP_LEFT_CHILD(i)  ((2 * (i)) + 1)
#define HEAP_RIGHT_CHILD(i) ((2 * (i)) + 2)

/*
 * Swap two passes in the heap.
 */
static void heap_swap(pass_priority_queue_t *q, int i, int j)
{
    pass_info_t temp = q->passes[i];
    q->passes[i] = q->passes[j];
    q->passes[j] = temp;
}

/*
 * Sift a node up to restore heap property after insertion.
 */
static void heap_sift_up(pass_priority_queue_t *q, int idx)
{
    while (idx > 0) {
        int parent = HEAP_PARENT(idx);
        if (pass_compare(&q->passes[idx], &q->passes[parent]) < 0) {
            heap_swap(q, idx, parent);
            idx = parent;
        } else {
            break;
        }
    }
}

/*
 * Sift a node down to restore heap property after extraction.
 */
static void heap_sift_down(pass_priority_queue_t *q, int idx)
{
    while (true) {
        int left = HEAP_LEFT_CHILD(idx);
        int right = HEAP_RIGHT_CHILD(idx);
        int smallest = idx;

        if (left < q->count &&
            pass_compare(&q->passes[left], &q->passes[smallest]) < 0) {
            smallest = left;
        }
        if (right < q->count &&
            pass_compare(&q->passes[right], &q->passes[smallest]) < 0) {
            smallest = right;
        }

        if (smallest != idx) {
            heap_swap(q, idx, smallest);
            idx = smallest;
        } else {
            break;
        }
    }
}

/*
 * Rebuild the entire heap (used after removing arbitrary elements).
 */
static void heap_rebuild(pass_priority_queue_t *q)
{
    /* Build heap from bottom up */
    for (int i = (q->count / 2) - 1; i >= 0; i--) {
        heap_sift_down(q, i);
    }
}

bool pass_overlaps(const pass_info_t *a, const pass_info_t *b)
{
    if (!a || !b) {
        return false;
    }
    /* Passes overlap if one starts before the other ends */
    return (a->aos_jd < b->los_jd) && (b->aos_jd < a->los_jd);
}

pass_compare_result_t pass_compare(const pass_info_t *a, const pass_info_t *b)
{
    if (!a || !b) {
        return PASS_EQUAL_PRIORITY;
    }

    bool overlaps = pass_overlaps(a, b);

    if (!overlaps) {
        /* Non-overlapping: earlier start wins */
        if (a->aos_jd < b->aos_jd) {
            return PASS_FIRST_HIGHER_PRIORITY;
        }
        if (a->aos_jd > b->aos_jd) {
            return PASS_SECOND_HIGHER_PRIORITY;
        }
        return PASS_EQUAL_PRIORITY;
    }

    /* Overlapping: higher max elevation wins */
    if (a->max_elevation_rad > b->max_elevation_rad) {
        return PASS_FIRST_HIGHER_PRIORITY;
    }
    if (a->max_elevation_rad < b->max_elevation_rad) {
        return PASS_SECOND_HIGHER_PRIORITY;
    }

    /* Tie-breaker: earlier start */
    if (a->aos_jd < b->aos_jd) {
        return PASS_FIRST_HIGHER_PRIORITY;
    }
    if (a->aos_jd > b->aos_jd) {
        return PASS_SECOND_HIGHER_PRIORITY;
    }

    return PASS_EQUAL_PRIORITY;
}

void pass_queue_init(pass_priority_queue_t *queue)
{
    if (!queue) {
        return;
    }
    memset(queue, 0, sizeof(*queue));
    queue->count = 0;
}

int pass_queue_insert(pass_priority_queue_t *queue, const pass_info_t *pass)
{
    if (!queue || !pass) {
        return -1;
    }
    if (queue->count >= PASS_QUEUE_CAPACITY) {
        return -1;
    }

    /* Insert at end and sift up */
    queue->passes[queue->count] = *pass;
    heap_sift_up(queue, queue->count);
    queue->count++;

    return 0;
}

const pass_info_t *pass_queue_peek(const pass_priority_queue_t *queue)
{
    if (!queue || queue->count == 0) {
        return NULL;
    }
    return &queue->passes[0];
}

int pass_queue_extract(pass_priority_queue_t *queue, pass_info_t *out)
{
    if (!queue || queue->count == 0) {
        return -1;
    }

    if (out) {
        *out = queue->passes[0];
    }

    /* Move last element to root and sift down */
    queue->count--;
    if (queue->count > 0) {
        queue->passes[0] = queue->passes[queue->count];
        heap_sift_down(queue, 0);
    }

    return 0;
}

int pass_queue_prune_expired(pass_priority_queue_t *queue, double current_jd)
{
    if (!queue) {
        return 0;
    }

    int removed = 0;
    int write_idx = 0;

    /* Compact the array, removing expired passes */
    for (int read_idx = 0; read_idx < queue->count; read_idx++) {
        if (queue->passes[read_idx].los_jd >= current_jd) {
            /* Pass is still active or future - keep it */
            if (write_idx != read_idx) {
                queue->passes[write_idx] = queue->passes[read_idx];
            }
            write_idx++;
        } else {
            removed++;
        }
    }

    queue->count = write_idx;

    /* Rebuild heap if we removed anything */
    if (removed > 0) {
        heap_rebuild(queue);
    }

    return removed;
}

int pass_queue_remove_by_norad(pass_priority_queue_t *queue, int norad_id)
{
    if (!queue) {
        return -1;
    }

    int found_idx = -1;
    for (int i = 0; i < queue->count; i++) {
        if (queue->passes[i].norad_id == norad_id) {
            found_idx = i;
            break;
        }
    }

    if (found_idx < 0) {
        return -1;
    }

    /* Remove by moving last element to this position */
    queue->count--;
    if (found_idx < queue->count) {
        queue->passes[found_idx] = queue->passes[queue->count];
        /* May need to sift up or down depending on relationship */
        heap_sift_up(queue, found_idx);
        heap_sift_down(queue, found_idx);
    }

    return 0;
}

bool pass_queue_is_empty(const pass_priority_queue_t *queue)
{
    return !queue || queue->count == 0;
}

int pass_queue_count(const pass_priority_queue_t *queue)
{
    return queue ? queue->count : 0;
}
