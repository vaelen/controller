/*
 * Priority Queue Unit Tests
 *
 * Host-compilable tests for the pass priority queue.
 * Compile with: gcc -std=c99 -Wall -Wextra -I.. -o test_priority_queue \
 *               test_priority_queue.c ../priority_queue.c
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "priority_queue.h"

/* Test counters */
static int tests_run = 0;
static int tests_passed = 0;

/* Test helper macros */
#define TEST_ASSERT(condition, msg) do { \
    tests_run++; \
    if (!(condition)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_EQ(expected, actual, msg) do { \
    tests_run++; \
    if ((expected) != (actual)) { \
        printf("  FAIL: %s - expected %d, got %d (line %d)\n", \
               msg, (int)(expected), (int)(actual), __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_DOUBLE_EQ(expected, actual, msg) do { \
    tests_run++; \
    if (fabs((expected) - (actual)) > 0.0001) { \
        printf("  FAIL: %s - expected %f, got %f (line %d)\n", \
               msg, (expected), (actual), __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

/* Helper to create a pass */
static pass_info_t make_pass(int norad_id, double aos_jd, double los_jd,
                              double max_el_rad)
{
    pass_info_t p = {
        .norad_id = norad_id,
        .aos_jd = aos_jd,
        .los_jd = los_jd,
        .max_elevation_rad = max_el_rad,
        .aos_azimuth_rad = 0.0,
        .los_azimuth_rad = 0.0
    };
    return p;
}

/* Test: Empty queue operations */
static int test_empty_queue(void)
{
    printf("test_empty_queue...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    TEST_ASSERT(pass_queue_is_empty(&q), "Queue should be empty after init");
    TEST_ASSERT_EQ(0, pass_queue_count(&q), "Count should be 0");
    TEST_ASSERT(pass_queue_peek(&q) == NULL, "Peek should return NULL");

    pass_info_t out;
    TEST_ASSERT_EQ(-1, pass_queue_extract(&q, &out), "Extract should fail on empty");

    printf("  PASSED\n");
    return 0;
}

/* Test: Single element insert/extract */
static int test_single_element(void)
{
    printf("test_single_element...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    pass_info_t pass = make_pass(25544, 2460000.5, 2460000.51, 0.5);

    TEST_ASSERT_EQ(0, pass_queue_insert(&q, &pass), "Insert should succeed");
    TEST_ASSERT_EQ(1, pass_queue_count(&q), "Count should be 1");
    TEST_ASSERT(!pass_queue_is_empty(&q), "Queue should not be empty");

    const pass_info_t *peek = pass_queue_peek(&q);
    TEST_ASSERT(peek != NULL, "Peek should not be NULL");
    TEST_ASSERT_EQ(25544, peek->norad_id, "Peek should return correct pass");

    pass_info_t out;
    TEST_ASSERT_EQ(0, pass_queue_extract(&q, &out), "Extract should succeed");
    TEST_ASSERT_EQ(25544, out.norad_id, "Extracted pass should be correct");
    TEST_ASSERT(pass_queue_is_empty(&q), "Queue should be empty after extract");

    printf("  PASSED\n");
    return 0;
}

/* Test: Non-overlapping passes sorted by AOS */
static int test_non_overlapping_sorted(void)
{
    printf("test_non_overlapping_sorted...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Insert in reverse order - should still extract in AOS order */
    pass_info_t pass3 = make_pass(3, 2460000.70, 2460000.75, 0.3);
    pass_info_t pass2 = make_pass(2, 2460000.50, 2460000.55, 0.4);
    pass_info_t pass1 = make_pass(1, 2460000.30, 2460000.35, 0.2);

    pass_queue_insert(&q, &pass3);
    pass_queue_insert(&q, &pass2);
    pass_queue_insert(&q, &pass1);

    TEST_ASSERT_EQ(3, pass_queue_count(&q), "Count should be 3");

    pass_info_t out;

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "First should be earliest AOS (pass 1)");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(2, out.norad_id, "Second should be pass 2");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Third should be latest AOS (pass 3)");

    printf("  PASSED\n");
    return 0;
}

/* Test: Overlapping passes sorted by max elevation */
static int test_overlapping_by_elevation(void)
{
    printf("test_overlapping_by_elevation...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Three passes that all overlap */
    pass_info_t low  = make_pass(1, 2460000.50, 2460000.60, 0.2);
    pass_info_t high = make_pass(2, 2460000.52, 2460000.58, 0.8);
    pass_info_t mid  = make_pass(3, 2460000.54, 2460000.59, 0.5);

    pass_queue_insert(&q, &low);
    pass_queue_insert(&q, &high);
    pass_queue_insert(&q, &mid);

    pass_info_t out;

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(2, out.norad_id, "First should be highest elevation (pass 2)");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Second should be mid elevation (pass 3)");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "Third should be lowest elevation (pass 1)");

    printf("  PASSED\n");
    return 0;
}

/* Test: Cascading overlaps */
static int test_cascading_overlaps(void)
{
    printf("test_cascading_overlaps...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* A overlaps B, B overlaps C, but A does NOT overlap C */
    pass_info_t A = make_pass(1, 2460000.50, 2460000.55, 0.3);
    pass_info_t B = make_pass(2, 2460000.54, 2460000.60, 0.8);
    pass_info_t C = make_pass(3, 2460000.58, 2460000.65, 0.5);

    /* Verify overlap conditions */
    TEST_ASSERT(pass_overlaps(&A, &B), "A and B should overlap");
    TEST_ASSERT(pass_overlaps(&B, &C), "B and C should overlap");
    TEST_ASSERT(!pass_overlaps(&A, &C), "A and C should NOT overlap");

    pass_queue_insert(&q, &A);
    pass_queue_insert(&q, &B);
    pass_queue_insert(&q, &C);

    pass_info_t out;

    /* B has highest elevation and overlaps with both A and C */
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(2, out.norad_id, "First should be B (highest overlapping)");

    /* Now A and C remain, they DON'T overlap, A has earlier AOS */
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "Second should be A (earlier AOS)");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Third should be C");

    printf("  PASSED\n");
    return 0;
}

/* Test: Queue full behavior */
static int test_queue_full(void)
{
    printf("test_queue_full...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Fill queue to capacity */
    for (int i = 0; i < PASS_QUEUE_CAPACITY; i++) {
        pass_info_t pass = make_pass(i, 2460000.5 + i * 0.1, 2460000.55 + i * 0.1, 0.5);
        TEST_ASSERT_EQ(0, pass_queue_insert(&q, &pass), "Insert should succeed");
    }

    TEST_ASSERT_EQ(PASS_QUEUE_CAPACITY, pass_queue_count(&q), "Queue should be at capacity");

    /* Try to insert one more */
    pass_info_t extra = make_pass(999, 2460010.5, 2460010.55, 0.5);
    TEST_ASSERT_EQ(-1, pass_queue_insert(&q, &extra), "Insert should fail when full");

    /* Count should still be at capacity */
    TEST_ASSERT_EQ(PASS_QUEUE_CAPACITY, pass_queue_count(&q), "Count unchanged after failed insert");

    printf("  PASSED\n");
    return 0;
}

/* Test: Prune expired passes */
static int test_prune_expired(void)
{
    printf("test_prune_expired...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Past pass - LOS already passed */
    pass_info_t expired = make_pass(1, 2460000.30, 2460000.35, 0.5);
    /* Current pass - still in progress */
    pass_info_t current = make_pass(2, 2460000.40, 2460000.55, 0.5);
    /* Future pass */
    pass_info_t future = make_pass(3, 2460000.60, 2460000.65, 0.5);

    pass_queue_insert(&q, &expired);
    pass_queue_insert(&q, &current);
    pass_queue_insert(&q, &future);

    TEST_ASSERT_EQ(3, pass_queue_count(&q), "Should have 3 passes");

    /* Prune at time 2460000.5 (after expired.los, during current, before future.aos) */
    int removed = pass_queue_prune_expired(&q, 2460000.50);
    TEST_ASSERT_EQ(1, removed, "Should have removed 1 pass");
    TEST_ASSERT_EQ(2, pass_queue_count(&q), "Should have 2 passes remaining");

    pass_info_t out;
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(2, out.norad_id, "First remaining should be current (pass 2)");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Second remaining should be future (pass 3)");

    printf("  PASSED\n");
    return 0;
}

/* Test: Equal elevation tie-breaking */
static int test_equal_elevation_tiebreak(void)
{
    printf("test_equal_elevation_tiebreak...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Two overlapping passes with equal elevation - earlier AOS should win */
    pass_info_t late  = make_pass(2, 2460000.55, 2460000.65, 0.5);
    pass_info_t early = make_pass(1, 2460000.50, 2460000.60, 0.5);

    pass_queue_insert(&q, &late);
    pass_queue_insert(&q, &early);

    pass_info_t out;
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "Earlier AOS should win on tie");

    printf("  PASSED\n");
    return 0;
}

/* Test: Remove by NORAD ID */
static int test_remove_by_norad(void)
{
    printf("test_remove_by_norad...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    pass_info_t p1 = make_pass(1, 2460000.50, 2460000.55, 0.5);
    pass_info_t p2 = make_pass(2, 2460000.60, 2460000.65, 0.5);
    pass_info_t p3 = make_pass(3, 2460000.70, 2460000.75, 0.5);

    pass_queue_insert(&q, &p1);
    pass_queue_insert(&q, &p2);
    pass_queue_insert(&q, &p3);

    TEST_ASSERT_EQ(3, pass_queue_count(&q), "Should have 3 passes");

    /* Remove the middle one */
    TEST_ASSERT_EQ(0, pass_queue_remove_by_norad(&q, 2), "Remove should succeed");
    TEST_ASSERT_EQ(2, pass_queue_count(&q), "Should have 2 passes");

    /* Try to remove non-existent */
    TEST_ASSERT_EQ(-1, pass_queue_remove_by_norad(&q, 2), "Remove should fail for missing");

    /* Verify remaining order */
    pass_info_t out;
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "First should be pass 1");
    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Second should be pass 3");

    printf("  PASSED\n");
    return 0;
}

/* Test: Mixed overlapping and non-overlapping */
static int test_mixed_passes(void)
{
    printf("test_mixed_passes...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Group 1: Two overlapping passes (low priority overall due to late start) */
    pass_info_t g1_low  = make_pass(4, 2460000.70, 2460000.80, 0.3);
    pass_info_t g1_high = make_pass(5, 2460000.72, 2460000.78, 0.7);

    /* Group 2: Single pass (mid-time, no overlap with others) */
    pass_info_t solo = make_pass(3, 2460000.50, 2460000.55, 0.5);

    /* Group 3: Two overlapping passes (earliest start) */
    pass_info_t g3_low  = make_pass(1, 2460000.30, 2460000.40, 0.2);
    pass_info_t g3_high = make_pass(2, 2460000.32, 2460000.38, 0.6);

    /* Insert in random order */
    pass_queue_insert(&q, &g1_low);
    pass_queue_insert(&q, &solo);
    pass_queue_insert(&q, &g3_high);
    pass_queue_insert(&q, &g1_high);
    pass_queue_insert(&q, &g3_low);

    pass_info_t out;

    /* Expected order:
     * 1. g3_high (overlaps with g3_low, higher elevation, earliest group)
     * 2. g3_low (next earliest, no longer overlapping with extracted)
     * 3. solo (next by AOS)
     * 4. g1_high (overlaps with g1_low, higher elevation)
     * 5. g1_low
     */

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(2, out.norad_id, "First: g3_high");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(1, out.norad_id, "Second: g3_low");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(3, out.norad_id, "Third: solo");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(5, out.norad_id, "Fourth: g1_high");

    pass_queue_extract(&q, &out);
    TEST_ASSERT_EQ(4, out.norad_id, "Fifth: g1_low");

    printf("  PASSED\n");
    return 0;
}

/* Test: Stress test with many passes */
static int test_stress(void)
{
    printf("test_stress...\n");

    pass_priority_queue_t q;
    pass_queue_init(&q);

    /* Insert 64 non-overlapping passes in reverse order */
    for (int i = PASS_QUEUE_CAPACITY - 1; i >= 0; i--) {
        pass_info_t pass = make_pass(i, 2460000.0 + i * 0.1, 2460000.05 + i * 0.1, 0.5);
        TEST_ASSERT_EQ(0, pass_queue_insert(&q, &pass), "Insert should succeed");
    }

    /* Extract all - should come out in NORAD ID order (which is also AOS order) */
    for (int i = 0; i < PASS_QUEUE_CAPACITY; i++) {
        pass_info_t out;
        TEST_ASSERT_EQ(0, pass_queue_extract(&q, &out), "Extract should succeed");
        TEST_ASSERT_EQ(i, out.norad_id, "Passes should come out in AOS order");
    }

    TEST_ASSERT(pass_queue_is_empty(&q), "Queue should be empty");

    printf("  PASSED\n");
    return 0;
}

/* Test: pass_overlaps function */
static int test_pass_overlaps(void)
{
    printf("test_pass_overlaps...\n");

    /* Test clear non-overlap */
    pass_info_t p1 = make_pass(1, 2460000.50, 2460000.55, 0.5);
    pass_info_t p2 = make_pass(2, 2460000.60, 2460000.65, 0.5);
    TEST_ASSERT(!pass_overlaps(&p1, &p2), "Non-overlapping passes");
    TEST_ASSERT(!pass_overlaps(&p2, &p1), "Commutative check");

    /* Test clear overlap */
    pass_info_t p3 = make_pass(3, 2460000.52, 2460000.58, 0.5);
    TEST_ASSERT(pass_overlaps(&p1, &p3), "Overlapping passes");
    TEST_ASSERT(pass_overlaps(&p3, &p1), "Commutative check");

    /* Test edge case: one starts exactly when other ends (not overlap) */
    pass_info_t p4 = make_pass(4, 2460000.55, 2460000.60, 0.5);
    TEST_ASSERT(!pass_overlaps(&p1, &p4), "Adjacent passes don't overlap");

    /* Test containment: one pass fully within another */
    pass_info_t outer = make_pass(5, 2460000.50, 2460000.70, 0.5);
    pass_info_t inner = make_pass(6, 2460000.55, 2460000.65, 0.5);
    TEST_ASSERT(pass_overlaps(&outer, &inner), "Contained passes overlap");
    TEST_ASSERT(pass_overlaps(&inner, &outer), "Commutative check");

    printf("  PASSED\n");
    return 0;
}

/* Test: pass_compare function */
static int test_pass_compare(void)
{
    printf("test_pass_compare...\n");

    /* Non-overlapping: earlier AOS wins */
    pass_info_t early = make_pass(1, 2460000.50, 2460000.55, 0.3);
    pass_info_t late  = make_pass(2, 2460000.60, 2460000.65, 0.8);

    TEST_ASSERT_EQ(PASS_FIRST_HIGHER_PRIORITY, pass_compare(&early, &late),
                   "Earlier AOS should win for non-overlapping");
    TEST_ASSERT_EQ(PASS_SECOND_HIGHER_PRIORITY, pass_compare(&late, &early),
                   "Reverse check");

    /* Overlapping: higher elevation wins */
    pass_info_t low  = make_pass(3, 2460000.50, 2460000.60, 0.3);
    pass_info_t high = make_pass(4, 2460000.52, 2460000.58, 0.8);

    TEST_ASSERT_EQ(PASS_SECOND_HIGHER_PRIORITY, pass_compare(&low, &high),
                   "Higher elevation should win for overlapping");
    TEST_ASSERT_EQ(PASS_FIRST_HIGHER_PRIORITY, pass_compare(&high, &low),
                   "Reverse check");

    /* Equal elevation, overlapping: earlier AOS wins */
    pass_info_t eq1 = make_pass(5, 2460000.50, 2460000.60, 0.5);
    pass_info_t eq2 = make_pass(6, 2460000.52, 2460000.58, 0.5);

    TEST_ASSERT_EQ(PASS_FIRST_HIGHER_PRIORITY, pass_compare(&eq1, &eq2),
                   "Earlier AOS should win on elevation tie");

    /* Identical passes */
    TEST_ASSERT_EQ(PASS_EQUAL_PRIORITY, pass_compare(&eq1, &eq1),
                   "Identical passes should be equal");

    printf("  PASSED\n");
    return 0;
}

/* Main test runner */
int main(void)
{
    printf("=== Priority Queue Tests ===\n\n");

    int failed = 0;

    failed += test_empty_queue();
    failed += test_single_element();
    failed += test_non_overlapping_sorted();
    failed += test_overlapping_by_elevation();
    failed += test_cascading_overlaps();
    failed += test_queue_full();
    failed += test_prune_expired();
    failed += test_equal_elevation_tiebreak();
    failed += test_remove_by_norad();
    failed += test_mixed_passes();
    failed += test_stress();
    failed += test_pass_overlaps();
    failed += test_pass_compare();

    printf("\n=== Results ===\n");
    printf("Tests run: %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_run - tests_passed);

    if (failed > 0) {
        printf("\n*** %d test function(s) FAILED ***\n", failed);
        return 1;
    }

    printf("\n*** All tests PASSED ***\n");
    return 0;
}
