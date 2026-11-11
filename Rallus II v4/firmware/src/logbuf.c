#include "logbuf.h"
#include "pico.h"
#include <string.h>

// Single-producer (core 0) / single-consumer (core 1) ring of log records.
// Free-running 32-bit counters; index = counter & (LOGQ_SIZE - 1). Plain
// 32-bit loads/stores are atomic on the M33, the barrier orders the record
// write against the index publish.

static log_record_t      buf[LOGQ_SIZE];
static volatile uint32_t head;
static volatile uint32_t tail;
static volatile uint32_t dropped_count;

void logq_init(void) {
    head = 0;
    tail = 0;
    dropped_count = 0;
}

bool logq_push(const log_record_t *rec) {
    if (head - tail >= LOGQ_SIZE) {
        dropped_count++;
        return false;
    }
    buf[head & (LOGQ_SIZE - 1)] = *rec;
    __compiler_memory_barrier();
    head++;
    return true;
}

bool logq_pop(log_record_t *out) {
    if (tail == head) return false;
    *out = buf[tail & (LOGQ_SIZE - 1)];
    __compiler_memory_barrier();
    tail++;
    return true;
}

uint32_t logq_count(void)   { return head - tail; }
uint32_t logq_dropped(void) { return dropped_count; }
