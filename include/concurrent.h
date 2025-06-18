#ifndef CONCURRENT_H
#define CONCURRENT_H

#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <linux/types.h>

// Structure for real-time scheduling attributes
struct sched_attr {
    __u32 size;
    __u32 sched_policy;
    __u64 sched_flags;
    __s32 sched_nice;
    __u32 sched_priority;
    __u64 sched_runtime;
    __u64 sched_deadline;
    __u64 sched_period;
    __u32 sched_util_min;
    __u32 sched_util_max;
};

// Task wrapper for thread management
typedef struct task {
    pthread_t tid;
    void *(*entry_point)(void *);
} task_t;

// Function declarations
int set_sched_deadline(uint64_t runtime_ns, uint64_t deadline_ns, uint64_t period_ns);
int mutex_init(pthread_mutex_t *lock);
int mutex_destroy(pthread_mutex_t *lock);
int mutex_lock(pthread_mutex_t *lock);
int mutex_unlock(pthread_mutex_t *lock);
int create_task(task_t *task, void* args);
int join_task(task_t *task);
int cancel_task(task_t *task);
void failure(const char *msg);

#endif // CONCURRENT_H
