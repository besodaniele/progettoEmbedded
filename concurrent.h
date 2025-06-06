#ifndef CONCURRENT_H
#define CONCURRENT_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <pthread.h>
#include <linux/sched.h>
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/types.h>
#include <sys/signal.h>

struct sched_attr
{
    __u32 size;           /* Size of this structure */
    __u32 sched_policy;   /* Policy (SCHED_*) */
    __u64 sched_flags;    /* Flags */
    __s32 sched_nice;     /* Nice value (SCHED_OTHER,
                           SCHED_BATCH) */
    __u32 sched_priority; /* Static priority (SCHED_FIFO,
                           SCHED_RR) */
    /* For SCHED_DEADLINE */
    __u64 sched_runtime; //time in nanoseconds
    __u64 sched_deadline;
    __u64 sched_period;

    /* Utilization hints */
    __u32 sched_util_min;
    __u32 sched_util_max;
};

typedef struct task
{
    pthread_t tid;
    void *(*entry_point)(void *);
} task_t;

void dl_miss_handler(int sig)
{
    puts("DEADLINE MISS!");
    fflush(stdout);
    (void)signal(SIGXCPU, SIG_DFL);
    exit(EXIT_FAILURE);
}

int set_sched_deadline(uint64_t runtime_ns, uint64_t deadline_ns, uint64_t period_ns)
{
    struct sched_attr attr = {
        .size = sizeof(attr),
        .sched_flags = 0 | SCHED_FLAG_DL_OVERRUN,
        .sched_policy = SCHED_DEADLINE,
        .sched_runtime = runtime_ns,
        .sched_deadline = deadline_ns,
        .sched_period = period_ns};
    if (syscall(__NR_sched_setattr, syscall(__NR_gettid), &attr, 0) != 0)
    {
        perror("set_sched_deadline: sched_setattr");
        return (1);
    }
    (void)signal(SIGXCPU, dl_miss_handler); // register signal handler
    return (0);
}

int mutex_init(pthread_mutex_t *lock)
{
    if (pthread_mutex_init(lock, NULL) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int mutex_destroy(pthread_mutex_t *lock)
{
    if (pthread_mutex_destroy(lock) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int mutex_lock(pthread_mutex_t *lock)
{
    if (pthread_mutex_lock(lock) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int mutex_unlock(pthread_mutex_t *lock)
{
    if (pthread_mutex_unlock(lock) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int create_task(task_t *task, void* args)
{
    if (pthread_create(&(task->tid), NULL, task->entry_point, args) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int join_task(task_t *task)
{
    if (pthread_join(task->tid, NULL) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int cancel_task(task_t *task)
{
    if (pthread_cancel(task->tid) != 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

void failure(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}
#endif