#include "include/concurrent.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/sched.h>
#include <sched.h>
#include <sys/syscall.h>
#include <linux/types.h>
#include <signal.h>
#include <pthread.h>
#include <pigpio.h>
// Handler for deadline misses
void dl_miss_handler(int sig) {
    puts("DEADLINE MISS!");
    fflush(stdout);
    (void)signal(SIGXCPU, SIG_DFL);
	gpioTerminate();
    exit(EXIT_FAILURE);
}

// Set SCHED_DEADLINE scheduling attributes
int set_sched_deadline(uint64_t runtime_ns, uint64_t deadline_ns, uint64_t period_ns) {
    struct sched_attr attr = {
        .size = sizeof(attr),
        .sched_flags = 0 | SCHED_FLAG_DL_OVERRUN,
        .sched_policy = SCHED_DEADLINE,
        .sched_runtime = runtime_ns,
        .sched_deadline = deadline_ns,
        .sched_period = period_ns
    };

    if (syscall(__NR_sched_setattr, syscall(__NR_gettid), &attr, 0) != 0) {
        perror("set_sched_deadline: sched_setattr");
        return 1;
    }

    (void)signal(SIGXCPU, dl_miss_handler);
    return 0;
}

// Mutex wrapper functions
int mutex_init(pthread_mutex_t *lock) {
    return pthread_mutex_init(lock, NULL) ;
}

int mutex_destroy(pthread_mutex_t *lock) {
    return pthread_mutex_destroy(lock) ;
}

int mutex_lock(pthread_mutex_t *lock) {
    return pthread_mutex_trylock(lock) ;
}

int mutex_unlock(pthread_mutex_t *lock) {
    return pthread_mutex_unlock(lock) ;
}

// Task (thread) management
int create_task(task_t *task, void* args) {
    return pthread_create(&(task->tid), NULL, task->entry_point, args) != 0;
}

int join_task(task_t *task) {
    return pthread_join(task->tid, NULL);
}

int cancel_task(task_t *task) {
    return pthread_cancel(task->tid);
}

// Print failure message and exit
void failure(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}
