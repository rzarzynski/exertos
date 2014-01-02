#include <limits.h>
#include <stddef.h>
#include <stdbool.h>

#include "stm32f10x.h"
#include "cmsis_os.h"

extern const uint32_t *_text_e, *_data_s, *_data_e, *_bss_s, *_bss_e;

extern osThreadDef_t * linker_tdefs_start;
extern const void * const linker_tdefs_end;

static void relocate( void )
{
    uint32_t *src = (uint32_t *)&_text_e;
    uint32_t *dst = (uint32_t *)&_data_s;

    while (dst < (uint32_t *)&_data_e)
        *dst++ = *src++;

    for (dst = (uint32_t *)&_bss_s; dst < (uint32_t *)&_bss_e; dst++)
        *dst = 0;

    return;
}

#define BITS_PER_THREADID   ( 4 )
#define BITS_PER_PRIORITY   ( 3 )

/* On Cortex M3 it should be 0x80000000. */
#define PRIO_SHIFT_BASE     ( 1 << ((CHAR_BIT * sizeof(queue_helper_prios.valid)) - 1) )

/** TCB */
#define FRUGAL_STRUCTS_ATTR(type, member, ptr, length)  \
    ((typeof( ((type *)0)->member) *)((int8_t *)(ptr) + ((length) * offsetof(type, member))))

#define FRUGAL_STRUCTS_SIZE(type, last_attr, max_pos)   \
    ((/*max_pos * */offsetof(type, last_attr)))

#define TCB_ATTR(attr, pos) \
    ((FRUGAL_STRUCTS_ATTR(struct tcb_t, attr, tcb_table_ptr, max_threads_num)[pos]))

#define TCB_SIZE           \
    (FRUGAL_STRUCTS_SIZE(struct tcb_t, __end_of_struct, max_threads_num))

typedef uint8_t osImplThreadId;
typedef uint8_t osImplPriorityId;

/**
 *  Structure that defines single element of queues for both ready and inactive threads
 *  (using a terminology from the CMSIS-RTOS specification).
 */
struct queue_unvrsl_elem_t
{
    osImplThreadId  next    : BITS_PER_THREADID;    /** < ID of next thread in the queue */
    osImplThreadId  prev    : BITS_PER_THREADID;    /** < ID of previous */
};

/** 
 *  Structure defining whole priority queues for speedup of certain operations
 *  (like insert and delete) on the threads ready queue.
 */
struct prio_helper_t
{
    /** ID of last thread with given priority in the ready threads queue */
    osImplThreadId  lastThreadId[ 1UL << BITS_PER_PRIORITY ];
    /** Bit array for fast lookup for first non-empty priority queue */
    uint32_t        valid;
};

struct tcb_t
{
    void        *stackPointer;

    uint8_t     priority; // : BITS_PER_PRIORITY;
    uint8_t     __end_of_struct[0];
};

/*restrict*/ struct queue_unvrsl_elem_t *queue_unvrsl_ptr;
/*restrict*/ void *tcb_table_ptr;

static struct prio_helper_t     queue_helper_prios;

static size_t max_threads_num              = 0;

static osImplThreadId queue_ready_first    = 0;
static osImplThreadId queue_inact_first    = 0;
static osImplThreadId running_tid          = 0;


uint8_t osImplKernelFlags = 0;



inline void queue_ready_add(const osImplThreadId tid, const osImplPriorityId prio);
inline void queue_ready_del (const osImplThreadId tid, const osImplPriorityId prio);
inline bool queue_ready_getfirst (osImplThreadId * const p_tid);
inline bool queue_ready_extract (osImplThreadId * const p_tid);
inline bool queue_ready_contains(const osImplThreadId tid);
inline bool queue_ready_empty (void);
void inline queue_inact_add (const osImplThreadId tid);
inline bool queue_inact_getfirst (osImplThreadId * const p_tid);
inline bool queue_inact_extract (osImplThreadId * const p_tid);
bool inline queue_inact_contains (const osImplThreadId tid);
bool inline queue_inact_empty (void);
inline void queue_blckd_add (const osImplThreadId tid);
inline void queue_blckd_del (const osImplThreadId tid);
inline bool queue_blckd_contains (const osImplThreadId tid);

void __osKernelBoot (void)
{
    relocate();
//    osKernelStart(osThread(main), NULL);
}

#define KFLAGS_OS_RUNNING           ( 1 <<  1 )
#define KFLAGS_INACTIVE_EMPTY       ( 1 <<  2 )

inline void queue_ready_add( const osImplThreadId tid, const osImplPriorityId prio )
{
    const uint32_t shifted_prios = queue_helper_prios.valid << prio;
    //const typeof(queue_helper_prios.valid) shifted_prios = queue_helper_prios.valid << prio;

    if (!(queue_helper_prios.valid << prio))
    {
        /* If we are here it means that there is no threads with higher
         * or equal priority in the ready queue.
         * Don't worry about ancestor and predecessor fields. Values
         * different than those which would make a cycle are completely
         * acceptable. Why? Please refer to queue_inact_contains()
         * and queue_blckd_contains() functions and look for the tole of
         * lastThreadId field in the queue_helper_prios structure. */
        if (queue_ready_empty())
            queue_unvrsl_ptr[tid].next  = tid + 1;
        else
        {
            /* OK, at least one thread with lower priority exists,
             * so we must preserve the order. */
            queue_unvrsl_ptr[tid].next  = queue_ready_first;
            queue_unvrsl_ptr[queue_ready_first].prev  = tid;
        }

        queue_unvrsl_ptr[tid].prev      = tid - 1;
        queue_ready_first               = tid;
    }
    else
    {
        /* If we are in this block, the argument for clz couldn't be zero. */
        const osImplPriorityId index    = clz(shifted_prios >> prio);

        const osImplThreadId ins_aft    = queue_helper_prios.lastThreadId[index];
        const osImplThreadId ia_next    = queue_unvrsl_ptr[ins_aft].next;

        queue_unvrsl_ptr[tid].prev      = ins_aft;
        queue_unvrsl_ptr[ins_aft].next  = tid;

        if (clz(queue_helper_prios.valid) < index)
        {
            /* Queue has at least one thread with lower priority. Add in the middle. */
            queue_unvrsl_ptr[tid].next      = ia_next;
            queue_unvrsl_ptr[ia_next].prev  = tid;
        }
        else
        {
            /* tid will become new tail of the whole ready queue, so avoid self-cycle. */
            queue_unvrsl_ptr[tid].next      = tid + 1;
        }
    }

    /* Update helper queue with reference to last thread with a given priority */
    queue_helper_prios.lastThreadId[prio] = tid;

    /* Any priority has a bit in the "valid" bit-field indicating that in the ready queue
     * is at least one thread with this priority. */
    queue_helper_prios.valid           |= PRIO_SHIFT_BASE >> prio;
}

inline void queue_ready_del (const osImplThreadId tid, const osImplPriorityId prio)
{
    /* Implicit thread state after deletion from the read queue is ready-or-undefined.
     * See comment for queue_ready_contains() below. */
    const osImplThreadId th_next        = queue_unvrsl_ptr[tid].next;
    const osImplThreadId th_prev        = queue_unvrsl_ptr[tid].prev;

    if (queue_helper_prios.lastThreadId[prio] == tid)
    {
        const uint32_t x = ((queue_helper_prios.valid << prio) ^ PRIO_SHIFT_BASE) >> prio;
        if (queue_ready_first == tid || (x && queue_helper_prios.lastThreadId[ clz(x) ] == th_prev))
            /* Invalidate marker if this thread is the only one with given priority. */
            queue_helper_prios.valid ^= PRIO_SHIFT_BASE >> prio;
        else
            /* Note: th_prev is always correct here. */
            queue_helper_prios.lastThreadId[prio] = th_prev;

        /* Thread isn't in the tail of the whole queue? */
        if (!queue_ready_empty() && clz(queue_helper_prios.valid) >= prio)
        {
            queue_unvrsl_ptr[th_prev].next  = th_prev + 1;
            return;
        }
    }

    if (!queue_ready_empty())
    {
        if (queue_ready_first == tid)
        {
            queue_unvrsl_ptr[th_next].prev  = th_next - 1;
            queue_ready_first               = th_next;
        }
        else
        {
            queue_unvrsl_ptr[th_next].prev  = th_prev;
            queue_unvrsl_ptr[th_prev].next  = th_next;
        }
    }
}

inline bool queue_ready_getfirst (osImplThreadId * const p_tid)
{
    if (queue_ready_empty())
        return false;

    *p_tid = queue_ready_first;

    return true;
}

inline bool queue_ready_extract (osImplThreadId * const p_tid)
{
    if (!queue_inact_getfirst(p_tid))
        return false;

    const int32_t x = queue_helper_prios.valid;
    /* x & -x set to zero all bits expect the least significant one (1). */
    queue_ready_del(*p_tid, clz( (uint32_t)(x & -x) ));
    
    return true;
}

#if 0
inline void queue_ready_shift (const osImplThreadId tid, const osImplPriorityId prio)
{
    /* Shifting threads when only one has given priority is pure nonsens. */
    if (queue_helper_prios.lastThreadId[prio] == tid)
        return;

    /* Make aliases (compiler shouldn't allocate space for this conststants). */
    const osImplThreadId ins_after      = queue_helper_prios.lastThreadId[prio];
    const osImplThreadId ia_next        = queue_unvrsl_ptr[ins_after].next;

    queue_unvrsl_ptr[tid].next          = ia_next;
    queue_unvrsl_ptr[ins_after].next    = tid;
    queue_unvrsl_ptr[tid].prev          = ins_after;
    queue_unvrsl_ptr[ia_next].prev      = tid;

    /* Global variable referencing the first ready task should be updated to point
     * on its successor in the ready queue. */
    queue_ready_first                   = queue_unvrsl_ptr[queue_ready_first].next;
    queue_helper_prios.lastThreadId[prio] = tid;
}
#endif

inline bool queue_ready_contains(const osImplThreadId tid)
{
    /* WARNING: default state is ready-or-undefined so if this function returns true,
     * it not neccessary means that thread is really in the ready queue.
     * Discrimination between them is achieved on higher level bacause of transient
     * nature of the undefined state. */
    return (!queue_inact_contains(tid) && !queue_blckd_contains(tid));
}

inline bool queue_ready_empty (void)
{
    return ((bool)(!queue_helper_prios.valid));
}


void inline queue_inact_add (const osImplThreadId tid)
{
    if (osImplKernelFlags & KFLAGS_INACTIVE_EMPTY)
    {
        /* Cycle on both "next" and "prev" means it is the sole element in the threads
         * inactive queue. */
        queue_unvrsl_ptr[tid].next  = tid;
        osImplKernelFlags          ^= KFLAGS_INACTIVE_EMPTY;
    }
    else
    {
        queue_unvrsl_ptr[tid].next  = queue_inact_first;
    }

    queue_inact_first               = tid;
    /* Self-cycle only on prev index is indicating an inactive thread. */
    queue_unvrsl_ptr[tid].prev      = tid;
}

inline bool queue_inact_getfirst (osImplThreadId * const p_tid)
{
    if (!queue_inact_contains(queue_inact_first))
        return false;

    *p_tid = queue_inact_first;

    return true;
}

inline bool queue_inact_extract (osImplThreadId * const p_tid)
{
    if (queue_inact_getfirst(p_tid) == false)
        return false;

    /* Check if this thread is the last one in the queue. */
    if (queue_unvrsl_ptr[*p_tid].next == *p_tid)
        /* It is, so set appropriate flag. */
        osImplKernelFlags          |= KFLAGS_INACTIVE_EMPTY;

    /* Put the thread in ready-or-undefined state. */
    queue_unvrsl_ptr[*p_tid].next   = (*p_tid) - 1;
    queue_unvrsl_ptr[*p_tid].prev   = (*p_tid) + 1;      /* it gently wrap-around! */
    
    return true;
}

bool inline queue_inact_contains (const osImplThreadId tid)
{
    /* Inactive thread has left (on "prev" attribute) cycle (and right both 
     * if is the last one in a queue). */
    return (queue_unvrsl_ptr[tid].prev == tid);
}

bool inline queue_inact_empty (void)
{
    return ((bool)(osImplKernelFlags & KFLAGS_INACTIVE_EMPTY));
}


inline void queue_blckd_add (const osImplThreadId tid)
{
    /* Blocked is really virtual queue. */
    queue_unvrsl_ptr[tid].next      = tid;
    queue_unvrsl_ptr[tid].prev      = tid + 1;      /* it gently wrap-around! */
}

inline void queue_blckd_del (const osImplThreadId tid)
{
    /* Here could be two lines reseting cycles but this is not neccessary. */
    queue_unvrsl_ptr[tid].next      = tid - 1;
    queue_unvrsl_ptr[tid].prev      = tid + 1;
}

inline bool queue_blckd_contains (const osImplThreadId tid)
{
    const struct queue_unvrsl_elem_t elem  = queue_unvrsl_ptr[tid];

    /* Blocked threads have cycle on next _ONLY_ and isn't running of course. */
    return (elem.next == tid && elem.prev != tid && tid != running_tid);
}



/* ########################## CONVERTION FUNCTIONS ############################### */


/* Convert CMSIS-RTOS standard thread id to implementation specific. */
inline osImplThreadId  s2i_tid (osThreadId tid)
{
    return (osImplThreadId)((uintptr_t)tid - 1);
}

/* Convert implementation specific thread ID to CMSIS-RTOS standard compilant. */
inline osThreadId      i2s_tid (osImplThreadId tid)
{
    return (osThreadId)(tid + 1);
}

inline osImplPriorityId s2i_prio (osPriority prio)
{
    return (osImplPriorityId)(prio + 3);
}

inline osPriority i2s_prio (osImplPriorityId prio)
{
    return (osPriority)(prio - 3);
}


osThreadId  osThreadCreate (osThreadDef_t *thread_def, void *argument)
{
    osImplThreadId tid;
    osImplThreadId first_tid;

    /* Kernel is running and  have we at least one available slot for new thread? */
    if (!osKernelRunning() || !queue_inact_extract(&tid))
        return NULL;

    /* Yes, we have, so tid is valid now. */
    TCB_ATTR(priority    , tid) = s2i_prio(thread_def->tpriority);
    TCB_ATTR(stackPointer, tid) = port_prepare_stack(thread_def->stacksize, thread_def->pthread, argument);

    queue_ready_add(tid, s2i_prio(thread_def->tpriority));

    if (queue_ready_getfirst(&first_tid) == false)
    {
        /* Something went terribly wrong. Really, it can? */
        return NULL;
    }

    /* Check prio and, if it is higher than the current one, reschedule */
    if (TCB_ATTR(priority, first_tid) > TCB_ATTR(priority, running_tid))
        port_resched();

    return i2s_tid(tid);
}

inline osThreadId   osThreadGetId (void)
{
    if (!(osImplKernelFlags & KFLAGS_OS_RUNNING))
        return NULL;
    else
        return i2s_tid(running_tid);
}


osStatus    osThreadTerminate (osThreadId thread_id)
{
    const osImplThreadId tid = s2i_tid(thread_id);

    if (queue_inact_contains(tid))
        return osErrorValue;
    else if (running_tid == tid) {
        /* If we are killing currently running thread, the scheduler procedure must be runned. */
        port_resched();

        if (!queue_ready_extract((osImplThreadId *)&running_tid))
            return osErrorResource; /* Lack of any ready thread. */
        /* Free resources now. */
#if 0
        free(TCB_ATTR(init_sp, tid));
#endif
    }
    else if (queue_blckd_contains(tid))
        /* To be honest, deleting from blocked queue in this implementation isn't neccessary. */
        queue_blckd_del(tid);
    else   
        /* Not inactive, running neither blocked? It must be in ready state. */
        queue_ready_del(tid, TCB_ATTR(priority, tid));

    queue_inact_add(tid);
 
    return osOK;
}

osStatus 	osThreadSetPriority (osThreadId thread_id, osPriority priority)
{
    const osImplThreadId   tid      = s2i_tid(thread_id);
    const osImplPriorityId dst_prio = s2i_prio(priority);

    if (queue_inact_contains(tid))
        return osErrorResource;

    if (queue_ready_contains(tid))
    {
        queue_ready_del(tid, TCB_ATTR(priority, tid));
        queue_ready_add(tid, dst_prio);
    }

    TCB_ATTR(priority, tid)         = dst_prio;
    return osOK;
}

osPriority 	osThreadGetPriority (osThreadId thread_id)
{
    const osImplThreadId   tid      = s2i_tid(thread_id);

    if (queue_inact_contains(tid))
        return osPriorityError;

    return i2s_prio(TCB_ATTR(priority, tid));
}

osStatus 	osThreadYield (void)
{
    osImplThreadId tid_first;

    if (!osKernelRunning())
        return osErrorOS;

    if (!queue_ready_getfirst(&tid_first))
        return osOK;
    
    /* Rescheduling should occur when at least one ready thread has priority
     * equal or greater to current running task. */
    if (TCB_ATTR(priority, running_tid) < TCB_ATTR(priority, tid_first))
        return osOK;

    /* Put current running thread in the ready queue and load new from top of it. */
    port_resched();

    return osOK;
}

osImplThreadId kernel_context_switch (void)
{
    queue_ready_add(running_tid, TCB_ATTR(priority, running_tid));
    queue_ready_extract(&running_tid);

    return running_tid;
}

/* This doesn't need a comment... */
int32_t     osKernelRunning (void)
{
    return ((int32_t)(osImplKernelFlags & KFLAGS_OS_RUNNING));
}


osStatus osKernelStart (osThreadDef_t *thread_def, void *argument)
{
    /* Symbols with "linker" prefix are defined in the linker script (see file stm32.ld). */
    const size_t tdefs_num  = ((uintptr_t)&linker_tdefs_start - (uintptr_t)&linker_tdefs_end) / sizeof (osThreadDef_t);

    /* Every kind of threads has information about maximum number of it instances
     * buried in thread_def struct (one per kind), so we have possibility to compute
     * maximum number of all threads in the system. */
    for (size_t i = 0; i < tdefs_num; i++)
        max_threads_num    += linker_tdefs_start[i].instances;

    /* Reserve space for important system data structures. Note: elements of TCB array
     * should be optimized to avoid loses due to an alignment - this is the reason why
     * the TCB_ATTR and TCB_SIZE macros are existing for. */
    tcb_table_ptr           = malloc(max_threads_num * TCB_SIZE);
    queue_unvrsl_ptr        = malloc(max_threads_num * sizeof(*queue_unvrsl_ptr));

    osImplKernelFlags      |= KFLAGS_OS_RUNNING;
    osImplKernelFlags      |= KFLAGS_INACTIVE_EMPTY;

    /* Add all except one possible threadsID to inactive queue. */
    for (osImplThreadId i = 1; i < max_threads_num; i++)
        queue_inact_add(i);

    /* First thread gets 0 as ID. */
    running_tid             = 0;

    /* Prepare thread control block for initial task. TCB contains data in internal
     * representation only, so conversion from CMSIS standard must be done. */
    TCB_ATTR(stackPointer, running_tid) = malloc(thread_def->stacksize);
    TCB_ATTR(priority    , running_tid) = s2i_prio(thread_def->tpriority);

    /* Enable interrupts, switch control to thread code and do other arch. specific things.
     * This never returns. */
    port_start_kernel(TCB_ATTR(stackPointer, running_tid), thread_def->pthread, argument);

    /* Control should never reach this. */
    return osOK;
}

