#ifndef TIMER_H
#define TIMER_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>


enum TimerType {
    TIMER_SECONDS,
    TIMER_MILLISECONDS,
    TIMER_MICROSECONDS
};

class Timer 
{
    typedef void (*FiredCB)(void *);

    TimerType m_type;
    FiredCB m_cb;
    void *m_cbPar; //! Callback parameters
    bool m_isRunning;
    bool m_isPeriodic;
    long m_currentDt;
    pthread_t m_thread;
    //!  Mutex to control the access to the internal data
    pthread_mutex_t m_mutex;
    void fired();


    static void *Func(void *arg);

public:
    Timer(TimerType t, FiredCB f, void *cbpar = NULL)
        :m_type(t)
        ,m_cb(f)
        ,m_cbPar(cbpar)
        ,m_isRunning(false)
        ,m_isPeriodic(false)
        ,m_currentDt(0)
    {
        if (pthread_mutex_init(&m_mutex, NULL) != 0)
        {
            fprintf(stderr, "mutex init failed\n");
            fflush(stderr);
            exit(-1);
        }
    }
    ~Timer()
    {
        pthread_mutex_destroy(&m_mutex);
    }

    /// basic interface
    void startPeriodic( unsigned int dt );
    void startOneShot( unsigned int dt );
    void stop();
    void setDt(unsigned int);
    long getDt();
    inline bool isRunning() { return m_isRunning;}
};
#endif
