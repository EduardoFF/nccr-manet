#include "timer.h"
#include "dbg.h"
#include <unistd.h>

//TODO pthread? should it be joinable or detached? do some research to see what is 
// 		more efficient

  void
*Timer::Func(void *arg)
{
  Timer *t = 
    (Timer *) arg;
  struct timespec start_time, gen_time, sleep_time;
  clock_gettime(CLOCK_REALTIME, &start_time);
  debug("Timer starts @ %ld %ld",
	start_time.tv_sec,
	start_time.tv_nsec);
  for(;;)
  {
    /// get next delay

    if(!t->m_isRunning)
    {
      debug("Timer not running - goodbye thread");
      break;
    }
    long currentDt = t->getDt();

    if( t->m_type == TIMER_SECONDS)
    {
      sleep_time.tv_sec = currentDt;
      sleep_time.tv_nsec = 0;
    } else {
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = currentDt;
      while( sleep_time.tv_nsec > 1000000000)
      {
	sleep_time.tv_sec++;
	sleep_time.tv_nsec-=1000000000;
      }
    }
    nanosleep(&sleep_time, NULL);


    time_t sec_diff;
    long nsec_diff;
    clock_gettime(CLOCK_REALTIME, &gen_time);
    if( gen_time.tv_sec > start_time.tv_sec)
    {
      sec_diff = gen_time.tv_sec - start_time.tv_sec - 1;
      nsec_diff = (1000000000 - start_time.tv_nsec) + gen_time.tv_nsec;
      while( nsec_diff > 1000000000)
      {
	sec_diff++;
	nsec_diff-=1000000000;
      }
    } else {
      sec_diff = 0;
      nsec_diff = gen_time.tv_nsec - start_time.tv_nsec;
    }
    debug("Timer fired @ %ld:%ld\n",
	   sec_diff, nsec_diff);
    start_time = gen_time;

    t->fired();
  }
  pthread_exit(0);

}
void
Timer::stop()
{
  if(!m_isRunning)
    return;
  m_isRunning = false;
  if( pthread_equal( pthread_self(), m_thread))
  {
    debug("same thread - suicide");
    pthread_exit(0);
  }else 
  {
    debug("waiting for the timer thread");
    pthread_join(m_thread,NULL);
  }

  m_isPeriodic = false;
  m_currentDt = 0;
}

long
Timer::getDt()
{
  long dt=0;
  pthread_mutex_lock(&m_mutex);
  dt=m_currentDt;
  pthread_mutex_unlock(&m_mutex);
  return dt;
}
void
Timer::setDt(unsigned int dt)
{
  pthread_mutex_lock(&m_mutex);
  if( m_type == TIMER_SECONDS)
  {
    /// thread will use sleep
    m_currentDt = dt;
  } else {
    /// thread will use nanosleep
    if( m_type == TIMER_MILLISECONDS)
    {
      m_currentDt = dt * 1000000;
    } else if( m_type == TIMER_MICROSECONDS)
    {
      m_currentDt = dt * 1000;
    }
  }
  pthread_mutex_unlock(&m_mutex);
}


void
Timer::startPeriodic(unsigned int dt)
{
  if( m_isRunning )
  {
    /// stop
    stop();
  }

  m_isRunning = true;
  m_isPeriodic = true;
  setDt(dt);
  if( pthread_create(&m_thread,NULL,&Func,(void *) this) )
  {
    log_err("Error creating thread");
    m_isRunning = false;
    m_isPeriodic = false;
    m_currentDt = 0;
  }
}

void
Timer::startOneShot(unsigned int dt)
{
  if( pthread_equal( pthread_self(), m_thread))
  {

    debug("another shot from same thread");
    // to be sure
    m_isRunning = true;
    m_isPeriodic = false;
    setDt(dt);
  } else 
  {
    if( m_isRunning )
    {
      debug("thread is running - stopping");
      /// stop
      stop();
    }
    m_isRunning = true;
    m_isPeriodic = false;
    setDt(dt);
    if( pthread_create(&m_thread,NULL,&Func,(void *) this) )
    {
      log_err("Error creating thread");
      m_isRunning = false;
      m_isPeriodic = false;
      m_currentDt = 0;
    }
  }
}

void
Timer::fired()
{
  if( !m_isPeriodic)
  {
    /// this should shut down the thread
    /// unless it is restarted from cb
    m_isRunning = false;
  }
  m_cb(m_cbPar);
}
