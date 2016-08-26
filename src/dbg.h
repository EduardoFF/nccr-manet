#ifndef __dbg_h__
#define __dbg_h__


/** Platform-independent (more or less) debug macros
*/

#include <stdio.h>
#include <errno.h>
#include <string.h>

#define USE_GLOG_FOR_DEBUG 1

#ifdef USE_GLOG_FOR_DEBUG
#include <glog/logging.h>
#endif

#ifdef NDEBUG
#define debug(M, ...)
#else
#ifdef USE_GLOG_FOR_DEBUG
#define debug(M, ...) { static char dbuff[255]; sprintf(dbuff, "DEBUG %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__); LOG(INFO) << dbuff;}
#else
#define debug(M, ...) fprintf(stderr, "DEBUG %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__); fflush(stderr)
#endif
#endif

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) fprintf(stderr, "[ERROR] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) fprintf(stderr, "[WARN] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) fprintf(stderr, "[INFO] (%s:%d) " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define check(A, M, ...) if(!(A)) { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define sentinel(M, ...)  { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define check_mem(A) check((A), "Out of memory.")

#define check_debug(A, M, ...) if(!(A)) { debug(M, ##__VA_ARGS__); errno=0; goto error; }

#endif
