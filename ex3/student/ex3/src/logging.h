/*
 * File name: logging.h
 * Date:      2005/10/09 15:09
 * Author:
 */

#ifndef __LOGGING_H__
#define __LOGGING_H__
#include <iostream>


//#include <log4cxx/logger.h>
namespace imr {

//   typedef log4cxx::Logger CLogger;
//   typedef log4cxx::LoggerPtr CLoggerPtr;

//#define INFO(str) LOG4CXX_INFO(logger, str);
//#define WARN(str) LOG4CXX_WARN(logger, str);
//#define ERROR(str) LOG4CXX_ERROR(logger, str);
//#define DEBUG(str) LOG4CXX_DEBUG(logger, str);

#define INFO(str)   std::cout << "INFO: " << str << std::endl;
#define WARN(str)   std::cout << "WARN: " << str << std::endl;
#define ERROR(str)  std::cout << "ERROR: " << str << std::endl;
#define DEBUG(str)  std::cout << "DEBUG: " << str << std::endl;

#define INFO_M(logger, str)     std::cout << "INFO: Logger: "<< logger << "Text: " str << std::endl;
#define WARN_M(logger, str)     std::cout << "WARN: Logger: "<< logger << "Text: " str << std::endl;
#define ERROR_M(logger, str)    std::cout << "ERROR: Logger: "<< logger << "Text: " str << std::endl;
#define DEBUG_M(logger, str)    std::cout << "DEBUG: Logger: "<< logger << "Text: " str << std::endl;

/*
#define MEM_CHECK(ptr, logger) {\
   if (!ptr) { \
      LOG4CXX_FATAL(logger, "Couldn't allocate memory");\
      exit(-2);\
   }\
}

#define PTR_CHECK(ptr, logger) {\
   if (!ptr) { \
      LOG4CXX_FATAL(logger, "Null pointer");\
      exit(-2);\
   }\
}


inline void warn(CLoggerPtr logger, std::string msg) {
    //LOG4CXX_WARN(logger, msg);
    WARN_M(logger, msg);
}

inline void error(CLoggerPtr logger, std::string msg) {
    //LOG4CXX_WARN(logger, msg);
    WARN_M(logger, msg);
}

void initLogger(const char * loggerName);

void initLogger(const char * loggerName, const char * configName);

CLoggerPtr getLogger(const char * loggerName);

void shutdownLogger(void);

//global available logger
extern CLoggerPtr logger;
*/
}
#endif

/* end of logging.h */
