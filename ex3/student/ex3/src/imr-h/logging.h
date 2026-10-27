/*
 * File name: logging.h
 * Date:      2005/10/09 15:09
 * Author:
 */

#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <iostream>
namespace imr {


#define INFO(str)   std::cout << "INFO: " << str << std::endl;
#define WARN(str)   std::cout << "WARN: " << str << std::endl;
#define ERROR(str)  std::cout << "ERROR: " << str << std::endl;
#define DEBUG(str)  std::cout << "DEBUG: " << str << std::endl;

#define INFO_M(logger, str)     std::cout << "INFO: Logger: "<< logger << "Text: " str << std::endl;
#define WARN_M(logger, str)     std::cout << "WARN: Logger: "<< logger << "Text: " str << std::endl;
#define ERROR_M(logger, str)    std::cout << "ERROR: Logger: "<< logger << "Text: " str << std::endl;
#define DEBUG_M(logger, str)    std::cout << "DEBUG: Logger: "<< logger << "Text: " str << std::endl;
}
#endif

/* end of logging.h */
