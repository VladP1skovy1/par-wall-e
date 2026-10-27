/*
 * File name: imr_assert.h
 * Date:      2008/08/05 20:23
 * Author:    Jan Faigl
 */

#ifndef __IMR_ASSERT_H__
#define __IMR_ASSERT_H__

#include "imr_exceptions.h"

namespace imr {

   template<class T>
      inline void assert_exception(bool cond, std::string msg) {
         if (!cond) {
            throw T(msg);
         }
      }

   inline void assert_argument(bool cond, std::string msg) {
      assert_exception<argument_error>(cond, msg);
   }

   inline void assert_usage(bool cond, std::string msg) {
      assert_exception<bad_usage>(cond, msg);
   }

   inline void assert_io(bool cond, std::string msg) {
      assert_exception<io_error>(cond, msg);
   }

} //end namespace imr


#endif

/* end of imr_assert.h */
