/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef RADAR2CARCENTER_LOGGING_HPP_
#define RADAR2CARCENTER_LOGGING_HPP_

#define OUTPUT
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

#ifdef OUTPUT
#define LOGI(...)                                                              \
  (printf("[INFO] [%d@%s] ", __LINE__, __FILENAME__), printf(__VA_ARGS__),     \
   printf("\n"))
#define LOGW(...)                                                              \
  (printf("\33[33m[WARN] [%d@%s] ", __LINE__, __FILENAME__),                   \
   printf(__VA_ARGS__), printf("\033[0m\n"))
#define LOGE(...)                                                              \
  (printf("\33[31m[ERROR] [%d@%s] ", __LINE__, __FILENAME__),                  \
   printf(__VA_ARGS__), printf("\033[0m\n"))
#else
#define LOGI(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGE(...) ((void)0)
#endif

#ifdef DEBUG
#define LOGDEBUG(...) (printf(__VA_ARGS__), printf("\n"))
#else
#define LOGDEBUG(...) ((void)0)
#endif

#endif //  RADAR2CARCENTER_LOGGING_HPP_