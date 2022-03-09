/*
 * Copyright (C) 2021-2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */
#ifndef COMMON_LOGGING_HPP_
#define COMMON_LOGGING_HPP_

#define DEBUG
#define __FILENAME__ \
    (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

#ifdef DEBUG
#define LOGI(...) ((void)0)
#define LOGE(...) ((void)0)
#define LOGW(...) \
    (printf("\33[33m[WARN] [%d@%s] ", __LINE__, __FILENAME__), printf(__VA_ARGS__), printf("\033[0m\n"))
#else
#define LOGI(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGE(...) ((void)0)
#endif

#endif  //  COMMON_LOG_HPP_