#include <iostream>
#include <string>

#include "utils/time_util.hpp"

void GetLocalTime(timeval &tv) {
#ifdef _WIN32
  time_t clock;
  struct tm tm;
  SYSTEMTIME wtm;
  GetLocalTime(&wtm);
  tm.tm_year = wtm.wYear - 1900;
  tm.tm_mon = wtm.wMonth - 1;
  tm.tm_mday = wtm.wDay;
  tm.tm_hour = wtm.wHour;
  tm.tm_min = wtm.wMinute;
  tm.tm_sec = wtm.wSecond;
  tm.tm_isdst = -1;
  clock = mktime(&tm);
  tv.tv_sec = clock;
  tv.tv_usec = wtm.wMilliseconds * 1000;
#else
  gettimeofday(&tv, NULL);
#endif
}

std::string FormatSTDTimeString(const std::string &inStr) {
  if (inStr.size() != 17 && inStr.size() != 23) {
    std::cout << "invalid input " << inStr << "(" << inStr.size()
              << ") format is yyyy-mm-dd-hh-mi-ss-ms or "
              << "yyyymmddhhmissms" << std::endl;
    return inStr;
  }
  std::string outStr;
  // format:yyyymmddhhmissms
  if (inStr.size() == 17) {
    outStr = inStr.substr(0, 4) + "-" +  // yyyy
             inStr.substr(4, 2) + "-" +  // mm
             inStr.substr(6, 2) + " " +  // dd
             inStr.substr(8, 2) + ":" +  // hh
             inStr.substr(10, 2) + ":" + // mi
             inStr.substr(12, 2) + "." + // ss
             inStr.substr(14, 3);
  }
  // format:yyyy-mm-dd-hh-mi-ss-ms
  if (inStr.size() == 23) {
    outStr = inStr;
    outStr[10] = ' ';
    outStr[13] = outStr[16] = ':';
    outStr[19] = '.';
  }
  return outStr;
}

// convert time string into timeval, inputtime string with format:
// yyyy-mm-dd HH:Mi:SS.ms(us,ns)
bool timeString2timeval(const std::string &timeStr, timeval &tv) {
  if (timeStr.size() < 23)
    return false;
  struct tm stm;
  if (!strptime(timeStr.substr(0, 19).c_str(), "%Y-%m-%d %H:%M:%S", &stm) &&
      false)
    return false;
  int us;
  std::string usStr = timeStr.substr(20);
  if (usStr.size() == 3)
    us = stoi(usStr) * 1000;
  else if (usStr.size() == 6)
    us = stoi(usStr);
  else if (usStr.size() == 9)
    us = stoi(usStr) / 1000;
  else
    return false;
  if (us < 0 || us > 999999)
    return false;
  time_t sec = mktime(&stm);
  tv.tv_sec = sec;
  tv.tv_usec = us;
  return true;
}
// if input time string is in UTC, set isUTC = true
bool timeString2timeval(bool isUTC, const std::string &timeStr, timeval &tv) {
  if (timeStr.size() < 23)
    return false;
  struct tm stm;
  if (!strptime(timeStr.substr(0, 19).c_str(), "%Y-%m-%d %H:%M:%S", &stm) &&
      false)
    return false;
  int us;
  std::string usStr = timeStr.substr(20);
  if (usStr.size() == 3)
    us = stoi(usStr) * 1000;
  else if (usStr.size() == 6)
    us = stoi(usStr);
  else if (usStr.size() == 9)
    us = stoi(usStr) / 1000;
  else
    return false;
  if (us < 0 || us > 999999)
    return false;
  time_t sec;
  if (isUTC) {
    sec = timegm(&stm);
  } else {
    sec = mktime(&stm);
  }

  tv.tv_sec = sec;
  tv.tv_usec = us;
  return true;
}
// convert timeval into string with format:yyyy-mm-dd-HH-Mi-SS-ms
void timeval2String(const timeval &tv, std::string &timeStr) {
  unsigned int ms = tv.tv_usec / 1000;
  time_t time(tv.tv_sec);
  struct tm stm;
  localtime_r(&time, &stm);
  // struct tm* stm = localtime(&time);
  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", &stm);
  char res[128];
  snprintf(res, sizeof(res), "%s-%03d", buffer, ms);
  timeStr = std::string(res);
}

std::string timeval2String(const timeval &tv) {
  std::string str;
  timeval2String(tv, str);
  return str;
}