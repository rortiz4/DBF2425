#ifndef DATALOGGER_H
#define DATALOGGER_H

extern float current_time;
extern bool serial_log;
extern bool SD_log;

void init_SD(bool serial_log, bool SD_log);
void log_data(void* pvParameters);

#endif