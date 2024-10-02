#ifndef DATALOGGER_H
#define DATALOGGER_H

extern float current_time;

void init_SD();
void log_data(void* pvParameters);

#endif