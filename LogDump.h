//
//  LogDump.h
//  SerialPortSample
//
//  Created by Juhani Vehvilainen on 10/02/16.
//
//

#ifndef LogDump_h
#define LogDump_h

#include <stdint.h>
#include <stdio.h>
#include "Log.h"

void logDump(FILE *output, struct LogInfo *info, const uint16_t *store, int len);

#endif /* LogDump_h */
