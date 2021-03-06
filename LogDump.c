//
//  LogDump.c
//  SerialPortSample
//
//  Created by Juhani Vehvilainen on 10/02/16.
//
//

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "LogDump.h"
#include "Log.h"

uint32_t logLen;
static FILE *outputFile;
static int col = 0;
static bool first = false, tick = false;

static void logOutputInit(void)
{
    col = 20;
    first = true;
    tick = false;
}

long valueCount;

void printNum(float v, int p)
{
    const char fmt[] = {'%', '.', '0'+p, 'f', '\0'};
    fprintf(outputFile, fmt, (double) v);
}

void printString(const char *f, ...)
{
    va_list args;
    va_start(args, f);
    vfprintf(outputFile, f, args);
    va_end(args);
}

static void logOutputValue(float v)
{
    float av = fabs(v);
    
    if(!first) {
        printString(",");
        col++;
    }
    
    if(col > 60) {
        float progress = 100.0 * (float) valueCount / logLen / lc_channels;
        printString(" // %d%%\n", (int) progress);
        col = 0;
    }
    
    if(av < 0.001) {
        col++;
        printString("0");
    } else if(fabs(av - 1.0) < 0.001){
        printString(v < 0.0 ? "-1" : "1");
        col += v < 0.0 ? 2 : 1;
    } else {
        int decimals = av < 1 ? 3 : av < 10 ? 2 : av < 100 ? 1 : 0;
        printNum(v, decimals);
        col += 3 + (v < 0.0 ? 1 : 0) + (decimals > 0 ? 1 : 0) + (av < 1.0 ? 1 : 0)
        + (av >= 1000.0 ? 1 : 0) + (av >= 10000.0 ? 1 : 0);
    }
    
    first = false;
}

static void logOutputString(const char *s)
{
    if(!first) {
        printString(", ");
        col++;
    }
    
    if(col > 72) {
        printString("");
        col = 0;
    }
    
    printString("\"%s\"", s);
    
    col += strlen(s) + 2;
    
    first = false;
}

static void logOutputVariableName(int stamp, const char *name)
{
    if(!first) {
        printString(";");
        col++;
    }
    
    if(col > 72) {
        printString("");
        col = 0;
    }
    
    printString("fdr_%d_%s", stamp, name);
    
    col += 4 + 3 + 1 + strlen(name);
    
    first = false;
}

static void logOutputValueInvalid(float small, float large)
{
    logOutputValue(tick ? small : large);
    tick = !tick;
}

void logDumpCh(int ch, int stamp, const uint16_t *store)
{
    printString("// CHANNEL %d (%s) DATA\n", ch, logChannels[ch].name);
    
    printString("fdr_%d_%s = [ ", stamp, logChannels[ch].name);
    
    float small = logChannels[ch].small, large = logChannels[ch].large;
    int currentCh = -1, nextCh = -1;
    uint16_t valueRaw = 0;
    bool valueValid = false;
    float value = 0.0;
    
    logOutputInit(); // Initialize
    
    for(int32_t i = 0; i < logLen; i++) {
        valueCount++;
        
        uint16_t entry = store[i];
        
        if(ENTRY_IS_TOKEN(entry) && ENTRY_VALUE(entry) < t_delta) {
            LogToken_t token = (LogToken_t) ENTRY_VALUE(entry);
            
            switch(token) {
                case t_stamp:
                    // End marker, a count follows, ignore both
                    i++;
                    break;
                    
                case t_start:
                    break;
                    
                case t_mark:
                    // Mark
                    
                    for(int j = 0; j < 10; j++)
                        logOutputValueInvalid(small, large);
                    break;
                    
                default:
                    if(token >= t_channel && token < t_channel+lc_channels) {
                        // Valid channel id
                        
                        nextCh = token - t_channel;
                    } else {
                        // Invalid token
                        
                        printString(" // *** Invalid entry 0x%X\n", token);
                        break;
                    }
            }
        } else {
            // A log value entry
            
            if(!ENTRY_IS_TOKEN(entry))
                currentCh = nextCh;
            
            if(logChannels[currentCh].tick) {
                if(valueValid)
                    logOutputValue(value);
                else
                    logOutputValueInvalid(small, large);
            }
            
            if(currentCh == ch) {
                if(ENTRY_IS_TOKEN(entry)) {
                    // Delta value
                    
                    valueRaw = ENTRY_VALUE(valueRaw + ((ENTRY_VALUE(entry) & DELTA_MASK)<< 1));
                    
                } else {
                    // Absolute value
                    
                    valueRaw = entry;
                }
                
                value = small + (float) valueRaw / (float) VALUE_MASK * (large - small);
                valueValid = true;
            }
            
            if(currentCh > -1)
                nextCh = currentCh + 1;
        }
    }
    
    printString(" ]\n");
}

void logDump(FILE *output, struct LogInfo *info, const uint16_t *store, int len)
{
    outputFile = output;
    logLen = len;
    
    valueCount = 0;
        
    for(int ch = 0; ch < lc_channels; ch++)
        logDumpCh(ch, info->stamp, store);
        
    printString("fdr_");
    printNum(info->stamp, 0);
    printString("_matrix = [ ");
        
    logOutputInit();
    
    for(int ch = 0; ch < lc_channels; ch++)
        logOutputVariableName(info->stamp, logChannels[ch].name);
        
    printString(" ]\n");
        
    printString("// FLIGHT DATA RECORD WITH INDEX\n");
        
    printString("fdr_");
    printNum(info->stamp, 0);
    printString(" = { fdr_");
    printNum(info->stamp, 0);
    printString("_matrix, ");
    
    logOutputInit();
        
    for(int ch = 0; ch < lc_channels; ch++)
        logOutputString(logChannels[ch].name);
        
    printString(" }\n");
    
    printString("fdr = fdr_%d\n", info->stamp);
}
