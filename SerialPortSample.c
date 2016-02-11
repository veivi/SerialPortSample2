/*
     File: SerialPortSample.c
 Abstract: Command line tool that demonstrates how to use IOKitLib to find all serial ports on OS X. Also shows how to open, write to, read from, and close a serial port.
  Version: 1.5
 
 Disclaimer: IMPORTANT:  This Apple software is supplied to you by Apple
 Inc. ("Apple") in consideration of your agreement to the following
 terms, and your use, installation, modification or redistribution of
 this Apple software constitutes acceptance of these terms.  If you do
 not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.
 
 In consideration of your agreement to abide by the following terms, and
 subject to these terms, Apple grants you a personal, non-exclusive
 license, under Apple's copyrights in this original Apple software (the
 "Apple Software"), to use, reproduce, modify and redistribute the Apple
 Software, with or without modifications, in source and/or binary forms;
 provided that if you redistribute the Apple Software in its entirety and
 without modifications, you must retain this notice and the following
 text and disclaimers in all such redistributions of the Apple Software.
 Neither the name, trademarks, service marks or logos of Apple Inc. may
 be used to endorse or promote products derived from the Apple Software
 without specific prior written permission from Apple.  Except as
 expressly stated in this notice, no other rights or licenses, express or
 implied, are granted by Apple herein, including but not limited to any
 patent rights that may be infringed by your derivative works or by other
 works in which the Apple Software may be incorporated.
 
 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE
 MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND
 OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
 
 IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
 Copyright (C) 2013 Apple Inc. All Rights Reserved.
 
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>

#include "Log.h"
#include "LogDump.h"
#include "Datagram.h"

// Find the first device that matches the callout device path MATCH_PATH.
// If this is undefined, return the first device found.
#define MATCH_PATH "/dev/cu.usbmodem1411"

const int kNumRetries = 3;

// Hold the original termios attributes so we can reset them
static struct termios gOriginalTTYAttrs;

// Function prototypes
static kern_return_t findModems(io_iterator_t *matchingServices);
static kern_return_t getModemPath(io_iterator_t serialPortIterator, char *bsdPath, CFIndex maxPathSize);
static int openSerialPort(const char *bsdPath);
static void closeSerialPort(int fileDescriptor);

// Returns an iterator across all known modems. Caller is responsible for
// releasing the iterator when iteration is complete.
static kern_return_t findModems(io_iterator_t *matchingServices)
{
    kern_return_t			kernResult;
    CFMutableDictionaryRef	classesToMatch;
    
    // Serial devices are instances of class IOSerialBSDClient.
    // Create a matching dictionary to find those instances.
    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
    if (classesToMatch == NULL) {
        printf("// IOServiceMatching returned a NULL dictionary.\n");
    }
    else {
        // Look for devices that claim to be modems.
        CFDictionarySetValue(classesToMatch,
                             CFSTR(kIOSerialBSDTypeKey),
                             CFSTR(kIOSerialBSDModemType));
        
		// Each serial device object has a property with key
        // kIOSerialBSDTypeKey and a value that is one of kIOSerialBSDAllTypes,
        // kIOSerialBSDModemType, or kIOSerialBSDRS232Type. You can experiment with the
        // matching by changing the last parameter in the above call to CFDictionarySetValue.
        
        // As shipped, this sample is only interested in modems,
        // so add this property to the CFDictionary we're matching on.
        // This will find devices that advertise themselves as modems,
        // such as built-in and USB modems. However, this match won't find serial modems.
    }
    
    // Get an iterator across all matching devices.
    kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, matchingServices);
    if (KERN_SUCCESS != kernResult) {
        printf("// IOServiceGetMatchingServices returned %d\n", kernResult);
		goto exit;
    }
    
exit:
    return kernResult;
}

// Given an iterator across a set of modems, return the BSD path to the first one with the callout device
// path matching MATCH_PATH if defined.
// If MATCH_PATH is not defined, return the first device found.
// If no modems are found the path name is set to an empty string.
static kern_return_t getModemPath(io_iterator_t serialPortIterator, char *bsdPath, CFIndex maxPathSize)
{
    io_object_t		modemService;
    kern_return_t	kernResult = KERN_FAILURE;
    Boolean			modemFound = false;
    
    // Initialize the returned path
    *bsdPath = '\0';
    
    // Iterate across all modems found. In this example, we bail after finding the first modem.
    
    while ((modemService = IOIteratorNext(serialPortIterator)) && !modemFound) {
        CFTypeRef	bsdPathAsCFString;
        
		// Get the callout device's path (/dev/cu.xxxxx). The callout device should almost always be
		// used: the dialin device (/dev/tty.xxxxx) would be used when monitoring a serial port for
		// incoming calls, e.g. a fax listener.
        
		bsdPathAsCFString = IORegistryEntryCreateCFProperty(modemService,
                                                            CFSTR(kIOCalloutDeviceKey),
                                                            kCFAllocatorDefault,
                                                            0);
        if (bsdPathAsCFString) {
            Boolean result;
            
            // Convert the path from a CFString to a C (NUL-terminated) string for use
			// with the POSIX open() call.
            
			result = CFStringGetCString(bsdPathAsCFString,
                                        bsdPath,
                                        maxPathSize,
                                        kCFStringEncodingUTF8);
            CFRelease(bsdPathAsCFString);
            
#ifdef MATCH_PATH
            if (strncmp(bsdPath, MATCH_PATH, strlen(MATCH_PATH)) != 0) {
                result = false;
            }
#endif
            
            if (result) {
                printf("// Modem found with BSD path: %s", bsdPath);
                modemFound = true;
                kernResult = KERN_SUCCESS;
            }
        }
        
        printf("\n");
        
        // Release the io_service_t now that we are done with it.
        
		(void) IOObjectRelease(modemService);
    }
    
    return kernResult;
}

bool initConsoleInput()
{
    struct termios attrs;

    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(STDIN_FILENO, &attrs) == -1) {
        printf("Error getting tty attributes - %s(%d).\n",
               strerror(errno), errno);
        return false;
    }
    
    // Set raw input (non-canonical) mode, with reads blocking until either a single character
    // has been received or a one second timeout expires.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> and termios(4) <x-man-page://4/termios> for details.
    
    attrs.c_lflag &= ~ICANON;
    attrs.c_cc[VMIN] = 0;
    attrs.c_cc[VTIME] = 1;
    attrs.c_lflag &= ~(ECHO);
    
    // Cause the new options to take effect immediately.
    if (tcsetattr(STDIN_FILENO, TCSANOW, &attrs) == -1) {
        printf("Error setting tty attributes - %s(%d).\n",
               strerror(errno), errno);
        return false;
    }

    return true;
}

// Given the path to a serial device, open the device and configure it.
// Return the file descriptor associated with the device.
static int openSerialPort(const char *bsdPath)
{
    int				fileDescriptor = -1;
    int				handshake;
    struct termios	options;
    
    // Open the serial port read/write, with no controlling terminal, and don't wait for a connection.
    // The O_NONBLOCK flag also causes subsequent I/O on the device to be non-blocking.
    // See open(2) <x-man-page://2/open> for details.
    
    fileDescriptor = open(bsdPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fileDescriptor == -1) {
        printf("Error opening serial port %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
    }
    
    // Note that open() follows POSIX semantics: multiple open() calls to the same file will succeed
    // unless the TIOCEXCL ioctl is issued. This will prevent additional opens except by root-owned
    // processes.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    if (ioctl(fileDescriptor, TIOCEXCL) == -1) {
        printf("Error setting TIOCEXCL on %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
    }
    
    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
    // See fcntl(2) <x-man-page//2/fcntl> for details.
    
    if (fcntl(fileDescriptor, F_SETFL, 0) == -1) {
        printf("Error clearing O_NONBLOCK %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
    }
    
    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(fileDescriptor, &gOriginalTTYAttrs) == -1) {
        printf("Error getting tty attributes %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
    }
    
    // The serial port attributes such as timeouts and baud rate are set by modifying the termios
    // structure and then calling tcsetattr() to cause the changes to take effect. Note that the
    // changes will not become effective without the tcsetattr() call.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> for details.
    
    options = gOriginalTTYAttrs;
    
    // Print the current input and output baud rates.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> for details.
    
    printf("// Current input baud rate is %d\n", (int) cfgetispeed(&options));
    printf("// Current output baud rate is %d\n", (int) cfgetospeed(&options));
    
    // Set raw input (non-canonical) mode, with reads blocking until either a single character
    // has been received or a one second timeout expires.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> and termios(4) <x-man-page://4/termios> for details.
    
    cfmakeraw(&options);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    
    // The baud rate, word length, and handshake options can be set as follows:
    
    cfsetspeed(&options, B115200);		// Set 19200 baud
    options.c_cflag |= (CS8);
    
	// The IOSSIOSPEED ioctl can be used to set arbitrary baud rates
	// other than those specified by POSIX. The driver for the underlying serial hardware
	// ultimately determines which baud rates can be used. This ioctl sets both the input
	// and output speed.
/*
	speed_t speed = 115200; // Set 14400 baud
    if (ioctl(fileDescriptor, IOSSIOSPEED, &speed) == -1) {
        printf("Error calling ioctl(..., IOSSIOSPEED, ...) %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
    }
*/
    // Print the new input and output baud rates. Note that the IOSSIOSPEED ioctl interacts with the serial driver
	// directly bypassing the termios struct. This means that the following two calls will not be able to read
	// the current baud rate if the IOSSIOSPEED ioctl was used but will instead return the speed set by the last call
	// to cfsetspeed.
    
    printf("// Input baud rate changed to %d\n", (int) cfgetispeed(&options));
    printf("// Output baud rate changed to %d\n", (int) cfgetospeed(&options));
    
    // Cause the new options to take effect immediately.
    if (tcsetattr(fileDescriptor, TCSANOW, &options) == -1) {
        printf("Error setting tty attributes %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
    }
    
    // To set the modem handshake lines, use the following ioctls.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    // Assert Data Terminal Ready (DTR)
    if (ioctl(fileDescriptor, TIOCSDTR) == -1) {
        printf("Error asserting DTR %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
    }
    
    // Clear Data Terminal Ready (DTR)
    if (ioctl(fileDescriptor, TIOCCDTR) == -1) {
        printf("Error clearing DTR %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
    }
    
    // Set the modem lines depending on the bits set in handshake
    handshake = TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
    if (ioctl(fileDescriptor, TIOCMSET, &handshake) == -1) {
        printf("Error setting handshake lines %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
    }
    
    // To read the state of the modem lines, use the following ioctl.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    // Store the state of the modem lines in handshake
    if (ioctl(fileDescriptor, TIOCMGET, &handshake) == -1) {
        printf("Error getting handshake lines %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
    }
    
    printf("// Handshake lines currently set to %d\n", handshake);
	
	unsigned long mics = 10000UL;
    
	// Set the receive latency in microseconds. Serial drivers use this value to determine how often to
	// dequeue characters received by the hardware. Most applications don't need to set this value: if an
	// app reads lines of characters, the app can't do anything until the line termination character has been
	// received anyway. The most common applications which are sensitive to read latency are MIDI and IrDA
	// applications.
	
	if (ioctl(fileDescriptor, IOSSDATALAT, &mics) == -1) {
		// set latency to 1 microsecond
        printf("Error setting read latency %s - %s(%d).\n",
               bsdPath, strerror(errno), errno);
        goto error;
	}
    
    // Success
    return fileDescriptor;
    
    // Failure path
error:
    if (fileDescriptor != -1) {
        close(fileDescriptor);
    }
    
    return -1;
}

void serialWrite(int fileDescriptor, const char *string)
{
    ssize_t stringLen = strlen(string);
    
    while(stringLen > 0) {
        ssize_t numBytes = write(fileDescriptor, string, stringLen);
        
        if(numBytes > 0) {
            string += numBytes;
            stringLen -= numBytes;
        }
    }
}

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
    crc ^= a;
    
    for (int i = 0; i < 8; ++i)
        crc = (crc >> 1) ^ ((crc & 1) * 0xA001U);
    
    return crc;
}

static uint16_t crc16(uint16_t initial, const uint8_t *data, int len)
{
    uint16_t crc = initial;
    
    for(int i = 0; i < len; i++)
        crc = crc16_update(crc, data[i]);
    
    return crc;
}

int datagramSize = 0;
const int maxSize = 1<<16;
uint8_t store[maxSize];

void storeByte(const uint8_t c)
{
    if(datagramSize < maxSize)
        store[datagramSize++] = c;
}

int datagrams, datagramsGood;

int hearbeatCount = 0;

#define MAX_LOG_SIZE (1<<24)
uint16_t logStorage[MAX_LOG_SIZE];
int logTotal;
struct LogInfo logInfo;

void logStore(const uint16_t *data, int count)
{
    if(logTotal + count < MAX_LOG_SIZE) {
        memcpy(&logStorage[logTotal], data, count*sizeof(uint16_t));
        logTotal += count;
        printf("// RECEIVED %d ENTRIES\r", logTotal);
        fflush(stdout);
    } else {
        printf("// LOG STORAGE OVERFLOW\n");
        fflush(stdout);
    }
}

void logDisplay()
{
    printf("\n// LOG DUMP COMPLETED\n");
    
    logDump(&logInfo, logStorage, logTotal);
    
    fflush(stdout);
    logTotal = 0;
}

void datagramInterpret(uint8_t t, const uint8_t *data, int size)
{
    switch(t) {
        case DG_HEARTBEAT:
            // Heartbeat
            
            hearbeatCount++;
            break;
            
        case DG_CONSOLE_OUT:
            // Console output
            
            write(1, data, size);
            break;
            
        case DG_LOGDATA:
            // Log data
            if(size > 0)
                logStore((const uint16_t*) data, size/2);
            else
                logDisplay();
            break;
            
        case DG_LOGINFO:
            // Log stamp
            memcpy(&logInfo, data, sizeof(logInfo));
            printf("\n// LOG %d OF MODEL %s\n", logInfo.stamp, logInfo.name);
            break;
    }
}

Boolean datagramEnd(void)
{
    Boolean success = false;
    
    if(datagramSize > 2) {
        datagrams++;
        uint16_t crcReceived = *((uint16_t*) &store[datagramSize-2]);
        uint16_t crc = crc16(0xFFFF, store, datagramSize - 2);
        success = crc == crcReceived;
        
        if(success) {
            datagramsGood++;
            datagramInterpret(store[0], &store[1], datagramSize-3);
        }
    }
    
    datagramSize = 0;
    return success;
}

Boolean binaryInputChar(const uint8_t c)
{
    static uint8_t prev;
    static Boolean busy = false;
    Boolean success = false;

    if(prev == 0x00 && c == 0x00) {
        if(busy)
            success = datagramEnd();
        
        busy = false;
    } else if(busy) {
        if(prev == 0x00)
            storeByte(0x00);
        else if(c != 0x00)
            storeByte(c);
    } else if(c != 0x00) {
        busy = true;
        datagramSize = 0;
        storeByte(c);
    }
    
    prev = c;
    
    return success;
}

static Boolean dumpLog(int fileDescriptor)
{
    char		buffer[1024];	// Input buffer
    ssize_t		numBytes;		// Number of bytes read or written
    Boolean		result = false;
    
    while (1) {
        // Local input
        
        numBytes = read(STDIN_FILENO, buffer, sizeof(buffer));

        for(int i = 0; i < numBytes; i++) {
            write(fileDescriptor, &buffer[i], 1);
            if((buffer[i] == '\r' || buffer[i] == '\n') && i < numBytes-1)
                usleep(0.05*1E6);
        }

        // Link input
        
        numBytes = read(fileDescriptor, buffer, sizeof(buffer));

        for(int i = 0; i < numBytes; i++)
            binaryInputChar(buffer[i]);
    }
    
    return result;
}

// Given the file descriptor for a serial device, close that device.
void closeSerialPort(int fileDescriptor)
{
    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
	// See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.
    if (tcdrain(fileDescriptor) == -1) {
        printf("Error waiting for drain - %s(%d).\n",
               strerror(errno), errno);
    }
    
    // Traditionally it is good practice to reset a serial port back to
    // the state in which you found it. This is why the original termios struct
    // was saved.
    if (tcsetattr(fileDescriptor, TCSANOW, &gOriginalTTYAttrs) == -1) {
        printf("Error resetting tty attributes - %s(%d).\n",
               strerror(errno), errno);
    }
    
    close(fileDescriptor);
}

int main(int argc, const char * argv[])
{
    int             fileDescriptor;
    kern_return_t	kernResult;
    io_iterator_t	serialPortIterator;
    char            bsdPath[MAXPATHLEN];
    
    kernResult = findModems(&serialPortIterator);
    if (KERN_SUCCESS != kernResult) {
        printf("No modems were found.\n");
    }
    
    kernResult = getModemPath(serialPortIterator, bsdPath, sizeof(bsdPath));
    if (KERN_SUCCESS != kernResult) {
        printf("Could not get path for modem.\n");
    }
    
    IOObjectRelease(serialPortIterator);	// Release the iterator.
    
    // Now open the modem port we found, initialize the modem, then close it
    if (!bsdPath[0]) {
        printf("No modem port found.\n");
        return EX_UNAVAILABLE;
    }
    
    if(!initConsoleInput())
        return 0;
    
    fileDescriptor = openSerialPort(bsdPath);
    if (-1 == fileDescriptor) {
        return EX_IOERR;
    }
    
    if (dumpLog(fileDescriptor)) {
//        printf("Log dumped successfully, size = %dk+%d entries.\n", (int) logSize/(1<<10), logSize%(1<<10));
    }
    else {
        printf("Could not initialize modem.\n");
    }
    
    closeSerialPort(fileDescriptor);
    printf("Modem port closed.\n");
    
    return EX_OK;
}
