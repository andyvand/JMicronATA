/*
 * Copyright (c) 2007 Dmitri Arekhta
 * Code derived from AppleOboardPCATA
 */

/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 2.0 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#ifndef _JMICRONATA_H_
#define _JMICRONATA_H_

#define UnsignedWide UInt64

#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOPCIATA.h>
#include <IOKit/ata/IOATAController.h>
#include <IOKit/ata/ATADeviceNub.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

#include "JMicronMisc.h"
#include "JMicronATAChannel.h"

#define kPIOModeCount    5		//PIO   mode 0 - 4
#define kDMAModeCount    3		//DMA   mode 0 - 2
#define kUltraModeCount  7		//Ultra mode 0 - 6

struct TimingParameter
{
    UInt16  cycleTimeNS;	//total cycle time
    UInt8   timingValue;	//timing register value
    UInt8   modeNumber;
};

static const TimingParameter PIOTimingTable[ kPIOModeCount ]=
{
    //Cycle  Value   Mode
    { 600,   0x5d,   0 },
    { 383,   0x47,   1 },
    { 240,   0x34,   2 },
    { 180,   0x22,   3 },
    { 120,   0x20,   4 }
};

static const TimingParameter DMATimingTable[ kDMAModeCount ]=
{
	//Cycle  Value   Mode
    { 480,   0x77,   0 },
    { 150,   0x21,   1 },
    { 120,   0x20,   2 }
};

struct BusTimings 
{
    UInt8                   pioModeNumber;
    const TimingParameter * pioTiming;
    UInt8                   dmaModeNumber;
    const TimingParameter * dmaTiming;
    UInt8                   ultraModeNumber;
    bool                    ultraEnabled;
};

struct HardwareInfo 
{
    UInt32			pciDeviceID;
    UInt8			minPCIRevID;
    UInt8			maxUltraMode;
	UInt8			maxPATAPorts;
    const char		*deviceName;
};

#define PCI_PIO_TIMING          0x40
#define PIO_TIMING_CH0D0_MASK   0x0000FF00
#define PIO_TIMING_CH0D1_MASK   0x000000FF
#define PIO_TIMING_CH1D0_MASK   0xFF000000
#define PIO_TIMING_CH1D1_MASK   0x00FF0000

#define PCI_DMA_TIMING          0x44
#define DMA_TIMING_CH0D0_MASK   0x0000FF00
#define DMA_TIMING_CH0D1_MASK   0x000000FF
#define DMA_TIMING_CH1D0_MASK   0xFF000000
#define DMA_TIMING_CH1D1_MASK   0x00FF0000

#define PCI_PIO_MODE            0x4A
#define PIO_MODE_CH0D0_MASK     0x000F
#define PIO_MODE_CH0D1_MASK     0x00F0
#define PIO_MODE_CH1D0_MASK     0x0F00
#define PIO_MODE_CH1D1_MASK     0xF000

#define PCI_ULTRA_ENABLE        0x54
#define ULTRA_CH0D0_ENABLE      0x01
#define ULTRA_CH0D1_ENABLE      0x02
#define ULTRA_CH1D0_ENABLE      0x04
#define ULTRA_CH1D1_ENABLE      0x08

#define PCI_ULTRA_MODE          0x56
#define ULTRA_MODE_CH0D0_MASK   0x000F
#define ULTRA_MODE_CH0D1_MASK   0x00F0
#define ULTRA_MODE_CH1D0_MASK   0x0F00
#define ULTRA_MODE_CH1D1_MASK   0xF000

#define PCI_ULTRA_CONTROL       0x5a
#define ULTRA_CTRL_DISABLE      0x40
#define ULTRA_CTRL_MODE_MASK    0x03
#define ULTRA_CTRL_MODE_4       0x02
#define ULTRA_CTRL_MODE_5       0x03

class JMicronATA : public IOPCIATA
{
    OSDeclareDefaultStructors( JMicronATA )

protected:
    JMicronATAChannel			*fChannelNub;
    UInt32						fChannelNumber;
    bool						fHardwareLostPower;
    IOInterruptEventSource		*fInterruptSource;
    IOWorkLoop					*fWorkLoop;
    BusTimings					fBusTimings[ kMaxDriveCount ];
    bool						f80PinCable[ kMaxDriveCount ];
    UInt16						fBMBaseAddr;
    UInt32						fUltraModeMask;
    const HardwareInfo			*fHWInfo;
	IOMemoryMap					*registerMap[ 3 ];
    IOBufferMemoryDescriptor*	_prdBuffer;

    virtual bool				publishDriveProperty(UInt32 driveUnit,
									const char * propKey,
									UInt32       value );

    virtual bool				openATAChannel( IOService * provider );

    virtual void				closeATAChannel( void );

    virtual void				attachATADeviceNubs( void );

    virtual IOReturn			synchronousIO( void );

    virtual void				initForPM( IOService * provider );

    virtual void				free( void );
	
    /* Interrupt event source action */

    static void					interruptOccurred( OSObject * owner,
									IOInterruptEventSource * source,
									int count );

    /* Interrupt event source filter */

    static bool					interruptFilter(
									OSObject * owner,
									IOFilterInterruptEventSource * source );
							   
	/* Hardware related code here */
    virtual bool				getBMBaseAddress( UInt16 * baseAddr );
	
	virtual void				resetBusTimings( void );
	
    virtual void				restoreHardwareState( void );
	
	virtual void				saveHardwareState( void );
	
	/* UDMA mode selection */
    virtual void				selectTimingParameter(IOATADevConfig * configRequest,
									UInt32           unitNumber );


public:
    virtual bool				start( IOService *provider );

    virtual void				stop( IOService *provider );

    virtual IOWorkLoop			*getWorkLoop( void ) const;
	
    virtual IOReturn			provideBusInfo( IOATABusInfo *infoOut );

    virtual IOReturn			getConfig( IOATADevConfig *configOut, UInt32 unit );

    virtual IOReturn			selectConfig( IOATADevConfig *config, UInt32 unit );

    virtual IOReturn			message( UInt32      type,
									IOService * provider,
									void *      argument );

    virtual bool				configureTFPointers( void );

    virtual UInt32				scanForDrives( void );

    virtual IOReturn			handleQueueFlush( void );

    virtual IOReturn			createChannelCommands( void );

    virtual bool				allocDMAChannel( void );

    virtual bool				freeDMAChannel( void );

    virtual void				initATADMAChains( PRD * descPtr );

    enum 
	{
        kPowerStateOff = 0,
        kPowerStateDoze,
        kPowerStateOn,
        kPowerStateCount
    };

    virtual IOReturn			setPowerState(unsigned long stateIndex, IOService *whatDevice );
};

#endif
