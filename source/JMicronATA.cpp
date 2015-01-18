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

#include <sys/systm.h>         // snprintf
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOMessage.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>

#include "JMicronATA.h"

// Increase the PRD table size to one full page or 512 descriptors to allow
// large transfers via the dma engine.

#define kMaxPRDCount				512
#define kMaxPRDSegmentSize			0x10000

// Limited to 2048 ATA sectors per transfer.

#define kMaxATATransferSize			(512 * 2048)

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer

#define kMaxATAXfer      512 * 2048
#define kPIOModeMask     ((1 << kPIOModeCount) - 1)
#define kDMAModeMask     ((1 << kDMAModeCount) - 1)

#define DRIVE_IS_PRESENT(u) \
        (_devInfo[u].type != kUnknownATADeviceType)

#define TIMING_PARAM_IS_VALID(p) \
        ((p) != 0)

#define CLASS JMicronATA
#define super IOPCIATA

#ifdef FEDE_DEBUG
#define FEDE_LOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#define FEDE_RELOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#else
#define FEDE_LOG(fmt,args...) 
#define FEDE_RELOG(fmt,args...)
#endif

OSDefineMetaClassAndStructors( JMicronATA, IOPCIATA )

static const HardwareInfo * getHardwareInfo( UInt32 pciID, UInt8 pciRevID )
{
    const HardwareInfo * info = 0;

    static const HardwareInfo hardwareTable[] =
    {
		{ 0x2360197b, 0x00, 7, 1, "JMB360"},
		{ 0x2361197b, 0x00, 7, 0, "JMB361"},
		{ 0x2363197b, 0x00, 7, 1, "JMB363"},
		{ 0x2365197b, 0x00, 7, 1, "JMB365"},
		{ 0x2366197b, 0x00, 7, 2, "JMB366"},
		{ 0x2368197b, 0x00, 7, 1, "JMB368"}		
    };

    for (UInt i = 0; i < sizeof(hardwareTable)/sizeof(hardwareTable[0]); i++)
    {
        if (hardwareTable[i].pciDeviceID == pciID)
        {
            info = &hardwareTable[i];
            break;
        }
    }

    return info;
}

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool CLASS::start( IOService * provider )
{
    bool superStarted = false;

    DbgPrint("%s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);

	/*setProperty(kIOPropertyPhysicalInterconnectTypeKey,
				kIOPropertyPhysicalInterconnectTypeSerialATA);
				
	setProperty(kIOPropertyPhysicalInterconnectLocationKey,
				kIOPropertyExternalKey);*/

    if (openATAChannel(provider) == false)
    {
        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    // Create a work loop.
    fWorkLoop = IOWorkLoop::workLoop();
    if (fWorkLoop == NULL)
    {
        DbgPrint("%s: new work loop failed\n", getName());

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    if (fChannelNumber > kSecondaryChannelID)
    {
        DbgPrint("%s: bad ATA channel number %u\n", getName(),
                  (unsigned int)fChannelNumber);

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }


    fHWInfo = getHardwareInfo(fChannelNub->pciConfigRead32(kIOPCIConfigVendorID),
				fChannelNub->pciConfigRead8(kIOPCIConfigRevisionID));
    if (!fHWInfo)
    {
        DbgPrint("%s: unsupported hardware\n", getName());

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    DbgPrint("chip revision = %02x\n",
              fChannelNub->pciConfigRead8(kIOPCIConfigRevisionID));

    fUltraModeMask = (1 << (fHWInfo->maxUltraMode + 1)) - 1;

    f80PinCable[0] = true;
    f80PinCable[1] = true;


#if defined(__i386__) || defined(__x86_64__)
    // Get the base address for the bus master registers in I/O space.
    if (!getBMBaseAddress(&fBMBaseAddr))
    {
        DbgPrint("%s: get bus-master base address failed\n", getName());

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    // Must setup these variables inherited from IOPCIATA before super::start
    _bmCommandReg   = IOATAIOReg8::withAddress(  fBMBaseAddr + BM_COMMAND );
    _bmStatusReg    = IOATAIOReg8::withAddress(  fBMBaseAddr + BM_STATUS );
    _bmPRDAddresReg = IOATAIOReg32::withAddress( fBMBaseAddr + BM_PRD_PTR );
#else
	//PPC platform (__ppc__ section)
	
	IOPCIDevice *pciNub = fChannelNub->getPCIDevice();
	
	registerMap[2] = pciNub->mapDeviceMemoryWithRegister(0x20);
	if (!registerMap[2]) 
	{
		DbgPrint("%s: Couldn't get Bus Master MMRs\n", getName());
		IOSleep(1000);	// Debug

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }
	
	DbgPrint("%s: registerMap[2] virtual address %x, length %d\n", 
			getName(), (int)registerMap[2]->getVirtualAddress(), (int)registerMap[2]->getLength());
		
	volatile UInt8 *bmAddr = (volatile UInt8 *)(registerMap[2]->getVirtualAddress());
	if (!bmAddr)
	{
		DbgPrint("%s: Couldn't get Bus Master MMRs\n", getName());
		IOSleep(1000);	// Debug

        if (fChannelNub)
            closeATAChannel();

        if (superStarted)
            super::stop( provider );
        
        return false;
	}
	
    if (fChannelNumber == kSecondaryChannelID) bmAddr += BM_SEC_OFFSET;
	
	DbgPrint("%s: Bus Master address %x\n", getName(), bmAddr);
	
	_bmCommandReg = (IOATARegPtr8)(bmAddr + BM_COMMAND);
	_bmStatusReg = (IOATARegPtr8)(bmAddr + BM_STATUS);
	_bmPRDAddresReg = (IOATARegPtr32)(bmAddr + BM_PRD_PTR);
#endif

    // Reset bus timings for both drives.
    resetBusTimings();

    // Now we are ready to call super::start

    if (super::start(provider) == false)
    {
        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    superStarted = true;

    // Create interrupt event source that will signal the
    // work loop (thread) when a device interrupt occurs.

    if ( fChannelNub->getInterruptVector() == 14 ||
         fChannelNub->getInterruptVector() == 15 )
    {
        fInterruptSource = IOInterruptEventSource::interruptEventSource(
                           this, &interruptOccurred,
                           fChannelNub, 0 );
    }
    else
    {
        fInterruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
                           this, &interruptOccurred, &interruptFilter,
                           fChannelNub, 0 );
    }

    if (!fInterruptSource ||
        (fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess))
    {
        DbgPrint("%s: interrupt event source error\n", getName());

        if (fChannelNub)
            closeATAChannel();
        
        if (superStarted)
            super::stop( provider );
        
        return false;
    }

    fInterruptSource->enable();

    attachATADeviceNubs();

	// Initializing Power Management
	initForPM( provider );
	
    IOLog("%s: %s (CMD 0x%x, CTR 0x%x, IRQ %u, BM 0x%x)\n",
          getName(),
          fHWInfo->deviceName,
          (unsigned int)(fChannelNub->getCommandBlockAddress()),
          (unsigned int)(fChannelNub->getControlBlockAddress()),
          (unsigned int)(fChannelNub->getInterruptVector()),
          fBMBaseAddr);

    return true;
}

void CLASS::stop( IOService * provider )
{
    DbgPrint("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    closeATAChannel();
    //Stop Power Management
	PMstop();

    if (fInterruptSource && fWorkLoop)
    {
        fWorkLoop->removeEventSource(fInterruptSource);
    }

    super::stop( provider );
}

IOWorkLoop * CLASS::getWorkLoop( void ) const
{
    return fWorkLoop;
}

bool CLASS::openATAChannel( IOService * provider )
{
    // Our provider is a 'nub' that represents a single PCI ATA channel,
    // and not an IOPCIDevice.

    fChannelNub = OSDynamicCast(JMicronATAChannel, provider);
    if (fChannelNub == 0)
    {
        DbgPrint("%s: ATA channel type mismatch\n", getName());
        return false;
    }

    // Retain and open our provider.

    fChannelNub->retain();
    if (fChannelNub->open(this) != true)
    {
        DbgPrint("%s: provider open failed\n", getName());
        return false;
    }

    fChannelNumber = fChannelNub->getChannelNumber();

    return true;
}

void CLASS::closeATAChannel( void )
{
    if (fChannelNub)
    {
        fChannelNub->close(this);
    }
}

void CLASS::free( void )
{
	DbgPrint("%s::%s(%p)\n", getName(), __FUNCTION__, this);

    RELEASE( fInterruptSource );
    RELEASE( fWorkLoop );

    RELEASE( fChannelNub );
    RELEASE( _nub[0] );
    RELEASE( _nub[1] );

#if defined(__i386__)
    // Release registers created by configureTFPointers().
    RELEASE( _tfDataReg );
    RELEASE( _tfFeatureReg );
    RELEASE( _tfSCountReg );
    RELEASE( _tfSectorNReg );
    RELEASE( _tfCylLoReg );
    RELEASE( _tfCylHiReg );
    RELEASE( _tfSDHReg );
    RELEASE( _tfStatusCmdReg );
    RELEASE( _tfAltSDevCReg );

    RELEASE( _bmCommandReg );
    RELEASE( _bmStatusReg );
    RELEASE( _bmPRDAddresReg );
#else
	//PPC platform (__ppc__ section)
	RELEASE( registerMap[0] );
	RELEASE( registerMap[1] );
	RELEASE( registerMap[2] );
#endif

    // IOATAController should release this.

    if ( _doubleBuffer.logicalBuffer )
    {
        IOFree((void *)_doubleBuffer.logicalBuffer,
               _doubleBuffer.bufferSize);
        _doubleBuffer.bufferSize     = 0;
        _doubleBuffer.logicalBuffer  = 0;
        _doubleBuffer.physicalBuffer = 0;
    }

    super::free();
}

#pragma mark -
#pragma mark - Controller mode -
#pragma mark -

void CLASS::selectTimingParameter( IOATADevConfig * configRequest, UInt32           unit )
{
    DbgPrint("[CH%u D%u] %s( %p )\n", (unsigned int)fChannelNumber, (unsigned int)unit, __FUNCTION__,
              configRequest);

    // Reset existing parameters for this unit.
    fBusTimings[unit].pioTiming = &PIOTimingTable[0];
    fBusTimings[unit].dmaTiming = 0;
    fBusTimings[unit].ultraEnabled = false;

    if (configRequest->getPIOMode())
    {
        UInt32  pioModeNumber;
        UInt32  pioCycleTime;
        UInt32  pioTimingEntry = 0;

        pioModeNumber = bitSigToNumeric( configRequest->getPIOMode() );
        pioModeNumber = min(pioModeNumber, kPIOModeCount - 1);

        // Use a default cycle time if the device didn't report a time to use.

        pioCycleTime = configRequest->getPIOCycleTime();
        pioCycleTime = max(pioCycleTime,
                           PIOTimingTable[pioModeNumber].cycleTimeNS);

        // Look for the fastest entry in the PIOMinCycleTime with a cycle time
        // which is larger than or equal to pioCycleTime.
    
        for (int i = kPIOModeCount - 1; i > 0; i--)
        {
            if (PIOTimingTable[i].cycleTimeNS >= pioCycleTime)
            {
                pioTimingEntry = i;
                break;
            }
        }

        fBusTimings[unit].pioTiming = &PIOTimingTable[pioTimingEntry];
        fBusTimings[unit].pioModeNumber = pioModeNumber;
        DbgPrint("  selected PIO timing entry %d\n", pioTimingEntry);
        publishDriveProperty(unit, kSelectedPIOModeKey, pioModeNumber);
    }

    if (configRequest->getDMAMode())
    {
        UInt32  dmaModeNumber;
        UInt32  dmaCycleTime;
        UInt32  dmaTimingEntry = 0;

        dmaModeNumber = bitSigToNumeric(configRequest->getDMAMode());
        dmaModeNumber = min(dmaModeNumber, kDMAModeCount - 1);

        dmaCycleTime = configRequest->getDMACycleTime();
        dmaCycleTime = max(dmaCycleTime,
                           DMATimingTable[dmaModeNumber].cycleTimeNS);

        // Look for the fastest entry in the DMAMinCycleTime with a cycle time
        // which is larger than or equal to dmaCycleTime.
    
        for (int i = kDMAModeCount - 1; i > 0; i--)
        {
            if (DMATimingTable[i].cycleTimeNS >= dmaCycleTime)
            {
                dmaTimingEntry = i;
                break;
            }
        }
        
        fBusTimings[unit].dmaTiming = &DMATimingTable[dmaTimingEntry];
        fBusTimings[unit].dmaModeNumber = dmaModeNumber;
        DbgPrint("  selected DMA timing entry %d\n", dmaTimingEntry);
        publishDriveProperty(unit, kSelectedDMAModeKey, dmaModeNumber);
    }

    if (configRequest->getUltraMode())
    {
        UInt32  ultraModeNumber;

        ultraModeNumber = bitSigToNumeric(configRequest->getUltraMode());
        ultraModeNumber = min(ultraModeNumber, fHWInfo->maxUltraMode);

        // For Ultra DMA mode 3 or higher, 80 pin cable must be present.
        // Otherwise, the drive will be limited to UDMA mode 2.

        if (ultraModeNumber > 2)
        {
            if ( f80PinCable[unit] == false )
            {
                DbgPrint("  80-conductor cable not detected\n");
                ultraModeNumber = 2;
            }
        }

        fBusTimings[unit].ultraEnabled = true;
        fBusTimings[unit].ultraModeNumber = ultraModeNumber;
        DbgPrint("  selected Ultra mode %d\n", ultraModeNumber);
        publishDriveProperty(unit, kSelectedUltraDMAModeKey, ultraModeNumber);
    }
}

IOReturn CLASS::provideBusInfo( IOATABusInfo * infoOut )
{
    DbgPrint("[CH%u] %s( %p )\n", (unsigned int)fChannelNumber, __FUNCTION__, infoOut);

    if (infoOut == 0)
    {
        return -1;
    }

    infoOut->zeroData();
    infoOut->setSocketType( kInternalATASocket );
    infoOut->setPIOModes( kPIOModeMask );
    infoOut->setDMAModes( kDMAModeMask );
    infoOut->setUltraModes( fUltraModeMask );
    infoOut->setExtendedLBA( true );
    infoOut->setMaxBlocksExtended( 0x0800 );  // 2048 sectors for ext LBA

    UInt8 units = 0;
    if ( _devInfo[0].type != kUnknownATADeviceType ) units++;
    if ( _devInfo[1].type != kUnknownATADeviceType ) units++;
    infoOut->setUnits( units );

    return kATANoErr;
}

IOReturn CLASS::getConfig( IOATADevConfig *configOut, UInt32 unit )
{
    DbgPrint("[CH%u D%u] %s( %p )\n", (unsigned int)fChannelNumber, (unsigned int)unit, __FUNCTION__,
              configOut);

    if ((configOut == 0) || (unit > kATADevice1DeviceID))
    {
        return -1;
    }

    configOut->setPIOMode( 0 );
    configOut->setDMAMode( 0 );
    configOut->setUltraMode( 0 );

    // Note that we need to report the bitmap of each mode,
    // not its mode number.

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].pioTiming))
    {
        configOut->setPIOMode( 1 << fBusTimings[unit].pioModeNumber );
        configOut->setPIOCycleTime( fBusTimings[unit].pioTiming->cycleTimeNS );
        DbgPrint("  PIO mode %hhu @ %u ns\n",
                  fBusTimings[unit].pioModeNumber,
                  fBusTimings[unit].pioTiming->cycleTimeNS);
    }

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].dmaTiming))
    {
        configOut->setDMAMode( 1 << fBusTimings[unit].dmaModeNumber );
        configOut->setDMACycleTime( fBusTimings[unit].dmaTiming->cycleTimeNS );
        DbgPrint("  DMA mode %hhu @ %u ns\n",
                  fBusTimings[unit].dmaModeNumber,
                  fBusTimings[unit].dmaTiming->cycleTimeNS);
    }

    if (fBusTimings[unit].ultraEnabled)
    {
        configOut->setUltraMode( 1 << fBusTimings[unit].ultraModeNumber );
        DbgPrint("  Ultra mode %hhu\n", fBusTimings[unit].ultraModeNumber);
    }

    configOut->setPacketConfig( _devInfo[unit].packetSend );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Select the bus timings for a given drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn CLASS::selectConfig( IOATADevConfig *configRequest, UInt32 unit )
{
    DbgPrint("[CH%u D%u] %s( %p )\n", (unsigned int)fChannelNumber, (unsigned int)unit, __FUNCTION__,
              configRequest);

    if ((configRequest == 0) || (unit > kATADevice1DeviceID))
    {
        return -1;
    }

    // All config requests must include a supported PIO mode

    if ((configRequest->getPIOMode() & kPIOModeMask) == 0)
    {
        DbgPrint("  missing PIO mode\n");
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() & ~kDMAModeMask)
    {
        DbgPrint("  DMA mode not supported\n");
        return kATAModeNotSupported;
    }

    if (configRequest->getUltraMode() & ~fUltraModeMask)
    {
        DbgPrint("  Ultra DMA mode not supported\n");
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() && configRequest->getUltraMode())
    {
        DbgPrint("  multiple DMA mode selection error\n");
        return kATAModeNotSupported;
    }

    _devInfo[unit].packetSend = configRequest->getPacketConfig();

    selectTimingParameter( configRequest, unit );

    return getConfig( configRequest, unit );
}

#pragma mark -
#pragma mark - Interrupts handler -
#pragma mark -

bool CLASS::interruptFilter( OSObject *owner, IOFilterInterruptEventSource *src )
{
    CLASS * driver = (CLASS *) owner;

    if (*(driver->_bmStatusReg) & BM_STATUS_INT)
	{
		//This is our interrupt
        return true;   
	}
    else
	{
        return false;  
	}
}

void CLASS::interruptOccurred( OSObject *owner, IOInterruptEventSource *source, int count )
{
    CLASS * me = (CLASS *) owner;

    // Clear interrupt latch

    *(me->_bmStatusReg) = BM_STATUS_INT;

    me->handleDeviceInterrupt();
}

#pragma mark -
#pragma mark - Hardware related code -
#pragma mark -

bool CLASS::getBMBaseAddress( UInt16 * baseAddr )
{
    UInt32 bmiba;

    DbgPrint("[CH%u] %s\n", (unsigned int)fChannelNumber, __FUNCTION__);

    bmiba = fChannelNub->pciConfigRead32( 0x20 );

    if ((bmiba & 0x01) == 0)
    {
        DbgPrint("  PCI BAR 0x20 (0x%08x) is not an I/O range\n", (unsigned int)bmiba);
        return false;
    }

    bmiba &= BM_ADDR_MASK;  // get the address portion
    if (bmiba == 0)
    {
        DbgPrint("  BMIBA is zero\n");
        return false;
    }

    if (fChannelNumber == kSecondaryChannelID)
        bmiba += BM_SEC_OFFSET;

    *baseAddr = (UInt16) bmiba;
    DbgPrint("  BMBaseAddr = %04x\n", *baseAddr);
    return true;
}

void CLASS::resetBusTimings( void )
{
    DbgPrint("[CH%u] %s\n", (unsigned int)fChannelNumber, __FUNCTION__);

    memset(&fBusTimings[0], 0, sizeof(fBusTimings));

    fBusTimings[0].pioTiming = &PIOTimingTable[0];
    fBusTimings[1].pioTiming = &PIOTimingTable[0];
}

void CLASS::restoreHardwareState( void )
{
	fChannelNub->restoreHwResources();
}

void CLASS::saveHardwareState( void )
{
	fChannelNub->prepareHwForSleep();
}

void CLASS::attachATADeviceNubs( void )
{
    for (UInt32 i = 0; i < kMaxDriveCount; i++)
    {
        if (_devInfo[i].type != kUnknownATADeviceType)
        {
            ATADeviceNub * nub;

            nub = ATADeviceNub::ataDeviceNub( (IOATAController *) this,
                                              (ataUnitID) i,
                                              _devInfo[i].type );

            if (nub)
            {
                if (_devInfo[i].type == kATAPIDeviceType)
                {
                    // Report less than the full PRD count to handle
                    // any PRD alignment restrictions.

                    nub->setProperty( kIOMaximumSegmentCountReadKey,
                                      kMaxPRDCount / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentCountWriteKey,
                                      kMaxPRDCount / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountReadKey,
                                      kMaxPRDSegmentSize, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountWriteKey,
                                      kMaxPRDSegmentSize, 64 );
									  
                }
				else	// For ATA devices
				{
					nub->setProperty( kJMicronDeviceType,
										kJMicronIDEDevice );
				}


                if (nub->attach(this))
                {
                    _nub[i] = (IOATADevice *) nub;
                    _nub[i]->retain();
                    _nub[i]->registerService();
                }
                nub->release();
            }
        }
    }
}

/*---------------------------------------------------------------------------
 *
 * Override IOATAController::synchronousIO()
 *
 ---------------------------------------------------------------------------*/

IOReturn CLASS::synchronousIO( void )
{
    IOWorkLoop *WorkLoop;
    IOReturn ret;


    WorkLoop = getWorkLoop();

    if (WorkLoop) WorkLoop->disableAllInterrupts();
    ret = super::synchronousIO();
    if (WorkLoop) WorkLoop->enableAllInterrupts();

    return ret;
}

#pragma mark -
#pragma mark - ATA Registers Setup -
#pragma mark -

bool CLASS::configureTFPointers( void )
{
    DbgPrint("[CH%u] %s\n", (unsigned int)fChannelNumber, __FUNCTION__);

    if (!fChannelNub) return false;

#if defined(__i386__)
    UInt16 cmdBlockAddr = fChannelNub->getCommandBlockAddress();
    UInt16 ctrBlockAddr = fChannelNub->getControlBlockAddress();
	
	DbgPrint("%s: Command block address %x, control block address %x\n", 
				getName(), cmdBlockAddr, ctrBlockAddr);

    _tfDataReg      = IOATAIOReg16::withAddress( cmdBlockAddr + 0 );
    _tfFeatureReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 1 );
    _tfSCountReg    = IOATAIOReg8::withAddress(  cmdBlockAddr + 2 );
    _tfSectorNReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 3 );
    _tfCylLoReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 4 );
    _tfCylHiReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 5 );
    _tfSDHReg       = IOATAIOReg8::withAddress(  cmdBlockAddr + 6 );
    _tfStatusCmdReg = IOATAIOReg8::withAddress(  cmdBlockAddr + 7 );
    _tfAltSDevCReg  = IOATAIOReg8::withAddress(  ctrBlockAddr + 2 );

#else
	//PPC platform (__ppc__ section)
	IOPCIDevice *pciNub = fChannelNub->getPCIDevice();
	
	if (fChannelNumber == kPrimaryChannelID)
	{
		registerMap[0] = pciNub->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
		registerMap[1] = pciNub->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);	
	}
	else if (fChannelNumber == kSecondaryChannelID)
	{
		registerMap[0] = pciNub->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress2);
		registerMap[1] = pciNub->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress3);		
	}
	
	if (!registerMap[0] || !registerMap[1])
	{
		DbgPrint("%s: Couldn't get ATA MMRs, registerMap[0] = %lx, registerMap[1] = %lx\n",
					getName(), (unsigned long)registerMap[0], (unsigned long)registerMap[1]);
		IOSleep(1000);	// Debug
		return false;
	}	
	
	DbgPrint("%s: registerMap[0] virtual address %llu, length %llu\n", 
				getName(), registerMap[0]->getVirtualAddress(), registerMap[0]->getLength());
	DbgPrint("%s: registerMap[1] virtual address %llu, length %llu\n", 
				getName(), registerMap[1]->getVirtualAddress(), registerMap[1]->getLength());
			
	IOSleep(1500);		// Debug sleep
			
	volatile UInt8 *cmdBlockBase = reinterpret_cast<volatile UInt8 *>(registerMap[0]->getVirtualAddress());
	volatile UInt8 *ctrBlockBase = reinterpret_cast<volatile UInt8 *>(registerMap[1]->getVirtualAddress());
	
	if (!cmdBlockBase || !ctrBlockBase)
	{
		DbgPrint("%s: Couldn't get ATA MMRs\n", getName());
		return false;
	}
	
    _tfDataReg = (IOATARegPtr16)(cmdBlockBase + 0);
    _tfFeatureReg = (IOATARegPtr8)(cmdBlockBase + 1);
    _tfSCountReg = (IOATARegPtr8)(cmdBlockBase + 2);
    _tfSectorNReg = (IOATARegPtr8)(cmdBlockBase + 3);
    _tfCylLoReg = (IOATARegPtr8)(cmdBlockBase + 4);
    _tfCylHiReg = (IOATARegPtr8)(cmdBlockBase + 5);
    _tfSDHReg = (IOATARegPtr8)(cmdBlockBase + 6);
    _tfStatusCmdReg = (IOATARegPtr8)(cmdBlockBase + 7);
    _tfAltSDevCReg = (IOATARegPtr8)(ctrBlockBase + 2);
#endif

    if ( !_tfDataReg || !_tfFeatureReg || !_tfSCountReg ||
         !_tfSectorNReg || !_tfCylLoReg || !_tfCylHiReg ||
         !_tfSDHReg || !_tfStatusCmdReg || !_tfAltSDevCReg )
    {
		DbgPrint("Failed to map ATA registers\n");
		IOSleep(1000);	// Debug
        return false;
    }
	
	UInt16 tmp16;
	UInt8 tmp8;
	
	tmp16 = *_tfDataReg;
	DbgPrint("_tfDataReg %x\n", tmp16);
	
	tmp8 = *_tfFeatureReg;
	DbgPrint("_tfFeatureReg %x\n", tmp8);
	tmp8 = *_tfSCountReg;
	DbgPrint("_tfSCountReg %x\n", tmp8);
	tmp8 = *_tfSectorNReg;
	DbgPrint("_tfSectorNReg %x\n", tmp8);
	tmp8 = *_tfCylLoReg;
	DbgPrint("_tfCylLoReg %x\n", tmp8);
	tmp8 = *_tfCylHiReg;
	DbgPrint("_tfCylHiReg %x\n", tmp8);
	tmp8 = *_tfSDHReg;
	DbgPrint("_tfSDHReg %x\n", tmp8);

    return true;
}

#pragma mark -
#pragma mark - Scan for Drives - 
#pragma mark -

UInt32 CLASS::scanForDrives( void )
{
    UInt32 unitsFound = 0;

    DbgPrint("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    *_tfAltSDevCReg = mATADCRReset;

    IODelay( 100 );

    *_tfAltSDevCReg = 0x0;

    IOSleep( 10 );


	DbgPrint("%s::%s( %p )\n", getName(), __FUNCTION__, this);
	
	UInt8 status = 0x00;
	// count total time spent searching max time allowed = 31 secs
	// it RARELY takes this long.
	UInt32 milsSpent = 0; 
	
	// wait for a not busy bus
	// should be ready, but some devices may be slow to wake or spin up.
	for( int loopMils = 0; milsSpent < 3100; loopMils++ )
	{
		OSSynchronizeIO();
		status = *_tfStatusCmdReg;
		if( (status & mATABusy) == 0x00 )
		{
			DbgPrint("Bus is ready for scan\n");
			break;
		}
		
		IOSleep( 10 );	
		milsSpent++;
	}

	// spun on BSY for too long, declare bus empty
	if( ! (milsSpent < 3100) )
		goto AllDone;
		
	
	// select each possible device on the bus, wait for BSY- 
	// then check for protocol signatures.	

	for( int unit = 0; unit < 2; unit++ )
	{

		// wait for a not busy bus
		for( int loopMils = 0; milsSpent < 3100; loopMils++ )
		{
			// write the selection bit
			OSSynchronizeIO();
			*_tfSDHReg	= ( unit << 4 );
			IODelay( 10 );
			// typically, devices respond quickly to selection
			// but we'll give it a chance in case it is slow for some reason.
			status = *_tfStatusCmdReg;
			if( (status & mATABusy) == 0x00 )
			{	
				DbgPrint("Unit %d, bus is ready for scan\n", unit);
				break;	
			}
			IOSleep( 10 );	
			milsSpent++;
		}

		// spun on BSY too long, probably bad device
		if( ! (milsSpent < 3100) )
			goto AllDone;
			
		UInt8 tmp8 = *_tfCylLoReg;
		DbgPrint("ATAPI regs _tfCylLoReg %x", tmp8);
		
		tmp8 = *_tfCylHiReg;
		DbgPrint("_tfCylHiReg %x\n", tmp8);

		// check for ATAPI device signature first
		if ( ( *_tfCylLoReg == 0x14) && ( *_tfCylHiReg == 0xEB) )
		{	
			IOLog("ATAPI Device checked\n");
		
			if(    (unit == 1 )
				&& ( _devInfo[0].type == kATAPIDeviceType )  )
			{

			// OK we've met the condition for an indeterminate bus, master is atapi and we see a slave atapi
			// signature. This is legal ATA, though we are fortunate enough that most devices don't do this.

				if( ATAPISlaveExists( ) != true )
				{
					IOLog("ATAPI device is not exists\n");
					_devInfo[unit].type = kUnknownATADeviceType;
					goto AllDone;
					
				} 

			} 

			_devInfo[unit].type = kATAPIDeviceType;	
			_devInfo[unit].packetSend = kATAPIDRQFast;  // this is the safest default setting
			unitsFound++;

		} // check for ATA signature, including status RDY=1 and ERR=0
		else if ( (*_tfCylLoReg == 0x00) && (*_tfCylHiReg == 0x00) &&
				  (*_tfSCountReg == 0x01) && (*_tfSectorNReg == 0x01) &&
				  ( (*_tfAltSDevCReg & 0x51) == 0x50) )
		{
			DbgPrint("ATA device checked\n");
		
			 _devInfo[unit].type = kATADeviceType;
			 _devInfo[unit].packetSend = kATAPIUnknown;  
			unitsFound++;
			
		}
		else
		{
			DbgPrint("Unknown device\n");
			_devInfo[unit].type = kUnknownATADeviceType;
			_devInfo[unit].packetSend = kATAPIUnknown;  
		}

	}

AllDone:
	// reselect device 0
	*_tfSDHReg	= 0x00;
	// enable device interrupts
	*_tfAltSDevCReg = 0x00;
	OSSynchronizeIO();

	// enforce ATA device selection protocol
	// before issuing the next command.
	_selectedUnit = kATAInvalidDeviceID;
	
    *_tfSDHReg = 0x00;  // Initialize device selection to device 0.
	return unitsFound;
}

/*---------------------------------------------------------------------------
 *
 * Flush the outstanding commands in the command queue.
 * Implementation borrowed from MacIOATA in IOATAFamily.
 *
 ---------------------------------------------------------------------------*/

IOReturn CLASS::handleQueueFlush( void )
{
    UInt32 savedQstate = _queueState;

    DbgPrint("[CH%u] %s\n", (unsigned int)fChannelNumber, __FUNCTION__);

    _queueState = IOATAController::kQueueLocked;

    IOATABusCommand * cmdPtr = 0;

    while ((cmdPtr = dequeueFirstCommand()))
    {
        cmdPtr->setResult( kIOReturnError );
        cmdPtr->executeCallback();
    }

    _queueState = savedQstate;

    return kATANoErr;
}

#pragma mark -
#pragma mark - DMA and PRD stuff -
#pragma mark -

IOReturn CLASS::createChannelCommands( void )
{
	FEDE_RELOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
	FEDE_RELOG("FEDE Casteo a IOAABUSCommand64\n");
	IOATABusCommand64* currentCommand64 = OSDynamicCast( IOATABusCommand64, _currentCommand );
	FEDE_RELOG("FEDE Obtengo el comaqndo DMA\n ");
	IODMACommand* currentDMACmd = currentCommand64->GetDMACommand();
	IODMACommand::Segment32 elSegmento;
	IOReturn DMAStatus = 0;
	
	if ( NULL == currentDMACmd
		|| currentDMACmd->getMemoryDescriptor() == NULL)
    {
        FEDE_RELOG("FEDE DMA BUFFER NOT SET ON COMMAND\n ");
		IOLog("%s: DMA buffer not set on command\n", getName());
		return -1;
    }


	FEDE_RELOG("FEDE obtengo el descriptor de memoria\n ");
    IOMemoryDescriptor * descriptor = _currentCommand->getBuffer();
    //IOMemoryCursor::PhysicalSegment physSegment;
    UInt32 index = 0;
    UInt8  *xferDataPtr, *ptr2EndData, *next64KBlock, *starting64KBlock;
    UInt32 xferCount, count2Next64KBlock;

    if ( !descriptor )
    {
        return -1;
    }

    // This form of DMA engine can only do 1 pass.
    // It cannot execute multiple chains.

	UInt32 numSegmentos = 1;

    IOByteCount bytesRemaining = _currentCommand->getByteCount() ;
    IOByteCount xfrPosition    = _currentCommand->getPosition() ;
    UInt64 transferSize   = 0; 

    // There's a unique problem with pci-style controllers, in that each
    // dma transaction is not allowed to cross a 64K boundary. This leaves
    // us with the yucky task of picking apart any descriptor segments that
    // cross such a boundary ourselves.  
//
//    while ( _DMACursor->getPhysicalSegments(
//                           /* descriptor */  descriptor,
//                           /* position   */  xfrPosition,
//                           /* segments   */  &physSegment,
//                           /* max segs   */  1,
//                           /* max xfer   */  bytesRemaining,
//                           /* xfer size  */  &transferSize) )
 
	FEDE_RELOG("FEDE entro al ciclo\n");
	while ( bytesRemaining )
	{
		FEDE_RELOG("FEDE genero 32IOVMSegments\n");
		DMAStatus = currentDMACmd->gen32IOVMSegments( &transferSize, &elSegmento, &numSegmentos);
		if ( ( DMAStatus != kIOReturnSuccess ) || ( numSegmentos != 1 ) || ( elSegmento.fLength == 0 ) )
		{
			
			panic ( "JMicronATA::createChannelCommands [%d] status %lx segs %d phys %qx:%qx \n", __LINE__, (unsigned long)DMAStatus, numSegmentos, (unsigned long long)elSegmento.fIOVMAddr, (unsigned long long)elSegmento.fLength );
		    break;
		    
		}

        xferDataPtr = (UInt8 *) ((UInt64)elSegmento.fIOVMAddr);
        xferCount   = elSegmento.fLength;

        if ((UInt64) xferDataPtr & 0x03)
        {
            IOLog("%s: DMA buffer %p not 4 byte aligned\n",
                  getName(), xferDataPtr);
            //return kIOReturnNotAligned;        
        }

        if (xferCount & 0x03)
        {
            IOLog("%s: DMA buffer length 0x%x is odd\n",
                  getName(), (unsigned int)xferCount);
        }

        // Update bytes remaining count after this pass.
        bytesRemaining -= xferCount;
        xfrPosition += xferCount;

        // Examine the segment to see whether it crosses (a) 64k boundary(s)
        starting64KBlock = (UInt8 *)((UInt64) xferDataPtr & 0xffff0000);
        ptr2EndData  = xferDataPtr + xferCount;
        next64KBlock = starting64KBlock + 0x10000;

        // Loop until this physical segment is fully accounted for.
        // It is not possible to have a memory descriptor which crosses
        // more than one 64K boundary given the max segment size passed
        // to the memory cursor.

          while ( xferCount > 0 )
        {
            if (ptr2EndData > next64KBlock)
            {
                count2Next64KBlock = (UInt32)(next64KBlock - xferDataPtr);
                if ( index < kATAMaxDMADesc )
                {
                    setPRD( xferDataPtr, (UInt16)count2Next64KBlock,
                            &_prdTable[index], kContinue_PRD);
                    
                    xferDataPtr = next64KBlock;
                    next64KBlock += 0x10000;
                    xferCount -= count2Next64KBlock;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 1\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
            else
            {
                if (index < kATAMaxDMADesc)
                {
                    setPRD( xferDataPtr, (UInt16) xferCount,
                            &_prdTable[index],
                            (bytesRemaining == 0) ? kLast_PRD : kContinue_PRD);
                    xferCount = 0;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 2\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
        }

    } // end of segment counting loop.

    if (index == 0)
    {
        IOLog("%s: rejected command with zero PRD count (0x%llx bytes)\n",
              getName(), _currentCommand->getByteCount());
        return kATADeviceError;
    }

    // Transfer is satisfied and only need to check status on interrupt.
    _dmaState = kATADMAStatus;

    // Chain is now ready for execution.
    return kATANoErr;
}

bool CLASS::allocDMAChannel( void )
{
    _prdBuffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                                                        kIODirectionInOut | kIOMemoryPhysicallyContiguous,
                                                        sizeof(PRD) * kATAMaxDMADesc,
                                                        0xFFFF0000UL);

    if (!_prdBuffer)
    {
        IOLog("%s: PRD buffer allocation failed\n", getName());
        return false;
    }

    _prdBuffer->prepare();

    _prdTable = (PRD *)_prdBuffer->getBytesNoCopy();
    _prdTablePhysical	= _prdBuffer->getPhysicalAddress();

    _DMACursor = IONaturalMemoryCursor::withSpecification(
                 /* max segment size  */ kMaxPRDSegmentSize,
                 /* max transfer size */ kMaxATATransferSize );

    if (!_DMACursor)
    {
        freeDMAChannel();
        IOLog("%s: Memory cursor allocation failed\n", getName());
        return false;
    }

    // fill the chain with stop commands to initialize it.    
    initATADMAChains(_prdTable);

    return true;
}

bool CLASS::freeDMAChannel( void )
{
    if (_prdTable)
    {
        // make sure the engine is stopped.
        stopDMA();

        // free the descriptor table.
        _prdBuffer->complete();
        _prdBuffer->release();
        _prdBuffer = NULL;
        _prdTable = NULL;
        _prdTablePhysical = 0;
    }

    return true;
}

void CLASS::initATADMAChains( PRD * descPtr )
{
    /* Initialize the data-transfer PRD channel command descriptors. */

    for (int i = 0; i < kMaxPRDCount; i++)
    {
        descPtr->bufferPtr = 0;
        descPtr->byteCount = 1;
        descPtr->flags = OSSwapHostToLittleConstInt16( kLast_PRD );
        descPtr++;
    }
}

#pragma mark -
#pragma mark - Notification Handler - 
#pragma mark -

IOReturn CLASS::message( UInt32 type, IOService *provider,
                         void *argument )
{
	DbgPrint("JMicronATA::message type %x\n", type);
	if (provider == fChannelNub) DbgPrint("Message from provider\n");
	
    if ((provider == fChannelNub) &&
        (type == kIOMessageServiceIsTerminated))
    {
        fChannelNub->close( this );
        return kIOReturnSuccess;
    }

    return super::message( type, provider, argument );
}

bool CLASS::publishDriveProperty( UInt32       driveUnit,
                                  const char * propKey,
                                  UInt32       value )
{
    char keyString[40];
    snprintf(keyString, 40, "Drive %u %s", (unsigned int)driveUnit, propKey);
    return super::setProperty( keyString, value, 32 );
}

#pragma mark -
#pragma mark - Power Management - 
#pragma mark -

void CLASS::initForPM( IOService * provider )
{
    static IOPMPowerState powerStates[ kPowerStateCount ] =
    {
        { 1, 0,				0,             0,             0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0,				IOPMSoftSleep, IOPMSoftSleep, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, IOPMPowerOn,	IOPMPowerOn,   IOPMPowerOn,   0, 0, 0, 0, 0, 0, 0, 0 }
    };

    PMinit();

    registerPowerDriver(this, powerStates, kPowerStateCount);

    provider->joinPMtree(this);
}

IOReturn CLASS::setPowerState( unsigned long stateIndex, IOService *   whatDevice )
{
	
    if (stateIndex == kPowerStateOff)
    {
        fHardwareLostPower = true;
		saveHardwareState();
    }
    else if (fHardwareLostPower)
    {
        restoreHardwareState();
        fHardwareLostPower = false;
    }

    return IOPMAckImplied;
}

