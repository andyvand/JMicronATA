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

#include <IOKit/IOLib.h>
#include <IOKit/IODeviceTreeSupport.h>
#include "JMicronATAPCIRoot.h"
#include "JMicronATAChannel.h"
#include "JMicronMisc.h"

#define CLASS JMicronATAPCIRoot
#define super IOService

OSDefineMetaClassAndStructors( JMicronATAPCIRoot, IOService )

IOService * CLASS::probe( IOService * provider, SInt32 * score )
{
    IOPCIDevice * pciDevice;

    if (super::probe( provider, score ) == 0)
        return 0;

    // Verify provider is an IOPCIDevice.
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if (pciDevice == 0)
        return 0;
	pciDevice->setIOEnable(true);
	IOSleep(10);
	
    // Fail if I/O space decoding is disabled.
    // Controller may be disabled in the BIOS.
    if ((pciDevice->configRead16(kIOPCIConfigCommand) &
         kIOPCICommandIOSpace) == 0)
    {
		DbgPrint("IO space decoding disabled\n");
        return 0;
    }

    return this;
}

static void registerClientApplier( IOService * service, void * context )
{
    if (service) service->registerService();
}

bool CLASS::start( IOService * provider )
{
	DbgPrint("JMicronATAPCIRoot start\n");

    UInt32 numChannels;

    if (super::start(provider) != true)
        return false;

    fProvider = OSDynamicCast( IOPCIDevice, provider );
    if (fProvider == 0)
        return false;

    fProvider->retain();

    // Enable bus master.

    fProvider->setBusMasterEnable( true );
	fProvider->setIOEnable(true);
	//saveHwRegisters(&fRegState);
	saveHwRegisters(fSavedPciRegs);

	fDeviceId = fProvider->configRead32(kIOPCIConfigVendorID);

    // Allocate a mutex to serialize access to PCI config space from
    // the primary and secondary ATA channels.
    fPCILock = IOLockAlloc();
    if (fPCILock == 0)
        return false;

    numChannels = getNumberPropertyValue( kATAChannelCount );
    if (numChannels == 0)
        numChannels = 2;

    fChannels = createATAChannels( numChannels );
    if (fChannels == 0)
	{
		IOLog("Couldn't create ATA channels");
        return false;
	}

    fOpenChannels = OSSet::withCapacity( fChannels->getCount() );
    if (fOpenChannels == 0)
	{
		IOLog("Couldn't open channels\n");
        return false;
	}

	// Start ATA channel client matching
    applyToClients( registerClientApplier, 0 );
	
	isSleaping_ = false;
	
	fExpansionRom = false;
	//fExpansionRomDesc = NULL;
	fExpansionRomMap = NULL;
	
	UInt32 expRomAddr = fProvider->configRead32(kIOPCIConfigExpansionROMBase);
	expRomAddr |= 0x01;
	fProvider->configWrite32(kIOPCIConfigExpansionROMBase, expRomAddr);
	
	fExpansionRomMap = fProvider->mapDeviceMemoryWithRegister(kIOPCIConfigExpansionROMBase);
	if (!fExpansionRomMap)
	{
		DbgPrint("%s: Couldn't map ROM Base memory\n", getName());
	}
	else if ((fExpansionRomMem = reinterpret_cast<UInt8 *>(fExpansionRomMap->getVirtualAddress())))
	{
		fExpansionRom = true;
	}
	else
	{
		DbgPrint("%s: Couldn't map ROM Base memory\n", getName());
	}

	/*IOSleep(5);
	expRomAddr = fProvider->configRead32(kIOPCIConfigExpansionROMBase);
	DbgPrint("Expansion ROM base address register %x\n", expRomAddr);
	
	//expRomAddr = expRomAddr >> 11;
	expRomAddr &= ~0x01;
	if (expRomAddr)
	{
		DbgPrint("ROM Address %x\n", expRomAddr);
		fExpansionRomDesc = IOMemoryDescriptor::withPhysicalAddress(static_cast<IOPhysicalAddress>(expRomAddr),
										0xFF,
										kIODirectionInOut);
									
		if (fExpansionRomDesc && fExpansionRomDesc->prepare() == kIOReturnSuccess)
		{
			fExpansionRomMap = fExpansionRomDesc->map();
			
			if (fExpansionRomMap)
			{
				fExpansionRomMem = reinterpret_cast<UInt8 *>(fExpansionRomMap->getVirtualAddress());
				if (fExpansionRomMem)
				{
					fExpansionRom = true;
				}
			}
		}
	}
	else
	{
		DbgPrint("Expansion ROM is not supported on this board\n");
	}*/
	
	if (fExpansionRom)
	{
		getExtVendorInfo(&fExtVendorInfo);
		publishExtVendorInfo(fExtVendorInfo);
	}

    return true;
}

void CLASS::free( void )
{
	DbgPrint("JMicronATAPCIRoot free\n");

	/*if (fExpansionRomDesc)
	{
		fExpansionRomDesc->complete();
		fExpansionRomDesc->release();
		fExpansionRomDesc = NULL;
	}*/
	
	if (fExpansionRomMap)
	{
		fExpansionRomMap->release();
		fExpansionRomMap = NULL;
	}

    if (fChannels)
    {
        fChannels->release();
        fChannels = 0;
    }

    if (fOpenChannels)
    {
        fOpenChannels->release();
        fOpenChannels = 0;
    }

    if (fProvider)
    {
        fProvider->release();
        fProvider = 0;
    }

    if (fPCILock)
    {
        IOLockFree( fPCILock );
        fPCILock = 0;
    }

    super::free();
}

IORegistryEntry *CLASS::getDTChannelEntry( UInt32 channelID )
{
    IORegistryEntry * entry = 0;
    const char *      location;

    OSIterator * iter = fProvider->getChildIterator( gIODTPlane );
    if (iter == 0) return 0;

    while ((entry = (IORegistryEntry *) iter->getNextObject()))
    {
        location = entry->getLocation();
        if (location && strtol(location, 0, 10) == (int)channelID)
        {
            entry->retain();
            break;
        }
    }

    iter->release();

    return entry;  // retain held on the entry
}

OSSet * CLASS::createATAChannels( UInt32 maxChannelCount )
{
    OSSet *           nubSet;
    OSDictionary *    channelInfo;
    IORegistryEntry * dtEntry;

    do {
        nubSet = OSSet::withCapacity(maxChannelCount);
        if (nubSet == 0)
            break;

        if (fProvider->open(this) != true)
            break;

        for ( UInt32 channelID = 0;
              channelID < maxChannelCount; channelID++ )
        {        
            // Create a dictionary for the channel info. Use native mode
            // settings if possible, else default to legacy mode.

            channelInfo = createNativeModeChannelInfo( channelID );
            if (channelInfo == 0)
			{
				IOLog("createNativeModeChannelInfo failed\n");
                /*channelInfo = createLegacyModeChannelInfo( channelID );
				if (channelInfo == 0)
				{
					IOLog("Couldn't create channelInfo\n");
					continue;
				}*/
			}

            // Create a nub for each ATA channel.

            JMicronATAChannel * channelNub;

            channelNub = new JMicronATAChannel;
            if ( channelNub )
            {
                dtEntry = getDTChannelEntry( channelID );

                // Invoke special init method in channel nub.

                if (channelNub->init( this, channelInfo, dtEntry ) &&
                    channelNub->attach( this ))
                {
                    nubSet->setObject( channelNub );
                }

                if ( dtEntry )
                {
                    dtEntry->release();
                }
                else
                {
                    // Platform did not create a device tree entry for
                    // this ATA channel. Do it here.

                    char channelName[5] = {'C','H','N','_','\0'};

                    channelName[3] = '0' + channelID;
                    channelNub->setName( channelName );

                    if (fProvider->inPlane(gIODTPlane))
                    {
                        channelNub->attachToParent( fProvider, gIODTPlane );
                    }
                }

                channelNub->release();
            }

            channelInfo->release();
        }

        fProvider->close( this );
    }
    while ( false );

    // Release and invalidate an empty set.

    if (nubSet && (nubSet->getCount() == 0))
    {
        nubSet->release();
        nubSet = 0;
    }

    return nubSet;
}

#define PRI_CMD_ADDR								0x1f0
#define PRI_CTR_ADDR								0x3f4
#define SEC_CMD_ADDR								0x170
#define SEC_CTR_ADDR								0x374

OSDictionary *CLASS::createNativeModeChannelInfo( UInt32 ataChannel )
{
    IOPCIDevice * pciDevice;
    UInt16        cmdPort = 0;
    UInt16        ctrPort = 0;

    pciDevice = OSDynamicCast(IOPCIDevice, fProvider);
    if (pciDevice == 0)
        return 0;

    switch ( ataChannel )
    {
        case kPrimaryChannelID:
            cmdPort = pciDevice->configRead16( kIOPCIConfigBaseAddress0 );
            ctrPort = pciDevice->configRead16( kIOPCIConfigBaseAddress1 );

            // Both address ranges must reside in I/O space.

            if (((cmdPort & 0x1) == 0) || ((ctrPort & 0x1) == 0))
            {
                cmdPort = ctrPort = 0;
                break;
            }

            cmdPort &= ~0x1;  // clear PCI I/O space indicator bit
            ctrPort &= ~0x1;

            if ((cmdPort > 0xFFF8) || (ctrPort > 0xFFF8) ||
                (cmdPort < 0x100)  || (ctrPort < 0x100)  ||
                ((cmdPort == PRI_CMD_ADDR) && (ctrPort == PRI_CTR_ADDR)))
            {
                cmdPort = ctrPort = 0;
            }
            break;

        case kSecondaryChannelID:
            cmdPort = pciDevice->configRead16( kIOPCIConfigBaseAddress2 );
            ctrPort = pciDevice->configRead16( kIOPCIConfigBaseAddress3 );

            // Both address ranges must reside in I/O space.

            if (((cmdPort & 0x1) == 0) || ((ctrPort & 0x1) == 0))
            {
                cmdPort = ctrPort = 0;
                break;
            }
			
            cmdPort &= ~0x1;  // clear PCI I/O space indicator bit
            ctrPort &= ~0x1;

            if ((cmdPort > 0xFFF8) || (ctrPort > 0xFFF8) ||
                (cmdPort < 0x100)  || (ctrPort < 0x100)  ||
                ((cmdPort == SEC_CMD_ADDR) && (ctrPort == SEC_CTR_ADDR)))
            {
                cmdPort = ctrPort = 0;
            }
            break;
    }

    if (cmdPort && ctrPort)
        return createChannelInfo( ataChannel, cmdPort, ctrPort,
                     pciDevice->configRead8(kIOPCIConfigInterruptLine) );
    else
        return 0;
}

OSDictionary * CLASS::createLegacyModeChannelInfo( UInt32 channelID )
{
    UInt16  cmdPort = 0;
    UInt16  ctrPort = 0;
    UInt8   isaIrq  = 0;

    switch ( channelID )
    {
        case kPrimaryChannelID:
            cmdPort = kPrimaryCommandPort;
            ctrPort = kPrimaryControlPort;
            isaIrq  = kPrimaryIRQ;
            break;
        
        case kSecondaryChannelID:
            cmdPort = kSecondaryCommandPort;
            ctrPort = kSecondaryControlPort;
            isaIrq  = kSecondaryIRQ;
            break;
    }

    return createChannelInfo( channelID, cmdPort, ctrPort, isaIrq );
}

OSDictionary * CLASS::createChannelInfo( UInt32 channelID,
                                         UInt32 commandPort,
                                         UInt32 controlPort,
                                         UInt32 interruptVector )
{
    OSDictionary * dict = OSDictionary::withCapacity( 4 );
    OSNumber *     num;

    if ( dict == 0 || commandPort == 0 || controlPort == 0 || 
         interruptVector == 0 || interruptVector == 0xFF )
    {
        if (dict) dict->release();
        return 0;
    }

    num = OSNumber::withNumber( channelID, 32 );
    if (num)
    {
        dict->setObject( kChannelNumberKey, num );
        num->release();
    }

    num = OSNumber::withNumber( commandPort, 32 );
    if (num)
    {
        dict->setObject( kCommandBlockAddressKey, num );
        num->release();
    }

    num = OSNumber::withNumber( controlPort, 32 );
    if (num)
    {
        dict->setObject( kControlBlockAddressKey, num );
        num->release();
    }

    num = OSNumber::withNumber( interruptVector, 32 );
    if (num)
    {
        dict->setObject( kInterruptVectorKey, num );
        num->release();
    }

    return dict;
}

bool CLASS::handleOpen( IOService *  client,
                        IOOptionBits options,
                        void *       arg )
{
    bool ret = true;

    // Reject open request from unknown clients, or if the client
    // already holds an open.
    if ((fChannels->containsObject(client) == false) ||
        (fOpenChannels->containsObject(client) == true))
        return false;

    // First client open will trigger an open to our provider.
    if (fOpenChannels->getCount() == 0)
        ret = fProvider->open(this);

    if (ret == true)
    {
        fOpenChannels->setObject(client);

        // Return the PCI device to the client
        if ( arg ) *((IOService **) arg) = fProvider;
    }

    return ret;
}

//---------------------------------------------------------------------------
//
// Handle a close request from a client.
//

void CLASS::handleClose( IOService *  client,
                         IOOptionBits options )
{
    // Reject close request from clients that do not hold an open.
    if (fOpenChannels->containsObject(client) == false) return;

    fOpenChannels->removeObject(client);

    // Last client close will trigger a close to our provider.
    if (fOpenChannels->getCount() == 0)
        fProvider->close(this);
}

//---------------------------------------------------------------------------
//
// Report if the specified client (or any client) has an open on us.
//

bool CLASS::handleIsOpen( const IOService * client ) const
{
    if (client) return fOpenChannels->containsObject(client);
    else return (fOpenChannels->getCount() != 0);
}

//---------------------------------------------------------------------------

UInt32 CLASS::getNumberPropertyValue( const char * propKey ) const
{
    OSNumber * num = OSDynamicCast(OSNumber, getProperty(propKey));

    if (num) return num->unsigned32BitValue();
    else return 0;
}

const char * CLASS::getStringPropertyValue( const char * propKey ) const
{
    OSString * str = OSDynamicCast(OSString, getProperty(propKey));

    if (str) return str->getCStringNoCopy();
    else return "";
}

void CLASS::getExtVendorInfo( JMicronExtVendorInfo *info )
{
	//const UInt32 
	
	bzero(info, sizeof(JMicronExtVendorInfo));
	
	info->VendorId[0] = *reinterpret_cast<UInt32 *>(fExpansionRomMem + 0x40 + 0x04);
	info->VendorId[1] = *reinterpret_cast<UInt32 *>(fExpansionRomMem + 0x40 + 0x08);
	
	info->DeviceId[0] = *reinterpret_cast<UInt32 *>(fExpansionRomMem + 0x40 + 0x10);
	info->DeviceId[1] = *reinterpret_cast<UInt32 *>(fExpansionRomMem + 0x40 + 0x14);
	
	for (UInt8 i = 0; i < 16; i++)
	{
		info->SerialNumber[i] = fExpansionRomMem[0x40 + 0x20 + i]; 
	}
	
	
	for (UInt8 i = 0; i < 128; i++)
	{
		info->CustomerString[i] = fExpansionRomMem[0x40 + 0x30 + i]; 
	}
	
	// Print out
	DbgPrint("ROM Signature byte 1 %x\n", fExpansionRomMem[0]);
	DbgPrint("ROM Signature byte 2 %x\n", fExpansionRomMem[1]);
	
	IOLog("Vendor ID %x %x\n", info->VendorId[0], info->VendorId[1]);
	IOLog("Device ID %x %x\n", info->DeviceId[0], info->DeviceId[1]);
	
	for (UInt8 i = 0; i < 16; i++)
	{
		IOLog("%x", info->SerialNumber[i]);
	}
	IOLog("\n");
	
	for (UInt8 i = 0; i < 128; i++)
	{
		IOLog("%c", info->CustomerString[i]);
	}
	IOLog("\n");
}

void CLASS::publishExtVendorInfo( const JMicronExtVendorInfo &info )
{

}

const char * CLASS::getHardwareVendorName( void ) const
{
    return getStringPropertyValue( kRootHardwareVendorNameKey );
}

const char * CLASS::getHardwareDeviceName( void ) const
{
    return getStringPropertyValue( kRootHardwareDeviceNameKey );
}

void CLASS::pciConfigWrite8( UInt8 offset, UInt8 data, UInt8 mask )
{
    UInt8 u8;

    IOLockLock( fPCILock );

    u8 = fProvider->configRead8( offset );
    u8 &= ~mask;
    u8 |= (mask & data);
    fProvider->configWrite8( offset, u8 );

    IOLockUnlock( fPCILock );
}

void CLASS::pciConfigWrite16( UInt8 offset, UInt16 data, UInt16 mask )
{
    UInt16 u16;

    IOLockLock( fPCILock );

    u16 = fProvider->configRead16( offset );
    u16 &= ~mask;
    u16 |= (mask & data);
    fProvider->configWrite16( offset, u16 );

    IOLockUnlock( fPCILock );
}

void CLASS::pciConfigWrite32( UInt8 offset, UInt32 data, UInt32 mask )
{
    UInt32 u32;

    IOLockLock( fPCILock );

    u32 = fProvider->configRead32( offset );
    u32 &= ~mask;
    u32 |= (mask & data);
    fProvider->configWrite32( offset, u32 );

    IOLockUnlock( fPCILock );
}

UInt8 CLASS::pciConfigRead8( UInt8 offset )
{
    return fProvider->configRead8( offset );
}

UInt16 CLASS::pciConfigRead16( UInt8 offset )
{
    return fProvider->configRead16( offset );
}

UInt32 CLASS::pciConfigRead32( UInt8 offset )
{
    return fProvider->configRead32( offset );
}

IOPCIDevice *CLASS::getPCIDevice ( void ) const
{
	return fProvider;
}

void CLASS::prepareHwForSleep( void )
{
	DbgPrint("JMicronATAPCIRoot prepareHwForSleep\n");
	
	isSleaping_ = true;
}

void CLASS::restoreHwResources( void )
{
	if (isSleaping_ == false) return ;
	isSleaping_ = false;

	DbgPrint("JMicronATAPCIRoot restoreHwResources\n");

	/*for(UInt32 i = 0; i <= 0xFF; i++)
	{
		fProvider->configWrite8(i, fSavedPciRegs[i]);
	}*/
}

void CLASS::saveHwRegisters( UInt8 *data )
{
	DbgPrint("JMicronATAPCIRoot saveHwRegisters\n");

	for(UInt32 i = 0; i <= 0xFF; i++)
	{
		data[i] = fProvider->configRead8(i);
	}
}

/*
void CLASS::restoreHwResources( void )
	fProvider->setIOEnable(true);

	if (fDeviceId != JMB368_VEN_DEV_ID)
	{
		fProvider->configWrite32(JMB_CONTR_CTRL1, fRegState.contr_ctrl1);
		fProvider->configWrite32(JMB_CONTR_CTRL2, fRegState.contr_ctrl2);
		fProvider->configWrite32(JMB_SW_BIOS_REG1, fRegState.sw_bios_reg1);
		fProvider->configWrite32(JMB_CONTR_CTRL5, fRegState.contr_ctrl5);
		fProvider->configWrite32(JMB_PATAIO_CTRL, fRegState.pataio_ctrl);
		fProvider->configWrite32(JMB_CLOCK_SELECT, fRegState.clock_select);
		fProvider->configWrite32(JMB_TIMER_COUNT, fRegState.timer_count);
		fProvider->configWrite32(JMB_SATAP1_PHY_CTRL, fRegState.satap1_phy_ctrl);
		fProvider->configWrite32(JMB_SW_BIOS_REG2, fRegState.sw_bios_reg2);
		fProvider->configWrite32(JMB_SATAP0_PHY_CTRL, fRegState.satap0_phy_ctrl);
	}
}

void CLASS::saveHwRegisters( JMicronRegistersState *regState )
	if (fDeviceId != JMB368_VEN_DEV_ID)
	{
		regState->contr_ctrl1 = fProvider->configRead32(JMB_CONTR_CTRL1);
		regState->contr_ctrl2 = fProvider->configRead32(JMB_CONTR_CTRL2);
		regState->sw_bios_reg1 = fProvider->configRead32(JMB_SW_BIOS_REG1);
		regState->contr_ctrl5 = fProvider->configRead32(JMB_CONTR_CTRL5);
		regState->pataio_ctrl = fProvider->configRead32(JMB_PATAIO_CTRL);
		regState->clock_select = fProvider->configRead32(JMB_CLOCK_SELECT);
		regState->timer_count = fProvider->configRead32(JMB_TIMER_COUNT);
		regState->satap1_phy_ctrl = fProvider->configRead32(JMB_SATAP1_PHY_CTRL);
		regState->sw_bios_reg2 = fProvider->configRead32(JMB_SW_BIOS_REG2);
		regState->satap0_phy_ctrl = fProvider->configRead32(JMB_SATAP0_PHY_CTRL);
	}
}
*/

