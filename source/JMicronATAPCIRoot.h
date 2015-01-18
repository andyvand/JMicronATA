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

#ifndef _JMICRONATAPCIROOT_H_
#define _JMICRONATAPCIROOT_H_

#include <IOKit/IOLocks.h>
#include <IOKit/pci/IOPCIDevice.h>

//JMicron specific

#define JMB360_VEN_DEV_ID			0x2360197b
#define JMB361_VEN_DEV_ID			0x2361197b
#define JMB363_VEN_DEV_ID			0x2363197b
#define JMB365_VEN_DEV_ID			0x2365197b
#define JMB366_VEN_DEV_ID			0x2366197b
#define JMB368_VEN_DEV_ID			0x2368197b

#define JMB_CONTR_CTRL1				0x40
#define JMB_CONTR_CTRL2				0x44
#define JMB_SW_BIOS_REG1			0x4C
#define JMB_CONTR_CTRL5				0x80
#define JMB_PATAIO_CTRL				0x88
#define JMB_CLOCK_SELECT			0xB8
#define JMB_TIMER_COUNT				0xBC
#define JMB_SATAP1_PHY_CTRL			0xCC
#define JMB_SW_BIOS_REG2			0xDC
#define JMB_SATAP0_PHY_CTRL			0xEC

struct JMicronRegistersState
{
	UInt32 contr_ctrl1;
	UInt32 contr_ctrl2;
	UInt32 sw_bios_reg1;
	UInt32 contr_ctrl5;
	UInt32 pataio_ctrl;
	UInt32 clock_select;
	UInt32 timer_count;
	UInt32 satap1_phy_ctrl;
	UInt32 sw_bios_reg2;
	UInt32 satap0_phy_ctrl;
};

struct JMicronExtVendorInfo
{
	UInt32	VendorId[2];
	UInt32	DeviceId[2];
	UInt8	SerialNumber[16];
	UInt8	CustomerString[128];	
};

class JMicronATAPCIRoot : public IOService
{
    OSDeclareDefaultStructors( JMicronATAPCIRoot )

protected:
    IOPCIDevice				*fProvider;
    IOLock					*fPCILock;
    OSSet					*fChannels;
    OSSet					*fOpenChannels;
	UInt32					fDeviceId;
	JMicronRegistersState	fRegState;
	UInt8					fSavedPciRegs[256];
	bool					isSleaping_;
	
	bool					fExpansionRom;
	//IOMemoryDescriptor		*fExpansionRomDesc;
	IOMemoryMap				*fExpansionRomMap;
	UInt8					*fExpansionRomMem;
	JMicronExtVendorInfo	fExtVendorInfo;

    virtual OSSet *           createATAChannels(
                                      UInt32 maxChannelCount );

    virtual OSDictionary *    createNativeModeChannelInfo(
                                      UInt32 channelID );

    virtual OSDictionary *    createLegacyModeChannelInfo(
                                      UInt32 channelID );

    virtual OSDictionary *    createChannelInfo(
                                      UInt32 channelID,
                                      UInt32 commandPort,
                                      UInt32 controlPort,
                                      UInt32 interruptVector );

    virtual IORegistryEntry * getDTChannelEntry(
                                      UInt32 channelID );

    virtual UInt32            getNumberPropertyValue(
                                      const char * propKey ) const;

    virtual const char *      getStringPropertyValue(
                                      const char * propKey ) const;
									  
	virtual void			  getExtVendorInfo( 
									  JMicronExtVendorInfo *info );
									  
	virtual void			  publishExtVendorInfo( 
									  const JMicronExtVendorInfo &info );

public:
    virtual IOService *       probe( IOService * provider,
                                     SInt32 *    score );

    virtual bool              start( IOService * provider );

    virtual void              free( void );

    virtual bool              handleOpen( IOService *  client,
                                          IOOptionBits options,
                                          void *       arg );
    
    virtual void              handleClose( IOService *  client,
                                           IOOptionBits options );

    virtual bool              handleIsOpen( const IOService * client ) const;

    virtual const char *      getHardwareVendorName( void ) const;

    virtual const char *      getHardwareDeviceName( void ) const;

    virtual void              pciConfigWrite8(
                                      UInt8  offset,
                                      UInt8  data,
                                      UInt8  mask = 0xff );

    virtual void              pciConfigWrite16(
                                      UInt8  offset,
                                      UInt16 data,
                                      UInt16 mask = 0xffff );

    virtual void              pciConfigWrite32(
                                      UInt8  offset,
                                      UInt32 data,
                                      UInt32 mask = 0xffffffff );

    virtual UInt8             pciConfigRead8(  UInt8 offset );

    virtual UInt16            pciConfigRead16( UInt8 offset );

    virtual UInt32            pciConfigRead32( UInt8 offset );
	
	virtual IOPCIDevice		  *getPCIDevice ( void ) const;
	
	virtual void			  prepareHwForSleep( void );

	virtual void			  restoreHwResources( void );

	virtual void			  saveHwRegisters( UInt8 *data );
};

#endif 
