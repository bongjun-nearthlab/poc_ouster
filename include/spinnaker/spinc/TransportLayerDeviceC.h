//=============================================================================
// Copyright (c) 2001-2021 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/* Auto-generated file. Do not modify. */

#include "QuickSpinC.h"

#ifndef FLIR_SPINNAKERC_TLDEVICE_H
#define FLIR_SPINNAKERC_TLDEVICE_H

/**	
* @defgroup TLDeviceC_h TLDevice Structures
* @ingroup CQuickSpin
*/
/*@{*/

typedef struct _quickSpinTLDevice
{
	quickSpinStringNode DeviceID;
	quickSpinStringNode DeviceSerialNumber;
	quickSpinStringNode DeviceUserID;
	quickSpinStringNode DeviceVendorName;
	quickSpinStringNode DeviceModelName;
	quickSpinStringNode DeviceVersion;
	quickSpinIntegerNode DeviceBootloaderVersion;
	quickSpinEnumerationNode DeviceType;
	quickSpinStringNode DeviceDisplayName;
	quickSpinEnumerationNode DeviceAccessStatus;
	quickSpinIntegerNode DeviceLinkSpeed;
	quickSpinStringNode DeviceDriverVersion;
	quickSpinBooleanNode DeviceIsUpdater;
	quickSpinEnumerationNode GenICamXMLLocation;
	quickSpinStringNode GenICamXMLPath;
	quickSpinEnumerationNode GUIXMLLocation;
	quickSpinStringNode GUIXMLPath;
	quickSpinEnumerationNode GevCCP;
	quickSpinIntegerNode GevDeviceMACAddress;
	quickSpinIntegerNode GevDeviceIPAddress;
	quickSpinIntegerNode GevDeviceSubnetMask;
	quickSpinIntegerNode GevDeviceGateway;
	quickSpinIntegerNode GevVersionMajor;
	quickSpinIntegerNode GevVersionMinor;
	quickSpinBooleanNode GevDeviceModeIsBigEndian;
	quickSpinIntegerNode GevDeviceReadAndWriteTimeout;
	quickSpinIntegerNode GevDeviceMaximumRetryCount;
	quickSpinIntegerNode GevDevicePort;
	quickSpinCommandNode GevDeviceDiscoverMaximumPacketSize;
	quickSpinIntegerNode GevDeviceMaximumPacketSize;
	quickSpinBooleanNode GevDeviceIsWrongSubnet;
	quickSpinCommandNode GevDeviceAutoForceIP;
	quickSpinCommandNode GevDeviceForceIP;
	quickSpinIntegerNode GevDeviceForceIPAddress;
	quickSpinIntegerNode GevDeviceForceSubnetMask;
	quickSpinIntegerNode GevDeviceForceGateway;
	quickSpinBooleanNode DeviceMulticastMonitorMode;
	quickSpinEnumerationNode DeviceEndianessMechanism;
	quickSpinStringNode DeviceInstanceId;
	quickSpinStringNode DeviceLocation;
	quickSpinEnumerationNode DeviceCurrentSpeed;
	quickSpinBooleanNode DeviceU3VProtocol;
	quickSpinStringNode DevicePortId;
} quickSpinTLDevice;

/*@}*/

#endif // FLIR_SPINNAKERC_TLDEVICE_H