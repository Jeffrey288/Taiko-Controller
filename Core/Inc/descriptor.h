#ifndef __DESCRIPTORS_H
#define __DESCRIPTORS_H

#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

//typedef struct {
//
//	uint8_t* CfgFSDesc;
//	uint8_t* CfgHSDesc;
//	uint8_t* OtherSpeedCfgDesc;
//	uint16_t size_CfgDesc;
//
//	uint8_t* Desc;
//	uint16_t size_Desc;
//
//} HID_ClassConfigStruct;

//
//typedef struct {
//
//	uint8_t* ReportDesc_FS;
//	uint16_t size_ReportDesc;
//
//} HID_ItfConfigStruct;
//

// Keyboard
#define Keyboard_size_CfgFSDesc 	(34)
#define Keyboard_size_Desc 			(9)
#define Keyboard_size_ReportDesc 	(63)

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t Keyboard_CfgDesc[Keyboard_size_CfgFSDesc] __ALIGN_END = {
	0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	Keyboard_size_CfgFSDesc,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
	the configuration*/
	0b11100000,    /*bmAttributes */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

	/************** Descriptor of CUSTOM HID interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x01,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: CUSTOM_HID*/
	0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/

	/******************** Descriptor of CUSTOM_HID *************************/
	/* 18 */
	0x09,         /*bLength: CUSTOM_HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
	0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	Keyboard_size_ReportDesc,/*wItemLength: Total length of Report descriptor*/
	0x00,

	/******************** Descriptor of Custom HID endpoints ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

	CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
	0x00,
	0x01,   /*bInterval: Polling Interval */
	/* 34 */

};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t Keyboard_Desc[Keyboard_size_Desc]  __ALIGN_END  =
{
	0x09,         /*bLength: HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x11,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	Keyboard_size_ReportDesc,/*wItemLength: Total length of Report descriptor*/
	0x00,
};

__ALIGN_BEGIN static uint8_t Keyboard_ReportDesc_FS[Keyboard_size_ReportDesc] __ALIGN_END = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

const HID_ClassConfigStruct Keyboard_ClassConfig = {
	Keyboard_CfgDesc,
	Keyboard_CfgDesc,
	Keyboard_CfgDesc,
	Keyboard_size_CfgFSDesc,

	Keyboard_Desc,
	Keyboard_size_Desc,
};

const HID_ItfConfigStruct Keyboard_ItfConfig = {
	Keyboard_ReportDesc_FS,
	Keyboard_size_ReportDesc,
};

// Nintendo Switch
// Reference: https://github.com/soonuse/stm32_joystick_for_nintendo_switch
#define Switch_size_CfgFSDesc 		(41)
#define Switch_size_Desc 			(9)
#define Switch_size_ReportDesc 		(86)

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t Switch_CfgDesc[Switch_size_CfgFSDesc] __ALIGN_END = {

	0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	Switch_size_CfgFSDesc,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
	the configuration*/
	0b11100000,    /*bmAttributes */
	0xFA,         /*MaxPower 500 mA: this current is used for detecting Vbus*/

	/************** Descriptor of CUSTOM HID interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x02,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: CUSTOM_HID*/
	0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/

	/******************** Descriptor of CUSTOM_HID *************************/
	/* 18 */
	0x09,         /*bLength: CUSTOM_HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
	0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	Switch_size_ReportDesc,/*wItemLength: Total length of Report descriptor*/
	0x00,

	/******************** Descriptor of Custom HID endpoints ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

	CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
	0x00,
	0x01,   /*bInterval: Polling Interval */
	/* 34 */

	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

	CUSTOM_HID_EPOUT_ADDR,     /*bEndpointAddress: Endpoint Address (OUT)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	CUSTOM_HID_EPOUT_SIZE, /*wMaxPacketSize: 2 Byte max */
	0x00,
	0x05,   	/*bInterval: Polling Interval */
	/* 41 */

};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t Switch_Desc[Switch_size_Desc]  __ALIGN_END  =
{
	0x09,         /*bLength: HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x11,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	Switch_size_ReportDesc,/*wItemLength: Total length of Report descriptor*/
	0x00,
};

__ALIGN_BEGIN static uint8_t Switch_ReportDesc_FS[Switch_size_ReportDesc] __ALIGN_END = {
	0x05, 0x01, 0x09, 0x05,   0xa1, 0x01, 0x15, 0x00,   0x25, 0x01, 0x35, 0x00,   0x45, 0x01, 0x75, 0x01,
	0x95, 0x10, 0x05, 0x09,   0x19, 0x01, 0x29, 0x10,   0x81, 0x02, 0x05, 0x01,   0x25, 0x07, 0x46, 0x3b,
	0x01, 0x75, 0x04, 0x95,   0x01, 0x65, 0x14, 0x09,   0x39, 0x81, 0x42, 0x65,   0x00, 0x95, 0x01, 0x81,
	0x01, 0x26, 0xff, 0x00,   0x46, 0xff, 0x00, 0x09,   0x30, 0x09, 0x31, 0x09,   0x32, 0x09, 0x35, 0x75,
	0x08, 0x95, 0x04, 0x81,   0x02, 0x06, 0x00, 0xff,   0x09, 0x20, 0x95, 0x01,   0x81, 0x02, 0x0a, 0x21,
	0x26, 0x95, 0x08, 0x91,   0x02, 0xc0,
};

const HID_ClassConfigStruct Switch_ClassConfig = {
	Switch_CfgDesc,
	Switch_CfgDesc,
	Switch_CfgDesc,
	Switch_size_CfgFSDesc,

	Switch_Desc,
	Switch_size_Desc,
};

const HID_ItfConfigStruct Switch_ItfConfig = {
	Switch_ReportDesc_FS,
	Switch_size_ReportDesc,
};

/**
 *
productName: Taiko Controller
vendorId:    0x0F0D (3853) HORI CO., LTD.
productId:   0x00F0 (240)
opened:      true
collections[0]
  Usage: 0001:0005 (Generic Desktop > Gamepad)
  Input reports: 0x00
Input report 0x00
  14 values * 1 bit (bits 0 to 13)
    Data,Var,Abs
    Usages: 0009:0001 (Button Button 1) to 0009:000E (Button Button 14)
    Logical bounds: 0 to 0
  2 bits (bits 14 to 15)
    Cnst,Ary,Abs
    Logical bounds: 0 to 0
  4 bits (bits 16 to 19)
    Data,Var,Abs,Null
    Usage: 0001:0039 (Generic Desktop > Hat Switch)
    Logical bounds: 0 to 7
    Physical bounds: 0 to 315
    Units: deg
  4 bits (bits 20 to 23)
    Cnst,Ary,Abs
    Logical bounds: 0 to 0
  8 bits (bits 24 to 31)
    Data,Var,Abs
    Usage: 0001:0030 (Generic Desktop > X)
    Logical bounds: 0 to 255
    Physical bounds: 0 to 255
  8 bits (bits 32 to 39)
    Data,Var,Abs
    Usage: 0001:0031 (Generic Desktop > Y)
    Logical bounds: 0 to 255
    Physical bounds: 0 to 255
  8 bits (bits 40 to 47)
    Data,Var,Abs
    Usage: 0001:0032 (Generic Desktop > Z)
    Logical bounds: 0 to 255
    Physical bounds: 0 to 255
  8 bits (bits 48 to 55)
    Data,Var,Abs
    Usage: 0001:0035 (Generic Desktop > Rz)
    Logical bounds: 0 to 255
    Physical bounds: 0 to 255
  8 bits (bits 56 to 63)
    Cnst,Ary,Abs
    Logical bounds: 0 to 0
 *
 */


#endif
