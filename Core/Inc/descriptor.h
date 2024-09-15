#ifndef __USB_CUSTOMHID_H
#define __USB_CUSTOMHID_H

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

const uint16_t Keyboard_size_CfgFSDesc = 34;
const uint16_t Keyboard_size_Desc = 9;
const uint16_t Keyboard_size_ReportDesc = 63;

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t Keyboard_CfgDesc[Keyboard_size_CfgFSDesc] __ALIGN_END = {
	0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	USB_CUSTOM_HID_CONFIG_DESC_SIZ,
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


#endif
