/**********************************************************************************************************************
*                                        (c) COPYRIGHT by MEGABRIDGE LTD.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
#
# NAME
# smp.h
#
# DESCRIPTION
# This module contains Shelf Management Package calls export defintions
#
# REFERENCEs
#  * http://www.kernel.org/doc/Documentation/CodingStyle
#  * HLD - Shelf Management Package - Ver. 3.2 ( or later ) document
#  * Cam-Proj    - Vission-Standard Platform HDL document
#  * CP_Ctrl_HSI - Interface Board Hardware/Software Interface document
#  * POES_HSI    - Power over Ethernet board Hardware/Software Interface document
#  * Small Form-factor Pluggable (SFP) Transceiver MultiSource Agreement (MSA)
#       http://www.schelto.com/SFP/SFP%20MSA.pdf
#  * Power over Ethernet IEEE Std 802.3af
#       www.lan-power.com/pdf/802.3af-2003.pdf
#  * Power over Ethernet MIB
#       http://tools.ietf.org/html/rfc3621
#  * Definitions of Managed Objects for IEEE 802.3 Medium Attachment Units (MAUs)
#       http://tools.ietf.org/html/rfc2668
#
# NOTEs
#
# The package provides the application with API which significantly
# simplifies access to the I2C devices, transactions over MDIO, control
# of the cooling subsystem, communication with the expansion boards and
# direct management of expansion boards carrying a CPU.
#
***********************************************************************************************************************
#
# VERSION CONTROL
#
#   $Workfile: $
#
#   $Revision: $	
# 
#   $Log: $
#   $ 
#
#   $NoKeywords$
**********************************************************************************************************************/



#ifndef _SMP_H
#define _SMP_H


/* ##### General Definitions ##### */
/* ##### */


/* ##### Types/Constants/Externals ##### */
/* ##### */


/*********************************************************************************************
	PUBLIC SECTION: BEGIN
*********************************************************************************************/

/* ##### Constant Definitions ##### */

/* Error codes */
#define SMP_STATUS_OK                1         /* the operation is completed successfully */
#define SMP_STATUS_ERROR            -1         /* an error has occurred */
#define SMP_STATUS_INV_ARG          -2         /* invalid argument */
#define SMP_STATUS_RESET            -3         /* the operation was interrupted by card reset */
#define SMP_STATUS_TIMEOUT          -4         /* semaphore timeout has occurred */
#define SMP_STATUS_NO_CRD_RESPONSE  -5         /* card does not response */
#define SMP_STATUS_NO_DEV_RESPONSE  -6         /* card hardware does not response */



/* Timeout constants*/
#define SMP_WAITFOREVER             -1

#define SMP_NO_WAIT                  0

/*
 The package communicates with the interface board and other board carrying a CPU 
 via RS232 communication channel. The package sends commands to the boards and 
 expects to get a reply for every sent command.

 The API synchronizes the access of the application to the devices.
 The API blocks concurrent calls to the same device until the current
 transaction to the device completes.
*/

/* Wait for synchronization timeout in ms. If expiries, 
   the SMP_STATUS_TIMEOUT error code will be returned by API
*/
#define SMP_DEF_SYNC_TIMEOUT         100

/* 
   Wait for card reply timeout in ms. If expiries,                           
   the SMP_STATUS_NO_CRD_RESPONSE error code will be returned by API
*/ 
#define SMP_DEF_RESP_TIMEOUT         50         


/* Wait for peripheral device timeout in ms using by Firmware. If expiries, 
   the SMP_STATUS_NO_DEV_RESPONSE error code will be returned by API
*/
#define SMP_DEF_I2C_TIMEOUT          10



/* ##### Macro Definitions ##### */
/* ##### */
#define SMP_PORT_BIT(port)  ( 0x00000001 << (port) )

/* ##### Type Definitions ##### */

/* Expansion boards slot enumeration */ 
enum smp_slot {
	SMP_SLOT_1,
	SMP_SLOT_2,
	SMP_SLOT_3
};

/* 48V ON/OFF action enumeration */ 
enum smp_48v_action {
	SMP_48V_OFF,
	SMP_48V_ON
};

/* FANs enumeration */ 
enum smp_fans {
	SMP_FAN_1,
	SMP_FAN_2
};

/* Boolean enumeration, for boolean flags */ 
enum smp_bool {
	SMP_FALSE,
	SMP_TRUE
};

/* Port status enumeration */ 
enum smp_port_status {
	SMP_PORT_DOWN,			   /* The port is disabled or there is a fault 
								  that prevents it from going to the up*/

	SMP_PORT_UP,			   /* Port is UP */

	SMP_PORT_NOT_PRESENT,	   /* the interface has  missing components */
};


/* Ethernet Port speed enumeration */ 
enum smp_eth_speed {
	SMP_10B_HD,			/*10 Mbps half-duplex*/
	SMP_10B_FD,			/*10 Mbps full-duplex*/
	SMP_100B_HD,		/*100 Mbps half-duplex*/ 
	SMP_100B_FD,		/*100 Mbps full-duplex*/ 
	SMP_1000B_HD,		/*1000 Mbps half-duplex*/ 
	SMP_1000B_FD,		/*1000 Mbps full-duplex*/ 
};


/* Ethernet Port auto-negotiation status enumeration */ 
enum smp_autoneg_status {
	SMP_AUTONEG_CONFIGURING,   /* auto-negotiation is in process */
	SMP_AUTONEG_COMPLETE,	   /* auto-negotiation is complete */
	SMP_AUTONEG_DISABLED,	   /* auto-negotiation mechanism is disabled */
	SMP_AUTONEG_FAULTY,		   /* auto-negotiation is faulty */
};

/* System Ports enumeration */ 
enum smp_port {
	SMP_IFC_HOST_PORT,		   /* interface board host GE interface */
	SMP_IFC_EXP1_PORT,		   /* interface board expansion slot#1 GE interface */
	SMP_IFC_EXP2_PORT,		   /* interface board expansion slot#2 GE interface */
	SMP_IFC_EXP3_PORT,		   /* interface board expansion slot#3 GE interface */
	SMP_IFC_SFP1_PORT,		   /* interface board SFP#1 GE interface */
	SMP_IFC_SFP2_PORT,		   /* interface board SFP#2 GE interface */

	SMP_EXP1_BCKP_PORT,		   /* expansion board#1 backplane GE interface */            
	SMP_EXP1_FET1_PORT,		   /* expansion board#1 fast-ethernet interface#1 */
	SMP_EXP1_FET2_PORT,		   /* expansion board#1 fast-ethernet interface#2 */
	SMP_EXP1_FET3_PORT,		   /* expansion board#1 fast-ethernet interface#3 */
	SMP_EXP1_FET4_PORT,		   /* expansion board#1 fast-ethernet interface#4 */
	SMP_EXP1_FET5_PORT,		   /* expansion board#1 fast-ethernet interface#5 */
	SMP_EXP1_FET6_PORT,		   /* expansion board#1 fast-ethernet interface#6 */
	SMP_EXP1_FET7_PORT,		   /* expansion board#1 fast-ethernet interface#7 */
	SMP_EXP1_FET8_PORT,		   /* expansion board#1 fast-ethernet interface#8 */
	SMP_EXP1_SFP1_PORT,		   /* expansion board#1 SFP#1 GE interface */  
	SMP_EXP1_SFP2_PORT,		   /* expansion board#1 SFP#2 GE interface */  


	SMP_EXP2_BCKP_PORT,		   /* expansion board#2 backplane GE interface */    
	SMP_EXP2_FET1_PORT,		   /* expansion board#2 fast-ethernet interface#1 */ 
	SMP_EXP2_FET2_PORT,		   /* expansion board#2 fast-ethernet interface#2 */ 
	SMP_EXP2_FET3_PORT,		   /* expansion board#2 fast-ethernet interface#3 */ 
	SMP_EXP2_FET4_PORT,		   /* expansion board#2 fast-ethernet interface#4 */ 
	SMP_EXP2_FET5_PORT,		   /* expansion board#2 fast-ethernet interface#5 */ 
	SMP_EXP2_FET6_PORT,		   /* expansion board#2 fast-ethernet interface#6 */ 
	SMP_EXP2_FET7_PORT,		   /* expansion board#2 fast-ethernet interface#7 */ 
	SMP_EXP2_FET8_PORT,		   /* expansion board#2 fast-ethernet interface#8 */ 
	SMP_EXP2_SFP1_PORT,		   /* expansion board#2 SFP#1 GE interface */        
	SMP_EXP2_SFP2_PORT,		   /* expansion board#2 SFP#2 GE interface */        


	SMP_EXP3_BCKP_PORT,		   /* expansion board#3 backplane GE interface */    
	SMP_EXP3_FET1_PORT,		   /* expansion board#3 fast-ethernet interface#1 */ 
	SMP_EXP3_FET2_PORT,		   /* expansion board#3 fast-ethernet interface#2 */ 
	SMP_EXP3_FET3_PORT,		   /* expansion board#3 fast-ethernet interface#3 */ 
	SMP_EXP3_FET4_PORT,		   /* expansion board#3 fast-ethernet interface#4 */ 
	SMP_EXP3_FET5_PORT,		   /* expansion board#3 fast-ethernet interface#5 */ 
	SMP_EXP3_FET6_PORT,		   /* expansion board#3 fast-ethernet interface#6 */ 
	SMP_EXP3_FET7_PORT,		   /* expansion board#3 fast-ethernet interface#7 */ 
	SMP_EXP3_FET8_PORT,		   /* expansion board#3 fast-ethernet interface#8 */ 
	SMP_EXP3_SFP1_PORT,		   /* expansion board#3 SFP#1 GE interface */        
	SMP_EXP3_SFP2_PORT,		   /* expansion board#3 SFP#2 GE interface */        
};

/* Auto-negotiation Port capabilities flags */
#define	SMP_ANC_10BT       0x0001   /* 10BASE-T    half duplex mode */
#define	SMP_ANC_10BTFD     0x0002   /* 10BASE-T    full duplex mode */
#define	SMP_ANC_100BTX     0x0004   /* 100BASE-TX  half duplex mode */
#define	SMP_ANC_100BTXFD   0x0008   /* 100BASE-TX  full duplex mode */
#define	SMP_ANC_1000BT     0x0010   /* 1000BASE-T  half duplex mode */
#define	SMP_ANC_1000BTFD   0x0020   /* 1000BASE-T  full duplex mode */
#define	SMP_ANC_1000BX     0x0040   /* 1000BASE-X  half duplex mode */
#define	SMP_ANC_1000BXFD   0x0080   /* 1000BASE-X  full duplex mode */

/* Jack type enumeration */ 
enum smp_jack_type {
	SMP_RJ45,
	SMP_RJ45S,
	SMP_SC_FIBER,
	SMP_LC_FIBER,
};

/* Power over Ethernet port status enumeration*/
enum smp_port_poe_status {
	SMP_POE_DISABLED,		   /* Power delivering is disabled */
	SMP_POE_SIG_IN_PROC,	   /* Signature in process */
	SMP_POE_CLASS_IN_PROC,	   /* Classification in process */
	SMP_POE_DELIV_POWER,	   /* Power is delivering */
	SMP_POE_FAULTY			   /* port is faulty */
};

/* Power over Ethernet port Classification status enumeration */
enum smp_poe_class_status {
	SMP_POE_CLS_CLASS1,
	SMP_POE_CLS_UNKNOWN,
	SMP_POE_CLS_OVER_CURRENT
};

/* Power over Ethernet port Signature status enumeration */
enum smp_poe_sig_status {
	SMP_POE_SIG_IS_VALID,
	SMP_POE_SIG_SHORT_CIRCUIT,
	SMP_POE_SIG_HIGH_PD_INPUT_CAPACITANCE,
	SMP_POE_SIG_LOW_RESISTANCE,
	SMP_POE_SIG_HIGH_RESISTANCE,
	SMP_POE_SIG_OPEN_CIRCUIT,
	SMP_POE_SIG_HIGH_PORT_VOLT_OFFSET,
	SMP_POE_SIG_UNKNOWN,
};

/* Expansion board access interface type enumeration*/
enum smp_exp_ifc_type {
	SMP_EXP_I2C,		/* GPIO lines are configerd as I2C */
	SMP_EXP_SPI,		/* GPIO lines are configerd as SPI */
	SMP_EXP_MDIO,		/* GPIO lines are configerd as MDIO */
	SMP_EXP_GPIO,		/* GPIO lines are configerd as GPIO */
};


/* Peripherial devices enumeration */
enum smp_peripherals {
	SMP_FTV,		 /* Fan, temperature and voltage controller */
	SMP_SFP_1,		  /* Interface board SFP#1 */
	SMP_SFP_2,		  /* Interface board SFP#2 */

	SMP_SFP_3,		  /* Power over Ethernet board#1 SFP#1 */
	SMP_SFP_4,		  /* Power over Ethernet board#1 SFP#2 */

	SMP_SFP_5,		  /* Power over Ethernet board#2 SFP#1 */
	SMP_SFP_6,		  /* Power over Ethernet board#2 SFP#2 */

	SMP_SFP_7,		  /* Power over Ethernet board#3 SFP#1 */
	SMP_SFP_8,		  /* Power over Ethernet board#3 SFP#2 */

	SMP_PSE_1,		  /* Power over Ethernet board#1 PSE#1 device */
	SMP_PSE_2,		  /* Power over Ethernet board#1 PSE#2 device */

	SMP_PSE_3,		  /* Power over Ethernet board#2 PSE#1 device */
	SMP_PSE_4,		  /* Power over Ethernet board#2 PSE#2 device */

	SMP_PSE_5,		  /* Power over Ethernet board#3 PSE#1 device */
	SMP_PSE_6,		  /* Power over Ethernet board#3 PSE#2 device */

	/* Flexible hardware interface which can be 
	   configured as I2C, SPI, MDIO or 
	   four input/output GPIO lines */

	SMP_EXP1_FLX,		 /* Expansion board#1 flexible interface */
	SMP_EXP2_FLX,		 /* Expansion board#2 flexible interface */
	SMP_EXP3_FLX,		 /* Expansion board#3 flexible interface */

	SMP_MDIO_1,			 /* Chassis Ethernet switch */
	SMP_MDIO_2,			 /* Power over Ethernet board#1 Ethernet switch */
	SMP_MDIO_3,			 /* Power over Ethernet board#2 Ethernet switch */
	SMP_MDIO_4,			 /* Power over Ethernet board#3 Ethernet switch */

};


/* GPIO lines bitmasks */
#define SMP_GPIO_1  0x01
#define SMP_GPIO_2  0x02
#define SMP_GPIO_3  0x04
#define SMP_GPIO_4  0x08

/* GPIO mode enumeration */
enum smp_gpio_mode {
	SMP_GPIO_DISABLED,
	SMP_GPIO_INPUT,
	SMP_GPIO_OUT,
};


/* Serial Port (RS232) mapping structure */
struct smp_serial_map {
	int ifc_board_fd;	  /* Specifies the file descriptor, for 
							 example result of call to open("/dev/ttyUSB0"), 
							 which shall be used for sending commands to 
							 the interface module*/

	int exp_board1_fd;	  /* Specifies the file descriptor, for 
							 example result of call to open("/dev/ttyUSB1"), 
							 which shall be used for sending commands to 
							 the expansion module #1*/

	int exp_board2_fd;	  /* Specifies the file descriptor, for 
							 example result of call to open("/dev/ttyUSB2"), 
							 which shall be used for sending commands to 
							 the expansion module #1*/

	int exp_board3_fd;	  /* Specifies the file descriptor, for 
							 example result of call to open("/dev/ttyUSB3"), 
							 which shall be used for sending commands to 
							 the expansion module #1*/
};

/* Chassis status structure */
struct smp_chassis_status {
	unsigned int  hw_revision;				  /* Represents VID field of EMC6D102 controller, installed on interface module */
	unsigned int  fw_revision;				  /* Interface Board firmware version */

	unsigned int fan1_speed;				  /* Speed in RPM of the FAN#1 */
	unsigned int fan2_speed;				  /* Speed in RPM of the FAN#2 */

	unsigned int left_sensor_temperature;	  /* Left sensor temperature reading in Celsius in the range -128..128 
	                                             unless temperature offset is not zero. */

	unsigned int center_sensor_temperature;	  /* Center sensor temperature reading in Celsius in the range -128..128
	                                             unless temperature offset is not zero. */

	unsigned int right_sensor_temperature;	  /* Right sensor temperature reading in Celsius in the range -128..128
	                                             unless temperature offset is not zero. */

	unsigned int battery_33V;				  /* 3.3V battery measurement */
	unsigned int battery_5V;				  /* 5V battery measurement */
	unsigned int battery_12V;				  /* 12V battery measurement */

	unsigned int expension_1;				  /*  expansion board#1 ID.  0xFF if board does not exist */
	unsigned int expension_2;				  /*  expansion board#2 ID.  0xFF if board does not exist */
	unsigned int expension_3;				  /*  expansion board#3 ID.  0xFF if board does not exist */

	enum smp_bool sfp_1;					  /* SFP#1 exist. SMP_TRUE means exist. */
	enum smp_bool sfp_2;					  /* SFP#2 exist. SMP_TRUE means exist. */
};

/* Chassis status structure */
struct smp_chassis_alarm {
	/* The alarm is set if the actual fan speed reading is above the threshold value */
	unsigned int fan1;
	unsigned int fan2;

	/* The The alarm is set if the actual temperature reading is less than or equal
	   or greater than configured limits  */
	unsigned int left_sensor;
	unsigned int center_sensor;
	unsigned int right_sensor;

	/* The alarm is set if the is either a short or open circuit fault on the left sensor */
	unsigned int left_sensor_fault;
	unsigned int right_sensor_fault;

	/* The alarm is set if the actual voltage reading is less than or greater than the configured limits */
	unsigned int battery_33V;
	unsigned int battery_v5; 
	unsigned int battery_v12;

	/*Alarms 48v may occur because of short circuit, over
	current, thermal shutdown or card which does not require 48V lines.
	The alarm is reported for each of three expansion boards*/  
	unsigned int battery_48V_1;
	unsigned int battery_48V_2;
	unsigned int battery_48V_3;
};

/* FAN auto-control algorithm configuration */
struct smp_fan_auto_ctr_cfg {
	unsigned int low_limit_temp;		  /* minimum temperature that will turn the fans on */

	unsigned int hysteresis_temp;		  /* hysteresis value for the minimum temperature\
											 that will turn the fans off */

	unsigned int absolute_limit_temp;	  /* if the actual temperature is equal to or exceeds
											 the absolute limit, all fans will be set to full on */

};

/* Battery alarm threshold configuration */
struct smp_battery_alarm_thresholds {
	unsigned int threshold_33V;	/* value in percent. If the measured voltage is
									   less than or greater than 3.3V by configured threshold,
									   the appropriate alarm will be set */

	unsigned int threshold_5V;		/* value in percent. If the measured voltage is
									   less than or greater than 5V by configured threshold,
									   the appropriate alarm will be set */

	unsigned int threshold_12V;	/* value in percent. If the measured voltage is
									   less than or greater than 12V by configured threshold,
									   the appropriate alarm will be set */
};


/* SFP vendor informanton */
struct smp_sfp_vendor_info {
	char vendor_name[16]; /* SFP transceiver vendor name (ASCII) */
	char vendor_oui[3];	  /* SFP transceiver vendor IEEE company ID */
	char vendor_pn[16];	  /* Part number provided by SFP transceiver vendor (ASCII) */
	char vendor_rev[4];	  /* Revision level for part number provided by vendor (ASCII) */
	char vendor_sn[16];	  /* Serial number provided by vendor (ASCII) */
	char data_code[8];	  /* Vendor’s manufacturing date code */

};

/* port statistics structure */
struct smp_port_stats {
	unsigned int unicast_pkt_in;	   /* number of incoming unicast packets */
	unsigned int unicast_pkt_out;	   /* number of outgoing unicast packets */
	unsigned int multicast_pkt_in;	   /* number of incoming multicast packets */
	unsigned int multicast_pkt_out;	   /* number of outgoing multicast packets */
	unsigned int broadcast_pkt_in;	   /* number of incoming broadcast packets */
	unsigned int broadcast_pkt_out;	   /* number of outgoing broadcast packets */
	unsigned int discarded_pkt_in;	   /* number of discarded incoming packets */
	unsigned int discarded_pkt_out;	   /* number of discarded outgoing packets*/
	unsigned int error_pkt_in;		   /* number of incoming packets contained errors */
	unsigned int error_pkt_out;		   /* number of outgoing packets contained errors */
};


/* Power over Ethernet alarm configuration structure */
struct smp_poe_alarm_cfg {
	enum smp_bool power_usage_alarm_enable;	 /* Enables or disables Power Usage alarm reporting.
												SMP_FALSE disables alarm reporting,
												SMP_TRUE  enables alarm reporting. */

	unsigned int power_usage_threshold;	     /* The usage threshold expressed in percent for
												comparing the measured power and initiating
												an alarm if the threshold is exceeded.
												If alarm reporting is disabled the atrubute has
												no effect*/
};

/* Power over Ethernet advanced information structure */
struct smp_poe_info {
	unsigned int nominal_power;			    /* The nominal power of the PSE expressed in Watts */
	unsigned int measured_power;		    /* Measured usage power expressed in Watts */
	unsigned int measured_current;		    /* Measured current in uA */
	unsigned int measured_voltage;		    /* Measured voltage in mV */

	enum smp_poe_class_status class_status;	/* Classification status */
	enum smp_poe_sig_status   sig_status;	/* Signature status */
};


/* Power over Ethernet alarm structure */
struct smp_poe_alarm {
	unsigned int power_usage_on;			/*  The usage  power is above the  Power Usage Threshold */
	unsigned int power_usage_off;			/*  The usage  power is below the  Power Usage Threshold */
};

/* Detailed expansion board info structure */
struct smp_card_info {
	char card_name[16];						/* Expansion board card name ( ASCII ) */
	char card_description[64];				/* Expansion board card description ( ASCII ) */

	enum smp_bool need_48V;					/* Flag. EMP_TRUE if the board needs 48V supply */

	enum smp_exp_ifc_type ifc_type;			/* Type of the expansion board access interface*/
};


/* ##### */

/*********************************************************************************************
	PUBLIC SECTION: END
*********************************************************************************************/


/*******************************************************************************************
	EXPORT SECTION: BEGIN
*******************************************************************************************/

/* ##### Global Variables ##### */
/* ##### */


/* ##### Function Prototypes ##### */
/*********************************************************************************************
Syntax:				int smp_init(struct smp_serial_map* serial_map_ptr)

Remarks:			This function initializes shelf management package

						Parameter		 							Description
						--------------------------------------------------------------------------------
						serial_map_ptr								Specifies the pointer to smp_serial_map
																	structure containes the file descriptors
																	for the sending commands to the interface
																	module and to the expansion boards.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 									Description
						--------------------------------------------------------------------------------
						= SMP_STATUS_OK									No Errors		(Success)
						< 0												Error Code 		(Failure)
																			SMP_STATUS_INV_ARG
																			SMP_STATUS_RESET
																			SMP_STATUS_TIMEOUT
																			SMP_STATUS_NO_CRD_RESPONSE
																			SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_init(struct smp_serial_map* serial_map_ptr);

/*********************************************************************************************
Syntax:				int smp_chassis_status_get(struct smp_chassis_status * chassis_status_ptr)

Remarks:			This function retrieves the actual chassis status.

							Parameter		 							Description
							--------------------------------------------------------------------------------
							chassis_status_ptr							 Specifies the pointer to the smp_chassis_status
																		 structure. The actual chassis status will be stored
																		 in memory pointed by the argument.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_chassis_status_get(struct smp_chassis_status * chassis_status_ptr);


/*********************************************************************************************
Syntax:				int smp_chassis_alarms_get(struct smp_chassis_alarm * chassis_alarm_ptr)

Remarks:			This function retrieves the actual chassis alarms.

							Parameter		 							Description
							--------------------------------------------------------------------------------
							chassis_alarm_ptr							 Specifies the pointer to the smp_chassis_alarm
																		 structure. The actual alarms will be stored
																		 in memory pointed by the argument.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_chassis_alarms_get(struct smp_chassis_alarm * chassis_alarm_ptr);


/*********************************************************************************************
Syntax:				int smp_48v_cfg_set(enum smp_slot slot, enum smp_48v_action action)

Remarks:			This function powers 48V ON or OFF for the expansion boards.

							Parameter		 							Description
							--------------------------------------------------------------------------------
							slot		            					 Slot number. SMP_SLOT_1, SMP_SLOT_2
																		 or SMP_SLOT_3.

							action		            					 SMP_48V_ON or SMP_48V_OFF.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_48v_on_off(enum smp_slot slot, enum smp_48v_action action);

/*********************************************************************************************
Syntax:				int smp_fan_auto_control_set(enum smp_bool enable, struct smp_fan_auto_ctr_cfg * cfg_ptr)

Remarks:			This function enables or disables FAN auto-control mode. 
					The mode is enabled by default. All thresholds and limits used by
					the algorithm will be configured to defaults during the Firmware startup.

							Parameter		 							Description
							--------------------------------------------------------------------------------
							enable		            					 SMP_TRUE to enable the mode. 
																		 SMP_FALSE to disable the mode.
																		 If the mode is disabled the next argument
																		 will be ignored.

							cfg_ptr                                      Specifies the pointer to the smp_fan_auto_ctr_cfg
																		 structure contains the limits values which will used
																		 by the auto-control algorithm.
																		 If the argument is NULL, default values will be used.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_fan_auto_control_set(enum smp_bool enable, struct smp_fan_auto_ctr_cfg * cfg_ptr);

/*********************************************************************************************
Syntax:				int smp_fan_auto_control_get(enum smp_bool* enable_ptr, struct smp_fan_auto_ctr_cfg* cfg_ptr)

Remarks:			This function retrieves fans auto-control mode configuration. If the mode is 
					disabled, the second argument will be ignored.

							Parameter		 							Description
							--------------------------------------------------------------------------------
							enable_ptr		            				Specifies the pointer to the buffer
																		where the enable status will be stored.
																		   SMP_TRUE if the mode is enabled
																		   SMP_FALSE if the mode is disabled
																		 If the mode is disabled the next argument
																		 will be ignored.

							cfg_ptr                                      Specifies the pointer to the smp_fan_auto_ctr_cfg
																		 structure where the limits values will be stored.
																		 
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_fan_auto_control_get(enum smp_bool* enable_ptr, struct smp_fan_auto_ctr_cfg* cfg_ptr);

/*********************************************************************************************
Syntax:				int smp_fan_speed_set(enum smp_fans fan, unsigned int speed)

Remarks:			This function sets speed to the specific FAN.
					If cooling system is in auto-control mode, the API will ignored. 

							Parameter		 							Description
							--------------------------------------------------------------------------------
							fan		            					    Specifies the target FAN: 
																		SMP_FAN_1 or SMP_FAN_2


							speed               					    Specifies the speed in RPM

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_fan_speed_set(enum smp_fans fan, unsigned int speed);

/*********************************************************************************************
Syntax:				int smp_fan_alarm_threshold_set(enum smp_fans fan, unsigned int threshold)

Remarks:			This function sets speed alarm threshold for both of FANs. If fan speed
					is less than configured value, alarm will be raised   

							Parameter		 							Description
							--------------------------------------------------------------------------------
							threshold             					    Specifies the minimum value of fan speed
																		in RPM.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_fan_alarm_threshold_set(unsigned int threshold);

/*********************************************************************************************
Syntax:				int smp_fan_alarm_threshold_get(unsigned int *threshold)

Remarks:			This function retrieves speed alarm threshold for both of FANs.   

							Parameter		 							Description
							--------------------------------------------------------------------------------
							threshold_ptr          					    Specifies the pointer where the 
																		threshold will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_fan_alarm_threshold_get(unsigned int *threshold_ptr);

/*********************************************************************************************
Syntax:				int smp_battery_alarm_thresholds_set(struct smp_battery_alarm_thresholds *thresholds_ptr)

Remarks:			This function sets threshold for battery alarm system in percent. If the measured
					voltage is less than or greater than by configured threshold, the appropriate
					alarm will be set.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							thresholds_ptr         					    Specifies the pointer to the smp_fan_auto_ctr_cfg
																		structure contains the threshold values to set.
																		If the argument is NULL, default values will be used.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_battery_alarm_thresholds_set(struct smp_battery_alarm_thresholds *thresholds_ptr);

/*********************************************************************************************
Syntax:				int smp_battery_alarm_thresholds_get(struct smp_battery_alarm_thresholds *thresholds_ptr)

Remarks:			This function retrieves the thresholds for battery alarm system.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							thresholds_ptr         					    Specifies the pointer to the smp_fan_auto_ctr_cfg
																		structure where the threshold values will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_battery_alarm_thresholds_get(struct smp_battery_alarm_thresholds *thresholds_ptr);

/*********************************************************************************************
Syntax:				int smp_card_inventory(unsigned int card_i, struct smp_card_info *info_ptr)

Remarks:			This function retrieves the detailed information about expansion board 
					according to its ID.
					
							Parameter		  			Description
							----------------------------------------------------------------
							card_id		    Specifies expansion card id
							
							info_ptr        Specifies pointer to smp_card_info structure 
											where the info will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 	  			Description
							-----------------------------------------------------------------
							= SMP_STATUS_OK	  	No Errors		(Success)
							< 0				  	Error Code 		(Failure)
												  SMP_STATUS_INV_ARG
												  SMP_STATUS_RESET
												  SMP_STATUS_TIMEOUT
												  SMP_STATUS_NO_CRD_RESPONSE
												  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_card_inventory(unsigned int card_id, struct smp_card_info *info_ptr);

/*********************************************************************************************
Syntax:				int smp_port_enable(unsigned int port_mask, enum smp_bool enable)

Remarks:			This function enables or disables specified chassis Ethernet port.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
																	
							enable								    SMP_TRUE to enable the port.
																	SMP_FALSE to disable the port.							                                        

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_enable(unsigned int port_mask, enum smp_bool enable);


/*********************************************************************************************
Syntax:				 int smp_port_status(unsigned int port_mask, enum smp_port_status *status_ptr)

Remarks:			This function retrieves specified chassis Ethernet port status.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port        					        Specifies a single Ethernet port.
																	
							status_ptr							    Specifies pointer to buffer
																	where the status will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_status(enum smp_port port, enum smp_port_status *status_ptr);


/*********************************************************************************************
Syntax:				int smp_port_speed_set(unsigned int port_mask, enum smp_eth_speed speed)

Remarks:			This function sets speed for the specified chassis Ethernet port.
					If the port is in auto-negotiation mode the command will be ignored.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
																	
							speed								    Specifies selected speed.							                                        

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_speed_set(unsigned int port_mask, enum smp_eth_speed speed);

/*********************************************************************************************
Syntax:				int smp_port_speed_get(enum smp_port port, enum smp_eth_speed *speed_ptr)

Remarks:			This function retrieves speed of the specified chassis Ethernet port.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port        					        Specifies a single Ethernet port.
																	
							speed_ptr							    Specifies pointer to the buffer where the
																	speed will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_speed_get(enum smp_port port, enum smp_eth_speed *speed_ptr);

/*********************************************************************************************
Syntax:				int smp_port_autoneg_enable(unsigned int port_mask, enum smp_bool enable)

Remarks:			This function enables or disables auto-negotiation mechanism.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
																	
							enable								    SMP_TRUE enables auto-negotiation.
																	SMP_FALSE disables auto-negotiation

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_autoneg_enable(unsigned int port_mask, enum smp_bool enable);

/*********************************************************************************************
Syntax:				int smp_port_autoneg_restart(unsigned int port_mask)

Remarks:			This function forces auto-negotiation to begin link renegotiation.
					If auto-negotiation is disabled, the API has no effect
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_autoneg_restart(unsigned int port_mask);

/*********************************************************************************************
Syntax:				int smp_port_autoneg_status(enum smp_port port, enum smp_autoneg_status *status_ptr)

Remarks:			This function forces auto-negotiation to begin link renegotiation.
					If auto-negotiation is disabled, the API has no effect
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port        					        Specifies a single Ethernet port.
							
							status_ptr                              Specifies pointer to buffer where the 
																	status  will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_autoneg_status(enum smp_port port, enum smp_autoneg_status *status_ptr);

/*********************************************************************************************
Syntax:				int smp_port_autoneg_capabilities_set(unsigned int port_mask, unsigned int capabilities_mask)

Remarks:			This function configures the set of capabilities of the local auto-negotiation
					entity.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
							
							capabilities_mask                       Specifies bit mask of local capabilities:							
																			SMP_ANC_10BT    
																			SMP_ANC_10BTFD  
																			SMP_ANC_100BTX  
																			SMP_ANC_100BTXFD
																			SMP_ANC_1000BT  
																			SMP_ANC_1000BTFD
																			SMP_ANC_1000BX  
																			SMP_ANC_1000BXFD
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_autoneg_capabilities_set(unsigned int port_mask, unsigned int capabilities_mask);

/*********************************************************************************************
Syntax:				int smp_port_autoneg_capabilities_get(enum smp_port port, 
                                                          unsigned int *capabilities_mask_prt,
														  enum smp_bool remote)

Remarks:			This function retrieves the set of capabilities of the local or remote
					auto-negotiation entity.
										
							Parameter		 				Description
							-------------------------------------------------------------------
							port        				Specifies a single Ethernet port.
							
							capabilities_mask_ptr       Specifies a pointer to the buffer were the
														bit mask of local capabilities will be stored.
																	
							remote        				Specifies a flag. If SMP_TRUE, the API retrieves
														the capabilities supported by remote entity.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 						Description
							---------------------------------------------------------------------
							= SMP_STATUS_OK				No Errors		(Success)
							< 0							Error Code 		(Failure)
														  SMP_STATUS_INV_ARG
														  SMP_STATUS_RESET
														  SMP_STATUS_TIMEOUT
														  SMP_STATUS_NO_CRD_RESPONSE
														  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_autoneg_capabilities_get(enum smp_port port, 
											 unsigned int *capabilities_mask_prt,
											 enum smp_bool remote);

/*********************************************************************************************
Syntax:				int smp_port_sfp_manufacture_info(enum smp_port port, struct smp_sfp_vendor_info *info_ptr)

Remarks:			This function retrieves the set of capabilities of the local or remote
					auto-negotiation entity.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port        					        Specifies a single Ethernet port.
							
							info_ptr                                Specifies a pointer to the smp_sfp_vendor_info
																	where the info will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_sfp_manufacture_info(enum smp_port port, struct smp_sfp_vendor_info *info_ptr);

/*********************************************************************************************
Syntax:				int smp_port_jack_type(enum smp_port port, enum smp_jack_type *jack_type_ptr)

Remarks:			This function retrieves the type of the jack attached to the interface.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port        					        Specifies a single Ethernet port.
																	
							jack_type_ptr						    Specifies pointer to buffer
																	where the jack type will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_jack_type(enum smp_port port, enum smp_jack_type *jack_type_ptr);

/*********************************************************************************************
Syntax:				int smp_chs_eth_port_statistics(enum smp_port port, struct smp_port_stats *stats_ptr)

Remarks:			This function retrieves specified chassis Ethernet port statistics.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port         					        Specifies a single Ethernet port.
																	
							stats_ptr							    Specifies pointer to smp_port_stats
																	structure where the status will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_statistics(enum smp_port port, struct smp_port_stats *stats_ptr);

/*********************************************************************************************
Syntax:				int smp_port_poe_enable(unsigned int port_mask, enum smp_bool remote)

Remarks:			This function enables or disables Power over Ethernet capabilities
					for the specified Ethernet port or group of ports.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
							
							enable                                  SMP_TRUE enables the capability.
																	SMP_FALSE disables the capability
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_enable(unsigned int port_mask, enum smp_bool enable);

/*********************************************************************************************
Syntax:				int smp_port_poe_status(enum smp_port port, enum smp_port_poe_status *status_ptr)

Remarks:			This function retrieves Power over Ethernet status
					for the specified Ethernet port.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port         					        Specifies a single Ethernet port.
																	
							stats_ptr							    Specifies pointer to the buffer
																	where the status will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_status(enum smp_port port, enum smp_port_poe_status *status_ptr);

/*********************************************************************************************
Syntax:				int smp_port_poe_alarm_cfg_set(unsigned int port_mask, struct smp_poe_alarm_cfg *cfg_ptr)

Remarks:			This function configures Power over Ethernet alarm conditions
					for the specified Ethernet port or group of ports.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port_mask      					        Specifies Ethernet port or set of ports
																	to which the configuration will be applied.
																	The list of port is defined in smp_port enumeration.
																	Application can use SMP_PORT_BIT macro to place the
																	appropriate bit to the corresponding place.
																	Each port is a one single bit in the mask.
																	Application can combine the bits to apply the 
																	configuration to a group of ports.
							
							cfg_ptr                                 Specifies a pointer to the smp_poe_alarm_cfg
																	structure, contains the configuration to set
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_alarm_cfg_set(unsigned int port_mask, struct smp_poe_alarm_cfg *cfg_ptr);

/*********************************************************************************************
Syntax:				int smp_port_poe_alarm_cfg_get(enum smp_port port, struct smp_poe_alarm_cfg *cfg_ptr);

Remarks:			This function retrieves Power over Ethernet alarm configuration
					for the specified Ethernet port.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port         					        Specifies a single Ethernet port.
																	
							cfg_ptr						            Specifies pointer to the smp_poe_alarm_cfg
																	structure where the retrieved configuration
																	will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_alarm_cfg_get(enum smp_port port, struct smp_poe_alarm_cfg *cfg_ptr);

/*********************************************************************************************
Syntax:				int smp_port_poe_info(enum smp_port port, struct smp_poe_info *info)

Remarks:			This function retrieves Power over Ethernet advanced information
					for the specified Ethernet port.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port         					        Specifies a single Ethernet port.
																	
							info_ptr					            Specifies pointer to the smp_poe_info
																	structure where the retrieved information
																	will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_info(enum smp_port port, struct smp_poe_info *info_ptr);

/*********************************************************************************************
Syntax:				int smp_port_poe_alarm(enum smp_port port, struct smp_poe_alarm *alarm_ptr)

Remarks:			This function retrieves Power over Ethernet alarms
					for the specified Ethernet port.
										
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port         					        Specifies a single Ethernet port.
																	
							alarm_ptr					            Specifies pointer to the smp_poe_alarm
																	structure where the retrieved information
																	will be stored.
																	
Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= SMP_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  SMP_STATUS_INV_ARG
																			  SMP_STATUS_RESET
																			  SMP_STATUS_TIMEOUT
																			  SMP_STATUS_NO_CRD_RESPONSE
																			  SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_port_poe_alarm(enum smp_port port, struct smp_poe_alarm *alarm_ptr);

/*********************************************************************************************
Syntax:				int smp_firmware_upgrade(int fd_device, int fd_firmware)

Remarks:			This function performs the firmware upgrade capabilities.
					
							Parameter		 	 		Description
							------------------------------------------------------------------
							fd_device         	 Specifies serial port file descriptor.
												 For example result of call to
												 open(“/dev/ttyUSB0”).
																	
							fd_firmware	  		 file descriptor of a file containing the firmware,
												 for example result of call to
												 open(“/system/shelf/interface.bin”)

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							-------------------------------------------------------------------
							= SMP_STATUS_OK		 No Errors		(Success)
							< 0					 Error Code 		(Failure)
													 SMP_STATUS_INV_ARG
													 SMP_STATUS_RESET
													 SMP_STATUS_TIMEOUT
													 SMP_STATUS_NO_CRD_RESPONSE
													 SMP_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int smp_firmware_upgrade(int fd_device, int fd_firmware);

/*********************************************************************************************
Syntax:				void smp_sync_timeout_set(unsigned int timeout)

Remarks:			This function modifies synchronization timeout

						Parameter		 							Description
						--------------------------------------------------------------------------------
						timeout 									Specifies the timeout value in 
																	milliseconds.

Return Value:	    None.

*********************************************************************************************/
extern void smp_sync_timeout_set(unsigned int timeout);

/*********************************************************************************************
Syntax:				int smp_sync_timeout_get(unsigned int *timeout_ptr)

Remarks:			This function retrieves synchronization timeout

						Parameter		 							Description
						--------------------------------------------------------------------------------
						timeout_ptr									Specifies the pointer to the buffer where
																	the timeout value in milliseconds will be
																	stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 									Description
						--------------------------------------------------------------------------------
						= SMP_STATUS_OK									No Errors		(Success)
						< 0												Error Code 		(Failure)
																			SMP_STATUS_INV_ARG

*********************************************************************************************/
extern int smp_sync_timeout_get(unsigned int *timeout_ptr);

/**************************************************************************************************
Syntax:				void smp_card_timeout_set(unsigned int timeout)

Remarks:			This function modifies card reply timeout

						Parameter		 							Description
						---------------------------------------------------------------------------
						timeout 									Specifies the timeout value in
																	milliseconds.

Return Value:	    None.
***************************************************************************************************/
extern void smp_card_timeout_set(unsigned int timeout);

/*********************************************************************************************
Syntax:				int smp_sem_timeout_get(unsigned int *timeout_ptr)

Remarks:			This function retrieves card reply timeout

						Parameter		 							Description
						--------------------------------------------------------------------------------
						timeout_ptr									Specifies the pointer to the buffer where
																	the timeout value in milliseconds will be
																	stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 									Description
						--------------------------------------------------------------------------------
						= SMP_STATUS_OK									No Errors		(Success)
						< 0												Error Code 		(Failure)
																			SMP_STATUS_INV_ARG

*********************************************************************************************/
extern int smp_card_timeout_get(unsigned int *timeout_ptr);

/**************************************************************************************************
Syntax:				void smp_card_timeout_set(unsigned int timeout)

Remarks:			This function modifies I2C attached devices timeout

						Parameter		 							Description
						---------------------------------------------------------------------------
						timeout 									Specifies the timeout value in
																	milliseconds.

Return Value:	    None.
***************************************************************************************************/
extern void smp_i2c_timeout_set(unsigned int timeout);

/*********************************************************************************************
Syntax:				int smp_sem_timeout_get(unsigned int *timeout_ptr)

Remarks:			This function retrieves I2C attached devices timeout

						Parameter		 							Description
						--------------------------------------------------------------------------------
						timeout_ptr									Specifies the pointer to the buffer where
																	the timeout value in milliseconds will be
																	stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 									Description
						--------------------------------------------------------------------------------
						= SMP_STATUS_OK									No Errors		(Success)
						< 0												Error Code 		(Failure)
																			SMP_STATUS_INV_ARG
																			SMP_STATUS_RESET
																			SMP_STATUS_TIMEOUT
																			SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_i2c_timeout_get(unsigned int *timeout_ptr);

/*********************************************************************************************
Syntax:			int smp_i2c_write(enum smp_peripherals peripheral, int address, char value)

Remarks:		This function writes specified register in the devices attached to
				the interface module or to the expansion board via I2C

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral        	Specifies the target device.
						
						address             I2C register address
						
						value               Value to write

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_i2c_write(enum smp_peripherals peripheral, int address, char value);

/*********************************************************************************************
Syntax:			int smp_i2c_read(enum smp_peripherals peripheral, int address, char *value_ptr)

Remarks:		This function reads specified register in the devices attached to
				the interface module or to the expansion board via I2C

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral        	Specifies the target device.
						
						address             I2C register address
						
						value_ptr           Pointer to buffer where the read value
											will be stored.

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_i2c_read(enum smp_peripherals peripheral, int address, char *value_ptr);


/*********************************************************************************************
Syntax:			int smp_spi_write(enum smp_peripherals peripheral, char *buffer, int num)

Remarks:		This function writes specified register in the devices attached to
				to the expansion board via SPI

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral		    Specifies the target device.
						
						buffer_ptr          Pointer to buffer, contains the data to write
						
						num                 Number of bytes to write

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_spi_write(enum smp_peripherals peripheral, char *buffer_ptr, int num);

/*********************************************************************************************
Syntax:			int smp_spi_read(enum smp_peripherals peripheral, char *buffer, int num)

Remarks:		This function reads specified register in the devices attached to
				to the expansion board via SPI

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral		    Specifies the target device.
						
						buffer_ptr          Pointer to buffer where the read data will be stored
						
						num                 Number of bytes to read

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_spi_read(enum smp_peripherals peripheral, char *buffer_ptr, int num);

/*********************************************************************************************
Syntax:			int smp_mdio_write(enum smp_peripherals peripheral, int dev, int addr, int val)

Remarks:		This function writes specified register in the devices via MDIO

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral		    Specifies the target MDIO line.
						
						dev                 MDIO device
						
						addr                Target address
						
						val                 Value to write

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_mdio_write(enum smp_peripherals peripheral, int dev, int addr, int val);

/*********************************************************************************************
Syntax:			int smp_mdio_read(enum smp_peripherals peripheral, int dev, int addr, int *val_prt)

Remarks:		This function reads specified register in the devices via MDIO

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral		    Specifies the target MDIO line.
						
						dev                 MDIO device
						
						addr                Target address
						
						val_ptr             Pointer to buffer where the read value will be stored

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_mdio_read(enum smp_peripherals peripheral, int dev, int addr, int *val_prt);

/*********************************************************************************************
Syntax:			int smp_gpio_setup(enum smp_peripherals peripheral, int lines, 
								   enum smp_gpio_mode mode)

Remarks:		This function configures GPIO line or group of lines. 

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral 	    Specifies the expansion board GPIO.
						
						lines               Bit mask. Specifies the group of GPIO lines:
						
													SMP_GPIO_1
													SMP_GPIO_2
													SMP_GPIO_3
													SMP_GPIO_4
											
											The group can be defined as following:
												SMP_GPIO_1 | SMP_GPIO_3
						
						mode                Specifies the mode ( In/Out/Disabled )

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_gpio_setup(enum smp_peripherals peripheral, int lines, enum smp_gpio_mode mode);

/*********************************************************************************************
Syntax:			int smp_gpio_write(enum smp_peripherals peripheral, int lines, int val_mask)

Remarks:		This function writes to GPIO line or to the group of lines. 

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral 	    Specifies the expansion board GPIO.
						
						lines               Bit mask. Specifies the group of GPIO lines:
						
													SMP_GPIO_1
													SMP_GPIO_2
													SMP_GPIO_3
													SMP_GPIO_4
											
											The group can be defined as following:
												SMP_GPIO_1 | SMP_GPIO_3
						
						val_mask            Specifies the bit mask to be written. The appropriate
						                    bit values should be placed in the offsets corresponding
											to the specified gpio lines.
						
				The example below shows how to write ‘0’ to the GPIO_1 and ‘1’ to GPIO_2
				of the expansion card#1 interface:
				
				if( smp_gpio_write(SMP_EXP1_FLX, SMP_GPIO_1|SMP_GPIO_2, 0x02) == SMP_STATUS_OK){
						
						do something;
				}

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_gpio_write(enum smp_peripherals peripheral, int lines, int val_mask);

/*********************************************************************************************
Syntax:			int smp_gpio_read(enum smp_peripherals peripheral, int lines, int *val_mask_ptr)

Remarks:		This function reads from GPIO line or from the group of lines. Only GPIO lines
                setup as SMP_GPIO_INPUT can be read. Reading of GPIO in the not read modes 
				returns undefined value

						Parameter					Description
						-----------------------------------------------------------------------
						peripheral 	    Specifies the expansion board GPIO.
						
						lines               Bit mask. Specifies the group of GPIO lines:
						
													SMP_GPIO_1
													SMP_GPIO_2
													SMP_GPIO_3
													SMP_GPIO_4
											
											The group can be defined as following:
												SMP_GPIO_1 | SMP_GPIO_3
						
						val_mask_ptr        Specifies the pointer to the buffer where readout value
						                    will be stored. The appropriate bit values can found
						                    at the offsets corresponding to the specified gpio lines.
						
				The example below shows how to read from two GPIO lines
				from the expansion card#1 interface:
				
				int value;
				
				if( smp_gpio_read(SMP_EXP1_FLX, SMP_GPIO_1|SMP_GPIO_2, &value) == SMP_STATUS_OK){
						
					int gpio1 = value & SMP_GPIO_1;
					int gpio2 = value & SMP_GPIO_2;
				}

Return Value:	Returns SMP_STATUS_OK on success, or an error code on failure.

						Value		 		 		Description
						------------------------------------------------------------------------
						= SMP_STATUS_OK		 	No Errors		(Success)
						< 0					 	Error Code 		(Failure)
													SMP_STATUS_INV_ARG
													SMP_STATUS_RESET
													SMP_STATUS_TIMEOUT
													SMP_STATUS_NO_CRD_RESPONSE

*********************************************************************************************/
extern int smp_gpio_read(enum smp_peripherals peripheral, int lines, int *val_mask_ptr);
/* ##### */

/*******************************************************************************************
	EXPORT SECTION: END
*******************************************************************************************/


#endif /* _SMP_H */
