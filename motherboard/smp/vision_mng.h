/**********************************************************************************************************************
*                                        (c) COPYRIGHT by MEGABRIDGE LTD.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
#
# NAME
# vision_mng.h
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

#ifndef _VISION_MNG_H
#define _VISION_MNG_H


/* ##### General Definitions ##### */
/* ##### */


/* ##### Types/Constants/Externals ##### */
/* ##### */

/* ##### Constant Definitions ##### */

/* Error codes */
#define VISION_MNG_STATUS_OK                1         /* the operation is completed successfully */
#define VISION_MNG_STATUS_ERROR            -1         /* an error has occurred */
#define VISION_MNG_STATUS_INV_ARG          -2         /* invalid argument */
#define VISION_MNG_STATUS_RESET            -3         /* the operation was interrupted by card reset */
#define VISION_MNG_STATUS_TIMEOUT          -4         /* semaphore timeout has occurred */
#define VISION_MNG_STATUS_NO_CRD_RESPONSE  -5         /* card does not response */
#define VISION_MNG_STATUS_NO_DEV_RESPONSE  -6         /* card hardware does not response */



/* Timeout constants*/
#define VISION_MNG_WAITFOREVER             -1

#define VISION_MNG_NO_WAIT                  0

/*
 The package communicates with the interface board and other board carrying a CPU 
 via RS232 communication channel. The package sends commands to the boards and 
 expects to get a reply for every sent command.

 The API synchronizes the access of the application to the devices.
 The API blocks concurrent calls to the same device until the current
 transaction to the device completes.
*/

/* Wait for synchronization timeout in ms. If expiries, 
   the VISION_MNG_STATUS_TIMEOUT error code will be returned by API
*/
#define VISION_MNG_DEF_SYNC_TIMEOUT         100

/* 
   Wait for card reply timeout in ms. If expiries,                           
   the VISION_MNG_STATUS_NO_CRD_RESPONSE error code will be returned by API
*/ 
#define VISION_MNG_DEF_RESP_TIMEOUT         50         


/* Wait for peripheral device timeout in ms using by Firmware. If expiries, 
   the VISION_MNG_STATUS_NO_DEV_RESPONSE error code will be returned by API
*/
#define VISION_MNG_DEF_I2C_TIMEOUT          10


/* ##### Type Definitions ##### */

/* Expansion boards slot enumeration */ 
enum vision_mng_slot {
	VISION_MNG_SLOT_1,
	VISION_MNG_SLOT_2,
	VISION_MNG_SLOT_3
};

/* 48V ON/OFF action enumeration */ 
enum vision_mng_48v_action {
	VISION_MNG_48V_OFF,
	VISION_MNG_48V_ON
};

/* FANs enumeration */ 
enum vision_mng_fans {
	VISION_MNG_FAN_1,
	VISION_MNG_FAN_2
};

/* Boolean enumeration, for boolean flags */ 
enum vision_mng_bool {
	VISION_MNG_FALSE,
	VISION_MNG_TRUE
};

/* Port status enumeration */ 
enum vision_mng_port_status {
	VISION_MNG_PORT_DOWN,			   /* The port is disabled or there is a fault 
								  that prevents it from going to the up*/

	VISION_MNG_PORT_UP,			   /* Port is UP */

	VISION_MNG_PORT_NOT_PRESENT,	   /* the interface has  missing components */
};


/* Ethernet Port speed enumeration */ 
enum vision_mng_eth_speed {
	VISION_MNG_10B_HD,			/*10 Mbps half-duplex*/
	VISION_MNG_10B_FD,			/*10 Mbps full-duplex*/
	VISION_MNG_100B_HD,		/*100 Mbps half-duplex*/ 
	VISION_MNG_100B_FD,		/*100 Mbps full-duplex*/ 
	VISION_MNG_1000B_HD,		/*1000 Mbps half-duplex*/ 
	VISION_MNG_1000B_FD,		/*1000 Mbps full-duplex*/ 
};


/* Ethernet Port auto-negotiation status enumeration */ 
enum vision_mng_autoneg_status {
	VISION_MNG_AUTONEG_CONFIGURING,   /* auto-negotiation is in process */
	VISION_MNG_AUTONEG_COMPLETE,	   /* auto-negotiation is complete */
	VISION_MNG_AUTONEG_DISABLED,	   /* auto-negotiation mechanism is disabled */
	VISION_MNG_AUTONEG_FAULTY,		   /* auto-negotiation is faulty */
};

/* System Ports enumeration */ 
enum vision_mng_port {
	VISION_MNG_IFC_COMBO1_PORT,		   /* interface board COMBO#1 GE interface */
	VISION_MNG_IFC_COMBO2_PORT,		   /* interface board COMBO#2 GE interface */

	VISION_MNG_EXP1_FET1_PORT,		   /* expansion board#1 fast-ethernet interface#1 */
	VISION_MNG_EXP1_FET2_PORT,		   /* expansion board#1 fast-ethernet interface#2 */
	VISION_MNG_EXP1_FET3_PORT,		   /* expansion board#1 fast-ethernet interface#3 */
	VISION_MNG_EXP1_FET4_PORT,		   /* expansion board#1 fast-ethernet interface#4 */
	VISION_MNG_EXP1_FET5_PORT,		   /* expansion board#1 fast-ethernet interface#5 */
	VISION_MNG_EXP1_FET6_PORT,		   /* expansion board#1 fast-ethernet interface#6 */
	VISION_MNG_EXP1_FET7_PORT,		   /* expansion board#1 fast-ethernet interface#7 */
	VISION_MNG_EXP1_FET8_PORT,		   /* expansion board#1 fast-ethernet interface#8 */
	VISION_MNG_EXP1_COMBO1_PORT,		   /* expansion board#1 COMBO#1 GE interface */  
	VISION_MNG_EXP1_COMBO2_PORT,		   /* expansion board#1 COMBO#2 GE interface */  


	VISION_MNG_EXP2_FET1_PORT,		   /* expansion board#2 fast-ethernet interface#1 */ 
	VISION_MNG_EXP2_FET2_PORT,		   /* expansion board#2 fast-ethernet interface#2 */ 
	VISION_MNG_EXP2_FET3_PORT,		   /* expansion board#2 fast-ethernet interface#3 */ 
	VISION_MNG_EXP2_FET4_PORT,		   /* expansion board#2 fast-ethernet interface#4 */ 
	VISION_MNG_EXP2_FET5_PORT,		   /* expansion board#2 fast-ethernet interface#5 */ 
	VISION_MNG_EXP2_FET6_PORT,		   /* expansion board#2 fast-ethernet interface#6 */ 
	VISION_MNG_EXP2_FET7_PORT,		   /* expansion board#2 fast-ethernet interface#7 */ 
	VISION_MNG_EXP2_FET8_PORT,		   /* expansion board#2 fast-ethernet interface#8 */ 
	VISION_MNG_EXP2_COMBO1_PORT,		   /* expansion board#2 COMBO#1 GE interface */  
	VISION_MNG_EXP2_COMBO2_PORT,		   /* expansion board#2 COMBO#2 GE interface */  


	VISION_MNG_EXP3_FET1_PORT,		   /* expansion board#3 fast-ethernet interface#1 */ 
	VISION_MNG_EXP3_FET2_PORT,		   /* expansion board#3 fast-ethernet interface#2 */ 
	VISION_MNG_EXP3_FET3_PORT,		   /* expansion board#3 fast-ethernet interface#3 */ 
	VISION_MNG_EXP3_FET4_PORT,		   /* expansion board#3 fast-ethernet interface#4 */ 
	VISION_MNG_EXP3_FET5_PORT,		   /* expansion board#3 fast-ethernet interface#5 */ 
	VISION_MNG_EXP3_FET6_PORT,		   /* expansion board#3 fast-ethernet interface#6 */ 
	VISION_MNG_EXP3_FET7_PORT,		   /* expansion board#3 fast-ethernet interface#7 */ 
	VISION_MNG_EXP3_FET8_PORT,		   /* expansion board#3 fast-ethernet interface#8 */ 
	VISION_MNG_EXP3_COMBO1_PORT,		   /* expansion board#3 COMBO#1 GE interface */  
	VISION_MNG_EXP3_COMBO2_PORT,		   /* expansion board#3 COMBO#2 GE interface */  
};

/* Auto-negotiation Port capabilities flags */
#define	VISION_MNG_ANC_10BT       0x0001   /* 10BASE-T    half duplex mode */
#define	VISION_MNG_ANC_10BTFD     0x0002   /* 10BASE-T    full duplex mode */
#define	VISION_MNG_ANC_100BTX     0x0004   /* 100BASE-TX  half duplex mode */
#define	VISION_MNG_ANC_100BTXFD   0x0008   /* 100BASE-TX  full duplex mode */
#define	VISION_MNG_ANC_1000BT     0x0010   /* 1000BASE-T  half duplex mode */
#define	VISION_MNG_ANC_1000BTFD   0x0020   /* 1000BASE-T  full duplex mode */
#define	VISION_MNG_ANC_1000BX     0x0040   /* 1000BASE-X  half duplex mode */
#define	VISION_MNG_ANC_1000BXFD   0x0080   /* 1000BASE-X  full duplex mode */

/* Jack type enumeration */ 
enum vision_mng_jack_type {
	VISION_MNG_RJ45,
	VISION_MNG_RJ45S,
	VISION_MNG_SC_FIBER,
	VISION_MNG_LC_FIBER,
};

/* Power over Ethernet port status enumeration*/
enum vision_mng_ethport_poe_status {
	VISION_MNG_POE_DISABLED,		   /* Power delivering is disabled */
	VISION_MNG_POE_SIG_IN_PROC,	   /* Signature in process */
	VISION_MNG_POE_CLASS_IN_PROC,	   /* Classification in process */
	VISION_MNG_POE_DELIV_POWER,	   /* Power is delivering */
	VISION_MNG_POE_FAULTY			   /* port is faulty */
};

/* Power over Ethernet port Classification status enumeration */
enum vision_mng_poe_class_status {
	VISION_MNG_POE_CLS_CLASS1,
	VISION_MNG_POE_CLS_UNKNOWN,
	VISION_MNG_POE_CLS_OVER_CURRENT
};

/* Power over Ethernet port Signature status enumeration */
enum vision_mng_poe_sig_status {
	VISION_MNG_POE_SIG_IS_VALID,
	VISION_MNG_POE_SIG_SHORT_CIRCUIT,
	VISION_MNG_POE_SIG_HIGH_PD_INPUT_CAPACITANCE,
	VISION_MNG_POE_SIG_LOW_RESISTANCE,
	VISION_MNG_POE_SIG_HIGH_RESISTANCE,
	VISION_MNG_POE_SIG_OPEN_CIRCUIT,
	VISION_MNG_POE_SIG_HIGH_PORT_VOLT_OFFSET,
	VISION_MNG_POE_SIG_UNKNOWN,
};

/* Expansion board access interface type enumeration*/
enum vision_mng_exp_ifc_type {
	VISION_MNG_EXP_I2C,		/* GPIO lines are configerd as I2C */
	VISION_MNG_EXP_SPI,		/* GPIO lines are configerd as SPI */
	VISION_MNG_EXP_MDIO,		/* GPIO lines are configerd as MDIO */
	VISION_MNG_EXP_GPIO,		/* GPIO lines are configerd as GPIO */
};

/* Serial Port (RS232) mapping structure */
struct vision_mng_serial_map {
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
struct vision_mng_shelf_status {
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

	enum vision_mng_bool sfp_1;					  /* SFP#1 exist. VISION_MNG_TRUE means exist. */
	enum vision_mng_bool sfp_2;					  /* SFP#2 exist. VISION_MNG_TRUE means exist. */
};

/* Chassis status structure */
struct vision_mng_shelf_alarm {
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
struct vision_mng_fan_auto_ctr_cfg {
	unsigned int low_limit_temp;		  /* minimum temperature that will turn the fans on */

	unsigned int hysteresis_temp;		  /* hysteresis value for the minimum temperature\
											 that will turn the fans off */

	unsigned int absolute_limit_temp;	  /* if the actual temperature is equal to or exceeds
											 the absolute limit, all fans will be set to full on */

};

/* Battery alarm threshold configuration */
struct vision_mng_battery_alarm_thresholds {
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
struct vision_mng_sfp_vendor_info {
	char vendor_name[16]; /* SFP transceiver vendor name (ASCII) */
	char vendor_oui[3];	  /* SFP transceiver vendor IEEE company ID */
	char vendor_pn[16];	  /* Part number provided by SFP transceiver vendor (ASCII) */
	char vendor_rev[4];	  /* Revision level for part number provided by vendor (ASCII) */
	char vendor_sn[16];	  /* Serial number provided by vendor (ASCII) */
	char data_code[8];	  /* Vendor's manufacturing date code */

};

/* port statistics structure */
struct vision_mng_ethport_stats {
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
struct vision_mng_poe_alarm_cfg {
	enum vision_mng_bool power_usage_alarm_enable;	 /* Enables or disables Power Usage alarm reporting.
												VISION_MNG_FALSE disables alarm reporting,
												VISION_MNG_TRUE  enables alarm reporting. */

	unsigned int power_usage_threshold;	     /* The usage threshold expressed in percent for
												comparing the measured power and initiating
												an alarm if the threshold is exceeded.
												If alarm reporting is disabled the atrubute has
												no effect*/
};

/* Power over Ethernet advanced information structure */
struct vision_mng_poe_info {
	unsigned int nominal_power;			    /* The nominal power of the PSE expressed in Watts */
	unsigned int measured_power;		    /* Measured usage power expressed in Watts */
	unsigned int measured_current;		    /* Measured current in uA */
	unsigned int measured_voltage;		    /* Measured voltage in mV */

	enum vision_mng_poe_class_status class_status;	/* Classification status */
	enum vision_mng_poe_sig_status   sig_status;	/* Signature status */
};


/* Power over Ethernet alarm structure */
struct vision_mng_poe_alarm {
	unsigned int power_usage_on;			/*  The usage  power is above the  Power Usage Threshold */
	unsigned int power_usage_off;			/*  The usage  power is below the  Power Usage Threshold */
};

/* Detailed expansion board info structure */
struct vision_mng_card_info {
	char card_name[16];						/* Expansion board card name ( ASCII ) */
	char card_description[64];				/* Expansion board card description ( ASCII ) */

	enum vision_mng_bool need_48V;					/* Flag. EMP_TRUE if the board needs 48V supply */

	enum vision_mng_exp_ifc_type ifc_type;			/* Type of the expansion board access interface*/
};


/*******************************************************************************************
	EXPORT SECTION: BEGIN
*******************************************************************************************/


/* ##### Function Prototypes ##### */
/*********************************************************************************************
Syntax:				int vision_mng_init(struct vision_mng_serial_map* serial_map_ptr)

Remarks:			This function initializes shelf management package

						Parameter		  		Description
						----------------------------------------------------------------------
						serial_map_ptr	  	Specifies the pointer to vision_mng_serial_map
										  	structure containes the file descriptors
										  	for the sending commands to the interface
										  	module and to the expansion boards.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

						Value		 	  			Description
						----------------------------------------------------------------------
						= VISION_MNG_STATUS_OK		No Errors		(Success)
						< 0					  		Error Code 		(Failure)
											  			VISION_MNG_STATUS_INV_ARG
														VISION_MNG_STATUS_RESET
														VISION_MNG_STATUS_TIMEOUT
														VISION_MNG_STATUS_NO_CRD_RESPONSE
														VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_init(struct vision_mng_serial_map* serial_map_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_shelf_status_get(struct vision_mng_shelf_status * shelf_status_ptr)

Remarks:			This function retrieves the actual shelf status.

							Parameter	  				Description
							------------------------------------------------------------------
							shelf_status_ptr	 Specifies the pointer to the vision_mng_shelf_status
												 structure. The actual shelf status will be stored
												 in memory pointed by the argument.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 		 		Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0					  		Error Code 		(Failure)
												  		  VISION_MNG_STATUS_INV_ARG
												  		  VISION_MNG_STATUS_RESET
												  		  VISION_MNG_STATUS_TIMEOUT
												  		  VISION_MNG_STATUS_NO_CRD_RESPONSE
												  		  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_shelf_status_get(struct vision_mng_shelf_status * shelf_status_ptr);


/*********************************************************************************************
Syntax:				int vision_mng_shelf_alarms_get(struct vision_mng_shelf_alarm * shelf_alarm_ptr)

Remarks:			This function retrieves the actual shelf alarms.

							Parameter		 			Description
							------------------------------------------------------------------
							shelf_alarm_ptr		Specifies the pointer to the vision_mng_shelf_alarm
												structure. The actual alarms will be stored
												in memory pointed by the argument.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK	    No Errors		(Success)
							< 0					  	    Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_shelf_alarms_get(struct vision_mng_shelf_alarm * shelf_alarm_ptr);


/*********************************************************************************************
Syntax:				int vision_mng_exp_48v_cfg_set(enum vision_mng_slot slot, enum vision_mng_48v_action action)

Remarks:			This function powers 48V ON or OFF for the expansion boards.

							Parameter		  		Description
							------------------------------------------------------------------
							slot		      Slot number. VISION_MNG_SLOT_1, VISION_MNG_SLOT_2
											  or VISION_MNG_SLOT_3.

							action		      VISION_MNG_48V_ON or VISION_MNG_48V_OFF.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 	  				Description
							--------------------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0					  		Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_exp_48v_on_off(enum vision_mng_slot slot, enum vision_mng_48v_action action);

/*********************************************************************************************
Syntax:				int vision_mng_fan_auto_control_set(enum vision_mng_bool enable, 
                                                struct vision_mng_fan_auto_ctr_cfg * cfg_ptr)

Remarks:			This function enables or disables FAN auto-control mode. 
					The mode is enabled by default. All thresholds and limits used by
					the algorithm will be configured to defaults during the Firmware startup.

							Parameter		 		Description
							------------------------------------------------------------------
							enable		        VISION_MNG_TRUE to enable the mode. 
												VISION_MNG_FALSE to disable the mode.
												If the mode is disabled the next argument
												will be ignored.

							cfg_ptr             Specifies the pointer to the vision_mng_fan_auto_ctr_cfg
												structure contains the limits values which will used
												by the auto-control algorithm.
												If the argument is NULL, default values will be used.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				 Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_fan_auto_control_set(enum vision_mng_bool enable, struct vision_mng_fan_auto_ctr_cfg * cfg_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_fan_auto_control_get(enum vision_mng_bool* enable_ptr, 
                                                 struct vision_mng_fan_auto_ctr_cfg* cfg_ptr)

Remarks:			This function retrieves fans auto-control mode configuration. If the mode is 
					disabled, the second argument will be ignored.

							Parameter		 		  	Description
							--------------------------------------------------------------------------------
							enable_ptr		   Specifies the pointer to the buffer
											   where the enable status will be stored.
											   VISION_MNG_TRUE if the mode is enabled
											   VISION_MNG_FALSE if the mode is disabled
											   If the mode is disabled the next argument
											   will be ignored.

							cfg_ptr            Specifies the pointer to the vision_mng_fan_auto_ctr_cfg
											   structure where the limits values will be stored.
																		 
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 					Description
							--------------------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_fan_auto_control_get(enum vision_mng_bool* enable_ptr, struct vision_mng_fan_auto_ctr_cfg* cfg_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_fan_speed_set(enum vision_mng_fans fan, unsigned int speed)

Remarks:			This function sets speed to the specific FAN.
					If cooling system is in auto-control mode, the API will ignored. 

							Parameter		 		 Description
							-------------------------------------------------------------------
							fan		       Specifies the target FAN: 
										   VISION_MNG_FAN_1 or VISION_MNG_FAN_2


							speed          Specifies the speed in RPM

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				 	Description
							--------------------------------------------------------------------
							= VISION_MNG_STATUS_OK  	No Errors		(Success)
							< 0					    	Error Code 		(Failure)
												    	  VISION_MNG_STATUS_INV_ARG
												    	  VISION_MNG_STATUS_RESET
												    	  VISION_MNG_STATUS_TIMEOUT
												    	  VISION_MNG_STATUS_NO_CRD_RESPONSE
												    	  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_fan_speed_set(enum vision_mng_fans fan, unsigned int speed);

/*********************************************************************************************
Syntax:				int vision_mng_fan_alarm_threshold_set(enum vision_mng_fans fan, 
                                                           unsigned int threshold)

Remarks:			This function sets speed alarm threshold for both of FANs. If fan speed
					is less than configured value, alarm will be raised   

							Parameter		 			   Description
							-------------------------------------------------------------------
							threshold             Specifies the minimum value of fan speed
												  in RPM.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 		  			Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK 		No Errors		(Success)
							< 0					   		Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_fan_alarm_threshold_set(unsigned int threshold);

/*********************************************************************************************
Syntax:				int vision_mng_fan_alarm_threshold_get(unsigned int *threshold)

Remarks:			This function retrieves speed alarm threshold for both of FANs.   

							Parameter		 				Description
							------------------------------------------------------------------
							threshold_ptr          	Specifies the pointer where the 
													threshold will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 			         Description
							-----------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_fan_alarm_threshold_get(unsigned int *threshold_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_battery_alarm_thresholds_set(struct vision_mng_battery_alarm_thresholds *thresholds_ptr)

Remarks:			This function sets threshold for battery alarm system in percent. If the measured
					voltage is less than or greater than by configured threshold, the appropriate
					alarm will be set.
					
							Parameter					Description
							------------------------------------------------------------------
							thresholds_ptr    Specifies the pointer to the vision_mng_fan_auto_ctr_cfg
											  structure contains the threshold values to set.
											  If the argument is NULL, default values will be used.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 			  	Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_battery_alarm_thresholds_set(struct vision_mng_battery_alarm_thresholds *thresholds_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_battery_alarm_thresholds_get(struct vision_mng_battery_alarm_thresholds *thresholds_ptr)

Remarks:			This function retrieves the thresholds for battery alarm system.
					
							Parameter		 	   		Description
							--------------------------------------------------------------------------------
							thresholds_ptr         Specifies the pointer to the vision_mng_fan_auto_ctr_cfg
												   structure where the threshold values will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 		   		Description
							--------------------------------------------------------------------------------
							= VISION_MNG_STATUS_OK 		No Errors		(Success)
							< 0					   		Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_battery_alarm_thresholds_get(struct vision_mng_battery_alarm_thresholds *thresholds_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_card_inventory(unsigned int card_i, struct vision_mng_card_info *info_ptr)

Remarks:			This function retrieves the detailed information about expansion board 
					according to its ID.
					
							Parameter		  			Description
							----------------------------------------------------------------
							card_id		    Specifies expansion card id
							
							info_ptr        Specifies pointer to vision_mng_card_info structure 
											where the info will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 	  			Description
							-----------------------------------------------------------------
							= VISION_MNG_STATUS_OK	  	No Errors		(Success)
							< 0				  	Error Code 		(Failure)
												  VISION_MNG_STATUS_INV_ARG
												  VISION_MNG_STATUS_RESET
												  VISION_MNG_STATUS_TIMEOUT
												  VISION_MNG_STATUS_NO_CRD_RESPONSE
												  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_card_inventory(unsigned int card_id, struct vision_mng_card_info *info_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_enable(enum vision_mng_port port, enum vision_mng_bool enable)

Remarks:			This function enables or disables specified shelf Ethernet port.
					
							Parameter		   	Description
							------------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
																	
							enable	       VISION_MNG_TRUE to enable the port.
								  		   VISION_MNG_FALSE to disable the port.							                                        

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		   				Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_enable(enum vision_mng_port port, enum vision_mng_bool enable);


/*********************************************************************************************
Syntax:				 int vision_mng_ethport_status(enum vision_mng_port port, 
                                                   enum vision_mng_port_status *status_ptr)

Remarks:			This function retrieves specified shelf Ethernet port status.
					
							Parameter		 				Description
							------------------------------------------------------------------
							port            Specifies a single Ethernet port.
																	
							status_ptr	    Specifies pointer to buffer
											where the status will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 					Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK				No Errors		(Success)
							< 0									Error Code 		(Failure)
																  VISION_MNG_STATUS_INV_ARG
																  VISION_MNG_STATUS_RESET
																  VISION_MNG_STATUS_TIMEOUT
																  VISION_MNG_STATUS_NO_CRD_RESPONSE
																  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_status(enum vision_mng_port port, enum vision_mng_port_status *status_ptr);


/*********************************************************************************************
Syntax:				int vision_mng_ethport_speed_set(enum vision_mng_port port, 
                                                     enum vision_mng_eth_speed speed)

Remarks:			This function sets speed for the specified shelf Ethernet port.
					If the port is in auto-negotiation mode the command will be ignored.
					
							Parameter		 							Description
							--------------------------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
																	
							speed		   Specifies selected speed.							                                        

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							--------------------------------------------------------------------------------
							= VISION_MNG_STATUS_OK									No Errors		(Success)
							< 0												Error Code 		(Failure)
																			  VISION_MNG_STATUS_INV_ARG
																			  VISION_MNG_STATUS_RESET
																			  VISION_MNG_STATUS_TIMEOUT
																			  VISION_MNG_STATUS_NO_CRD_RESPONSE
																			  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_speed_set(enum vision_mng_port port, enum vision_mng_eth_speed speed);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_speed_get(enum vision_mng_port port, 
                                                     enum vision_mng_eth_speed *speed_ptr)

Remarks:			This function retrieves speed of the specified shelf Ethernet port.
					
							Parameter		 		 	Description
							------------------------------------------------------------------
							port              Specifies a single Ethernet port.
																	
							speed_ptr 		  Specifies pointer to the buffer where the
											  speed will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							--------------------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_speed_get(enum vision_mng_port port, enum vision_mng_eth_speed *speed_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_autoneg_enable(enum vision_mng_port port, 
                                                          enum vision_mng_bool enable)

Remarks:			This function enables or disables auto-negotiation mechanism.
					
							Parameter		 							Description
							------------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
																	
							enable			VISION_MNG_TRUE enables auto-negotiation.
											VISION_MNG_FALSE disables auto-negotiation

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				  Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		  No Errors		(Success)
							< 0							  Error Code 		(Failure)
														  	  VISION_MNG_STATUS_INV_ARG
														  	  VISION_MNG_STATUS_RESET
														  	  VISION_MNG_STATUS_TIMEOUT
														  	  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  	  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_autoneg_enable(enum vision_mng_port port, enum vision_mng_bool enable);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_autoneg_restart(enum vision_mng_port port)

Remarks:			This function forces auto-negotiation to begin link renegotiation.
					If auto-negotiation is disabled, the API has no effect
					
							Parameter		 							Description
							------------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 					Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_autoneg_restart(enum vision_mng_port port);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_autoneg_status(enum vision_mng_port port, 
                                                  enum vision_mng_autoneg_status *status_ptr)

Remarks:			This function forces auto-negotiation to begin link renegotiation.
					If auto-negotiation is disabled, the API has no effect
					
							Parameter		 	  	Description
							-----------------------------------------------------------------
							port        		  Specifies a single Ethernet port.
							
							status_ptr            Specifies pointer to buffer where the 
												  status  will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_autoneg_status(enum vision_mng_port port, enum vision_mng_autoneg_status *status_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_autoneg_capabilities_set(enum vision_mng_port port,
                                                              unsigned int capabilities_mask)

Remarks:			This function configures the set of capabilities of the local auto-negotiation
					entity.
										
							Parameter		 		 	Description
							-------------------------------------------------------------------
							port                    Specifies Ethernet port to which the configuration 
							                         will be applied.
							
							capabilities_mask       Specifies bit mask of local capabilities:							
														VISION_MNG_ANC_10BT    
														VISION_MNG_ANC_10BTFD  
														VISION_MNG_ANC_100BTX  
														VISION_MNG_ANC_100BTXFD
														VISION_MNG_ANC_1000BT  
														VISION_MNG_ANC_1000BTFD
														VISION_MNG_ANC_1000BX  
														VISION_MNG_ANC_1000BXFD
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_autoneg_capabilities_set(enum vision_mng_port port, unsigned int capabilities_mask);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_autoneg_capabilities_get(enum vision_mng_port port, 
                                                          unsigned int *capabilities_mask_prt,
														  enum vision_mng_bool remote)

Remarks:			This function retrieves the set of capabilities of the local or remote
					auto-negotiation entity.
										
							Parameter		 				Description
							-------------------------------------------------------------------
							port        				Specifies a single Ethernet port.
							
							capabilities_mask_ptr       Specifies a pointer to the buffer were the
														bit mask of local capabilities will be stored.
																	
							remote        				Specifies a flag. If VISION_MNG_TRUE, the API retrieves
														the capabilities supported by remote entity.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 						Description
							---------------------------------------------------------------------
							= VISION_MNG_STATUS_OK				No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_autoneg_capabilities_get(enum vision_mng_port port, 
											 unsigned int *capabilities_mask_prt,
											 enum vision_mng_bool remote);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_sfp_manufacture_info(enum vision_mng_port port, 
                                                  struct vision_mng_sfp_vendor_info *info_ptr)

Remarks:			This function retrieves the set of capabilities of the local or remote
					auto-negotiation entity.
										
							Parameter		 		Description
							------------------------------------------------------------------
							port        	Specifies a single Ethernet port.
							
							info_ptr        Specifies a pointer to the vision_mng_sfp_vendor_info
											where the info will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_sfp_manufacture_info(enum vision_mng_port port, struct vision_mng_sfp_vendor_info *info_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_jack_type(enum vision_mng_port port, 
                                                     enum vision_mng_jack_type *jack_type_ptr)

Remarks:			This function retrieves the type of the jack attached to the interface.
					
							Parameter		 		 	Description
							------------------------------------------------------------------
							port        			Specifies a single Ethernet port.
																	
							jack_type_ptr			Specifies pointer to buffer
													where the jack type will be stored.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_jack_type(enum vision_mng_port port, enum vision_mng_jack_type *jack_type_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_chs_eth_port_statistics(enum vision_mng_port port, 
                                                    struct vision_mng_ethport_stats *stats_ptr)

Remarks:			This function retrieves specified shelf Ethernet port statistics.
					
							Parameter		 		Description
							------------------------------------------------------------------
							port         		Specifies a single Ethernet port.
																	
							stats_ptr			Specifies pointer to vision_mng_ethport_stats
												structure where the status will be stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_statistics(enum vision_mng_port port, struct vision_mng_ethport_stats *stats_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_enable(enum vision_mng_port port, 
                                                      enum vision_mng_bool remote)

Remarks:			This function enables or disables Power over Ethernet capabilities
					for the specified Ethernet port or group of ports.
										
							Parameter		 		  Description
							------------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
							
							enable      VISION_MNG_TRUE enables the capability.
								  		VISION_MNG_FALSE disables the capability
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_enable(enum vision_mng_port port, enum vision_mng_bool enable);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_status(enum vision_mng_port port, 
                                             enum vision_mng_ethport_poe_status *status_ptr)

Remarks:			This function retrieves Power over Ethernet status
					for the specified Ethernet port.
										
							Parameter		 			Description
							------------------------------------------------------------------
							port         	 Specifies a single Ethernet port.
																	
							stats_ptr		 Specifies pointer to the buffer
											 where the status will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_status(enum vision_mng_port port, enum vision_mng_ethport_poe_status *status_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_alarm_cfg_set(enum vision_mng_port port, 
                                                    struct vision_mng_poe_alarm_cfg *cfg_ptr)

Remarks:			This function configures Power over Ethernet alarm conditions
					for the specified Ethernet port or group of ports.
										
							Parameter	   			Description
							-----------------------------------------------------------------
							port           Specifies Ethernet port to which the configuration 
							               will be applied.
							
							cfg_ptr        Specifies a pointer to the vision_mng_poe_alarm_cfg
										   structure, contains the configuration to set
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_alarm_cfg_set(enum vision_mng_port port, struct vision_mng_poe_alarm_cfg *cfg_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_alarm_cfg_get(enum vision_mng_port port, 
                                                     struct vision_mng_poe_alarm_cfg *cfg_ptr);

Remarks:			This function retrieves Power over Ethernet alarm configuration
					for the specified Ethernet port.
										
							Parameter		 		 Description
							-----------------------------------------------------------------
							port         	Specifies a single Ethernet port.
																	
							cfg_ptr			Specifies pointer to the vision_mng_poe_alarm_cfg
                                            structure where the retrieved configuration
											will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_alarm_cfg_get(enum vision_mng_port port, struct vision_mng_poe_alarm_cfg *cfg_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_info(enum vision_mng_port port, 
                                                    struct vision_mng_poe_info *info)

Remarks:			This function retrieves Power over Ethernet advanced information
					for the specified Ethernet port.
										
							Parameter		 			Description
							-------------------------------------------------------------------
							port         			Specifies a single Ethernet port.
																	
							info_ptr				Specifies pointer to the vision_mng_poe_info
													structure where the retrieved information
													will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_info(enum vision_mng_port port, struct vision_mng_poe_info *info_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_ethport_poe_alarm(enum vision_mng_port port, 
                                                     struct vision_mng_poe_alarm *alarm_ptr)

Remarks:			This function retrieves Power over Ethernet alarms
					for the specified Ethernet port.
										
							Parameter		 		 Description
							------------------------------------------------------------------
							port         	Specifies a single Ethernet port.
															
							alarm_ptr		Specifies pointer to the vision_mng_poe_alarm
											structure where the retrieved information
											will be stored.
																	
Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 				Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		No Errors		(Success)
							< 0							Error Code 		(Failure)
														  VISION_MNG_STATUS_INV_ARG
														  VISION_MNG_STATUS_RESET
														  VISION_MNG_STATUS_TIMEOUT
														  VISION_MNG_STATUS_NO_CRD_RESPONSE
														  VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_ethport_poe_alarm(enum vision_mng_port port, struct vision_mng_poe_alarm *alarm_ptr);

/*********************************************************************************************
Syntax:				int vision_mng_firmware_upgrade(int fd_device, int fd_firmware)

Remarks:			This function performs the firmware upgrade capabilities.
					
							Parameter		 	 		Description
							------------------------------------------------------------------
							fd_device         	 Specifies serial port file descriptor.
												 For example result of call to
												 open("/dev/ttyUSB0").
																	
							fd_firmware	  		 file descriptor of a file containing the firmware,
												 for example result of call to
												 open("/system/shelf/interface.bin")

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

							Value		 									Description
							-------------------------------------------------------------------
							= VISION_MNG_STATUS_OK		 No Errors		(Success)
							< 0					 Error Code 		(Failure)
													 VISION_MNG_STATUS_INV_ARG
													 VISION_MNG_STATUS_RESET
													 VISION_MNG_STATUS_TIMEOUT
													 VISION_MNG_STATUS_NO_CRD_RESPONSE
													 VISION_MNG_STATUS_NO_DEV_RESPONSE
*********************************************************************************************/
extern int vision_mng_firmware_upgrade(int fd_device, int fd_firmware);

/*********************************************************************************************
Syntax:				void vision_mng_sync_timeout_set(unsigned int timeout)

Remarks:			This function modifies synchronization timeout

						Parameter		  		Description
						--------------------------------------------------------------------
						timeout 		  Specifies the timeout value in  milliseconds.

Return Value:	    None.

*********************************************************************************************/
extern void vision_mng_sync_timeout_set(unsigned int timeout);

/*********************************************************************************************
Syntax:				int vision_mng_sync_timeout_get(unsigned int *timeout_ptr)

Remarks:			This function retrieves synchronization timeout

						Parameter				Description
						---------------------------------------------------------------------
						timeout_ptr		Specifies the pointer to the buffer where
								   		the timeout value in milliseconds will be
								   		stored.

Return Value:	Returns VISION_MNG_STATUS_OK on success, or an error code on failure.

						Value	   				Description
						---------------------------------------------------------------------
						= VISION_MNG_STATUS_OK		No Errors		(Success)
						< 0				  		    Error Code 		(Failure)
										  			VISION_MNG_STATUS_INV_ARG

*********************************************************************************************/
extern int vision_mng_sync_timeout_get(unsigned int *timeout_ptr);


#endif /* _VISION_MNG_H */
