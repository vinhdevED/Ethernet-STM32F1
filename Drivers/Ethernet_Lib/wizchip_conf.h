#ifndef _WIZCHIP_CONF_H
#define _WIZCHIP_CONF_H


#include <stdint.h>

#define W5500			5500

#ifndef _WIZCHIP_
#define _WIZCHIP_		5500
#endif


/*
 * These macros define different I/O modes for a WIZCHIP,
 * specifying whether it will communicate via a bus interface - spi inteface
 */
#define _WIZCHIP_IO_MODE_NONE_			0x0000		/* <NONE MODE> */
#define _WIZCHIP_IO_MODE_BUS_			0x0100		/* <BUS INTERFACE MODE> */
#define _WIZCHIP_IO_MODE_SPI_			0x0200		/* <SPI INTERFACE MODE> */
//#define _WIZCHIP_IO_MODE_IIC_          0x0400
//#define _WIZCHIP_IO_MODE_SDIO_         0x0800

#define _WIZCHIP_IO_MODE_BUS_DIR_      (_WIZCHIP_IO_MODE_BUS_ + 1) 			/* <BUS DIRECT - 0x0101 (Value)>  */
#define _WIZCHIP_IO_MODE_BUS_INDIR_    (_WIZCHIP_IO_MODE_BUS_ + 2)			/* <BUS INDRECT - 0x0102 (Value)> */
#define _WIZCHIP_IO_MODE_SPI_VDM_      (_WIZCHIP_IO_MODE_SPI_ + 1)			/* <SPI VARIABLE DATA MODE -0x0201 (Value)> */
#define _WIZCHIP_IO_MODE_SPI_FDM_      (_WIZCHIP_IO_MODE_SPI_ + 2)			/* <SPI FIXED LENGTH DATA MODE - 0x0202 (Value)> */
#define _WIZCHIP_IO_MODE_SPI_5500_     (_WIZCHIP_IO_MODE_SPI_ + 2)			/* <SPI FIXED LENGTH DATA MODE - 0x0203 (Value)> */


#if (_WIZCHIP_ == 5500)
	#define _WIZCHIP_ID_			"W5500\0"

	/*
	 *  Define interface mode.
	 *  Should select interface mode as chip.
	 */
	#ifndef _WIZCHIP_IO_MODE_
		#define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_SPI_VDM_
	#endif

	typedef uint8_t iodata_t;		/*Define the unit of IO DATA.*/
	#include "W5500/w5500.h"

#else
	#error "Unknown defined _WIZCHIP_. You should define 5500"
#endif


#ifndef _WIZCHIP_IO_MODE_
   #error "Undefined _WIZCHIP_IO_MODE_. You should define it !!!"
#endif


/*
 *  Define I/O base address when BUS IF mode
 *  which is used to define the base address for accessing the I/O registers of a WIZnet chip
 *  based on the communication mode being used.
 */
#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_BUS_
	#define _WIZCHIP_IO_BASE_				0x60000000
#elif _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
	#define _WIZCHIP_IO_BASE_				0x00000000
#else
    #error "You should define _WIZCHIP_IO_BASE_ to fit your system memory map."
#endif


/*
 *  The chip has 8 independent sockets available for network communication.
 */
#define _WIZCHIP_SOCK_NUM_   		8



/********************************************************
* WIZCHIP BASIC IF functions for SPI, SDIO, I2C , ETC.
*********************************************************/

typedef struct _WIZCHIP
{
	uint16_t if_mode;					/// < Host interface mode
	uint8_t id[6];						/// < Holds the ID of the WIZnet chip

	/*
	 * The set of critical section callback func.
	 */
	struct _CRIS
    {
		void (*_enter) (void);       	/// < crtical section enter
		void (*_exit) (void);         	/// < critial section exit
    }CRIS;

    /*
     * The set of select control callback func.
     */
    struct _CS
    {
        void (*_select)  (void);      	/// < _WIZCHIP_ selected
        void (*_deselect)(void);      	/// < _WIZCHIP_ deselected
    }CS;

    union _IF
    {
    	/*
		 * For BUS interface IO
		 */
    	struct
		{
			uint8_t  (*_read_byte)  (uint32_t AddrSel);
			void     (*_write_byte) (uint32_t AddrSel, uint8_t wb);
		}BUS;

	    /*
	     * For SPI interface IO
	     */
	    struct
		{
		 uint8_t (*_read_byte)   (void);
		 void    (*_write_byte)  (uint8_t wb);
		}SPI;
    }IF;


}_WIZCHIP;


extern _WIZCHIP WIZCHIP;


/* -----------------------------------------------------
 * WIZCHIP control type enumration used in ctlwizchip().
 * -----------------------------------------------------
 */
typedef enum {
	CW_RESET_WIZCHIP,   ///< Resets WIZCHIP by softly
	CW_INIT_WIZCHIP,    ///< Inializes to WIZCHIP with SOCKET buffer size 2 or 1 dimension array typed uint8_t.
	CW_GET_INTERRUPT,   ///< Get Interrupt status of WIZCHIP
	CW_CLR_INTERRUPT,   ///< Clears interrupt
	CW_SET_INTRMASK,    ///< Masks interrupt
	CW_GET_INTRMASK,    ///< Get interrupt mask
	CW_SET_INTRTIME,    ///< Set interval time between the current and next interrupt.
	CW_GET_INTRTIME,    ///< Set interval time between the current and next interrupt.
	CW_GET_ID,          ///< Gets WIZCHIP name.
#if _WIZCHIP_ ==  5500
	CW_RESET_PHY,       ///< Resets internal PHY. Valid Only W5000
	CW_SET_PHYCONF,     ///< When PHY configured by interal register, PHY operation mode (Manual/Auto, 10/100, Half/Full). Valid Only W5000
	CW_GET_PHYCONF,     ///< Get PHY operation mode in interal register. Valid Only W5000
	CW_GET_PHYSTATUS,   ///< Get real PHY status on operating. Valid Only W5000
	CW_SET_PHYPOWMODE,  ///< Set PHY power mode as noraml and down when PHYSTATUS.OPMD == 1. Valid Only W5000
#endif
	CW_GET_PHYPOWMODE,  ///< Get PHY Power mode as down or normal
	CW_GET_PHYLINK      ///< Get PHY Link status

}ctlwizchip_type;



/* -----------------------------------------------------
 * Network control type enumration used in ctlnetwork().
 * -----------------------------------------------------
 */
typedef enum
{
	CN_SET_NETINFO,  ///< Set Network with @ref wiz_NetInfo
	CN_GET_NETINFO,  ///< Get Network with @ref wiz_NetInfo
	CN_SET_NETMODE,  ///< Set network mode as WOL, PPPoE, Ping Block, and Force ARP mode - <MODE REGISTER>
	CN_GET_NETMODE,  ///< Get network mode as WOL, PPPoE, Ping Block, and Force ARP mode - <MODE REGISTER>
	CN_SET_TIMEOUT,  ///< Set network timeout as retry count and time.
	CN_GET_TIMEOUT,  ///< Get network timeout as retry count and time.
}ctlnetwork_type;


/* ----------------------------------------------------------------------
 * These flags are associated with the Socket Interrupt Register (SIR) and the Interrupt Register (IR)
 * It can be used with OR operation
 * ----------------------------------------------------------------------
 */
typedef enum {
	/* ------------------------Interrupt Register------------------------------------
	 * It indicates whether any global interrupt (like WOL, IP conflict) has occurred.
	 * Bits 3-0: Reserved (usually 0)
	 */
	IK_WOL               	= (1 << 4),		///< Wake On Lan by receiving the magic packet
	IK_PPPOE_TERMINATED  	= (1 << 5),		///< PPPoE Disconnected
	IK_DEST_UNREACH      	= (1 << 6),   	///< Destination IP & Port Unreable
	IK_IP_CONFLICT       	= (1 << 7),   	///< IP conflict occurred

	/* ------------------------Sọcket Interrupt Register------------------------------------
	 * It shows which specific socket has triggered an interrupt.
	 */
	IK_SOCK_0            	= (1 << 8),   	///< Socket 0 interrupt
	IK_SOCK_1            	= (1 << 9),   	///< Socket 1 interrupt
	IK_SOCK_2            	= (1 << 10),  	///< Socket 2 interrupt
	IK_SOCK_3            	= (1 << 11),  	///< Socket 3 interrupt
	IK_SOCK_4            	= (1 << 12),  	///< Socket 4 interrupt
	IK_SOCK_5            	= (1 << 13),  	///< Socket 5 interrupt
	IK_SOCK_6            	= (1 << 14),  	///< Socket 6 interrupt
	IK_SOCK_7            	= (1 << 15),  	///< Socket 7 interrupt

	IK_SOCK_ALL          	= (0xFF << 8) 	///< All Socket interrpt
}intr_kind;


#define PHY_CONFBY_HW            0     ///< Configured PHY operation mode by HW pin
#define PHY_CONFBY_SW            1     ///< Configured PHY operation mode by SW register
#define PHY_MODE_MANUAL          0     ///< Configured PHY operation mode with user setting.
#define PHY_MODE_AUTONEGO        1     ///< Configured PHY operation mode with auto-negotiation
#define PHY_SPEED_10             0     ///< Link Speed 10
#define PHY_SPEED_100            1     ///< Link Speed 100
#define PHY_DUPLEX_HALF          0     ///< Link Half-Duplex
#define PHY_DUPLEX_FULL          1     ///< Link Full-Duplex
#define PHY_LINK_OFF             0     ///< Link Off
#define PHY_LINK_ON              1     ///< Link On
#define PHY_POWER_NORM           0     ///< PHY power normal mode
#define PHY_POWER_DOWN           1     ///< PHY power down mode



/* --------------------------------------------------------------------------------------
 * It is used to configure and manage the physical layer (PHY) settings of a WIZnet chip.
 * It configures PHY configuration when CW_SET PHYCONF or CW_GET_PHYCONF in W5500.
 * --------------------------------------------------------------------------------------
 */
typedef struct wiz_PhyConf_t
{
	uint8_t by;       ///< set by @ref PHY_CONFBY_HW or @ref PHY_CONFBY_SW
	uint8_t mode;     ///< set by @ref PHY_MODE_MANUAL or @ref PHY_MODE_AUTONEGO
	uint8_t speed;    ///< set by @ref PHY_SPEED_10 or @ref PHY_SPEED_100
	uint8_t duplex;   ///< set by @ref PHY_DUPLEX_HALF @ref PHY_DUPLEX_FULL
	//uint8_t power;  ///< set by @ref PHY_POWER_NORM or @ref PHY_POWER_DOWN
	//uint8_t link;   ///< Valid only in CW_GET_PHYSTATUS. set by @ref PHY_LINK_ON or PHY_DUPLEX_OFF
}wiz_PhyConf;


/* -------------------------------------------------
 * It used in setting dhcp_mode of @ref wiz_NetInfo.
 * -------------------------------------------------
 */
typedef enum
{
   NETINFO_STATIC = 1,    ///< Static IP configuration by manually.
   NETINFO_DHCP           ///< Dynamic IP configruation from a DHCP sever
}dhcp_mode;


/* --------------------------------
 * Network Information for WIZCHIP.
 * --------------------------------
 */
typedef struct wiz_NetInfo_t
{
   uint8_t mac[6];  ///< Source Mac Address - Source Hardware Address
   uint8_t ip[4];   ///< Source IP Address
   uint8_t sn[4];   ///< Subnet Mask
   uint8_t gw[4];   ///< Gateway IP Address
   uint8_t dns[4];  ///< DNS server IP Address
   dhcp_mode dhcp;  ///< 1 - Static, 2 - DHCP
}wiz_NetInfo;


/*---------------------------------------------
 *In Mode Register
 * Network Mode
 *  7  - 6 -  5  -  4  -    3    - 2 -  1   - 0
 * RST - 0 - WOL - PB  -  PPPoE  - 0 - FARP - 0
 * --------------------------------------------
 */
typedef enum {
#if (_WIZCHIP_ == 5500)
	NM_FORCEARP = (1<<1),		///< Force to APP send whenever udp data is sent. Valid only in W5500
#endif
	NM_WAKEONLAN   = (1<<5),  	///< Wake On Lan
	NM_PINGBLOCK   = (1<<4),  	///< Block ping-request
	NM_PPPOE       = (1<<3),  	///< PPPoE mode
}netmode_type;


/*  ----------------------------------------------------------------------------------------
 *  Used in CN_SET_TIMEOUT or CN_GET_TIMEOUT of @ref ctlwizchip() for timeout configruation.
 *  ----------------------------------------------------------------------------------------
 */
typedef struct wiz_NetTimeout_t
{
   uint8_t  retry_cnt;     ///< retry count
   uint16_t time_100us;    ///< time unit 100us
}wiz_NetTimeout;


/* -----------------------------------------------------------------------------------
 * Registers call back function for critical section of I/O functions such as
 * WIZCHIP_READ, @ref WIZCHIP_WRITE, @ref WIZCHIP_READ_BUF and @ref WIZCHIP_WRITE_BUF.
 * -----------------------------------------------------------------------------------
 */
void reg_wizchip_cris_cbfunc(void(*cris_en)(void), void(*cris_ex)(void));

/* -----------------------------------------------------------------------------------
 * Registers call back function for WIZCHIP select & deselect.
 * -----------------------------------------------------------------------------------
 */
void reg_wizchip_cs_cbfunc(void(*cs_sel)(void), void(*cs_desel)(void));

void reg_wizchip_bus_cbfunc(uint8_t (*bus_rb)(uint32_t addr), void (*bus_wb)(uint32_t addr, uint8_t wb));

void reg_wizchip_spi_cbfunc(uint8_t (*spi_rb)(void), void (*spi_wb)(uint8_t wb));


/* --------------------------------------------------------------------------------------------------------
 * @brief 	Controls to the WIZCHIP.
 * @details Resets WIZCHIP & internal PHY, Configures PHY mode, Monitor PHY(Link,Speed,Half/Full/Auto),
 * controls interrupt & mask and so on.
 * @param 	cwtype : Decides to the control type
 * @param 	arg : arg type is dependent on cwtype.
 * @return  0 : Success \n
 *         	-1 : Fail because of invalid \ref ctlwizchip_type or unsupported \ref ctlwizchip_type in WIZCHIP
 * --------------------------------------------------------------------------------------------------------
 */
int8_t ctlwizchip(ctlwizchip_type cwtype, void* arg);



/* --------------------------------------------------------------------------------------------------
 * @brief 	Controls to network.
 * @details Controls to network environment, mode, timeout and so on.
 * @param 	cntype : Input. Decides to the control type
 * @param 	arg : Inout. arg type is dependent on cntype.
 * @return 	-1 : Fail because of invalid ctlnetwork_type or unsupported ctlnetwork_type in WIZCHIP \n
 *         	 0 : Success
 * --------------------------------------------------------------------------------------------------
 */
int8_t ctlnetwork(ctlnetwork_type cntype, void* arg);



/*  ------------------------
 *  Reset WIZCHIP by softly.
 *  ------------------------
 */
void   wizchip_sw_reset(void);


/* --------------------------------------------------------------------------------
 * @brief 	Initializes WIZCHIP with socket buffer size
 * @param 	txsize Socket tx buffer sizes. If null, initialized the default size 2KB.
 * @param 	rxsize Socket rx buffer sizes. If null, initialized the default size 2KB.
 * @return 	0 : succcess \n
 *        	-1 : fail. Invalid buffer size
 * (You need to make sure that the total transmit and receive buffer size
 *  for each socket does not exceed the buffer size supported by the chip)
 *  -------------------------------------------------------------------------------
 */
int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize);



/* -----------------------------------------------------------------------------
 * @brief Clear Interrupt of WIZCHIP.
 * @param intr : @ref intr_kind value operated OR. It can type-cast to uint16_t.
 * -----------------------------------------------------------------------------
 */
void wizchip_clrinterrupt(intr_kind intr);



/* -----------------------------------------------------------------------
 * @brief Get Interrupt of WIZCHIP.
 * @return @ref intr_kind value operated OR. It can type-cast to uint16_t.
 * -----------------------------------------------------------------------
 */
intr_kind wizchip_getinterrupt(void);


/* -----------------------------------------------------------------------------
 * @brief Mask or Unmask Interrupt of WIZCHIP.
 * @param intr : @ref intr_kind value operated OR. It can type-cast to uint16_t.
 * -----------------------------------------------------------------------------
 */
void wizchip_setinterruptmask(intr_kind intr);


/* --------------------------------------------------------------------------------
 * @brief Get Interrupt mask of WIZCHIP.
 * @return : The operated OR vaule of @ref intr_kind. It can type-cast to uint16_t.
 * --------------------------------------------------------------------------------
 */
intr_kind wizchip_getinterruptmask(void);



#if _WIZCHIP_ > 5100
   int8_t wizphy_getphylink(void);              ///< get the link status of phy in WIZCHIP. No use in W5100
   int8_t wizphy_getphypmode(void);             ///< get the power mode of PHY in WIZCHIP. No use in W5100
#endif

#if _WIZCHIP_ == 5500
   void   wizphy_reset(void);                   ///< Reset phy. Vailid only in W5500

/**
 * @brief Set the phy information for WIZCHIP without power mode
 * @param phyconf : @ref wiz_PhyConf
 */
   void   wizphy_setphyconf(wiz_PhyConf* phyconf);


 /**
 * @brief Get phy configuration information.
 * @param phyconf : @ref wiz_PhyConf
 */
   void   wizphy_getphyconf(wiz_PhyConf* phyconf);


 /*
 * @brief Get phy status.
 * @param phyconf : @ref wiz_PhyConf
 */
   void   wizphy_getphystat(wiz_PhyConf* phyconf);

	/*
	* @brief set the power mode of phy inside WIZCHIP. Refer to @ref PHYCFGR in W5500, @ref PHYSTATUS in W5200
	* @param pmode Settig value of power down mode.
	*/
   int8_t wizphy_setphypmode(uint8_t pmode);
#endif


/* ----------------------------------------------
 * @brief Set the network information for WIZCHIP
 * @param pnetinfo : @ref wizNetInfo
 * ----------------------------------------------
*/
void wizchip_setnetinfo(wiz_NetInfo* pnetinfo);

/*
* @brief Get the network information for WIZCHIP
* @param pnetinfo : @ref wizNetInfo
*/
void wizchip_getnetinfo(wiz_NetInfo* pnetinfo);

/*
* @brief Set the network mode such WOL, PPPoE, Ping Block, and etc.
* @param pnetinfo Value of network mode. Refer to @ref netmode_type.
*/
int8_t wizchip_setnetmode(netmode_type netmode);

/*
* @brief Get the network mode such WOL, PPPoE, Ping Block, and etc.
* @return Value of network mode. Refer to @ref netmode_type.
*/
netmode_type wizchip_getnetmode(void);

/*
* @brief Set retry time value(@ref RTR) and retry count(@ref RCR).
* @details @ref RTR configures the retransmission timeout period and @ref RCR configures the number of time of retransmission.
* @param nettime @ref RTR value and @ref RCR value. Refer to @ref wiz_NetTimeout.
*/
void wizchip_settimeout(wiz_NetTimeout* nettime);

/*
* @brief Get retry time value(@ref RTR) and retry count(@ref RCR).
* @details @ref RTR configures the retransmission timeout period and @ref RCR configures the number of time of retransmission.
* @param nettime @ref RTR value and @ref RCR value. Refer to @ref wiz_NetTimeout.
*/
void wizchip_gettimeout(wiz_NetTimeout* nettime);

#endif	// _WIZCHIP_CONF_H_


