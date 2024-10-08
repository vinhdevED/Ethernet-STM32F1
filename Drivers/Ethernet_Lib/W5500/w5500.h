#ifndef  _W5500_H_
#define  _W5500_H_

#include <stdint.h>
#include "../wizchip_conf.h"


#define _W5500_IO_BASE_              	0x00000000

#define _W5500_SPI_READ_			   	(0x00 << 2) 	//< SPI interface Read operation in Control Phase
#define _W5500_SPI_WRITE_			   	(0x01 << 2) 	//< SPI interface Write operation in Control Phase

#define WIZCHIP_CREG_BLOCK          	0x00 			//< Common register block
#define WIZCHIP_SREG_BLOCK(N)       	(1+4*N) 		//< Socket N register block
#define WIZCHIP_TXBUF_BLOCK(N)      	(2+4*N) 		//< Socket N Tx buffer address block
#define WIZCHIP_RXBUF_BLOCK(N)      	(3+4*N) 		//< Socket N Rx buffer address bloc

#define WIZCHIP_OFFSET_INC(ADDR, N)    	(ADDR + (N<<8)) //< Increase offset address

///////////////////////////////////////
// Definition For Legacy Chip Driver //
///////////////////////////////////////
#define IINCHIP_READ(ADDR)                WIZCHIP_READ(ADDR)               ///< The defined for legacy chip driver
#define IINCHIP_WRITE(ADDR,VAL)           WIZCHIP_WRITE(ADDR,VAL)          ///< The defined for legacy chip driver
#define IINCHIP_READ_BUF(ADDR,BUF,LEN)    WIZCHIP_READ_BUF(ADDR,BUF,LEN)   ///< The defined for legacy chip driver
#define IINCHIP_WRITE_BUF(ADDR,BUF,LEN)   WIZCHIP_WRITE(ADDR,BUF,LEN)      ///< The defined for legacy chip driver

/**
 * @defgroup WIZCHIP_register WIZCHIP register
 * @ingroup W5500
 *
 * @brief WHIZCHIP register defines register group of @b W5500.
 *
 * - @ref Common_register_group : Common register group
 * - @ref Socket_register_group : \c SOCKET n register group
 */

/**
 * @defgroup WIZCHIP_IO_Functions WIZCHIP I/O functions
 * @ingroup W5500
 *
 * @brief This supports the basic I/O functions for @ref WIZCHIP_register.
 *
 * - <b> Basic I/O function </b> \n
 *   WIZCHIP_READ(), WIZCHIP_WRITE(), WIZCHIP_READ_BUF(), WIZCHIP_WRITE_BUF() \n\n
 *
 * - @ref Common_register_group <b>access functions</b> \n
 * 	-# @b Mode \n
 *    getMR(), setMR()
 * 	-# @b Interrupt \n
 *    getIR(), setIR(), getIMR(), setIMR(), getSIR(), setSIR(), getSIMR(), setSIMR(), getINTLEVEL(), setINTLEVEL()
 * 	-# <b> Network Information </b> \n
 *    getSHAR(), setSHAR(), getGAR(), setGAR(), getSUBR(), setSUBR(), getSIPR(), setSIPR()
 * 	-# @b Retransmission \n
 *    getRCR(), setRCR(), getRTR(), setRTR()
 * 	-# @b PPPoE \n
 *    getPTIMER(), setPTIMER(), getPMAGIC(), getPMAGIC(), getPSID(), setPSID(), getPHAR(), setPHAR(), getPMRU(), setPMRU()
 * 	-# <b> ICMP packet </b>\n
 *    getUIPR(), getUPORTR()
 * 	-# @b etc. \n
 *    getPHYCFGR(), setPHYCFGR(), getVERSIONR() \n\n
 *
 * - \ref Socket_register_group <b>access functions</b> \n
 *   -# <b> SOCKET control</b> \n
 *      getSn_MR(), setSn_MR(), getSn_CR(), setSn_CR(), getSn_IMR(), setSn_IMR(), getSn_IR(), setSn_IR()
 *   -# <b> SOCKET information</b> \n
 *      getSn_SR(), getSn_DHAR(), setSn_DHAR(), getSn_PORT(), setSn_PORT(), getSn_DIPR(), setSn_DIPR(), getSn_DPORT(), setSn_DPORT()
 *      getSn_MSSR(), setSn_MSSR()
 *   -# <b> SOCKET communication </b> \n
 *      getSn_RXBUF_SIZE(), setSn_RXBUF_SIZE(), getSn_TXBUF_SIZE(), setSn_TXBUF_SIZE() \n
 *      getSn_TX_RD(), getSn_TX_WR(), setSn_TX_WR() \n
 *      getSn_RX_RD(), setSn_RX_RD(), getSn_RX_WR() \n
 *      getSn_TX_FSR(), getSn_RX_RSR(), getSn_KPALVTR(), setSn_KPALVTR()
 *   -# <b> IP header field </b> \n
 *      getSn_FRAG(), setSn_FRAG(),  getSn_TOS(), setSn_TOS() \n
 *      getSn_TTL(), setSn_TTL()
 */


/**
 * @defgroup Common_register_group Common register
 * @ingroup WIZCHIP_register
 *
 * @brief Common register group\n
 * It set the basic for the networking\n
 * It set the configuration such as interrupt, network information, ICMP, etc.
 * @details
 * @sa MR : Mode register.
 * @sa GAR, SUBR, SHAR, SIPR
 * @sa INTLEVEL, IR, IMR, SIR, SIMR : Interrupt.
 * @sa RTR, RCR : Data retransmission.
 * @sa PTIMER, PMAGIC, PHAR, PSID, PMRU : PPPoE.
 * @sa UIPR, UPORTR : ICMP message.
 * @sa PHYCFGR, VERSIONR : etc.
 */


/**
 * @defgroup Socket_register_group Socket register
 * @ingroup WIZCHIP_register
 *
 * @brief Socket register group.\n
 * Socket register configures and control SOCKETn which is necessary to data communication.
 * @details
 * @sa Sn_MR, Sn_CR, Sn_IR, Sn_IMR : SOCKETn Control
 * @sa Sn_SR, Sn_PORT, Sn_DHAR, Sn_DIPR, Sn_DPORT : SOCKETn Information
 * @sa Sn_MSSR, Sn_TOS, Sn_TTL, Sn_KPALVTR, Sn_FRAG : Internet protocol.
 * @sa Sn_RXBUF_SIZE, Sn_TXBUF_SIZE, Sn_TX_FSR, Sn_TX_RD, Sn_TX_WR, Sn_RX_RSR, Sn_RX_RD, Sn_RX_WR : Data communication
 */

 /**
 * @defgroup Basic_IO_function Basic I/O function
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are basic input/output functions to read values from register or write values to register.
 */

/**
 * @defgroup Common_register_access_function Common register access functions
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are functions to access <b>common registers</b>.
 */

/**
 * @defgroup Socket_register_access_function Socket register access functions
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are functions to access <b>socket registers</b>.
 */

//----------------------------- W5500 Common Registers IOMAP -----------------------------
/**
 * @ingroup Common_register_group
 * @brief Mode Register address(R/W)\n
 * @ref MR is used for S/W reset, ping block mode, PPPoE mode and etc.
 * @details Each bit of @ref MR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>RST</td> <td>Reserved</td> <td>WOL</td> <td>PB</td> <td>PPPoE</td> <td>Reserved</td> <td>FARP</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref MR_RST		 	: Reset
 * - \ref MR_WOL       		: Wake on LAN
 * - \ref MR_PB         	: Ping block
 * - \ref MR_PPPOE      	: PPPoE mode
 * - \ref MR_FARP			: Force ARP mode
 */


#define MR                 (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_CREG_BLOCK << 3))


/**
 * @ingroup Common_register_group
 * @brief Gateway IP Register address(R/W)
 * @details @ref GAR configures the default gateway address.
 */
#define GAR                (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_CREG_BLOCK << 3))


/**
 * @ingroup Common_register_group
 * @brief Subnet mask Register address(R/W)
 * @details @ref SUBR configures the subnet mask address.
 */
#define SUBR               (_W5500_IO_BASE_ + (0x0005 << 8) + (WIZCHIP_CREG_BLOCK << 3))


/**
 * @ingroup Common_register_group
 * @brief Source MAC Register address(R/W)
 * @details @ref SHAR configures the source hardware address.
 */
#define SHAR               (_W5500_IO_BASE_ + (0x0009 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Source IP Register address(R/W)
 * @details @ref SIPR configures the source IP address.
 */
#define SIPR               (_W5500_IO_BASE_ + (0x000F << 8) + (WIZCHIP_CREG_BLOCK << 3))


/**
 * @ingroup Common_register_group
 * @brief Set Interrupt low level timer register address(R/W)
 * @details @ref INTLEVEL configures the Interrupt Assert Time.
 */
#define INTLEVEL           (_W5500_IO_BASE_ + (0x0013 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Interrupt Register(R/W)
 * @details @ref IR indicates the interrupt status. Each bit of @ref IR will be still until the bit will be written to by the host.
 * If @ref IR is not equal to x00 INTn PIN is asserted to low until it is x00\n\n
 * Each bit of @ref IR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>CONFLICT</td> <td>UNREACH</td> <td>PPPoE</td> <td>MP</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IR_CONFLICT : IP conflict
 * - \ref IR_UNREACH  : Destination unreachable
 * - \ref IR_PPPoE	  : PPPoE connection close
 * - \ref IR_MP		  : Magic packet
 */
#define IR                 (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Interrupt mask register(R/W)
 * @details @ref IMR is used to mask interrupts. Each bit of @ref IMR corresponds to each bit of @ref IR.
 * When a bit of @ref IMR is and the corresponding bit of @ref IR is  an interrupt will be issued. In other words,
 * if a bit of @ref IMR is  an interrupt will not be issued even if the corresponding bit of @ref IR is \n\n
 * Each bit of @ref IMR defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>IM_IR7</td> <td>IM_IR6</td> <td>IM_IR5</td> <td>IM_IR4</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IM_IR7 : IP Conflict Interrupt Mask
 * - \ref IM_IR6 : Destination unreachable Interrupt Mask
 * - \ref IM_IR5 : PPPoE Close Interrupt Mask
 * - \ref IM_IR4 : Magic Packet Interrupt Mask
 */
#define IMR                (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Socket Interrupt Register(R/W)
 * @details @ref SIR indicates the interrupt status of Socket.\n
 * Each bit of @ref SIR be still until @ref Sn_IR is cleared by the host.\n
 * If @ref Sn_IR is not equal to x00 the n-th bit of @ref SIR is and INTn PIN is asserted until @ref SIR is x00 */
#define SIR                (_W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Socket Interrupt Mask Register(R/W)
 * @details Each bit of @ref SIMR corresponds to each bit of @ref SIR.
 * When a bit of @ref SIMR is and the corresponding bit of @ref SIR is  Interrupt will be issued.
 * In other words, if a bit of @ref SIMR is  an interrupt will be not issued even if the corresponding bit of @ref SIR is
 */
#define SIMR               (_W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Timeout register address( 1 is 100us )(R/W)
 * @details @ref RTR configures the retransmission timeout period. The unit of timeout period is 100us and the default of @ref RTR is x07D0or 000
 * And so the default timeout period is 200ms(100us X 2000). During the time configured by @ref RTR, W5500 waits for the peer response
 * to the packet that is transmitted by \ref Sn_CR (CONNECT, DISCON, CLOSE, SEND, SEND_MAC, SEND_KEEP command).
 * If the peer does not respond within the @ref RTR time, W5500 retransmits the packet or issues timeout.
 */
#define RTR                (_W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Retry count register(R/W)
 * @details @ref RCR configures the number of time of retransmission.
 * When retransmission occurs as many as ref RCR+1 Timeout interrupt is issued (@ref Sn_IR[TIMEOUT] = .
 */
#define RCR                (_W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PPP LCP Request Timer register  in PPPoE mode(R/W)
 * @details @ref PTIMER configures the time for sending LCP echo request. The unit of time is 25ms.
 */
#define PTIMER             (_W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PPP LCP Magic number register  in PPPoE mode(R/W)
 * @details @ref PMAGIC configures the 4bytes magic number to be used in LCP negotiation.
 */
#define PMAGIC             (_W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PPP Destination MAC Register address(R/W)
 * @details @ref PHAR configures the PPPoE server hardware address that is acquired during PPPoE connection process.
 */
#define PHAR                (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PPP Session Identification Register(R/W)
 * @details @ref PSID configures the PPPoE sever session ID acquired during PPPoE connection process.
 */
#define PSID               (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PPP Maximum Segment Size(MSS) register(R/W)
 * @details @ref PMRU configures the maximum receive unit of PPPoE.
 */
#define PMRU               (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Unreachable IP register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR indicates
 * the destination IP address & port number respectively.
 */
#define UIPR               (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief Unreachable Port register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR
 * indicates the destination IP address & port number respectively.
 */
#define UPORTR              (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief PHY Status Register(R/W)
 * @details @ref PHYCFGR configures PHY operation mode and resets PHY. In addition, @ref PHYCFGR indicates the status of PHY such as duplex, Speed, Link.
 */
#define PHYCFGR            (_W5500_IO_BASE_ + (0x002E << 8) + (WIZCHIP_CREG_BLOCK << 3))

// Reserved			         (_W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0030 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0031 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0032 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0033 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0034 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0035 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0036 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0037 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0038 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief chip version register address(R)
 * @details @ref VERSIONR always indicates the W5500 version as @b 0x04.
 */
#define VERSIONR           (_W5500_IO_BASE_ + (0x0039 << 8) + (WIZCHIP_CREG_BLOCK << 3))


//----------------------------- W5500 Socket Registers IOMAP -----------------------------
/**
 * @ingroup Socket_register_group
 * @brief socket Mode register(R/W)
 * @details @ref Sn_MR configures the option or protocol type of Socket n.\n\n
 * Each bit of @ref Sn_MR defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>MULTI/MFEN</td> <td>BCASTB</td> <td>ND/MC/MMB</td> <td>UCASTB/MIP6B</td> <td>Protocol[3]</td> <td>Protocol[2]</td> <td>Protocol[1]</td> <td>Protocol[0]</td> </tr>
 * </table>
 * - @ref Sn_MR_MULTI	: Support UDP Multicasting
 * - @ref Sn_MR_BCASTB	: Broadcast block <b>in UDP Multicasting</b>
 * - @ref Sn_MR_ND		: No Delayed Ack(TCP) flag
 * - @ref Sn_MR_MC   	: IGMP version used <b>in UDP mulitcasting</b>
 * - @ref Sn_MR_MMB    	: Multicast Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - @ref Sn_MR_UCASTB	: Unicast Block <b>in UDP Multicating</b>
 * - @ref Sn_MR_MIP6B   : IPv6 packet Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - <b>Protocol</b>
 * <table>
 * 		<tr>   <td><b>Protocol[3]</b></td> <td><b>Protocol[2]</b></td> <td><b>Protocol[1]</b></td> <td><b>Protocol[0]</b></td> <td>@b Meaning</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>0</td> <td>Closed</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>1</td> <td>TCP</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>1</td> <td>0</td> <td>UDP</td>   </tr>
 * 		<tr>   <td>0</td> <td>1</td> <td>0</td> <td>0</td> <td>MACRAW</td>   </tr>
 * </table>
 *	- @ref Sn_MR_MACRAW	: MAC LAYER RAW SOCK \n
 *  - @ref Sn_MR_UDP		: UDP
 *  - @ref Sn_MR_TCP		: TCP
 *  - @ref Sn_MR_CLOSE	: Unused socket
 *  @note MACRAW mode should be only used in Socket 0.
 */
#define Sn_MR(N)           (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Socket command register(R/W)
 * @details This is used to set the command for Socket n such as OPEN, CLOSE, CONNECT, LISTEN, SEND, and RECEIVE.\n
 * After W5500 accepts the command, the @ref Sn_CR register is automatically cleared to 0x00.
 * Even though @ref Sn_CR is cleared to 0x00, the command is still being processed.\n
 * To check whether the command is completed or not, please check the @ref Sn_IR or @ref Sn_SR.
 * - @ref Sn_CR_OPEN 		: Initialize or open socket.
 * - @ref Sn_CR_LISTEN 		: Wait connection request in TCP mode(<b>Server mode</b>)
 * - @ref Sn_CR_CONNECT 	: Send connection request in TCP mode(<b>Client mode</b>)
 * - @ref Sn_CR_DISCON 		: Send closing request in TCP mode.
 * - @ref Sn_CR_CLOSE   	: Close socket.
 * - @ref Sn_CR_SEND    	: Update TX buffer pointer and send data.
 * - @ref Sn_CR_SEND_MAC	: Send data with MAC address, so without ARP process.
 * - @ref Sn_CR_SEND_KEEP 	: Send keep alive message.
 * - @ref Sn_CR_RECV		: Update RX buffer pointer and receive data.
 */
#define Sn_CR(N)           (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Socket interrupt register(R)
 * @details @ref Sn_IR indicates the status of Socket Interrupt such as establishment, termination, receiving data, timeout).\n
 * When an interrupt occurs and the corresponding bit of @ref Sn_IMR is  the corresponding bit of @ref Sn_IR becomes \n
 * In order to clear the @ref Sn_IR bit, the host should write the bit to \n
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>SEND_OK</td> <td>TIMEOUT</td> <td>RECV</td> <td>DISCON</td> <td>CON</td> </tr>
 * </table>
 * - \ref Sn_IR_SENDOK : <b>SEND_OK Interrupt</b>
 * - \ref Sn_IR_TIMEOUT : <b>TIMEOUT Interrupt</b>
 * - \ref Sn_IR_RECV : <b>RECV Interrupt</b>
 * - \ref Sn_IR_DISCON : <b>DISCON Interrupt</b>
 * - \ref Sn_IR_CON : <b>CON Interrupt</b>
 */
#define Sn_IR(N)           (_W5500_IO_BASE_ + (0x0002 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Socket status register(R)
 * @details @ref Sn_SR indicates the status of Socket n.\n
 * The status of Socket n is changed by @ref Sn_CR or some special control packet as SYN, FIN packet in TCP.
 * @par Normal status
 * - @ref SOCK_CLOSED 		: Closed
 * - @ref SOCK_INIT   		: Initiate state
 * - @ref SOCK_LISTEN    	: Listen state
 * - @ref SOCK_ESTABLISHED 	: Success to connect
 * - @ref SOCK_CLOSE_WAIT   : Closing state
 * - @ref SOCK_UDP   		: UDP socket
 * - @ref SOCK_MACRAW  		: MAC raw mode socket
 *@par Temporary status during changing the status of Socket n.
 * - @ref SOCK_SYNSENT   	: This indicates Socket n sent the connect-request packet (SYN packet) to a peer.
 * - @ref SOCK_SYNRECV    	: It indicates Socket n successfully received the connect-request packet (SYN packet) from a peer.
 * - @ref SOCK_FIN_WAIT		: Connection state
 * - @ref SOCK_CLOSING		: Closing state
 * - @ref SOCK_TIME_WAIT	: Closing state
 * - @ref SOCK_LAST_ACK 	: Closing state
 */
#define Sn_SR(N)           (_W5500_IO_BASE_ + (0x0003 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief source port register(R/W)
 * @details @ref Sn_PORT configures the source port number of Socket n.
 * It is valid when Socket n is used in TCP/UPD mode. It should be set before OPEN command is ordered.
 */
#define Sn_PORT(N)         (_W5500_IO_BASE_ + (0x0004 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Peer MAC register address(R/W)
 * @details @ref Sn_DHAR configures the destination hardware address of Socket n when using SEND_MAC command in UDP mode or
 * it indicates that it is acquired in ARP-process by CONNECT/SEND command.
 */
#define Sn_DHAR(N)         (_W5500_IO_BASE_ + (0x0006 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Peer IP register address(R/W)
 * @details @ref Sn_DIPR configures or indicates the destination IP address of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In TCP client mode, it configures an IP address of �TCP serverbefore CONNECT command.
 * In TCP server mode, it indicates an IP address of �TCP clientafter successfully establishing connection.
 * In UDP mode, it configures an IP address of peer to be received the UDP packet by SEND or SEND_MAC command.
 */
#define Sn_DIPR(N)         (_W5500_IO_BASE_ + (0x000C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Peer port register address(R/W)
 * @details @ref Sn_DPORT configures or indicates the destination port number of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In �TCP clientmode, it configures the listen port number of �TCP serverbefore CONNECT command.
 * In �TCP Servermode, it indicates the port number of TCP client after successfully establishing connection.
 * In UDP mode, it configures the port number of peer to be transmitted the UDP packet by SEND/SEND_MAC command.
 */
#define Sn_DPORT(N)        (_W5500_IO_BASE_ + (0x0010 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Maximum Segment Size(Sn_MSSR0) register address(R/W)
 * @details @ref Sn_MSSR configures or indicates the MTU(Maximum Transfer Unit) of Socket n.
 */
#define Sn_MSSR(N)         (_W5500_IO_BASE_ + (0x0012 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// Reserved			         (_W5500_IO_BASE_ + (0x0014 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief IP Type of Service(TOS) Register(R/W)
 * @details @ref Sn_TOS configures the TOS(Type Of Service field in IP Header) of Socket n.
 * It is set before OPEN command.
 */
#define Sn_TOS(N)          (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
/**
 * @ingroup Socket_register_group
 * @brief IP Time to live(TTL) Register(R/W)
 * @details @ref Sn_TTL configures the TTL(Time To Live field in IP header) of Socket n.
 * It is set before OPEN command.
 */
#define Sn_TTL(N)          (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Receive memory size register(R/W)
 * @details @ref Sn_RXBUF_SIZE configures the RX buffer block size of Socket n.
 * Socket n RX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data cannot be normally received from a peer.
 * Although Socket n RX Buffer Block size is initially configured to 2Kbytes,
 * user can re-configure its size using @ref Sn_RXBUF_SIZE. The total sum of @ref Sn_RXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data reception error is occurred.
 */
#define Sn_RXBUF_SIZE(N)   (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory size register(R/W)
 * @details @ref Sn_TXBUF_SIZE configures the TX buffer block size of Socket n. Socket n TX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data can�t be normally transmitted to a peer.
 * Although Socket n TX Buffer Block size is initially configured to 2Kbytes,
 * user can be re-configure its size using @ref Sn_TXBUF_SIZE. The total sum of @ref Sn_TXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data transmission error is occurred.
 */
#define Sn_TXBUF_SIZE(N)   (_W5500_IO_BASE_ + (0x001F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Transmit free memory size register(R)
 * @details @ref Sn_TX_FSR indicates the free size of Socket n TX Buffer Block. It is initialized to the configured size by @ref Sn_TXBUF_SIZE.
 * Data bigger than @ref Sn_TX_FSR should not be saved in the Socket n TX Buffer because the bigger data overwrites the previous saved data not yet sent.
 * Therefore, check before saving the data to the Socket n TX Buffer, and if data is equal or smaller than its checked size,
 * transmit the data with SEND/SEND_MAC command after saving the data in Socket n TX buffer. But, if data is bigger than its checked size,
 * transmit the data after dividing into the checked size and saving in the Socket n TX buffer.
 */
#define Sn_TX_FSR(N)       (_W5500_IO_BASE_ + (0x0020 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory read pointer register address(R)
 * @details @ref Sn_TX_RD is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.
 * After its initialization, it is auto-increased by SEND command.
 * SEND command transmits the saved data from the current @ref Sn_TX_RD to the @ref Sn_TX_WR in the Socket n TX Buffer.
 * After transmitting the saved data, the SEND command increases the @ref Sn_TX_RD as same as the @ref Sn_TX_WR.
 * If its increment value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
#define Sn_TX_RD(N)        (_W5500_IO_BASE_ + (0x0022 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory write pointer register address(R/W)
 * @details @ref Sn_TX_WR is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.\n
 * It should be read or be updated like as follows.\n
 * 1. Read the starting address for saving the transmitting data.\n
 * 2. Save the transmitting data from the starting address of Socket n TX buffer.\n
 * 3. After saving the transmitting data, update @ref Sn_TX_WR to the increased value as many as transmitting data size.
 * If the increment value exceeds the maximum value 0xFFFF(greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.\n
 * 4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND command
 */
#define Sn_TX_WR(N)        (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Received data size register(R)
 * @details @ref Sn_RX_RSR indicates the data size received and saved in Socket n RX Buffer.
 * @ref Sn_RX_RSR does not exceed the @ref Sn_RXBUF_SIZE and is calculated as the difference between
 * �Socket n RX Write Pointer (@ref Sn_RX_WR)and �Socket n RX Read Pointer (@ref Sn_RX_RD)
 */
#define Sn_RX_RSR(N)       (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Read point of Receive memory(R/W)
 * @details @ref Sn_RX_RD is initialized by OPEN command. Make sure to be read or updated as follows.\n
 * 1. Read the starting save address of the received data.\n
 * 2. Read data from the starting address of Socket n RX Buffer.\n
 * 3. After reading the received data, Update @ref Sn_RX_RD to the increased value as many as the reading size.
 * If the increment value exceeds the maximum value 0xFFFF, that is, is greater than 0x10000 and the carry bit occurs,
 * update with the lower 16bits value ignored the carry bit.\n
 * 4. Order RECV command is for notifying the updated @ref Sn_RX_RD to W5500.
 */
#define Sn_RX_RD(N)        (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Write point of Receive memory(R)
 * @details @ref Sn_RX_WR is initialized by OPEN command and it is auto-increased by the data reception.
 * If the increased value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
#define Sn_RX_WR(N)        (_W5500_IO_BASE_ + (0x002A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief socket interrupt mask register(R)
 * @details @ref Sn_IMR masks the interrupt of Socket n.
 * Each bit corresponds to each bit of @ref Sn_IR. When a Socket n Interrupt is occurred and the corresponding bit of @ref Sn_IMR is
 * the corresponding bit of @ref Sn_IR becomes  When both the corresponding bit of @ref Sn_IMR and @ref Sn_IR are and the n-th bit of @ref IR is
 * Host is interrupted by asserted INTn PIN to low.
 */
#define Sn_IMR(N)          (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Fragment field value in IP header register(R/W)
 * @details @ref Sn_FRAG configures the FRAG(Fragment field in IP header).
 */
#define Sn_FRAG(N)         (_W5500_IO_BASE_ + (0x002D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Keep Alive Timer register(R/W)
 * @details @ref Sn_KPALVTR configures the transmitting timer of �KEEP ALIVE(KA)packet of SOCKETn. It is valid only in TCP mode,
 * and ignored in other modes. The time unit is 5s.
 * KA packet is transmittable after @ref Sn_SR is changed to SOCK_ESTABLISHED and after the data is transmitted or received to/from a peer at least once.
 * In case of '@ref Sn_KPALVTR > 0', W5500 automatically transmits KA packet after time-period for checking the TCP connection (Auto-keepalive-process).
 * In case of '@ref Sn_KPALVTR = 0', Auto-keep-alive-process will not operate,
 * and KA packet can be transmitted by SEND_KEEP command by the host (Manual-keep-alive-process).
 * Manual-keep-alive-process is ignored in case of '@ref Sn_KPALVTR > 0'.
 */
#define Sn_KPALVTR(N)      (_W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

//#define Sn_TSR(N)          (_W5500_IO_BASE_ + (0x0030 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))


/* MODE register values */
/**
 * @brief Reset
 * @details If this bit is  All internal registers will be initialized. It will be automatically cleared as after S/W reset.
 */
#define MR_RST                       0x80	//1000 0000


/**
 * @brief Wake on LAN
 * @details 0 : Disable WOL mode\n
 * 1 : Enable WOL mode\n
 * If WOL mode is enabled and the received magic packet over UDP has been normally processed, the Interrupt PIN (INTn) asserts to low.
 * When using WOL mode, the UDP Socket should be opened with any source port number. (Refer to Socket n Mode Register (@ref Sn_MR) for opening Socket.)
 * @note The magic packet over UDP supported by W5500 consists of 6 bytes synchronization stream (xFFFFFFFFFFFF and
 * 16 times Target MAC address stream in UDP payload. The options such like password are ignored. You can use any UDP source port number for WOL mode.
 */
#define MR_WOL                       0x20	//0010 0000


/**
 * @brief Ping block
 * @details 0 : Disable Ping block\n
 * 1 : Enable Ping block\n
 * If the bit is  it blocks the response to a ping request.
 */
#define MR_PB                        0x10

/**
 * @brief Enable PPPoE
 * @details 0 : DisablePPPoE mode\n
 * 1 : EnablePPPoE mode\n
 * If you use ADSL, this bit should be
 */
#define MR_PPPOE                     0x08

/**
 * @brief Enable UDP_FORCE_ARP CHECHK
 * @details 0 : Disable Force ARP mode\n
 * 1 : Enable Force ARP mode\n
 * In Force ARP mode, It forces on sending ARP Request whenever data is sent.
 */
#define MR_FARP                      0x02

#endif	_W5500_H_
