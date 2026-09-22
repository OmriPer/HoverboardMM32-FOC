/*
 * Board role: master or slave.
 *
 * The master and slave boards of this hoverboard are the same PCB. The slave gets its
 * power through the master-slave cable and talks to the master over that cable's UART:
 *
 *   PC --UART1 (PD0/PD1)--> master --UART2 (TX PA2, RX PA3, crossed cable)--> slave (same pins)
 *
 * The master relays RemoteUartBus frames addressed to other SLAVE_IDs to UART2, and the slave's
 * answers back to the PC (Src/remoteUartBus.c). The slave firmware is built with BOARD_SLAVE
 * defined: VS Code target type SPIN27-Slave (Hoverboard.csolution.yml).
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef BOARD_SLAVE
#define BOARD_IS_SLAVE   1
#else
#define BOARD_IS_SLAVE   0
#endif

/* Master only: relay frames for other SLAVE_IDs between UART1 (PC) and UART2 (slave). */
#define RELAY_ENABLE     (!BOARD_IS_SLAVE)

/* The master-slave link uses UART2 on TX = PA2, RX = PA3 (AF1), set up by LinkUartInit() with
 * direct register writes. Verified with a pin scan (TX on PA2 and PA14, RX alternating PA3/PA15:
 * frames only with RX on PA3) and then PA2/PA3 alone. An earlier PA2/PA3 build that configured the
 * pins through pinModeAF()/pinMode() got no bytes through; the cause was not found. */

#endif
