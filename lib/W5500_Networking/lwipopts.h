/* lwipopts.h — LwIP configuration for Remora RP2350 ETH build.
 *
 * NO_SYS mode (no RTOS).  IPv4 only.  UDP for Remora data channel,
 * TFTP for config uploads.  TCP disabled.
 * W5500 provides the physical Ethernet via MACRAW socket.
 */

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* ---- OS / threading ---- */
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0

/* ---- Memory ---- */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (16 * 1024)

/* pbufs: need enough for one Ethernet MTU (1500 B) plus TFTP blocks (512 B) */
#define PBUF_POOL_SIZE              8
#define PBUF_POOL_BUFSIZE           1536    /* >= ETHERNET_MTU */

/* memp pool sizes — keep small; we only use UDP */
#define MEMP_NUM_PBUF               16
#define MEMP_NUM_UDP_PCB            4       /* remora data + TFTP init + TFTP data */
#define MEMP_NUM_TCP_PCB            0
#define MEMP_NUM_TCP_PCB_LISTEN     0
#define MEMP_NUM_TCP_SEG            0
#define MEMP_NUM_SYS_TIMEOUT        6

/* ---- Protocol options ---- */
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_IPV4                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_RAW                    0
#define LWIP_DHCP                   0
#define LWIP_AUTOIP                 0
#define LWIP_SNMP                   0
#define LWIP_IGMP                   1       /* needed for NETIF_FLAG_IGMP */
#define LWIP_DNS                    0

/* ---- Checksum ---- */
/* W5500 hardware does not offload checksums in MACRAW mode;
   compute them in software via the Thumb-2 routines in arch/checksum.c. */
#define CHECKSUM_GEN_IP             1
#define CHECKSUM_GEN_UDP            1
#define CHECKSUM_GEN_TCP            0
#define CHECKSUM_CHECK_IP           1
#define CHECKSUM_CHECK_UDP          0
#define CHECKSUM_CHECK_TCP          0

/* ---- Netif callbacks (required by W5500_Networking.cpp) ---- */
#define LWIP_NETIF_STATUS_CALLBACK  1   /* netif_set_status_callback */
#define LWIP_NETIF_LINK_CALLBACK    1   /* netif_set_link_callback */

/* ---- Sequential / socket API (requires NO_SYS=0 — must be disabled) ---- */
#define LWIP_NETCONN                0
#define LWIP_SOCKET                 0

/* ---- Misc ---- */
#define LWIP_NETIF_HOSTNAME         0
#define LWIP_NETIF_API              0
#define LWIP_STATS                  0
#define LWIP_DEBUG                  0

/* BYTE_ORDER: RP2350 is little-endian */
#define BYTE_ORDER                  LITTLE_ENDIAN

#endif /* LWIPOPTS_H */
