#pragma once

// ---------------------------------------------------------------------------
// W5500_Networking stub for RP2350 Milestone 1
//
// remora-core's W5500_Networking driver is not yet wired into the build.
// This stub provides the minimum declarations needed for RP2350_EthComms.cpp
// to compile during early milestones.
//
// TODO: Remove this file and restore the real include in RP2350_EthComms.cpp
// when implementing Milestone 5 (Ethernet comms operational).  The real header
// is at remora-core/drivers/W5500_Networking/W5500_Networking.h and requires
// the W5500_Networking source to be added back to CMakeLists.txt.
// ---------------------------------------------------------------------------

class CommsInterface;  // forward declaration

namespace network {
    inline void EthernetInit(CommsInterface* /*comms*/) {}
    inline void EthernetTasks() {}
}
