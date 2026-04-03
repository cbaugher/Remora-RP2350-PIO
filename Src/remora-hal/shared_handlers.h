#pragma once

// RP2350 stub — shared peripheral handle management is not needed.
// The pico-sdk manages all peripheral state internally, and its initialisation
// functions are idempotent.  This header exists only so that any remora-core
// include chain that references it continues to compile.
