/*
 * Gearlynx - Lynx Emulator
 * Copyright (C) 2025  Ignacio Sanchez
 *
 * GDB Stub Interface for remote debugging
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/
 *
 */

#ifndef GDB_INTERFACE_H
#define GDB_INTERFACE_H

#include "common.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class GearlynxCore;

// 65C02 Register indices for GDB protocol
// Order matches MAME's gdb_register_map_m6502: A, X, Y, P, SP, PC
// Plus llvm-mos imaginary registers for variable inspection
enum GDB_6502_Registers
{
    GDB_REG_A = 0,      // Accumulator (1 byte)
    GDB_REG_X = 1,      // X Index (1 byte)
    GDB_REG_Y = 2,      // Y Index (1 byte)
    GDB_REG_P = 3,      // Processor Status (1 byte)
    GDB_REG_SP = 4,     // Stack Pointer (1 byte)
    GDB_REG_PC = 5,     // Program Counter (2 bytes)
    // Imaginary 8-bit registers RC0-RC31 (at zero page 0x00-0x1F)
    GDB_REG_RC0 = 6,    // Start of 8-bit imaginary registers
    GDB_REG_RC31 = 37,  // End of 8-bit imaginary registers (6 + 31)
    // Imaginary 16-bit registers RS0-RS15 (pairs of RC registers)
    GDB_REG_RS0 = 38,   // Start of 16-bit imaginary registers
    GDB_REG_RS15 = 53,  // End of 16-bit imaginary registers (38 + 15)
    GDB_REG_COUNT = 54  // Total registers
};

// llvm-mos DWARF register numbers
// Real registers: A=0, X=2, Y=4, S=6 (but we remap to 0-5 for GDB)
// Imaginary 8-bit: RC0=16, RC1=18, RC2=20... (every 2)
// Imaginary 16-bit: RS0=528, RS1=529...

// GDB execution state
enum GDB_State
{
    GDB_STATE_HALTED = 0,    // Waiting for GDB command
    GDB_STATE_RUNNING,       // Emulator running (continue)
    GDB_STATE_STEPPING,      // Single step requested
    GDB_STATE_DISCONNECTED   // No GDB connection
};

class GDBInterface
{
public:
    GDBInterface();
    ~GDBInterface();

    // Initialize GDB server on specified port (non-blocking, starts thread)
    bool Init(GearlynxCore* core, int port);

    // Shutdown GDB server
    void Shutdown();

    // Check if GDB server is active
    bool IsActive() const;

    // Check if GDB client is connected
    bool IsConnected() const;

    // Get current GDB state
    GDB_State GetState() const;

    // Called by main loop: check what to do
    bool ShouldRun() const;      // Should emulator run this frame?
    bool ShouldStep() const;     // Should single-step?

    // Called by main loop when breakpoint is hit
    void SignalBreakpoint();

    // Called by main loop after step completes
    void SignalStepComplete();

    // Called by main loop to clear step flag after processing
    void ClearStepFlag();

    // Internal: called by GDB thread when continue/step received
    void RequestContinue();
    void RequestStep();

    // Internal: wait for breakpoint (called from gdb_cont in GDB thread)
    void WaitForBreakpoint();

private:
    void GDBThreadFunc();

    GearlynxCore* m_core;
    std::atomic<bool> m_active;
    std::atomic<bool> m_connected;
    std::atomic<GDB_State> m_state;
    int m_port;

    // Threading
    std::thread m_gdb_thread;
    std::mutex m_mutex;
    std::condition_variable m_breakpoint_cv;
    std::atomic<bool> m_breakpoint_hit;
    std::atomic<bool> m_shutdown_requested;
    std::atomic<bool> m_step_pending;  // True while step is in progress

    // Opaque pointer to gdbstub_t (avoid including C header in C++ header)
    void* m_gdbstub;

    // Allow standalone function access to internals
    friend bool RunGDBServer(GearlynxCore* core, int port);
};

// Global GDB interface (for use by gdb_cont callback)
extern GDBInterface* g_gdb_interface;

// Standalone function to run GDB server in blocking mode
// Use this for headless/command-line debugging
bool RunGDBServer(GearlynxCore* core, int port);

#endif /* GDB_INTERFACE_H */
