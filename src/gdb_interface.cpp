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

#include "gdb_interface.h"
#include "gearlynx_core.h"
#include "m6502.h"
#include "memory.h"

#include <cstring>
#include <cstdio>

extern "C" {
#include "gdbstub/include/gdbstub.h"
}

// Global GDB interface pointer for callbacks
GDBInterface* g_gdb_interface = nullptr;

// 65C02 Target description for GDB - matches MAME's gdb_register_map_m6502
// Register order: A, X, Y, P, SP, PC (MAME format for llvm-mos LLDB compatibility)
// Plus imaginary registers for llvm-mos variable inspection
// DWARF numbers must match MOSRegisterInfo.td:
//   - Real regs: A=0, X=1, Y=2, P=3, SP=4, PC=5 (sequential for GDB)
//   - RC0-RC31: DWARF 16, 18, 20, ... 78 (every 2, due to LSB subregs)
//   - RS0-RS15: DWARF 528, 529, 530, ... 543
#define TARGET_6502 \
    "<?xml version=\"1.0\"?>" \
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">" \
    "<target version=\"1.0\">" \
    "<feature name=\"mame.m6502\">" \
    "<reg name=\"a\" bitsize=\"8\" type=\"int\" regnum=\"0\"/>" \
    "<reg name=\"x\" bitsize=\"8\" type=\"int\" regnum=\"1\"/>" \
    "<reg name=\"y\" bitsize=\"8\" type=\"int\" regnum=\"2\"/>" \
    "<reg name=\"p\" bitsize=\"8\" type=\"int\" regnum=\"3\"/>" \
    "<reg name=\"sp\" bitsize=\"8\" type=\"data_ptr\" regnum=\"4\"/>" \
    "<reg name=\"pc\" bitsize=\"16\" type=\"code_ptr\" regnum=\"5\"/>" \
    "</feature>" \
    "<feature name=\"mos.imaginary\">" \
    "<reg name=\"rc0\" bitsize=\"8\" type=\"int\" regnum=\"6\" dwarf_regnum=\"16\"/>" \
    "<reg name=\"rc1\" bitsize=\"8\" type=\"int\" regnum=\"7\" dwarf_regnum=\"18\"/>" \
    "<reg name=\"rc2\" bitsize=\"8\" type=\"int\" regnum=\"8\" dwarf_regnum=\"20\"/>" \
    "<reg name=\"rc3\" bitsize=\"8\" type=\"int\" regnum=\"9\" dwarf_regnum=\"22\"/>" \
    "<reg name=\"rc4\" bitsize=\"8\" type=\"int\" regnum=\"10\" dwarf_regnum=\"24\"/>" \
    "<reg name=\"rc5\" bitsize=\"8\" type=\"int\" regnum=\"11\" dwarf_regnum=\"26\"/>" \
    "<reg name=\"rc6\" bitsize=\"8\" type=\"int\" regnum=\"12\" dwarf_regnum=\"28\"/>" \
    "<reg name=\"rc7\" bitsize=\"8\" type=\"int\" regnum=\"13\" dwarf_regnum=\"30\"/>" \
    "<reg name=\"rc8\" bitsize=\"8\" type=\"int\" regnum=\"14\" dwarf_regnum=\"32\"/>" \
    "<reg name=\"rc9\" bitsize=\"8\" type=\"int\" regnum=\"15\" dwarf_regnum=\"34\"/>" \
    "<reg name=\"rc10\" bitsize=\"8\" type=\"int\" regnum=\"16\" dwarf_regnum=\"36\"/>" \
    "<reg name=\"rc11\" bitsize=\"8\" type=\"int\" regnum=\"17\" dwarf_regnum=\"38\"/>" \
    "<reg name=\"rc12\" bitsize=\"8\" type=\"int\" regnum=\"18\" dwarf_regnum=\"40\"/>" \
    "<reg name=\"rc13\" bitsize=\"8\" type=\"int\" regnum=\"19\" dwarf_regnum=\"42\"/>" \
    "<reg name=\"rc14\" bitsize=\"8\" type=\"int\" regnum=\"20\" dwarf_regnum=\"44\"/>" \
    "<reg name=\"rc15\" bitsize=\"8\" type=\"int\" regnum=\"21\" dwarf_regnum=\"46\"/>" \
    "<reg name=\"rc16\" bitsize=\"8\" type=\"int\" regnum=\"22\" dwarf_regnum=\"48\"/>" \
    "<reg name=\"rc17\" bitsize=\"8\" type=\"int\" regnum=\"23\" dwarf_regnum=\"50\"/>" \
    "<reg name=\"rc18\" bitsize=\"8\" type=\"int\" regnum=\"24\" dwarf_regnum=\"52\"/>" \
    "<reg name=\"rc19\" bitsize=\"8\" type=\"int\" regnum=\"25\" dwarf_regnum=\"54\"/>" \
    "<reg name=\"rc20\" bitsize=\"8\" type=\"int\" regnum=\"26\" dwarf_regnum=\"56\"/>" \
    "<reg name=\"rc21\" bitsize=\"8\" type=\"int\" regnum=\"27\" dwarf_regnum=\"58\"/>" \
    "<reg name=\"rc22\" bitsize=\"8\" type=\"int\" regnum=\"28\" dwarf_regnum=\"60\"/>" \
    "<reg name=\"rc23\" bitsize=\"8\" type=\"int\" regnum=\"29\" dwarf_regnum=\"62\"/>" \
    "<reg name=\"rc24\" bitsize=\"8\" type=\"int\" regnum=\"30\" dwarf_regnum=\"64\"/>" \
    "<reg name=\"rc25\" bitsize=\"8\" type=\"int\" regnum=\"31\" dwarf_regnum=\"66\"/>" \
    "<reg name=\"rc26\" bitsize=\"8\" type=\"int\" regnum=\"32\" dwarf_regnum=\"68\"/>" \
    "<reg name=\"rc27\" bitsize=\"8\" type=\"int\" regnum=\"33\" dwarf_regnum=\"70\"/>" \
    "<reg name=\"rc28\" bitsize=\"8\" type=\"int\" regnum=\"34\" dwarf_regnum=\"72\"/>" \
    "<reg name=\"rc29\" bitsize=\"8\" type=\"int\" regnum=\"35\" dwarf_regnum=\"74\"/>" \
    "<reg name=\"rc30\" bitsize=\"8\" type=\"int\" regnum=\"36\" dwarf_regnum=\"76\"/>" \
    "<reg name=\"rc31\" bitsize=\"8\" type=\"int\" regnum=\"37\" dwarf_regnum=\"78\"/>" \
    "<reg name=\"rs0\" bitsize=\"16\" type=\"data_ptr\" regnum=\"38\" dwarf_regnum=\"528\"/>" \
    "<reg name=\"rs1\" bitsize=\"16\" type=\"data_ptr\" regnum=\"39\" dwarf_regnum=\"529\"/>" \
    "<reg name=\"rs2\" bitsize=\"16\" type=\"data_ptr\" regnum=\"40\" dwarf_regnum=\"530\"/>" \
    "<reg name=\"rs3\" bitsize=\"16\" type=\"data_ptr\" regnum=\"41\" dwarf_regnum=\"531\"/>" \
    "<reg name=\"rs4\" bitsize=\"16\" type=\"data_ptr\" regnum=\"42\" dwarf_regnum=\"532\"/>" \
    "<reg name=\"rs5\" bitsize=\"16\" type=\"data_ptr\" regnum=\"43\" dwarf_regnum=\"533\"/>" \
    "<reg name=\"rs6\" bitsize=\"16\" type=\"data_ptr\" regnum=\"44\" dwarf_regnum=\"534\"/>" \
    "<reg name=\"rs7\" bitsize=\"16\" type=\"data_ptr\" regnum=\"45\" dwarf_regnum=\"535\"/>" \
    "<reg name=\"rs8\" bitsize=\"16\" type=\"data_ptr\" regnum=\"46\" dwarf_regnum=\"536\"/>" \
    "<reg name=\"rs9\" bitsize=\"16\" type=\"data_ptr\" regnum=\"47\" dwarf_regnum=\"537\"/>" \
    "<reg name=\"rs10\" bitsize=\"16\" type=\"data_ptr\" regnum=\"48\" dwarf_regnum=\"538\"/>" \
    "<reg name=\"rs11\" bitsize=\"16\" type=\"data_ptr\" regnum=\"49\" dwarf_regnum=\"539\"/>" \
    "<reg name=\"rs12\" bitsize=\"16\" type=\"data_ptr\" regnum=\"50\" dwarf_regnum=\"540\"/>" \
    "<reg name=\"rs13\" bitsize=\"16\" type=\"data_ptr\" regnum=\"51\" dwarf_regnum=\"541\"/>" \
    "<reg name=\"rs14\" bitsize=\"16\" type=\"data_ptr\" regnum=\"52\" dwarf_regnum=\"542\"/>" \
    "<reg name=\"rs15\" bitsize=\"16\" type=\"data_ptr\" regnum=\"53\" dwarf_regnum=\"543\"/>" \
    "</feature>" \
    "</target>"

// Global context for GDB callbacks
struct GDBContext
{
    GearlynxCore* core;
    GDBInterface* interface;
};

static GDBContext g_gdb_context;

// Register size callback - order: A, X, Y, P, SP, PC, RC0-RC31, RS0-RS15
static size_t gdb_get_reg_bytes(int regno)
{
    // Real 6502 registers
    if (regno >= GDB_REG_A && regno <= GDB_REG_SP)
        return 1;  // A, X, Y, P, SP are 8-bit
    if (regno == GDB_REG_PC)
        return 2;  // PC is 16-bit

    // Imaginary 8-bit registers RC0-RC31
    if (regno >= GDB_REG_RC0 && regno <= GDB_REG_RC31)
        return 1;

    // Imaginary 16-bit registers RS0-RS15
    if (regno >= GDB_REG_RS0 && regno <= GDB_REG_RS15)
        return 2;

    return 0;  // Unknown register
}

// Read register callback
static int gdb_read_reg(void* args, int regno, void* value)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return -1;

    M6502* cpu = ctx->core->GetM6502();
    if (!cpu)
        return -1;

    M6502::M6502_State* state = cpu->GetState();

    // Real 6502 registers
    switch (regno)
    {
        case GDB_REG_A:
        {
            u8 val = state->A.GetValue();
            std::memcpy(value, &val, 1);
            return 0;
        }
        case GDB_REG_X:
        {
            u8 val = state->X.GetValue();
            std::memcpy(value, &val, 1);
            return 0;
        }
        case GDB_REG_Y:
        {
            u8 val = state->Y.GetValue();
            std::memcpy(value, &val, 1);
            return 0;
        }
        case GDB_REG_SP:
        {
            u8 val = state->S.GetValue();
            std::memcpy(value, &val, 1);
            return 0;
        }
        case GDB_REG_PC:
        {
            u16 val = state->PC.GetValue();
            std::memcpy(value, &val, 2);
            return 0;
        }
        case GDB_REG_P:
        {
            u8 val = state->P.GetValue();
            std::memcpy(value, &val, 1);
            return 0;
        }
    }

    // Imaginary 8-bit registers RC0-RC31 are at zero page 0x00-0x1F
    if (regno >= GDB_REG_RC0 && regno <= GDB_REG_RC31)
    {
        Memory* mem = ctx->core->GetMemory();
        if (!mem)
            return -1;
        int rc_index = regno - GDB_REG_RC0;  // 0-31
        u8 val = mem->Read(static_cast<u16>(rc_index));
        std::memcpy(value, &val, 1);
        return 0;
    }

    // Imaginary 16-bit registers RS0-RS15 are pairs of RC registers
    // RS0 = {RC1, RC0} at addresses 0x00-0x01 (little endian)
    // RS1 = {RC3, RC2} at addresses 0x02-0x03
    // etc.
    if (regno >= GDB_REG_RS0 && regno <= GDB_REG_RS15)
    {
        Memory* mem = ctx->core->GetMemory();
        if (!mem)
            return -1;
        int rs_index = regno - GDB_REG_RS0;  // 0-15
        u16 addr = static_cast<u16>(rs_index * 2);  // Each RS uses 2 bytes
        u8 lo = mem->Read(addr);
        u8 hi = mem->Read(addr + 1);
        u16 val = static_cast<u16>(lo) | (static_cast<u16>(hi) << 8);
        std::memcpy(value, &val, 2);
        return 0;
    }

    return -1;
}

// Write register callback
static int gdb_write_reg(void* args, int regno, void* value)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return -1;

    M6502* cpu = ctx->core->GetM6502();
    if (!cpu)
        return -1;

    M6502::M6502_State* state = cpu->GetState();

    // Real 6502 registers
    switch (regno)
    {
        case GDB_REG_A:
        {
            u8 val;
            std::memcpy(&val, value, 1);
            state->A.SetValue(val);
            return 0;
        }
        case GDB_REG_X:
        {
            u8 val;
            std::memcpy(&val, value, 1);
            state->X.SetValue(val);
            return 0;
        }
        case GDB_REG_Y:
        {
            u8 val;
            std::memcpy(&val, value, 1);
            state->Y.SetValue(val);
            return 0;
        }
        case GDB_REG_SP:
        {
            u8 val;
            std::memcpy(&val, value, 1);
            state->S.SetValue(val);
            return 0;
        }
        case GDB_REG_PC:
        {
            u16 val;
            std::memcpy(&val, value, 2);
            state->PC.SetValue(val);
            return 0;
        }
        case GDB_REG_P:
        {
            u8 val;
            std::memcpy(&val, value, 1);
            state->P.SetValue(val);
            return 0;
        }
    }

    // Imaginary 8-bit registers RC0-RC31 are at zero page 0x00-0x1F
    if (regno >= GDB_REG_RC0 && regno <= GDB_REG_RC31)
    {
        Memory* mem = ctx->core->GetMemory();
        if (!mem)
            return -1;
        int rc_index = regno - GDB_REG_RC0;  // 0-31
        u8 val;
        std::memcpy(&val, value, 1);
        mem->Write(static_cast<u16>(rc_index), val);
        return 0;
    }

    // Imaginary 16-bit registers RS0-RS15 are pairs of RC registers
    if (regno >= GDB_REG_RS0 && regno <= GDB_REG_RS15)
    {
        Memory* mem = ctx->core->GetMemory();
        if (!mem)
            return -1;
        int rs_index = regno - GDB_REG_RS0;  // 0-15
        u16 addr = static_cast<u16>(rs_index * 2);  // Each RS uses 2 bytes
        u16 val;
        std::memcpy(&val, value, 2);
        mem->Write(addr, static_cast<u8>(val & 0xFF));        // Low byte
        mem->Write(addr + 1, static_cast<u8>(val >> 8));      // High byte
        return 0;
    }

    return -1;
}

// Read memory callback
// For debugger reads, we read directly from RAM for hardware I/O regions
// to avoid side effects, assertions, and crashes
static int gdb_read_mem(void* args, size_t addr, size_t len, void* val)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return -1;

    Memory* mem = ctx->core->GetMemory();
    if (!mem)
        return -1;

    if (addr > 0xFFFF || (addr + len) > 0x10000)
        return -1;

    // Get raw RAM pointer for direct access to hardware I/O regions
    u8* ram = mem->GetRAM();

    u8* dest = static_cast<u8*>(val);
    for (size_t i = 0; i < len; i++)
    {
        u16 read_addr = static_cast<u16>(addr + i);
        u8 page = read_addr >> 8;

        // For hardware I/O regions, read directly from RAM to avoid:
        // - Side effects from hardware register reads
        // - Assertions for unhandled addresses
        // - Crashes from uninitialized cartridge data
        // These regions are: Suzy (0xFC), Mikey (0xFD), BIOS (0xFE), vectors (0xFF)
        if (page >= 0xFC)
        {
            dest[i] = ram[read_addr];
        }
        else
        {
            dest[i] = mem->Read(read_addr);
        }
    }

    return 0;
}

// Write memory callback
static int gdb_write_mem(void* args, size_t addr, size_t len, void* val)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return -1;

    Memory* mem = ctx->core->GetMemory();
    if (!mem)
        return -1;

    if (addr > 0xFFFF || (addr + len) > 0x10000)
        return -1;

    u8* src = static_cast<u8*>(val);
    for (size_t i = 0; i < len; i++)
    {
        mem->Write(static_cast<u16>(addr + i), src[i]);
    }

    return 0;
}

// Continue execution callback - NON-BLOCKING VERSION
// This sets state and waits for main loop to signal breakpoint
static gdb_action_t gdb_cont(void* args)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->interface)
        return ACT_SHUTDOWN;

    ctx->interface->RequestContinue();
    ctx->interface->WaitForBreakpoint();

    return ACT_RESUME;
}

static gdb_action_t gdb_stepi(void* args)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->interface)
        return ACT_SHUTDOWN;

    ctx->interface->RequestStep();
    ctx->interface->WaitForBreakpoint();

    return ACT_RESUME;
}

// Set breakpoint callback
static bool gdb_set_bp(void* args, size_t addr, bp_type_t type)
{
    if (type != BP_SOFTWARE)
        return false;

    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return false;

    M6502* cpu = ctx->core->GetM6502();
    if (!cpu)
        return false;

    if (addr > 0xFFFF)
        return false;

    return cpu->AddBreakpoint(static_cast<u16>(addr));
}

static bool gdb_del_bp(void* args, size_t addr, bp_type_t type)
{
    if (type != BP_SOFTWARE)
        return false;

    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (!ctx || !ctx->core)
        return false;

    M6502* cpu = ctx->core->GetM6502();
    if (!cpu)
        return false;

    if (addr > 0xFFFF)
        return false;

    cpu->RemoveBreakpoint(static_cast<u16>(addr));
    return true;
}

static void gdb_on_interrupt(void* args)
{
    GDBContext* ctx = static_cast<GDBContext*>(args);
    if (ctx && ctx->interface)
    {
        ctx->interface->SignalBreakpoint();
    }
}

static int gdb_get_cpu(void* args)
{
    (void)args;
    return 1;
}

static void gdb_set_cpu(void* args, int cpuid)
{
    (void)args;
    (void)cpuid;
}

// Target operations structure
static struct target_ops g_target_ops = {
    .cont = gdb_cont,
    .stepi = gdb_stepi,
    .get_reg_bytes = gdb_get_reg_bytes,
    .read_reg = gdb_read_reg,
    .write_reg = gdb_write_reg,
    .read_mem = gdb_read_mem,
    .write_mem = gdb_write_mem,
    .set_bp = gdb_set_bp,
    .del_bp = gdb_del_bp,
    .on_interrupt = gdb_on_interrupt,
    .set_cpu = gdb_set_cpu,
    .get_cpu = gdb_get_cpu,
};

// GDBInterface implementation

GDBInterface::GDBInterface()
    : m_core(nullptr)
    , m_active(false)
    , m_connected(false)
    , m_state(GDB_STATE_DISCONNECTED)
    , m_port(0)
    , m_breakpoint_hit(false)
    , m_shutdown_requested(false)
    , m_step_pending(false)
    , m_gdbstub(nullptr)
{
}

GDBInterface::~GDBInterface()
{
    Shutdown();
}

bool GDBInterface::Init(GearlynxCore* core, int port)
{
    if (m_active.load())
        return false;

    m_core = core;
    m_port = port;
    m_shutdown_requested.store(false);
    m_breakpoint_hit.store(false);
    m_state.store(GDB_STATE_HALTED);

    // Setup global context
    g_gdb_context.core = core;
    g_gdb_context.interface = this;
    g_gdb_interface = this;

    // Allocate gdbstub structure
    m_gdbstub = new gdbstub_t();
    gdbstub_t* stub = static_cast<gdbstub_t*>(m_gdbstub);

    // Build connection string
    char conn_str[64];
    snprintf(conn_str, sizeof(conn_str), "0.0.0.0:%d", port);

    arch_info_t arch = {
        .target_desc = const_cast<char*>(TARGET_6502),
        .smp = 1,
        .reg_num = GDB_REG_COUNT,
    };

    if (!gdbstub_init(stub, &g_target_ops, arch, conn_str))
    {
        delete stub;
        m_gdbstub = nullptr;
        return false;
    }

    m_active.store(true);
    m_connected.store(true);

    m_gdb_thread = std::thread(&GDBInterface::GDBThreadFunc, this);

    return true;
}

void GDBInterface::Shutdown()
{
    if (!m_active.load())
        return;

    m_shutdown_requested.store(true);
    m_state.store(GDB_STATE_DISCONNECTED);

    // Wake up any waiting thread
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_breakpoint_hit.store(true);
    }
    m_breakpoint_cv.notify_all();

    // Wait for thread to finish
    if (m_gdb_thread.joinable())
    {
        m_gdb_thread.join();
    }

    if (m_gdbstub)
    {
        gdbstub_t* stub = static_cast<gdbstub_t*>(m_gdbstub);
        gdbstub_close(stub);
        delete stub;
        m_gdbstub = nullptr;
    }

    m_active.store(false);
    m_connected.store(false);
    g_gdb_interface = nullptr;
}

bool GDBInterface::IsActive() const
{
    return m_active.load();
}

bool GDBInterface::IsConnected() const
{
    return m_connected.load();
}

GDB_State GDBInterface::GetState() const
{
    return m_state.load();
}

bool GDBInterface::ShouldRun() const
{
    GDB_State state = m_state.load();
    return state == GDB_STATE_RUNNING || state == GDB_STATE_STEPPING;
}

bool GDBInterface::ShouldStep() const
{
    return m_state.load() == GDB_STATE_STEPPING && m_step_pending.load();
}

void GDBInterface::SignalBreakpoint()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_breakpoint_hit.store(true);
        m_step_pending.store(false);
        m_state.store(GDB_STATE_HALTED);
    }
    m_breakpoint_cv.notify_all();
}

void GDBInterface::SignalStepComplete()
{
    SignalBreakpoint();
}

void GDBInterface::ClearStepFlag()
{
    // Not needed with new state machine
}

void GDBInterface::RequestContinue()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_breakpoint_hit.store(false);
    m_state.store(GDB_STATE_RUNNING);
}

void GDBInterface::RequestStep()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_breakpoint_hit.store(false);
    m_step_pending.store(true);
    m_state.store(GDB_STATE_STEPPING);
}

void GDBInterface::WaitForBreakpoint()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_breakpoint_cv.wait(lock, [this] {
        // Also wake up if disconnected (debugger detached)
        return m_breakpoint_hit.load() || m_shutdown_requested.load() ||
               !m_connected.load();
    });
}

void GDBInterface::GDBThreadFunc()
{
    gdbstub_t* stub = static_cast<gdbstub_t*>(m_gdbstub);
    if (!stub)
        return;

    gdbstub_run(stub, &g_gdb_context);

    // Signal disconnection and wake up any waiting threads
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_connected.store(false);
        m_state.store(GDB_STATE_DISCONNECTED);
        m_active.store(false);
    }
    m_breakpoint_cv.notify_all();
}

bool RunGDBServer(GearlynxCore* core, int port)
{
    GDBInterface gdb;

    if (!gdb.Init(core, port))
    {
        fprintf(stderr, "Failed to initialize GDB server on port %d\n", port);
        return false;
    }

    // In headless mode, we need to run the emulator ourselves
    // This is the old blocking behavior
    u8 frame_buffer[160 * 102 * 4];
    s16 sample_buffer[2048];
    int sample_count = 0;

    GearlynxCore::GLYNX_Debug_Run debug_run;
    debug_run.step_debugger = false;
    debug_run.stop_on_breakpoint = true;
    debug_run.stop_on_run_to_breakpoint = true;
    debug_run.stop_on_irq = 0;

    while (gdb.IsActive())
    {
        if (gdb.ShouldRun())
        {
            M6502* cpu = core->GetM6502();
            cpu->EnableBreakpoints(true, 0);

            if (gdb.ShouldStep())
            {
                cpu->RunInstruction();
                gdb.SignalStepComplete();
            }
            else
            {
                bool hit = core->RunToVBlank(frame_buffer, sample_buffer, &sample_count, &debug_run);
                if (hit)
                {
                    gdb.SignalBreakpoint();
                }
            }
        }
        else
        {
            // Idle - let GDB thread process commands
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return true;
}
