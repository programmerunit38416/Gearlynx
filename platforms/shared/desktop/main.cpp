/*
 * Gearlynx - Lynx Emulator
 * Copyright (C) 2025  Ignacio Sanchez

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/
 *
 */

#include "gearlynx.h"
#include "application.h"
#include "gdb_interface.h"
#include "emu.h"
#include <cstdlib>

int main(int argc, char* argv[])
{
    char* rom_file = NULL;
    char* symbol_file = NULL;
    bool show_usage = false;
    bool force_fullscreen = false;
    bool force_windowed = false;
    int gdb_port = 0;  // 0 = disabled
    int ret = 0;

    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "-?") == 0) ||
                (strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "/?") == 0))
            {
                show_usage = true;
                ret = 0;
            }
            else if ((strcmp(argv[i], "-v") == 0) || (strcmp(argv[i], "--version") == 0))
            {
                printf("%s\n", GLYNX_TITLE_ASCII);
                printf("Build: %s\n", GLYNX_VERSION);
                printf("Author: Ignacio Sánchez (drhelius)\n");
                return 0;
            }
            else if ((strcmp(argv[i], "-f") == 0) || (strcmp(argv[i], "--fullscreen") == 0))
            {
                force_fullscreen = true;
            }
            else if ((strcmp(argv[i], "-w") == 0) || (strcmp(argv[i], "--windowed") == 0))
            {
                force_windowed = true;
            }
            else if (strncmp(argv[i], "--gdb-port=", 11) == 0)
            {
                gdb_port = atoi(argv[i] + 11);
                if (gdb_port <= 0 || gdb_port > 65535)
                {
                    printf("Invalid GDB port: %s\n", argv[i] + 11);
                    show_usage = true;
                    ret = -1;
                }
            }
            else if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--gdb") == 0)
            {
                gdb_port = 1234;  // Default GDB port
            }
            else
            {
                printf("Unknown option: %s\n", argv[i]);
                show_usage = true;
                ret = -1;
            }
        }
    }

    int non_option_count = 0;
    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] != '-')
        {
            if (non_option_count == 0)
                rom_file = argv[i];
            else if (non_option_count == 1)
                symbol_file = argv[i];
            
            non_option_count++;
            
            if (non_option_count > 2)
            {
                show_usage = true;
                ret = -1;
                break;
            }
        }
    }

    if (show_usage)
    {
        printf("Usage: %s [options] [game_file] [symbol_file]\n", argv[0]);
        printf("  [game_file]         Game file: accepts ROMs (.lyx, .lnx, .o) or ZIP (.zip)\n");
        printf("\nOptions:\n");
        printf("  -f, --fullscreen    Start in fullscreen mode\n");
        printf("  -w, --windowed      Start in windowed mode with menu visible\n");
        printf("  -g, --gdb           Enable GDB server on default port 1234 (with GUI)\n");
        printf("  --gdb-port=PORT     Enable GDB server on specified port (with GUI)\n");
        printf("  -v, --version       Display version information\n");
        printf("  -h, --help          Display this help message\n");
        return ret;
    }

    if (force_fullscreen && force_windowed)
        force_fullscreen = false;

    // Initialize GUI application
    ret = application_init(rom_file, symbol_file, force_fullscreen, force_windowed);

    if (ret != 0)
    {
        application_destroy();
        return ret;
    }

    // Start GDB server if requested (runs in background thread with GUI)
    GDBInterface* gdb = nullptr;
    if (gdb_port > 0)
    {
        // Print connection info BEFORE Init() because Init() blocks on accept()
        printf("Starting GDB server on port %d...\n", gdb_port);
        printf("GDB server started. Connect with:\n");
        printf("  lldb -o \"target create t.elf\" -o \"gdb-remote %d\"\n", gdb_port);
        fflush(stdout);

        gdb = new GDBInterface();
        GearlynxCore* core = emu_get_core();

        if (!core || !gdb->Init(core, gdb_port))
        {
            printf("Warning: Failed to start GDB server\n");
            delete gdb;
            gdb = nullptr;
        }
    }

    // Run main loop (with GUI and optional GDB)
    application_mainloop();

    // Cleanup
    if (gdb)
    {
        gdb->Shutdown();
        delete gdb;
    }

    application_destroy();

    return ret;
}
