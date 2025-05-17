/* vim: set ts=8 sw=8 et: */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <stdarg.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <SDL2/SDL.h>

#include "serialdevice.h"
#include "ttycon.h"
#include "fake65c02.h"
#include "6522.h"
#include "6551.h"
#include "sdcard.h"
#include "tms9918a.h"
#include "tms9918a_render.h"

#define TRACE_MEM       1
#define TRACE_IRQ       2
#define TRACE_CPU       4
#define TRACE_6551      8
#define TRACE_SD        16
#define TRACE_SPI       32
#define TRACE_TMS9918A  64
#define TRACE_VIA       128

#define SD_CLK          1 << 0
#define SD_CS           1 << 1
#define ROM_SWITCH      1 << 6
#define SD_MOSI         1 << 7

static struct serial_device *con;
static struct termios saved_term, term;

static struct sdcard *sdcard;
static struct tms9918a *vdp;
static struct tms9918a_renderer *vdprend;
static struct via6522 *via1;
static struct m6551 *uart;

static uint8_t via_pa = 0xC7; // LED OFF, ROM SWITCH ON - other active lows are disabled

volatile int emulator_done;
static uint8_t fast = 0;

int sdl_live;

static int trace = 0;

static uint8_t ram[0x10000];
static uint8_t paged_ram[64 * 0x2000];
static uint8_t rom[0x2000];

uint8_t do_read_6502(uint16_t addr, unsigned debug)
{
        bool flash_in = via_get_port_a(via1) & ROM_SWITCH;
        if (flash_in && (addr >= 0xE000))
        {
                if (debug) fprintf(stderr, "Read from rom\n");
                return rom[addr - 0xE000];
        }

        if (uart && (addr & 0xFFF0) == 0xBF10)
        {
                if (debug) fprintf(stderr, "Read from uart\n");
                return m6551_read(uart, addr & 0x03);
        }

        if (via1 && (addr & 0xFFF0) == 0xBF20)
        {
                if (debug) fprintf(stderr, "Read from via\n");
                return via_read(via1, addr & 0x0F);
        }

        if (vdp && (addr & 0xFFF0) == 0xBF30)
        {
                if (debug) fprintf(stderr, "Read from vdp\n");
                return tms9918a_read(vdp, addr);
        }

        if (addr >= 0xC000 && addr < 0xE000)
        {
                uint8_t bank = ram[0xBF00];
                if (debug) fprintf(stderr, "Read from bank [%d]\n",bank);
                return paged_ram[(bank * 0x2000) + (0xC000-addr)];
        }
        return ram[addr];
}

uint8_t read6502_debug(uint16_t addr)
{
        uint8_t r = do_read_6502(addr, 1);
        if (trace & TRACE_MEM) {
                fprintf(stderr, "%04X -> %02X\n", addr, r);
        }
        return r;
}

uint8_t read6502(uint16_t addr)
{
        uint8_t r = do_read_6502(addr, 0);
        if (trace & TRACE_MEM) {
                fprintf(stderr, "%04X -> %02X\n", addr, r);
        }
        return r;
}

bool old_spi_clk = 0;

void write6502(uint16_t addr, uint8_t val)
{
        bool flash_in = via_get_port_a(via1) & ROM_SWITCH;
        if (flash_in && (addr >= 0xE000))
        {
            return;
        }

        if (uart && (addr & 0xFFF0) == 0xBF10)
        {
                m6551_write(uart, addr & 0x03, val);
                return;
        }

        if (via1 && (addr & 0xFFF0) == 0xBF20)
        {
                // we want to make sure that ROM_SWITCH is pulled up when setting the direction to input.
                if ((addr == 0xBF23) && ((val & ROM_SWITCH) == 0))
                {
                        uint8_t pa = via_get_port_a(via1);
                        pa |= ROM_SWITCH;
                        via_write(via1, 1, pa);
                }
                via_write(via1, addr & 0x0F, val);
                return;
        }

        if (vdp && (addr & 0xFFF0) == 0xBF30)
        {
                tms9918a_write(vdp, addr, val);
                return;
        }

        if (addr >= 0xC000 && addr < 0xE000)
        {
                uint8_t bank = ram[0xBF00];
                paged_ram[(bank * 0x2000) + (0xC000-addr)] = val;
                return;
        }
        ram[addr] = val;
        return;
}

/* We do this in the 6502 loop instead. Provide a dummy for the device models */
void recalc_interrupts(void)
{
}


void via_recalc_outputs(struct via6522 *via)
{
        uint8_t port = via_get_port_a(via);

        bool clk = port & 1;
        bool is_sdcard = !((port >> 1) & 1);
        bool mosi = port >> 7;

        if (trace & TRACE_SPI)
                fprintf(stderr, "SPI: sdcs=%d, sdclk=%d, mosi=%d\n", is_sdcard, clk, mosi);

        static bool last_sdcard;
        static uint8_t bit_counter = 0;
        if (!last_sdcard && is_sdcard) {
                bit_counter = 0;
                sd_spi_lower_cs(sdcard);
        }
        last_sdcard = is_sdcard;

        static int init_counter = 0;
        static bool initialized = false;
        // For initialization, the client has to pull&release CLK 74 times.
        // The SD card should be deselected, because it's not actual
        // data transmission (we ignore this).
        if (!initialized) { // TODO FIXME move to sdcard.c
                if (clk) {
                        init_counter++;
                        if (init_counter >= 74) {
                                sd_spi_lower_cs(sdcard);
                                initialized = true;
                        }
                }
                return;
        }

        if (!is_sdcard) {
                return;
        }
        static bool last_clk = false;
        if (clk == last_clk) {
                return;
        }
        last_clk = clk;
        if (!clk) {     // only care about rising clock
                return;
        }

        static uint8_t inbyte, outbyte;
        inbyte <<= 1;
        inbyte |= mosi;
        bit_counter++;


        if (bit_counter != 8) {
                return;
        }

        bit_counter = 0;

        if (is_sdcard) {
                outbyte = sd_spi_in(sdcard, inbyte);
                if (trace & TRACE_SPI)
                        fprintf(stderr, "SDSPI: %02X -> %02X\n", inbyte, outbyte);
        }
        via_write(via, 10, outbyte);
}


void via_handshake_a(struct via6522 *via)
{
}

void via_handshake_b(struct via6522 *via)
{
}

static int romload(const char *path, uint8_t *mem, unsigned int maxsize)
{
        int fd;
        int size;
        fd = open(path, O_RDONLY);
        if (fd == -1) {
                perror(path);
                exit(1);
        }
        size = read(fd, mem, maxsize);
        close(fd);
        return size;
}

static void cleanup(int sig)
{
        tcsetattr(0, TCSADRAIN, &saved_term);
        emulator_done = 1;
}

static void exit_cleanup(void)
{
        tcsetattr(0, TCSADRAIN, &saved_term);
        SDL_Quit();
}

void termon(void)
{
    if (tcgetattr(0, &term) == 0) {
        saved_term = term;
        atexit(exit_cleanup);
        signal(SIGINT, cleanup);
        signal(SIGQUIT, cleanup);
        signal(SIGPIPE, cleanup);
        term.c_lflag &= ~(ICANON | ECHO);
        term.c_cc[VMIN] = 0;
        term.c_cc[VTIME] = 1;
        term.c_cc[VINTR] = 0;
        term.c_cc[VSUSP] = 0;
        term.c_cc[VSTOP] = 0;
        tcsetattr(0, TCSADRAIN, &term);
    }

}
static void irqnotify(void)
{
        if (via1 && via_irq_pending(via1))
                irq6502();
        else if (uart && m6551_irq_pending(uart))
                irq6502();
        else if (vdp && tms9918a_irq_pending(vdp))
                irq6502();
}

void ui_event(void)
{
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
                switch(ev.type) {
                        case SDL_QUIT:
                                emulator_done = 1;
                                break;
                        }
        }
}
static void usage(void)
{
        fprintf(stderr, "6502retro: [-1] [-r rompath] [-S sdcard] [-T] [-f] [-d debug]\n");
        exit_cleanup();
        exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
        static struct timespec tc;
        int opt;
        int fd;
        char *rompath = "6502retro.rom";
        char *sdpath = NULL;
        unsigned have_tms = 0;
        static int tstates = 4000;      /* 1MHz */

        while ((opt = getopt(argc, argv, "d:fr:S:T")) != -1) {
                switch (opt) {
                case 'r':
                        rompath = optarg;
                        break;
                case 'S':
                        sdpath = optarg;
                        break;
                case 'd':
                        trace = atoi(optarg);
                        break;
                case 'f':
                        fast = 1;
                        break;
                case 'T':
                        have_tms = 1;
                        break;
                default:
                        usage();
                }
        }
        if (optind < argc)
                usage();

        uint16_t rsize = romload(rompath, rom, 0x2000);
        fprintf(stderr,"Loaded %04x bytes from %s into rom\n", rsize, rompath);
        if (rsize != 0x2000) {
                fprintf(stderr, "6502retro: invalid BOOT ROM\n");
                exit(EXIT_FAILURE);
        }

        via1 = via_create();
        if (via1 && trace & TRACE_VIA)
                via_trace(via1, 1);

        sdcard = sd_create("sd0");
        if (sdpath) {
                fd = open(sdpath, O_RDWR);
                if (fd == -1) {
                        perror(sdpath);
                        exit(1);
                }
                sd_attach(sdcard, fd);
                via_write(via1, 1, via_pa);     /* init LED OFF, ROM SWITCH ON, other active lows disabled */
                via_write(via1, 3, 0xD7);       /* PA5 and PA3 are inputs */
        }

        if (trace & TRACE_SD)
                sd_trace(sdcard, 1);

        sd_blockmode(sdcard);

        if (have_tms) {
                vdp = tms9918a_create();
                tms9918a_trace(vdp, !!(trace & TRACE_TMS9918A));
                vdprend = tms9918a_renderer_create(vdp);
                /* SDL init called in tms9918a_renderer_create */
        }

        /* 60Hz for the VDP */
        tc.tv_sec = 0;
        tc.tv_nsec = 12500000L;

        termon();

        con = &console;
        uart = m6551_create();
        m6551_trace(uart, trace & TRACE_6551);
        m6551_attach(uart, con);


        //if (trace & TRACE_CPU)
        //        log_6502 = 1;

        hookexternal(irqnotify);
        reset6502();

        /* This is the wrong way to do it but it's easier for the moment. We
           should track how much real time has occurred and try to keep cycle
           matched with that. The scheme here works fine except when the host
           is loaded though */

        /* We run 1000000 t-states per second */
        while (!emulator_done) {
                int i;
                for (i = 0; i < 10; i++) {
                        exec6502(tstates);
                        via_tick(via1, tstates);
                }
                ui_event();
                /* Do 20ms of I/O and delays */
                if (vdp) {
                        tms9918a_rasterize(vdp);
                        tms9918a_render(vdprend);
                }

                if (uart)
                        m6551_timer(uart);

                if (!fast)
                        nanosleep(&tc, NULL);
        }

        exit(0);
}

