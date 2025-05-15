/* vim: set ts=8 sw=8 et: */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <SDL2/SDL.h>

#include "6502.h"
#include "serialdevice.h"
#include "ttycon.h"
#include "6522.h"
#include "6551.h"
#include "sdcard.h"
#include "tms9918a.h"
#include "tms9918a_render.h"

#define TRACE_MEM       0x000001
#define TRACE_IRQ       0x000002
#define TRACE_CPU       0x000004
#define TRACE_6551      0x000008
#define TRACE_SD        0x000010
#define TRACE_SPI       0x000020
#define TRACE_TMS9918A  0x000040
#define TRACE_VIA       0x000060

#define SD_CLK          0x01
#define SD_CS           0x02
#define ROM_SWITCH      0x40
#define SD_MOSI         0x80

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

static void spi_clock_high();

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
                if (addr == 0xBF21)             // via port a
                {
                        if (!(val & SD_CS))             // SD_CS is asserted
                        {
                                if ((val & SD_CLK) && (old_spi_clk == 0))    // Clock is high and old clock was low
                                {
                                        old_spi_clk = 1;
                                        spi_clock_high();
                                }
                                else if (!(val & SD_CLK) && (old_spi_clk == 1)) // clock is low and old clock was high
                                {
                                        old_spi_clk = 0;
                                }
                        }
                }
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
}

void via_handshake_a(struct via6522 *via)
{
}

void via_handshake_b(struct via6522 *via)
{
}

static uint8_t bitcnt;
static uint8_t txbits, rxbits;

static void spi_clock_high(void)
{
        return;
        txbits <<=1;
        txbits |= (via_get_port_a(via1) & 0x7F) >> 7; // Gather MOSI and send it to sdcard after 8th bit
        bitcnt++;
        if (bitcnt == 8) {
                rxbits = sd_spi_in(sdcard, txbits);
                via_write(via1,10, rxbits);           // Just save the received bits in via->sr so the bios can collect
                if (trace & TRACE_SPI)
                        fprintf(stderr, "spi %02X | %02X\n", rxbits, txbits);
                bitcnt = 0;
        }
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
}

static void irqnotify(void)
{
        if (via1 && via_irq_pending(via1))
                irq6502();
        else if (uart && m6551_irq_pending(uart))
                irq6502();
}

static void usage(void)
{
        fprintf(stderr, "6502retro: [-1] [-r rompath] [-S sdcard] [-T] [-f] [-d debug]\n");
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
        static int tstates = 100;      /* 1MHz */

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
                via_write(via1, 1, via_pa);       /* init LED OFF, ROM SWITCH ON, other active lows disabled */
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
        tc.tv_nsec = 16666667L;

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

        uart = m6551_create();
        m6551_trace(uart, trace & TRACE_6551);
        m6551_attach(uart, &console);


        if (trace & TRACE_CPU)
                log_6502 = 1;

        init6502();
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
                /* Do 20ms of I/O and delays */
                if (vdp) {
                        tms9918a_rasterize(vdp);
                        tms9918a_render(vdprend);
                }

                m6551_timer(uart);
                if (!fast)
                        nanosleep(&tc, NULL);
        }
        exit(0);
}

