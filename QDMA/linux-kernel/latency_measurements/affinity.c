#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

static volatile uint32_t *reg_girq_ctrl; 
static volatile uint32_t *reg_girq_trig; 
static volatile uint32_t *reg_girq_stat;
static volatile uint32_t *reg_girq_cmpt; 
static volatile uint32_t *reg_girq_ts0;
static volatile uint32_t *reg_girq_ts1;
static volatile uint32_t *reg_girq_ts2;
static volatile uint32_t *reg_girq_counter;

static off_t GIRQ_BASE_ADDR = 0x101000;
static int N_REGS = 7;
static int bar2_fd;
static void *map_base;
static int map_size = 4096UL;

static int mm_registers_init() {	
	if((bar2_fd = open("/sys/bus/pci/devices/0000:0a:00.0/resource2", O_RDWR | O_SYNC)) == -1) return -1;

	off_t target_base = GIRQ_BASE_ADDR & ~(sysconf(_SC_PAGE_SIZE)-1);
	if (GIRQ_BASE_ADDR + N_REGS*4 - target_base > map_size) 
		map_size = GIRQ_BASE_ADDR + N_REGS*4 - target_base;

	map_base = mmap(0, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, bar2_fd, target_base);
    if(map_base == (void *) -1) return -1;

	reg_girq_ctrl = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 0*4 - target_base);
	reg_girq_trig = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 1*4 - target_base);
	reg_girq_stat = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 2*4 - target_base);
	reg_girq_cmpt = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 3*4 - target_base);
	reg_girq_ts0 = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 4*4 - target_base);
	reg_girq_ts1 = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 5*4 - target_base);
	reg_girq_ts2 = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 6*4 - target_base);
	reg_girq_counter = (uint32_t *) (map_base + GIRQ_BASE_ADDR + 7*4 - target_base);
	return 0;
}

static int mm_registers_teardown() {
	if(munmap(map_base, map_size) == -1) return -1;
    close(bar2_fd);
	return 0;
}

int main(int argc, char **argv) {

	if (argc <= 1) {
        printf("Usage: ./affinity <number of interrupts per vector>");
        exit(1);
    }

	mm_registers_init();
	
	int n = atoi(argv[1]);

    for (int j = 0; j < 8; j++) {
        *(reg_girq_ctrl) = (uint32_t) j;
        for (int i = 0; i < n; i++) {
            *(reg_girq_trig) = 1;
            while (!*(reg_girq_cmpt));
            // usleep(1000);
        }
    }
    
	if (mm_registers_teardown()) exit(1);
    exit(0);
}