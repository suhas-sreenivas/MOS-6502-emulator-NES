#include <membus.h>
#include<mos6502/mos6502.h>

static inline uint8_t
read8 (mos6502_t * cpu, uint16_t addr)
{
	return membus_read(cpu->bus, addr);
}

static inline void
write8 (mos6502_t * cpu, uint16_t addr, uint8_t val)
{
	membus_write(cpu->bus, addr, val);
}

static inline uint16_t
read16 (mos6502_t * cpu, uint16_t addr)
{
	uint16_t lo = (uint16_t)read8(cpu, addr);
	uint16_t hi = (uint16_t)read8(cpu, addr + 1);
	uint16_t val = lo | (uint16_t)(hi << 8);
	return val;
}

static inline uint16_t
buggy_read16 (mos6502_t * cpu, uint16_t addr)
{
	uint16_t first = addr;
    uint16_t msb = addr & 0xff00;
    uint16_t lsb = ((addr & 0xff) == 0xff) ? 0 : ((addr & 0xff) + 1);
	uint16_t secnd = msb | lsb;
	uint16_t lo = (uint16_t)read8(cpu, first);
	uint16_t hi = (uint16_t)read8(cpu, secnd);
	uint16_t val = (uint16_t)(hi << 8) | lo;
	return val;
}

uint16_t ind_abs_addr(mos6502_t * cpu, uint16_t addr);

uint16_t abs_addr(mos6502_t * cpu, uint8_t offset);

uint16_t index_indirect_addr(mos6502_t * cpu, uint8_t offset);

uint16_t indirect_index_addr(mos6502_t * cpu, uint8_t offset);

void add(mos6502_t * cpu, uint16_t value);

void and(mos6502_t * cpu, uint8_t value);

void or(mos6502_t * cpu, uint8_t value);

void eor(mos6502_t * cpu, uint8_t value);

void rot(mos6502_t * cpu, uint8_t * target, uint8_t rot, bool left);

void inc_dec(mos6502_t * cpu, uint16_t addr, uint8_t val);

void inc_dec_ireg(mos6502_t * cpu, uint8_t * reg, uint8_t val);

void transfer(mos6502_t * cpu, uint8_t val, uint8_t * dest);

void push(mos6502_t * cpu, uint8_t val);

uint8_t pop(mos6502_t * cpu);

void sub(mos6502_t * cpu, uint16_t value);

void compare(mos6502_t * cpu, uint8_t val1, uint8_t val2);

void rel_addr_branch(mos6502_t * cpu);

