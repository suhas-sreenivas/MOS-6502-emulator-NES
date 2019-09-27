#include <rc.h>
#include <base.h>
#include <membus.h>
#include <timekeeper.h>
#include <mos6502/vmcall.h>
#include <mos6502/mos6502.h>

#include <string.h>

static const uint8_t instr_cycles[256] = {
	7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
};

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

size_t
mos6502_instr_repr (mos6502_t * cpu, uint16_t addr, char * buffer, size_t buflen)
{
	// FILL ME IN

	// Delete this line when you're done
	buffer[0] = 0;
	return 0;
}

void lda_imm(mos6502_t * cpu){
	// cpu->pc = cpu->pc + 1; //added
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->a = imm_operand;
	cpu->pc = cpu->pc+1;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void ldx_imm(mos6502_t * cpu){
	// cpu->pc = cpu->pc + 1; //added
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->x = imm_operand;
	cpu->pc = cpu->pc+1;
	cpu->p.z = cpu->x == 0 ? 1 : 0;
	cpu->p.n = (cpu->x & ( 1 << 7 )) >> 7;
}

void ldy_imm(mos6502_t * cpu){
	// cpu->pc = cpu->pc + 1; //added
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->y = imm_operand;
	cpu->pc = cpu->pc+1;
	cpu->p.z = cpu->y == 0 ? 1 : 0;
	cpu->p.n = (cpu->y & ( 1 << 7 )) >> 7;
}

void sta_abs(mos6502_t * cpu){
	uint8_t low_byte = read8(cpu, cpu->pc);
	uint8_t high_byte = read8(cpu, cpu->pc+1);
	uint16_t addr = high_byte;
	addr = (addr << 8) | low_byte;
	write8(cpu, addr, cpu->a);
	cpu->pc = cpu->pc + 2;
}

void rom_end(mos6502_t * cpu){
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->pc = cpu->pc+1;
	handle_vmcall(cpu, imm_operand);
}

void (*instr_handler_array[1000])(mos6502_t *)= {
	[0xA9] = lda_imm,
	[0x80] = rom_end,
	[0xA2] = ldx_imm,
	[0xA0] = ldy_imm,
	[0x8D] = sta_abs
};

mos6502_step_result_t
mos6502_step (mos6502_t * cpu)
{
	uint8_t opcode = read8(cpu, cpu->pc);

	cpu->pc = cpu->pc + 1;
	(*instr_handler_array[opcode])(cpu);
	mos6502_advance_clk(cpu, instr_cycles[opcode]);
	return MOS6502_STEP_RESULT_SUCCESS;
}
