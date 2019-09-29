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

uint16_t abs_addr(mos6502_t * cpu, uint8_t offset){
	uint8_t low_byte = read8(cpu, cpu->pc++);
	uint8_t high_byte = read8(cpu, cpu->pc++);
	return ((uint16_t) high_byte << 8) + (uint16_t) low_byte + (uint16_t) offset;
}

uint16_t index_indirect_addr(mos6502_t * cpu, uint8_t offset){
	uint8_t target = read8(cpu, cpu->pc++) + offset;
	return read16(cpu, target);
}

uint16_t indirect_index_addr(mos6502_t * cpu, uint8_t offset){
	uint8_t target = read8(cpu, cpu->pc++);
	return read16(cpu, target) + offset;
}

void lda_imm(mos6502_t * cpu){
	// cpu->pc = cpu->pc + 1; //added
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->a = imm_operand;
	cpu->pc = cpu->pc+1;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void lda_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	cpu->a = read8(cpu, addr);
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void lda_zp(mos6502_t * cpu){
	cpu->a = read8(cpu, read8(cpu, cpu->pc++));
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void lda_zp_x(mos6502_t * cpu){
	cpu->a = read8(cpu, (uint8_t)(read8(cpu, cpu->pc++) + cpu->x));
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void lda_idx_idr(mos6502_t * cpu){
	uint16_t addr = index_indirect_addr(cpu, cpu->x);
	cpu->a = read8(cpu, addr);
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = (cpu->a & ( 1 << 7 )) >> 7;
}

void lda_idr_idx(mos6502_t * cpu){
	uint16_t addr = indirect_index_addr(cpu, cpu->y);
	cpu->a = read8(cpu, addr);
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
	// uint8_t low_byte = read8(cpu, cpu->pc);
	// uint8_t high_byte = read8(cpu, cpu->pc+1);
	// uint16_t addr = high_byte;
	// addr = (addr << 8) | low_byte;
	// write8(cpu, addr, cpu->a);
	write8(cpu, abs_addr(cpu,0), cpu->a);
	// cpu->pc = cpu->pc + 2;
}

void sta_abs_x(mos6502_t * cpu){
	write8(cpu,abs_addr(cpu,cpu->x),cpu->a);
}

void sta_abs_y(mos6502_t * cpu){
	write8(cpu,abs_addr(cpu,cpu->y),cpu->a);
}

void sta_zp(mos6502_t * cpu){
	uint16_t zp_addr = read8(cpu, cpu->pc++);
	write8(cpu,zp_addr,cpu->a);
}

void sta_zp_x(mos6502_t * cpu){
	uint16_t zp_addr = read8(cpu, cpu->pc++);
	write8(cpu,zp_addr+cpu->x,cpu->a);
}

void stx_abs(mos6502_t * cpu){
	write8(cpu, abs_addr(cpu,0), cpu->x);
}

void stx_zp(mos6502_t * cpu){
	write8(cpu, read8(cpu, cpu->pc++), cpu->x);
}

void stx_zp_y(mos6502_t * cpu){
	uint16_t zp_addr = read8(cpu, cpu->pc++);
	write8(cpu,(uint8_t)(zp_addr+cpu->y),cpu->x);
}

void sty_abs(mos6502_t * cpu){
	write8(cpu, abs_addr(cpu,0), cpu->y);
}

void sty_zp(mos6502_t * cpu){
	uint16_t zp_addr = read8(cpu, cpu->pc++);
	write8(cpu,zp_addr,cpu->y);
}

void sty_zp_x(mos6502_t * cpu){
	uint16_t zp_addr = read8(cpu, cpu->pc++);
	write8(cpu,zp_addr+cpu->x,cpu->y);
}

void add(mos6502_t * cpu, uint16_t value){
	uint16_t sum = value + cpu->a + cpu->p.c;
	//flags set or unset below
	cpu->p.n = sum & 0x80 ? 1 : 0;
	cpu->p.c = sum & 0xFF00 ? 1:0;
	// cpu->p.v = (!((cpu->a ^ value) & 0x80) && ((cpu->a ^ sum) & 0x80));
	cpu->p.v = ((cpu->a ^ sum)  & (value ^ sum) & 0x80) ? 1 : 0;

	cpu->a = sum;
	cpu->p.z = cpu->a == 0 ? 1 : 0;

}

void adc_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	add(cpu, read8(cpu, addr));
}

void adc_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	add(cpu, read8(cpu, addr));
}

void adc_imm(mos6502_t * cpu){
	add(cpu, read8(cpu, cpu->pc++));
}

void and(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a & value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void and_imm(mos6502_t * cpu){
	and(cpu,read8(cpu, cpu->pc++));
}

void and_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	and(cpu, read8(cpu,addr));
}

void and_zp(mos6502_t * cpu){
	uint8_t addr = read8(cpu, cpu->pc++);
	and(cpu, read8(cpu, addr));
}

void rot(mos6502_t * cpu, uint8_t * target, uint8_t rot, bool left){
	// uint8_t value = read8(cpu, addr);
	if(left){
		cpu->p.c = *target & 0x80;
		*target = (*target << 1) | rot;
	}
	else{
		if(rot) rot = 128;
		cpu->p.c = *target & 0x01;
		*target = (*target >> 1) | rot;
	}

	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = *target & 0x80 ? 1 : 0;

	// write8(cpu, addr, value);
}

void asl_a(mos6502_t * cpu){
	// cpu->p.c = cpu->a & 0x80;

	// cpu->a = cpu->a << 1;

	// cpu->p.z = cpu->a == 0 ? 1 : 0;
	// cpu->p.n = cpu->a & 0x80 ? 1 : 0;
	rot(cpu, &cpu->a, 0, 1);
}

void asl_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	uint8_t to_shift = read8(cpu, addr);
	rot(cpu, &to_shift, 0, 1);
	write8(cpu, addr, to_shift);

}

void lsr_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	uint8_t to_shift = read8(cpu, addr);
	rot(cpu, &to_shift, 0, 0);
	write8(cpu, addr, to_shift);

}

void rol_a(mos6502_t * cpu){
	rot(cpu, &cpu->a, cpu->p.c, 1);
}

void rol_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	uint8_t to_shift = read8(cpu, addr);
	rot(cpu, &to_shift, cpu->p.c, 1);
	write8(cpu, addr, to_shift);

}

void ror_a(mos6502_t * cpu){
	rot(cpu, &cpu->a, cpu->p.c, 0);
}

void ror_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	uint8_t to_shift = read8(cpu, addr);
	rot(cpu, &to_shift, cpu->p.c, 0);
	write8(cpu, addr, to_shift);

}

void sec(mos6502_t * cpu){
	cpu->p.c = 1;
}

void clc(mos6502_t * cpu){
	cpu->p.c = 0;
}

void sed(mos6502_t * cpu){
	cpu->p.d = 1;
}

void cld(mos6502_t * cpu){
	cpu->p.d = 0;
}

void sei(mos6502_t * cpu){
	cpu->p.i = 1;
}

void cli(mos6502_t * cpu){
	cpu->p.i = 0;
}

void clv(mos6502_t * cpu){
	cpu->p.v = 0;
}

void or(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a | value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void or_imm(mos6502_t * cpu){
	or(cpu,read8(cpu, cpu->pc++));
}

void or_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	or(cpu, read8(cpu,addr));
}

void eor(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a ^ value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void eor_imm(mos6502_t * cpu){
	eor(cpu,read8(cpu, cpu->pc++));
}

void eor_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	eor(cpu, read8(cpu,addr));
}

void eor_abs_y(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, cpu->y);
	eor(cpu, read8(cpu,addr));
}

void inc_dec(mos6502_t * cpu, uint16_t addr, uint8_t val){
	uint8_t target = read8(cpu, addr) + val;

	cpu->p.z = target == 0 ? 1 : 0;
	cpu->p.n = target & 0x80 ? 1 : 0;

	write8(cpu, addr, target);
}

void inc_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	inc_dec(cpu, addr, 1);
}

void inc_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	inc_dec(cpu, addr, 1);
}

void dec_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	inc_dec(cpu, addr, -1);
}

void dec_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	inc_dec(cpu, addr, -1);
}

void inc_dec_ireg(mos6502_t * cpu, uint8_t * reg, uint8_t val){
	*reg = *reg + val;
	cpu->p.z = *reg == 0 ? 1 : 0;
	cpu->p.n = *reg & 0x80 ? 1 : 0;
}

void inx(mos6502_t * cpu){
	inc_dec_ireg(cpu, &cpu->x, 1);
}

void iny(mos6502_t * cpu){
	inc_dec_ireg(cpu, &cpu->y, 1);
}

void dex(mos6502_t * cpu){
	inc_dec_ireg(cpu, &cpu->x, -1);
}

void dey(mos6502_t * cpu){
	inc_dec_ireg(cpu, &cpu->y, -1);
}

void transfer(mos6502_t * cpu, uint8_t val, uint8_t * dest){
	*dest = val;
	cpu->p.z = *dest == 0 ? 1 : 0;
	cpu->p.n = *dest & 0x80 ? 1 : 0;

}

void tax(mos6502_t * cpu){
	transfer(cpu, cpu->a, &cpu->x);
}

void tay(mos6502_t * cpu){
	transfer(cpu, cpu->a, &cpu->y);
}

void txa(mos6502_t * cpu){
	transfer(cpu, cpu->x, &cpu->a);
}

void tya(mos6502_t * cpu){
	transfer(cpu, cpu->y, &cpu->a);
}

void push(mos6502_t * cpu, uint8_t val){
	write8(cpu, 0x0100 | cpu->sp--,val);
}

uint8_t pop(mos6502_t * cpu){
	return read8(cpu, 0x0100 | ++cpu->sp);
}

void pha(mos6502_t * cpu){
	push(cpu, cpu->a);
}

void pla(mos6502_t * cpu){
	cpu->a = pop(cpu);
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void php(mos6502_t * cpu){
	push(cpu, cpu->p.val);
}

void plp(mos6502_t * cpu){
	cpu->p.val = 0x20 | pop(cpu);
}

void tsx(mos6502_t * cpu){
	transfer(cpu, cpu->sp, &cpu->x);
}

void txs(mos6502_t * cpu){
	cpu->sp = cpu->x;
}

void sub(mos6502_t * cpu, uint16_t value){
	uint16_t res = cpu->a - value - (1 - cpu->p.c);
	//flags set or unset below
	cpu->p.n = res & 0x80 ? 1 : 0;
	cpu->p.c = res & 0xFF00 ? 0 : 1;
	cpu->p.v = (cpu->a ^ value) & (cpu->a ^ res) & 0x80 ? 1 : 0;
	// cpu->p.v = (!((cpu->a ^ value) & 0x80) && ((cpu->a ^ res) & 0x80));
	// if(cpu->p.v) INFO_PRINT("a:%04x, val:%04x, res:%04x", cpu->a, value, res);
	cpu->a = res;
	cpu->p.z = cpu->a == 0 ? 1 : 0;

}

void sbc_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	sub(cpu, read8(cpu, addr));
}

void sbc_zp(mos6502_t * cpu){
	uint16_t addr = read8(cpu, cpu->pc++);
	sub(cpu, read8(cpu, addr));
}

void jmp_abs(mos6502_t * cpu){
	cpu->pc = abs_addr(cpu,0);
}

uint16_t ind_abs_addr(mos6502_t * cpu, uint16_t addr){
	// uint16_t addr = abs_addr(cpu,0);
	uint8_t low_byte = read8(cpu, addr++);
	uint8_t high_byte = read8(cpu, addr);
	return ((uint16_t) high_byte << 8) + (uint16_t) low_byte;
}

void jmp_ind(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	if(addr & 0x00FF) cpu->pc = buggy_read16(cpu, addr);
	else cpu->pc = ind_abs_addr(cpu, addr);
}

void jsr_abs(mos6502_t * cpu){
	push(cpu, (cpu->pc+1 & 0xFF00) >> 8);
	push(cpu, cpu->pc +1 & 0x00FF);
	cpu->pc = abs_addr(cpu, 0);
}

void rts(mos6502_t * cpu){
	uint8_t low_byte = pop(cpu);
	uint8_t high_byte = pop(cpu);
	cpu->pc = 1 + ((uint16_t) high_byte << 8) + (uint16_t) low_byte;
}

void compare(mos6502_t * cpu, uint8_t val1, uint8_t val2){
	uint8_t test = val1 - val2;
	cpu->p.z = test == 0 ? 1 : 0;
	cpu->p.n = test & 0x80 ? 1 : 0;
	cpu->p.c = test >= 0 ? 1 : 0;

}

void cmp_abs(mos6502_t * cpu){
	compare(cpu, cpu->a, read8(cpu, abs_addr(cpu,0)));
}

void cmp_imm(mos6502_t * cpu){
	compare(cpu, cpu->a, read8(cpu, cpu->pc++));
}

void cpx_imm(mos6502_t * cpu){
	compare(cpu, cpu->x, read8(cpu, cpu->pc++));
}

void cpx_abs(mos6502_t * cpu){
	compare(cpu, cpu->x, read8(cpu, abs_addr(cpu,0)));
}

void cpy_abs(mos6502_t * cpu){
	compare(cpu, cpu->y, read8(cpu, abs_addr(cpu,0)));
}

void rel_addr_branch(mos6502_t * cpu){
	uint8_t t = read8(cpu, cpu->pc++);
	if(t & 0x80)
       	cpu->pc += (t-0x100);
    else
       	cpu->pc += t;
}

void bcc(mos6502_t * cpu){
	if(cpu->p.c == 0){
		// uint8_t t = read8(cpu, cpu->pc++);
		// if(t & 0x80)
        // 	cpu->pc += (t-0x100);
      	// else
        // 	cpu->pc += t;
		rel_addr_branch(cpu);
	}
	else
		cpu->pc ++;
}

void bcs(mos6502_t * cpu){
	if(cpu->p.c == 1)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void beq(mos6502_t * cpu){
	if(cpu->p.z == 1)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bne(mos6502_t * cpu){
	if(cpu->p.z == 0)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bpl(mos6502_t * cpu){
	if(cpu->p.n == 0)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bmi(mos6502_t * cpu){
	if(cpu->p.n == 1)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bvc(mos6502_t * cpu){
	if(cpu->p.v == 0)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bvs(mos6502_t * cpu){
	if(cpu->p.v == 1)
		rel_addr_branch(cpu);
	else
		cpu->pc ++;
}

void bit_abs(mos6502_t * cpu){
	uint16_t addr = abs_addr(cpu, 0);
	uint8_t mem_operand = read8(cpu, addr);
	uint8_t res = cpu->a & mem_operand;

	cpu->p.n = mem_operand & 0x80 ? 1 : 0;
	cpu->p.v = mem_operand & 0x40 ? 1 : 0;
	cpu->p.z = res == 0 ? 1 : 0;
}

void brk(mos6502_t * cpu){
	cpu->pc++;
	push(cpu, (cpu->pc & 0xFF00) >> 8);
	push(cpu,  cpu->pc & 0x00FF);
	push(cpu, cpu->p.val);
	cpu->pc = read16(cpu, 0xFFFE);
	cpu->p.b = 1;
}

void rti(mos6502_t * cpu){
	cpu->p.val = pop(cpu);
	uint8_t low_byte = pop(cpu);
	uint8_t high_byte = pop(cpu);
	cpu->pc = ((uint16_t) high_byte << 8) + (uint16_t) low_byte;
}

void nop(mos6502_t * cpu){

}

void rom_end(mos6502_t * cpu){
	uint8_t imm_operand = read8(cpu, cpu->pc);
	cpu->pc = cpu->pc+1;
	handle_vmcall(cpu, imm_operand);
}

void (*instr_handler_array[1000])(mos6502_t *)= {
	[0x80] = rom_end,

	[0xA9] = lda_imm,
	[0xAD] = lda_abs,
	[0xA5] = lda_zp,
	[0xB5] = lda_zp_x,
	[0xA1] = lda_idx_idr,
	[0xB1] = lda_idr_idx,

	[0xA2] = ldx_imm,

	[0xA0] = ldy_imm,

	[0x8D] = sta_abs,
	[0x9D] = sta_abs_x,
	[0x99] = sta_abs_y,

	[0x85] = sta_zp,
	[0x95] = sta_zp_x,

	[0x8E] = stx_abs,
	[0x86] = stx_zp,
	[0x96] = stx_zp_y,

	[0x8C] = sty_abs,
	[0x84] = sty_zp,
	[0x94] = sty_zp_x,

	[0x6D] = adc_abs,
	[0x69] = adc_imm,
	[0x65] = adc_zp,

	[0x29] = and_imm,
	[0x2D] = and_abs,
	[0x25] = and_zp,

	[0x09] = or_imm,
	[0x0D] = or_abs,

	[0x49] = eor_imm,
	[0x4D] = eor_abs,
	[0x59] = eor_abs_y,

	[0x0A] = asl_a,
	[0x0E] = asl_abs,

	[0x46] = lsr_zp,

	[0x2A] = rol_a,
	[0x2E] = rol_abs,

	[0x6A] = ror_a,
	[0x66] = ror_zp,

	[0x38] = sec,
	[0x18] = clc,
	[0xF8] = sed,
	[0xD8] = cld,
	[0x78] = sei,
	[0x58] = cli,
	[0xB8] = clv,

	[0xEA] = nop,

	[0xEE] = inc_abs,
	[0xE6] = inc_zp,

	[0xCE] = dec_abs,
	[0xC6] = dec_zp,

	[0xE8] = inx,
	[0xC8] = iny,
	[0xCA] = dex,
	[0x88] = dey,

	[0xAA] = tax,
	[0xA8] = tay,
	[0x8A] = txa,
	[0x98] = tya,

	[0x48] = pha,
	[0x68] = pla,
	[0x08] = php,
	[0x28] = plp,

	[0xBA] = tsx,
	[0x9A] = txs,

	[0xED] = sbc_abs,
	[0xE5] = sbc_zp,

	[0x4C] = jmp_abs,
	[0x6C] = jmp_ind,

	[0x20] = jsr_abs,
	[0x60] = rts,

	[0xCD] = cmp_abs,
	[0xC9] = cmp_imm,

	[0xE0] = cpx_imm,
	[0xEC] = cpx_abs,

	[0xCC] = cpy_abs,

	[0x90] = bcc,
	[0xB0] = bcs,
	[0xF0] = beq,
	[0xD0] = bne,
	[0x10] = bpl,
	[0x30] = bmi,
	[0x50] = bvc,
	[0x70] = bvs,

	[0x2C] = bit_abs,

	[0x00] = brk,
	[0x40] = rti
};

mos6502_step_result_t
mos6502_step (mos6502_t * cpu)
{
	uint8_t opcode = read8(cpu, cpu->pc);
	// INFO_PRINT("P:%04x", cpu->p.val);
	// INFO_PRINT("%04x", opcode);
	cpu->pc = cpu->pc + 1;
	(*instr_handler_array[opcode])(cpu);
	mos6502_advance_clk(cpu, instr_cycles[opcode]);
	return MOS6502_STEP_RESULT_SUCCESS;
}
