#include<mos6502/mos6502.h>
#include<mos6502/helper-functions.h>

uint16_t ind_abs_addr(mos6502_t * cpu, uint16_t addr){
	// uint16_t addr = abs_addr(cpu,0);
	uint8_t low_byte = read8(cpu, addr++);
	uint8_t high_byte = read8(cpu, addr);
	return ((uint16_t) high_byte << 8) + (uint16_t) low_byte;
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

void and(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a & value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void or(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a | value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
}

void eor(mos6502_t * cpu, uint8_t value){
	cpu->a = cpu->a ^ value;
	cpu->p.z = cpu->a == 0 ? 1 : 0;
	cpu->p.n = cpu->a & 0x80 ? 1 : 0;
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

void inc_dec(mos6502_t * cpu, uint16_t addr, uint8_t val){
	uint8_t target = read8(cpu, addr) + val;

	cpu->p.z = target == 0 ? 1 : 0;
	cpu->p.n = target & 0x80 ? 1 : 0;

	write8(cpu, addr, target);
}

void inc_dec_ireg(mos6502_t * cpu, uint8_t * reg, uint8_t val){
	*reg = *reg + val;
	cpu->p.z = *reg == 0 ? 1 : 0;
	cpu->p.n = *reg & 0x80 ? 1 : 0;
}

void transfer(mos6502_t * cpu, uint8_t val, uint8_t * dest){
	*dest = val;
	cpu->p.z = *dest == 0 ? 1 : 0;
	cpu->p.n = *dest & 0x80 ? 1 : 0;

}

void push(mos6502_t * cpu, uint8_t val){
	write8(cpu, 0x0100 | cpu->sp--,val);
}

uint8_t pop(mos6502_t * cpu){
	return read8(cpu, 0x0100 | ++cpu->sp);
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

void compare(mos6502_t * cpu, uint8_t val1, uint8_t val2){
	uint8_t test = val1 - val2;
	cpu->p.z = test == 0 ? 1 : 0;
	cpu->p.n = test & 0x80 ? 1 : 0;
	cpu->p.c = test >= 0 ? 1 : 0;

}

void rel_addr_branch(mos6502_t * cpu){
	uint8_t t = read8(cpu, cpu->pc++);
	if(t & 0x80)
       	cpu->pc += (t-0x100);
    else
       	cpu->pc += t;
}