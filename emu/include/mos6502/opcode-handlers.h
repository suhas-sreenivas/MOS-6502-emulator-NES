#include <mos6502/mos6502.h>

void lda_imm(mos6502_t * cpu);
void lda_abs(mos6502_t * cpu);
void lda_zp(mos6502_t * cpu);
void lda_zp_x(mos6502_t * cpu);
void lda_idx_idr(mos6502_t * cpu);
void lda_idr_idx(mos6502_t * cpu);
void ldx_imm(mos6502_t * cpu);
void ldy_imm(mos6502_t * cpu);

void sta_abs(mos6502_t * cpu);
void sta_abs_x(mos6502_t * cpu);
void sta_abs_y(mos6502_t * cpu);
void sta_zp(mos6502_t * cpu);
void sta_zp_x(mos6502_t * cpu);

void stx_abs(mos6502_t * cpu);
void stx_zp(mos6502_t * cpu);
void stx_zp_y(mos6502_t * cpu);

void sty_abs(mos6502_t * cpu);
void sty_zp(mos6502_t * cpu);
void sty_zp_x(mos6502_t * cpu);

void adc_abs(mos6502_t * cpu);
void adc_zp(mos6502_t * cpu);
void adc_imm(mos6502_t * cpu);

void and_imm(mos6502_t * cpu);
void and_abs(mos6502_t * cpu);
void and_zp(mos6502_t * cpu);

void asl_a(mos6502_t * cpu);
void asl_abs(mos6502_t * cpu);

void lsr_zp(mos6502_t * cpu);

void rol_a(mos6502_t * cpu);
void rol_abs(mos6502_t * cpu);

void ror_a(mos6502_t * cpu);
void ror_zp(mos6502_t * cpu);

void sec(mos6502_t * cpu);
void clc(mos6502_t * cpu);
void sed(mos6502_t * cpu);
void cld(mos6502_t * cpu);
void sei(mos6502_t * cpu);
void cli(mos6502_t * cpu);
void clv(mos6502_t * cpu);

void or_imm(mos6502_t * cpu);
void or_abs(mos6502_t * cpu);

void eor_imm(mos6502_t * cpu);
void eor_abs(mos6502_t * cpu);
void eor_abs_y(mos6502_t * cpu);

void inc_abs(mos6502_t * cpu);
void inc_zp(mos6502_t * cpu);

void dec_abs(mos6502_t * cpu);
void dec_zp(mos6502_t * cpu);

void inx(mos6502_t * cpu);
void iny(mos6502_t * cpu);
void dex(mos6502_t * cpu);
void dey(mos6502_t * cpu);

void tax(mos6502_t * cpu);
void tay(mos6502_t * cpu);
void txa(mos6502_t * cpu);
void tya(mos6502_t * cpu);

void pha(mos6502_t * cpu);
void pla(mos6502_t * cpu);
void php(mos6502_t * cpu);
void plp(mos6502_t * cpu);

void tsx(mos6502_t * cpu);
void txs(mos6502_t * cpu);

void sbc_abs(mos6502_t * cpu);
void sbc_zp(mos6502_t * cpu);

void jmp_abs(mos6502_t * cpu);
void jmp_ind(mos6502_t * cpu);
void jsr_abs(mos6502_t * cpu);

void rts(mos6502_t * cpu);

void cmp_abs(mos6502_t * cpu);
void cmp_imm(mos6502_t * cpu);

void cpx_imm(mos6502_t * cpu);
void cpx_abs(mos6502_t * cpu);

void cpy_abs(mos6502_t * cpu);

void bcc(mos6502_t * cpu);
void bcs(mos6502_t * cpu);
void beq(mos6502_t * cpu);
void bne(mos6502_t * cpu);
void bpl(mos6502_t * cpu);
void bmi(mos6502_t * cpu);
void bvc(mos6502_t * cpu);
void bvs(mos6502_t * cpu);

void bit_abs(mos6502_t * cpu);

void brk(mos6502_t * cpu);

void rti(mos6502_t * cpu);

void nop(mos6502_t * cpu);

void rom_end(mos6502_t * cpu);

void (*instr_handler_array[1000])(mos6502_t *);