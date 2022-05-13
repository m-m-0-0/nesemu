#pragma once

#ifndef CPU_H
#define CPU_H

#include "addressing_modes.h"

class ppu;

#include <limits.h>
#include <stdint.h>
#include <map>
#include <iostream>
#include <fstream>
#include <functional>
#include <bitset>

class cpu {
private:
	// Memory
	// ======
	// 0x100 => Zero Page
	// 0x200 => Stack
	// 0x800 => RAM
	// 0x2000 => Mirrors (0-0x7FF)
	// 0x2008 => I/O Registers
	// 0x4000 => Mirrors (0x2000-0x2007)
	// 0x4020 => I/O Registers
	// 0x6000 => Expansion ROM
	// 0x8000 => SRAM
	// 0xC000 => PRG-ROM (Lower Bank)
	// 0x10000 => PRG-ROM (Upper Bank)

	ppu* PPU;

	uint8_t memory[0xFFFF];

	//registers
	uint8_t A;
	uint8_t B;
	uint8_t X;
	uint8_t Y;
	uint16_t PC;
	uint8_t S;
	uint8_t P;

	//flags
	bool flag_C;
	bool flag_Z;
	bool flag_U;
	bool flag_I;
	bool flag_D;
	bool flag_B;
	bool flag_V;
	bool flag_N;

	struct instruction {
		int (cpu::*func)(uint16_t, bool);
		addressing_mode addr_mode;
		bool reg_only;
		int cycles;
	};

	void bind_ppu(ppu* p);

	//flags methods
	uint8_t flags_to_byte(bool irq = false);
	uint8_t flags_to_byte_p();
	void byte_to_flags(uint8_t byte, bool irq = false);
	
	bool same_page(uint16_t a1, uint16_t a2);
	
	//stack methods
	void push_stack(uint8_t value);
	void push_stack_16(uint16_t value);
	uint8_t pop_stack();
	uint16_t pop_stack_16();
	
	//set instruction method
	void set_instr(int pos, int(cpu::*f)(uint16_t, bool), addressing_mode a, int c);

	//INSTRUCTION MAP
	instruction instr[0xFF];

	int adc(uint16_t address, bool reg_only = false);
	int and(uint16_t address, bool reg_only = false);
	int asl(uint16_t address, bool reg_only = false);
	int bcc(uint16_t address, bool reg_only = true);
	int bcs(uint16_t address, bool reg_only = true);
	int beq(uint16_t address, bool reg_only = true);
	int bit(uint16_t address, bool reg_only = false);
	int bmi(uint16_t address, bool reg_only = true);
	int bne(uint16_t address, bool reg_only = true);
	int bpl(uint16_t address, bool reg_only = true);
	int brk(uint16_t address, bool reg_only = true);
	int bvc(uint16_t address, bool reg_only = true); 
	int bvs(uint16_t address, bool reg_only = true);
	int clc(uint16_t address, bool reg_only = true);
	int cld(uint16_t address, bool reg_only = true);
	int cli(uint16_t address, bool reg_only = true);
	int clv(uint16_t address, bool reg_only = true);
	int cmp(uint16_t address, bool reg_only = false); 
	int cpx(uint16_t address, bool reg_only = false);
	int cpy(uint16_t address, bool reg_only = false);
	int dec(uint16_t address, bool reg_only = false);
	int dex(uint16_t address, bool reg_only = true);
	int dey(uint16_t address, bool reg_only = true);
	int eor(uint16_t address, bool reg_only = false);
	int inc(uint16_t address, bool reg_only = false);
	int inx(uint16_t address, bool reg_only = true);
	int iny(uint16_t address, bool reg_only = true);
	int jmp(uint16_t address, bool reg_only = false);
	int jsr(uint16_t address, bool reg_only = false);
	int lda(uint16_t address, bool reg_only = false);
	int ldx(uint16_t address, bool reg_only = false);
	int ldy(uint16_t address, bool reg_only = false);
	int lsr(uint16_t address, bool reg_only = false);
	int nop(uint16_t address, bool reg_only = true);
	int ora(uint16_t address, bool reg_only = false);
	int pha(uint16_t address, bool reg_only = true);
	int php(uint16_t address, bool reg_only = true);
	int pla(uint16_t address, bool reg_only = true);
	int plp(uint16_t address, bool reg_only = true);
	int rol(uint16_t address, bool reg_only = false);
	int ror(uint16_t address, bool reg_only = false);
	int rti(uint16_t address, bool reg_only = true);
	int rts(uint16_t address, bool reg_only = true);
	int sbc(uint16_t address, bool reg_only = false);
	int sec(uint16_t address, bool reg_only = true);
	int sed(uint16_t address, bool reg_only = true);
	int sei(uint16_t address, bool reg_only = true);
	int sta(uint16_t address, bool reg_only = true);
	int stx(uint16_t address, bool reg_only = true);
	int sty(uint16_t address, bool reg_only = true);
	int tax(uint16_t address, bool reg_only = true);
	int tay(uint16_t address, bool reg_only = true);
	int tsx(uint16_t address, bool reg_only = true);
	int txa(uint16_t address, bool reg_only = true);
	int txs(uint16_t address, bool reg_only = true);
	int tya(uint16_t address, bool reg_only = true);

public:
	uint8_t read_memory(uint16_t address);
	void write_memory(uint16_t address, uint8_t byte);

	void reset();

	inline bool check_overflow(int a, int b, int size);
	inline bool check_overflow_signed(int a, int b, int size);
	inline bool check_overflow_signed(uint8_t a, uint8_t b);
	inline bool check_negative(uint8_t V);

	void init();

	uint8_t fetch();
	uint16_t fetch_16();

	int exec(uint8_t opcode);

	int cycles = 0;
	uint8_t prg_size;
	uint8_t chr_size;
	bool trainer;
	uint8_t ram_size;
	uint8_t mapper;
	std::string romstr;
	char* romchr;
	uint8_t* rom;

	void load(std::string path);

	//MAPPERS
	const bool READ = 0;
	const bool WRITE = 1;
	const int HEADER_SIZE = 16;

	uint8_t mapper_0(uint16_t address, bool mode, uint8_t value = 0);

	void p_r();

	void run();
	int advance();
};
#endif