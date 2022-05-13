#include "ppu.h"
#include "cpu.h"
#include "addressing_modes.h"

#include "utils.cpp"

#include <limits.h>
#include <stdint.h>
#include <map>
#include <iostream>
#include <fstream>
#include <functional>
#include <bitset>

bool u8_contains(uint8_t A[], int size, uint8_t elem) {
	for (int i = 0; i < size; i++)
		if (A[i] == elem)
			return true;

	return false;
}

char* readFileBytes(std::string name) {
	std::ifstream fl(name, std::ios::binary | std::ios::ate);
	fl.seekg(0, std::ios::end);
	size_t len = fl.tellg();
	char *ret = new char[len];
	fl.seekg(0, std::ios::beg);
	fl.read(&ret[0], len);
	fl.close();
	return ret;
}

//add 1 to cycles if page boundary is crossed
uint8_t check_boundary[23] = { 0x7d, 0x79, 0x71, 0x3d, 0x39, 0x31, 0xdd, 0xd9, 0xd1, 0x5d, 0x59, 0x51, 0xbd, 0xb9, 0xb1, 0xbe, 0xbc, 0x1d, 0x19, 0x11, 0xfd, 0xf9, 0xf1 };

const uint16_t irqVectorH = 0xFFFF;
const uint16_t irqVectorL = 0xFFFE;
const uint16_t rstVectorH = 0xFFFD;
const uint16_t rstVectorL = 0xFFFC;
const uint16_t nmiVectorH = 0xFFFB;
const uint16_t nmiVectorL = 0xFFFA;

void cpu::bind_ppu(ppu* p) {
	PPU = p;
}

uint8_t cpu::flags_to_byte(bool irq) {
	return (flag_N << 7) | (flag_V << 6) | (1 << 5) | (!irq << 4) | (flag_D << 3) | (flag_I << 2) | (flag_Z << 1) | flag_C;
}

uint8_t cpu::flags_to_byte_p() {
	return (flag_N << 7) | (flag_V << 6) | (flag_U << 5) | (flag_B << 4) | (flag_D << 3) | (flag_I << 2) | (flag_Z << 1) | flag_C;
}

void cpu::byte_to_flags(uint8_t status, bool irq) { //PULLING STATUS
	flag_N = (bool)(status & 128);
	flag_V = (bool)(status & 64);
	//flag_U = 1;//(bool)(status & 32);
	//flag_B = 0;//(bool)(status & 16);
	flag_D = (bool)(status & 8);
	flag_I = (bool)(status & 4);
	flag_Z = (bool)(status & 2);
	flag_C = (bool)(status & 1);
}

bool cpu::same_page(uint16_t a1, uint16_t a2) {
	return (a1 / 256) == (a2 / 256);
}

void cpu::push_stack(uint8_t value) {
	write_memory(0x100 + P, value);
	if (P == 0x00) P = 0xFF;
	else P--;
}

uint8_t cpu::pop_stack() {
	if (P == 0xFF) P = 0x00;
	else P++;
	return read_memory(0x100 + P);
}

void cpu::push_stack_16(uint16_t value) {
	push_stack((value >> 8) & 0xFF);
	push_stack(value & 0xFF);
}

uint16_t cpu::pop_stack_16() {
	uint16_t lo = pop_stack();
	uint16_t hi = pop_stack();

	return (hi << 8) + lo;
}

void cpu::set_instr(int pos, int(cpu::*f)(uint16_t, bool), addressing_mode a, int c) {
	instr[pos].func = f;
	instr[pos].addr_mode = a;
	if (a == addressing_mode::IMPLIED || a == addressing_mode::RELATIVE || a == addressing_mode::IMMEDIATE || a == addressing_mode::ACCUMULATOR)
		instr[pos].reg_only = true;
	else
		instr[pos].reg_only = false;

	instr[pos].cycles = c;
	return;
}

//INSTRUCTIONS
int cpu::adc(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);
	unsigned int tmp = A + V + flag_C;
	bool old_C = flag_C;
	flag_C = check_overflow(A, V + flag_C, sizeof(uint8_t));
	//flag_V = check_overflow_signed(A, V + flag_C);//(bool)(!((A ^ V) & 0x80) && ((A ^ uint8_t(A+V+old_C) & 0x80)));
	flag_V = (!((A ^ V) & 0x80) && ((A ^ tmp) & 0x80));
	A += V + old_C;
	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::and(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);

	A = A & V;
	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::asl(uint16_t address, bool reg_only) {
	if (reg_only) {
		flag_C = check_negative(A);
		A = A * 2;
		flag_N = check_negative(A);
		flag_Z = A == 0;
	}
	else {
		uint8_t V = read_memory(address);
		flag_C = check_negative(V);
		V = V * 2;
		write_memory(address, V);
		flag_N = check_negative(V);
		flag_Z = V == 0;
	}
	return 0;
}

int cpu::bcc(uint16_t address, bool reg_only) {
	if (!flag_C) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::bcs(uint16_t address, bool reg_only) {
	if (flag_C) {
		bool sp = same_page(PC, PC + address);
		std::cout << sp;
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::beq(uint16_t address, bool reg_only) {
	if (flag_Z) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::bit(uint16_t address, bool reg_only) {
	uint8_t V = read_memory(address);
	uint8_t result = A & V;

	flag_Z = result == 0;
	flag_V = (bool)(V & 64);
	flag_N = check_negative(V);
	return 0;
}

int cpu::bmi(uint16_t address, bool reg_only) {
	if (flag_N) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::bne(uint16_t address, bool reg_only) {
	if (!flag_Z) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::bpl(uint16_t address, bool reg_only) {
	if (!flag_N) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::brk(uint16_t address, bool reg_only) {
	PC++;
	push_stack_16(PC);
	push_stack(flags_to_byte() | 0x10);
	flag_B = 1;
	PC = (read_memory(irqVectorH) << 8) + read_memory(irqVectorH);
	return 0;
}


int cpu::bvc(uint16_t address, bool reg_only) {
	if (!flag_V) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::bvs(uint16_t address, bool reg_only) {
	if (flag_V) {
		bool sp = same_page(PC, PC + address);
		PC += address;
		return sp == true ? 3 : 6;
	}
	return 0;
}

int cpu::clc(uint16_t address, bool reg_only) {
	flag_C = 0;
	return 0;
}

int cpu::cld(uint16_t address, bool reg_only) {
	flag_D = 0;
	return 0;
}

int cpu::cli(uint16_t address, bool reg_only) {
	flag_I = 0;
	return 0;
}

int cpu::clv(uint16_t address, bool reg_only) {
	flag_V = 0;
	return 0;
}	

int cpu::cmp(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);

	flag_C = A >= V;
	flag_Z = A == V;
	flag_N = check_negative(A - V);
	return 0;
}

int cpu::cpx(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);

	flag_C = X >= V;
	flag_Z = X == V;
	flag_N = check_negative(X - V);
	return 0;
}

int cpu::cpy(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);

	flag_C = Y >= V;
	flag_Z = Y == V;
	flag_N = check_negative(Y - V);
	return 0;
}

int cpu::dec(uint16_t address, bool reg_only) {
	uint8_t V = read_memory(address);
	V--;
	write_memory(address, V);

	flag_Z = V == 0;
	flag_N = check_negative(V);
	return 0;
}

int cpu::dex(uint16_t address, bool reg_only) {
	X--;

	flag_Z = X == 0;
	flag_N = check_negative(X);
	return 0;
}

int cpu::dey(uint16_t address, bool reg_only) {
	Y--;

	flag_Z = Y == 0;
	flag_N = check_negative(Y);
	return 0;
}

int cpu::eor(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);
	A = V ^ A;

	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::inc(uint16_t address, bool reg_only) {
	uint8_t V = read_memory(address);
	V++;
	write_memory(address, V);

	flag_Z = V == 0;
	flag_N = check_negative(V);
	return 0;
}

int cpu::inx(uint16_t address, bool reg_only) {
	X++;

	flag_Z = X == 0;
	flag_N = check_negative(X);
	return 0;
}

int cpu::iny(uint16_t address, bool reg_only) {
	Y++;

	flag_Z = Y == 0;
	flag_N = check_negative(Y);
	return 0;
}

int cpu::jmp(uint16_t address, bool reg_only) {
	PC = address;
	return 0;
}

int cpu::jsr(uint16_t address, bool reg_only) {
	PC--;
	push_stack_16(PC);
	PC = address;
	return 0;
}

int cpu::lda(uint16_t address, bool reg_only) {
	if (reg_only)
		A = address;
	else
		A = read_memory(address);

	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::ldx(uint16_t address, bool reg_only) {
	if (reg_only)
		X = address;
	else
		X = read_memory(address);

	flag_Z = X == 0;
	flag_N = check_negative(X);
	return 0;
}

int cpu::ldy(uint16_t address, bool reg_only) {
	if (reg_only)
		Y = (uint8_t)address;
	else
		Y = read_memory(address);

	flag_Z = Y == 0;
	flag_N = check_negative(Y);
	return 0;
}

int cpu::lsr(uint16_t address, bool reg_only) {
	if (reg_only) {
		flag_C = A & 1;
		A = A >> 1;

		flag_Z = A == 0;
		flag_N = check_negative(A);
	}
	else {
		uint8_t V = read_memory(address);
		flag_C = V & 1;
		V = V >> 1;
		write_memory(address, V);

		flag_Z = V == 0;
		flag_N = check_negative(V);
	}
	return 0;
}

int cpu::nop(uint16_t address, bool reg_only) {
	return 0;
}

int cpu::ora(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);
	A = V | A;

	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::pha(uint16_t address, bool reg_only) {
	push_stack(A);
	return 0;
}

int cpu::php(uint16_t address, bool reg_only) {
	flag_B = 1;
	push_stack(flags_to_byte());
	flag_B = 0;
	return 0;
}

int cpu::pla(uint16_t address, bool reg_only) {
	A = pop_stack();

	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::plp(uint16_t address, bool reg_only) {
	byte_to_flags(pop_stack());
	return 0;
}

int cpu::rol(uint16_t address, bool reg_only) {
	if (reg_only) {
		bool old_C = flag_C;
		flag_C = check_negative(A);
		A = (A << 1) + old_C;

		flag_Z = A == 0;
		flag_N = check_negative(A);
	}
	else {
		uint8_t V = read_memory(address);
		bool old_C = flag_C;
		flag_C = check_negative(V);
		V = (V << 1) + old_C;
		write_memory(address, V);

		flag_Z = V == 0;
		flag_N = check_negative(V);
	}
	return 0;
}

int cpu::ror(uint16_t address, bool reg_only) {
	if (reg_only) {
		bool old_C = flag_C;
		flag_C = (bool)(A & 1);
		A = (A >> 1) + (old_C << 7);

		flag_Z = A == 0;
		flag_N = check_negative(A);
	}
	else {
		uint8_t V = read_memory(address);
		bool old_C = flag_C;
		flag_C = (bool)(V & 1);
		V = (V >> 1) + (old_C << 7);
		write_memory(address, V);

		flag_Z = V == 0;
		flag_N = check_negative(V);
	}
	return 0;
}

int cpu::rti(uint16_t address, bool reg_only) {
	byte_to_flags(pop_stack());
	PC = pop_stack_16();
	return 0;
}

int cpu::rts(uint16_t address, bool reg_only) {
	PC = pop_stack_16() + 1; //CHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECKCHECK
	return 0;
}


int cpu::sbc(uint16_t address, bool reg_only) {
	uint8_t V;
	if (reg_only)
		V = address;
	else
		V = read_memory(address);

	unsigned int tmp = A - V - !flag_C;
	flag_V = (A ^ tmp) & (A ^ V) & 0x80;
	flag_C = !(tmp >> 8);
	A = (uint8_t)tmp;
	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::sec(uint16_t address, bool reg_only) {
	flag_C = 1;
	return 0;
}

int cpu::sed(uint16_t address, bool reg_only) {
	flag_D = 1;
	return 0;
}

int cpu::sei(uint16_t address, bool reg_only) {
	flag_I = 1;
	return 0;
}

int cpu::sta(uint16_t address, bool reg_only) {
	write_memory(address, A);
	return 0;
}

int cpu::stx(uint16_t address, bool reg_only) {
	write_memory(address, X);
	return 0;
}

int cpu::sty(uint16_t address, bool reg_only) {
	write_memory(address, Y);
	return 0;
}

int cpu::tax(uint16_t address, bool reg_only) {
	X = A;
	flag_Z = X == 0;
	flag_N = check_negative(X);
	return 0;
}

int cpu::tay(uint16_t address, bool reg_only) {
	Y = A;
	flag_Z = Y == 0;
	flag_N = check_negative(Y);
	return 0;
}

int cpu::tsx(uint16_t address, bool reg_only) {
	X = P;
	flag_Z = X == 0;
	flag_N = check_negative(X);
	return 0;
}

int cpu::txa(uint16_t address, bool reg_only) {
	A = X;
	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

int cpu::txs(uint16_t address, bool reg_only) {
	P = X;
	return 0;
}

int cpu::tya(uint16_t address, bool reg_only) {
	A = Y;
	flag_Z = A == 0;
	flag_N = check_negative(A);
	return 0;
}

uint8_t cpu::read_memory(uint16_t address) {
	if (0x2000 <= address <= 0x3FFF)
		return PPU->ppu_read(address);

	switch (mapper) {
	case((uint8_t)0):
		return mapper_0(address, READ);
	}
	return memory[address];
}

void cpu::write_memory(uint16_t address, uint8_t byte) {

	if (0x2000 <= address <= 0x3FFF) {
		PPU->ppu_write(address, byte); // TODO
		return;
	}

	switch (mapper) {
	case((uint8_t)0):
		mapper_0(address, WRITE, byte);
	}
	memory[address] = byte;
}

void cpu::reset() {
	A = 0;
	B = 0;
	X = 0;
	Y = 0;
	S = 0;
	P = 0xFD;
	cycles = 0;


	for (int i = 0; i < 0xFFFF; i++)
		memory[i] = i > 0x2000 ? 0x00 : 0xFF;

	flag_C = 0; // Carry: overflow/underflow
	flag_Z = 0; // Zero: result was 0
	flag_U = 1; // unused, set to 1
	flag_I = 1; // Interrupt Disable: prevent system from responding to interrupts
	flag_D = 0; // Decimal mode: unsupported
	flag_B = 0; // Break Command: break instruction was used
	flag_V = 0; // Overflow: 
	flag_Z = 0; // Negative: result is negative
}

inline bool cpu::check_overflow(int a, int b, int size) {
	if (a + b >= pow(2, size * 8) || a + b < 0)
		return true;

	return false;
}

inline bool cpu::check_overflow_signed(int a, int b, int size) {
	if (a + b >= pow(2, (size * 8) - 1) || a + b < 0)
		return true;

	return false;
}

inline bool cpu::check_overflow_signed(uint8_t a, uint8_t b) {
	if (check_negative(a) && check_negative(b)) {
		if (check_negative(a + b))
			return false;
	}
	else {
		if (!check_negative(a + b))
			return false;
		else
			return true;
	}
}

inline bool cpu::check_negative(uint8_t V) {
	return (bool)(128 & V);
}

void cpu::init() {
	set_instr(0x69, &cpu::adc, addressing_mode::IMMEDIATE, 2);
	set_instr(0x65, &cpu::adc, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x75, &cpu::adc, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x6d, &cpu::adc, addressing_mode::ABSOLUTE, 4);
	set_instr(0x7d, &cpu::adc, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0x79, &cpu::adc, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0x61, &cpu::adc, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0x71, &cpu::adc, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0x29, &cpu::and, addressing_mode::IMMEDIATE, 2);
	set_instr(0x25, &cpu::and, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x35, &cpu::and, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x2d, &cpu::and, addressing_mode::ABSOLUTE, 4);
	set_instr(0x3d, &cpu::and, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0x39, &cpu::and, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0x21, &cpu::and, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0x31, &cpu::and, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0x0a, &cpu::asl, addressing_mode::ACCUMULATOR, 2);
	set_instr(0x06, &cpu::asl, addressing_mode::ZERO_PAGE, 5);
	set_instr(0x16, &cpu::asl, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0x0e, &cpu::asl, addressing_mode::ABSOLUTE, 6);
	set_instr(0x1e, &cpu::asl, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0x90, &cpu::bcc, addressing_mode::RELATIVE, 2);

	set_instr(0xb0, &cpu::bcs, addressing_mode::RELATIVE, 2);

	set_instr(0xf0, &cpu::beq, addressing_mode::RELATIVE, 2);

	set_instr(0x24, &cpu::bit, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x2c, &cpu::bit, addressing_mode::ABSOLUTE, 4);

	set_instr(0x30, &cpu::bmi, addressing_mode::RELATIVE, 2);

	set_instr(0xd0, &cpu::bne, addressing_mode::RELATIVE, 2);

	set_instr(0x10, &cpu::bpl, addressing_mode::RELATIVE, 2);

	set_instr(0x00, &cpu::brk, addressing_mode::IMPLIED, 7);

	set_instr(0x50, &cpu::bvc, addressing_mode::RELATIVE, 2);

	set_instr(0x70, &cpu::bvs, addressing_mode::RELATIVE, 2);

	set_instr(0x18, &cpu::clc, addressing_mode::IMPLIED, 2);

	set_instr(0xd8, &cpu::cld, addressing_mode::IMPLIED, 2);

	set_instr(0x58, &cpu::cli, addressing_mode::IMPLIED, 2);

	set_instr(0xb8, &cpu::clv, addressing_mode::IMPLIED, 2);

	set_instr(0xc9, &cpu::cmp, addressing_mode::IMMEDIATE, 2);
	set_instr(0xc5, &cpu::cmp, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xd5, &cpu::cmp, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0xcd, &cpu::cmp, addressing_mode::ABSOLUTE, 4);
	set_instr(0xdd, &cpu::cmp, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0xd9, &cpu::cmp, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0xc1, &cpu::cmp, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0xd1, &cpu::cmp, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0xe0, &cpu::cpx, addressing_mode::IMMEDIATE, 2);
	set_instr(0xe4, &cpu::cpx, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xec, &cpu::cpx, addressing_mode::ABSOLUTE, 4);

	set_instr(0xc0, &cpu::cpy, addressing_mode::IMMEDIATE, 2);
	set_instr(0xc4, &cpu::cpy, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xcc, &cpu::cpy, addressing_mode::ABSOLUTE, 4);

	set_instr(0xc6, &cpu::dec, addressing_mode::ZERO_PAGE, 5);
	set_instr(0xd6, &cpu::dec, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0xce, &cpu::dec, addressing_mode::ABSOLUTE, 6);
	set_instr(0xde, &cpu::dec, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0xca, &cpu::dex, addressing_mode::IMPLIED, 2);

	set_instr(0x88, &cpu::dey, addressing_mode::IMPLIED, 2);

	set_instr(0x49, &cpu::eor, addressing_mode::IMMEDIATE, 2);
	set_instr(0x45, &cpu::eor, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x55, &cpu::eor, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x4d, &cpu::eor, addressing_mode::ABSOLUTE, 4);
	set_instr(0x5d, &cpu::eor, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0x59, &cpu::eor, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0x41, &cpu::eor, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0x51, &cpu::eor, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0xe6, &cpu::inc, addressing_mode::ZERO_PAGE, 5);
	set_instr(0xf6, &cpu::inc, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0xee, &cpu::inc, addressing_mode::ABSOLUTE, 6);
	set_instr(0xfe, &cpu::inc, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0xe8, &cpu::inx, addressing_mode::IMPLIED, 2);

	set_instr(0xc8, &cpu::iny, addressing_mode::IMPLIED, 2);

	set_instr(0x4c, &cpu::jmp, addressing_mode::ABSOLUTE, 3);
	set_instr(0x6c, &cpu::jmp, addressing_mode::INDIRECT, 5);

	set_instr(0x20, &cpu::jsr, addressing_mode::ABSOLUTE, 6);

	set_instr(0xa9, &cpu::lda, addressing_mode::IMMEDIATE, 2);
	set_instr(0xa5, &cpu::lda, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xb5, &cpu::lda, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0xad, &cpu::lda, addressing_mode::ABSOLUTE, 4);
	set_instr(0xbd, &cpu::lda, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0xb9, &cpu::lda, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0xa1, &cpu::lda, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0xb1, &cpu::lda, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0xa2, &cpu::ldx, addressing_mode::IMMEDIATE, 2);
	set_instr(0xa6, &cpu::ldx, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xb6, &cpu::ldx, addressing_mode::INDEXED_ZERO_PAGE_Y, 4);
	set_instr(0xae, &cpu::ldx, addressing_mode::ABSOLUTE, 4);
	set_instr(0xbe, &cpu::ldx, addressing_mode::INDEXED_ABSOLUTE_Y, 4);

	set_instr(0xa0, &cpu::ldy, addressing_mode::IMMEDIATE, 2);
	set_instr(0xa4, &cpu::ldy, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xb4, &cpu::ldy, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0xac, &cpu::ldy, addressing_mode::ABSOLUTE, 4);
	set_instr(0xbc, &cpu::ldy, addressing_mode::INDEXED_ABSOLUTE_X, 4);

	set_instr(0x4a, &cpu::lsr, addressing_mode::ACCUMULATOR, 2);
	set_instr(0x46, &cpu::lsr, addressing_mode::ZERO_PAGE, 5);
	set_instr(0x56, &cpu::lsr, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0x4e, &cpu::lsr, addressing_mode::ABSOLUTE, 6);
	set_instr(0x5e, &cpu::lsr, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0xea, &cpu::nop, addressing_mode::IMPLIED, 2);

	set_instr(0x09, &cpu::ora, addressing_mode::IMMEDIATE, 2);
	set_instr(0x05, &cpu::ora, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x15, &cpu::ora, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x0d, &cpu::ora, addressing_mode::ABSOLUTE, 4);
	set_instr(0x1d, &cpu::ora, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0x19, &cpu::ora, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0x01, &cpu::ora, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0x11, &cpu::ora, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0x48, &cpu::pha, addressing_mode::IMPLIED, 3);

	set_instr(0x08, &cpu::php, addressing_mode::IMPLIED, 3);

	set_instr(0x68, &cpu::pla, addressing_mode::IMPLIED, 4);

	set_instr(0x28, &cpu::plp, addressing_mode::IMPLIED, 4); //check

	set_instr(0x2a, &cpu::rol, addressing_mode::ACCUMULATOR, 2);
	set_instr(0x26, &cpu::rol, addressing_mode::ZERO_PAGE, 5);
	set_instr(0x36, &cpu::rol, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0x2e, &cpu::rol, addressing_mode::ABSOLUTE, 6);
	set_instr(0x3e, &cpu::rol, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0x6a, &cpu::ror, addressing_mode::ACCUMULATOR, 2);
	set_instr(0x66, &cpu::ror, addressing_mode::ZERO_PAGE, 5);
	set_instr(0x76, &cpu::ror, addressing_mode::INDEXED_ZERO_PAGE_X, 6);
	set_instr(0x6e, &cpu::ror, addressing_mode::ABSOLUTE, 6);
	set_instr(0x7e, &cpu::ror, addressing_mode::INDEXED_ABSOLUTE_X, 7);

	set_instr(0x40, &cpu::rti, addressing_mode::IMPLIED, 6);

	set_instr(0x60, &cpu::rts, addressing_mode::IMPLIED, 6);

	set_instr(0xe9, &cpu::sbc, addressing_mode::IMMEDIATE, 2);
	set_instr(0xe5, &cpu::sbc, addressing_mode::ZERO_PAGE, 3);
	set_instr(0xf5, &cpu::sbc, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0xed, &cpu::sbc, addressing_mode::ABSOLUTE, 4);
	set_instr(0xfd, &cpu::sbc, addressing_mode::INDEXED_ABSOLUTE_X, 4);
	set_instr(0xf9, &cpu::sbc, addressing_mode::INDEXED_ABSOLUTE_Y, 4);
	set_instr(0xe1, &cpu::sbc, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0xf1, &cpu::sbc, addressing_mode::INDEXED_INDIRECT_Y, 5);

	set_instr(0x38, &cpu::sec, addressing_mode::IMPLIED, 2);

	set_instr(0xf8, &cpu::sed, addressing_mode::IMPLIED, 2);

	set_instr(0x78, &cpu::sei, addressing_mode::IMPLIED, 2);

	set_instr(0x85, &cpu::sta, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x95, &cpu::sta, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x8d, &cpu::sta, addressing_mode::ABSOLUTE, 4);
	set_instr(0x9d, &cpu::sta, addressing_mode::INDEXED_ABSOLUTE_X, 5);
	set_instr(0x99, &cpu::sta, addressing_mode::INDEXED_ABSOLUTE_Y, 5);
	set_instr(0x81, &cpu::sta, addressing_mode::INDEXED_INDIRECT_X, 6);
	set_instr(0x91, &cpu::sta, addressing_mode::INDEXED_INDIRECT_Y, 6);

	set_instr(0x86, &cpu::stx, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x96, &cpu::stx, addressing_mode::INDEXED_ZERO_PAGE_Y, 4);
	set_instr(0x8e, &cpu::stx, addressing_mode::ABSOLUTE, 4);

	set_instr(0x84, &cpu::sty, addressing_mode::ZERO_PAGE, 3);
	set_instr(0x94, &cpu::sty, addressing_mode::INDEXED_ZERO_PAGE_X, 4);
	set_instr(0x8c, &cpu::sty, addressing_mode::ABSOLUTE, 4);

	set_instr(0xaa, &cpu::tax, addressing_mode::IMPLIED, 2);

	set_instr(0xa8, &cpu::tay, addressing_mode::IMPLIED, 2);

	set_instr(0xba, &cpu::tsx, addressing_mode::IMPLIED, 2);

	set_instr(0x8a, &cpu::txa, addressing_mode::IMPLIED, 2);

	set_instr(0x9a, &cpu::txs, addressing_mode::IMPLIED, 2);

	set_instr(0x98, &cpu::tya, addressing_mode::IMPLIED, 2);

	reset();
}

uint8_t cpu::fetch() {
	return read_memory(PC++);
}

uint16_t cpu::fetch_16() {
	uint16_t lo = fetch();
	uint16_t hi = fetch();

	return (hi << 8) + lo;
}

int cpu::exec(uint8_t opcode) {
	int cyc = 0;
	uint16_t address = 0;
	uint8_t t_address;
	uint16_t lo;
	uint16_t hi;
	//std::cout << instr[opcode].addr_mode << "\n";
	switch (instr[opcode].addr_mode) {
	case(addressing_mode::ACCUMULATOR):
		break;
	case(addressing_mode::IMPLIED):
		break;
	case(addressing_mode::IMMEDIATE):
		address = fetch();
		break;
	case(addressing_mode::RELATIVE):
		address = fetch();
		break;
	case(addressing_mode::ABSOLUTE):
		address = fetch_16();
		break;
	case(addressing_mode::ZERO_PAGE):
		address = fetch();
		break;
	case(addressing_mode::INDIRECT):
		address = fetch_16();
		lo = read_memory(address);
		if (address & 0xFF == 0xFF) {
			address -= 0xFF;
			//cyc += 3; //??????
		}
		else
			address++;
		hi = read_memory(address);
		address = (hi << 8) + lo;
		break;
	case(addressing_mode::INDEXED_ABSOLUTE_X):
		address = fetch_16();
		if (u8_contains(check_boundary, 23, opcode) && !same_page(address, address + X)) //check if address + X crosses page boundary
			cyc += 3;
		address += X;
		break;
	case(addressing_mode::INDEXED_ABSOLUTE_Y):
		address = fetch_16();// + Y;
		if (u8_contains(check_boundary, 23, opcode) && !same_page(address, address + Y)) //check if address + Y crosses page boundary
			cyc += 3;
		address += Y;
		break;
	case(addressing_mode::INDEXED_ZERO_PAGE_X):
		t_address = fetch() + X;
		address = t_address & 0xFF;
		break;
	case(addressing_mode::INDEXED_ZERO_PAGE_Y):
		t_address = fetch() + Y;
		address = t_address & 0xFF;
		break;
	case(addressing_mode::INDEXED_INDIRECT_X):
		t_address = ((uint16_t)fetch() + X);
		lo = read_memory(t_address++);
		hi = read_memory(t_address);
		address = (hi << 8) + lo;
		break;
	case(addressing_mode::INDEXED_INDIRECT_Y):
		t_address = fetch();
		lo = read_memory(t_address++);
		hi = read_memory(t_address);
		address = (hi << 8) + lo;
		if (u8_contains(check_boundary, 23, opcode) && !same_page(address, address + Y)) //check if address + Y crosses page boundary
			cyc += 3;
		address += Y;
		break;
	}

	std::cout << " " << address;
	p_r();
	auto f = std::bind(instr[opcode].func, this, address, instr[opcode].reg_only);
	cyc += instr[opcode].cycles * 3;
	cyc += f();
	return cyc;
}

void cpu::load(std::string path) {
	romchr = readFileBytes(path);
	rom = (uint8_t*)(void*)romchr;

	mapper = (((uint8_t)rom[6] & 0xF0) >> 4) + (uint8_t)rom[7] & 0xF0;
	prg_size = rom[4];
	chr_size = rom[5];
	ram_size = rom[8];
	trainer = (bool)((rom[6] >> 3) & 0x1);
	PC = (uint16_t)(read_memory(0xfffc)) + ((uint16_t)read_memory(0xfffd) << 8);
}

const bool READ = 0;
const bool WRITE = 1;
const int HEADER_SIZE = 16;

//0
uint8_t cpu::mapper_0(uint16_t address, bool mode, uint8_t value) {
	//std::cout << "attempting to read address " << address << " from memory/rom\n";
	if (address >= 0x8000 && address <= 0xbfff) {
		if (mode == READ)
			return rom[HEADER_SIZE + (512 * trainer) + address - 0x8000];
		else
			return 0;
	}
	if (address >= 0xc000 && address <= 0xffff) {
		if (mode == READ)
			return rom[HEADER_SIZE + (512 * trainer) + address - (prg_size > 1 ? 0x8000 : 0xc000)];
		else
			return 0;
	}

	if (mode == READ)
		return memory[address];
	else
		memory[address] = value;

	return -1;
}

void cpu::p_r() {
	std::cout << "\tA:" << (int)A << " X:" << (int)X << " Y:" << (int)Y << " P:" << (int)flags_to_byte_p() << " SP:" << (int)P << " CYC:" << std::dec << cycles % 341 << std::hex << "\n";
}

void cpu::run() {
	PC = (uint16_t)(read_memory(0xfffc)) + ((uint16_t)read_memory(0xfffd) << 8);
	std::cout << "PC SET TO: " << PC << "\n";
	while (1) {
		std::cout << PC << "   ";
		uint8_t opcode = fetch();
		std::cout << (int)opcode << " ";
		//p_r();
		cycles += exec(opcode);
		std::cout << "";
	}
}

int cpu::advance() {
	uint8_t opcode = fetch();
	int cyc = exec(opcode);
	cycles += cyc;
	return cyc;
}