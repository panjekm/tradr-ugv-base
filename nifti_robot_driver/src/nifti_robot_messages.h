#ifndef _NIFTI_ROBOT_MESSAGES_H
#define _NIFTI_ROBOT_MESSAGES_H

#include <string.h>
#include <sstream>
#include <map>


//! Diagnostic messages for the battery
static const std::string battery_messages[3] = {
	"OK",
	"Warning",
	"Critical"
};

//! Error messages for CAN communication (returned by the librover function calls)
static const std::string CAN_error_messages[6]=
		{"No error",
		 "Open error",
		 "Setup error",
		 "Close error",
		 "Send error",
		 "Receive error"};

//! Messages for bits 1-3 of the status register of the controllers
static const std::string servo_drive_status_messages[8]=
		{"OK (000b)",
		 "Under voltage (001b)",
		 "Over voltage (010b)",
		 "unknown (011b)",
		 "unknown (100b)",
		 "Short circuit (101b)",
		 "Overheating (110b)",
		 "unknown (111b)"};

//! Messages for the Motor on bit (4) of the status register of the controllers
static const std::string MO_messages[2]=
		{"Disabled",
		 "Enabled"};

//! Messages for the Unit mode bits (7-8) of the status register of the
//controllers
static const std::string UM_messages[8]=
		{"unknown (0)",
		 "Torque control (1)",
		 "Speed control (2)",
		 "Micro-stepper (3)",
		 "Dual feedback position control (4)",
		 "Single loop position control (5)",
		 "unknown (6)",
		 "unknown (7)"};

//! Messages for the Motion status reflection bits (14-15) of the status
//register of the controllers
static const std::string MS_messages[4]=
		{"Motor position stabilized (0)",
		 "Reference stationary or motor off (1)",
		 "Reference dynamically controlled (2)",
		 "Reserved (3)"};

//! Messages for the Motor Failure register (index is the bit)
static const std::string MF_messages[32]=
		{"Resolver or Analog Halls feedback, angle not found or \
amplitude too low.",
		 "Reserved.",
		 "Feedback loss: no match between encode and Hall location.",
		 "The peak current has been exceeded.",
		 "Inhibit.",
		 "Reserved.",
		 "Two digital Hall sensors were changed at the same time.",
		 "Speed tracking error DV[2] - VX exceeded speed error limit \
ER[2] (for UM=2 or UM=4,5).",
		 "Position tracking error DV[3] - PX (UM=5) or DV[3] - PY \
(UM=4) exceeded position error limit ER[3].",
		 "Cannot start because of inconsistent database.",
		 "Too large a difference in ECAM table.",
		 "Heartbeat failure.",
		 "Servo drive fault see bits 13-15.",
		 "Servo drive fault detail bit 1.",
		 "Servo drive fault detail bit 2.",
		 "Servo drive fault detail bit 3.",
		 "Failed to find the electrical zero of the motor in an \
attempt to start it with an incremental encoder and no digital \
Hall sensors.",
		 "Speed limit exceeded: VX<LL[2] or VX>HL[2].",
		 "Stack overflow - fatal exception (need a power reset).",
		 "CPU exception - fatal exception (need a power reset).",
		 "Reserved.",
		 "Motor stuck - the motor is powered but is not moving \
according to the definition of CL[2] and CL[3]",
		 "Position limit exceeded: PX<LL[3] or PX>HL[3] (UM=5), \
or PY<LL[3] or PY>HL[3] (UM=4).",
		 "unknown",
		 "unknown",
		 "unknown",
		 "unknown",
		 "unknown",
		 "Reserved.",
		 "Cannot start motor.",
		 "unknown",
		 "Reserved."};

#define MF_GET_FAULT_DETAIL(mf)	((mf >> 13)&0x07)
//! Messages for the servo drive fault bits (13-15) of the motor failure
//register of the controllers
static const std::string MF131415_messages[8]=
		{"OK (0)",
		 "Under voltage (1)",
		 "Over voltage (2)",
		 "Reserved (3)",
		 "Reserved (4)",
		 "Short circuit (5)",
		 "Temperature. Drive overheating (6)",
		 "Reserved (7)"};


//! Messages for the EC error code of the controllers
struct EC_messages: public std::map<int, std::string> {
	EC_messages() {
		(*this)[2] = "Bad command.";
		(*this)[3] = "Bad index.";
		(*this)[5] = "Has no interpreter meaning.";
		(*this)[6] = "Program is not running.";
		(*this)[7] = "Mode cannot be started; bad initialization data.";
		(*this)[11] = "Cannot write to flash memory.";
		(*this)[12] = "Command not available in this unit mode.";
		(*this)[13] = "Cannot reset communication; UART is nusy.";
		(*this)[18] = "Empty assign.";
		(*this)[19] = "Bad command format.";
		(*this)[21] = "Operand out of range.";
		(*this)[22] = "Zero division.";
		(*this)[23] = "Command cannot be assigned.";
		(*this)[24] = "Bad operator.";
		(*this)[25] = "Command not valid while moving.";
		(*this)[26] = "Motion mode not valid.";
		(*this)[27] = "Out of limit range.";
		(*this)[30] = "No program to continue.";
		(*this)[32] = "Communication overrun, parity, noise or framing error.";
		(*this)[37] = "Two or more Hall sensors are defined for the same location.";
		(*this)[39] = "Motion start specified for the past.";
		(*this)[41] = "Command not supported by this product.";
		(*this)[42] = "No such label.";
		(*this)[43] = "CAN state machine fault.";
		(*this)[45] = "Returned error from subroutine.";
		(*this)[46] = "May not use multi-capture homing mode with stop event.";
		(*this)[47] = "User program does not exist.";
		(*this)[50] = "Stack overflow.";
		(*this)[53] = "Only for current.";
		(*this)[54] = "Bad database.";
		(*this)[55] = "Bad context.";
		(*this)[56] = "The product grade does not support this command.";
		(*this)[57] = "Motor must be off.";
		(*this)[58] = "Motor must be on.";
		(*this)[60] = "Bad unit mode.";
		(*this)[61] = "Data base reset.";
		(*this)[64] = "Cannot set the index of an active table.";
		(*this)[65] = "Disabled by SW.";
		(*this)[66] = "Drive not ready.";
		(*this)[67] = "Recorder is busy.";
		(*this)[69] = "Recorder usage error.";
		(*this)[70] = "Recorder data invalid.";
		(*this)[71] = "Homing is busy.";
		(*this)[72] = "Must be even.";
		(*this)[73] = "Please set position.";
		(*this)[77] = "Buffer too large.";
		(*this)[78] = "Out of program range.";
		(*this)[80] = "ECAM data inconsistent.";
		(*this)[81] = "In \"Quick stop\" mode.";
		(*this)[82] = "Program is running.";
		(*this)[83] = "CMD not for program.";
		(*this)[84] = "The system is not in point to point mode.";
		(*this)[90] = "CAN state machine is not ready.";
		(*this)[93] = "There is a wrong initiation value for this command.";
		(*this)[95] = "Too large or modulo setting.";
		(*this)[96] = "User program time out.";
		(*this)[97] = "RS232 receive buffer overflow.";
		(*this)[99] = "The auviliary feedback entry does not configure as \
output during the activation of Output Compare.";
		(*this)[100] = "The requested PWM value is not supported.";
		(*this)[101] = "Abortive attempt to read  aposition value from the \
absolute position sensor.";
		(*this)[105] = "Speed loop KP out of range.";
		(*this)[106] = "Position loop KP out of range.";
		(*this)[111] = "KV[N] vector is invalid.";
		(*this)[112] = "KV[N] defines a scheduled block but scheduling is off.";
		(*this)[113] = "Exp task queue is full.";
		(*this)[114] = "Exp task quere is empty.";
		(*this)[115] = "Exp output queue is full.";
		(*this)[116] = "Exp output queue is empty.";
		(*this)[117] = "Bad KV setting for sensor filter.";
		(*this)[118] = "Bad KV vector.";
		(*this)[119] = "Bad Analog sensor filter.";
		(*this)[120] = "Bad number of blocks for Analog sensor filter.";
		(*this)[121] = "Analog sensor is not ready.";
		(*this)[127] = "Modulo range must be positive.";
		(*this)[128] = "Bad variable index in database; internal compiler error.";
		(*this)[129] = "Variable is not an array.";
		(*this)[130] = "Variable name does not exist.";
		(*this)[131] = "Cannot record local variables.";
		(*this)[132] = "Variable is not an array.";
		(*this)[133] = "Mismatched number of user/system function input \
arguments.";
		(*this)[134] = "Cannot run local label with XQ command.";
		(*this)[137] = "Program already compiled.";
		(*this)[139] = "The number of breakpoints exceeds the maximum number.";
		(*this)[140] = "An attempt to set/clear breakpoint atthe non-relevant \
time.";
		(*this)[141] = "Boot identity parameters section is not clear.";
		(*this)[142] = "Checksum of data is not correct.";
		(*this)[143] = "Missing bott identity parameters.";
		(*this)[144] = "Numeric stack underflow.";
		(*this)[145] = "Numeric stack overflow.";
		(*this)[146] = "Expression stack overflow.";
		(*this)[147] = "Executable command within math expression.";
		(*this)[148] = "Nothing in the expression.";
		(*this)[149] = "Unexpected sentence termination.";
		(*this)[150] = "Sentence terminator not found.";
		(*this)[151] = "Parentheses mismatch.";
		(*this)[152] = "Bad operand type.";
		(*this)[154] = "Address is out of data memory segment.";
		(*this)[155] = "Beyond stack range.";
		(*this)[156] = "Bad opcode.";
		(*this)[157] = "No available program stack.";
		(*this)[158] = "Out of flash memory range.";
		(*this)[159] = "Flash verify error.";
		(*this)[160] = "Program aborted by another threat.";
		(*this)[161] = "Program is not halted.";
		(*this)[162] = "Badly formatted number.";
		(*this)[164] = "EC command (not an error).";
		(*this)[165] = "Attempt to access serial flash while busy.";
		(*this)[166] = "Out of modulo range.";
		(*this)[167] = "Infinite loop in for loop; zero step.";
		(*this)[168] = "Speed too large to start motor.";
	}

	std::string get(int index) const {
		std::ostringstream message;
		std::map<int, std::string>::const_iterator elem = this->find(index);
		if (elem==this->end())
			message << "unknown";
		else
			message << elem->second;
		message << " (" << index << ")";
		return message.str();
	}
};

#endif
