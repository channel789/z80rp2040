start:
  LD HL,0xFFFD
  LD A,(HL)
  ADD A,0
  JR Z,start
  LD HL,0xFFFE
  LD A,(HL)
  CP 0x61
  JP C,out
  SUB 0x20
out:
  LD HL,0xFFFF
  LD (HL),A
  JP start
  HALT
