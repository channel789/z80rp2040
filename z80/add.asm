LD HL,0x1000
LD A,(HL)
LD HL,0x1001
ADD A,(HL)
LD HL,0x1002
LD (HL),A
HALT
