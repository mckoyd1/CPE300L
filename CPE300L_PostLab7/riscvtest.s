# riscvtest.s
# Sarah.Harris@unlv.edu
# David_Harris@hmc.edu
# 27 Oct 2020
#
# Test the RISC-V processor.  
#  add, sub, and, or, slt, addi, lw, sw, beq, jal
# If successful, it should write the value 25 to address 100

#       RISC-V Assembly         Description               Address   Machine Code
main:   addi x2, x0, 5          # x2 = 5                  0         00500113   
        addi x3, x0, 12         # x3 = 12                 4         00C00193
        addi x7, x3, -9         # x7 = (12 - 9) = 3       8         FF718393
        or   x4, x7, x2         # x4 = (3 OR 5) = 7       C         0023E233
        xor  x4, x3, x2         # x4 = (12 ^ 5) = 9       10        0021c233
        lui  x3, 0xABCDE        # x3 = 0xABCDE            14        abcde1b7
        and  x5, x3, x4         # x5 = (12 AND 7) = 4     18        0041F2B3
        add  x5, x5, x4         # x5 = (4 + 7) = 11       1C        004282B3
        beq  x5, x7, end        # shouldn't be taken      20        02728863
        slt  x4, x3, x4         # x4 = (12 < 7) = 0       24        0041A233
        beq  x4, x0, around     # should be taken         28        00020463
        addi x5, x0, 0          # shouldn't happen        2C        00000293
around: slt  x4, x7, x2         # x4 = (3 < 5)  = 1       30        0023A233
        add  x7, x4, x5         # x7 = (1 + 11) = 12      34        005203B3
        sub  x7, x7, x2         # x7 = (12 - 5) = 7       38        402383B3
        sw   x7, 84(x3)         # [96] = 7                3C        0471AA23 
        lw   x2, 96(x0)         # x2 = [96] = 7           40        06002103 
        add  x9, x2, x5         # x9 = (7 + 11) = 18      44        005104B3
        jal  x3, end            # jump to end, x3 = 0x44  48        008001EF
        addi x2, x0, 1          # shouldn't happen        4C        00100113
end:    add  x2, x2, x9         # x2 = (7 + 18)  = 25     50        00910133
        sw   x2, 0x20(x3)       # mem[100] = 25           54        0221A023 
done:   beq  x2, x2, done       # infinite loop           58        00210063
		
		