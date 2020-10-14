\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 271 C0 - 45 C1 + C2 - 271 C3 - 45 C4 + C5 + 37733 Constant + [ 2 C0 ^2
   + 2 C1 ^2 + 2 C3 ^2 + 2 C4 ^2 ] / 2 
Subject To
 R0: C0 = 135.1631071551507
 R1: C1 = 20.38050277049319
 R2: C2 = 0
 R3: C3 = 135.5
 R4: C4 = 22.5
 R5: C0 + 1e+06 alpha_ob0_seg0[0] <= 1.000113e+06
 R6: C3 + 1e+06 alpha_ob0_seg0[0] <= 1.000113e+06
 R7: - C0 + 1e+06 alpha_ob0_seg0[1] <= 999868
 R8: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999868
 R9: C1 + 1e+06 alpha_ob0_seg0[2] <= 1.000009e+06
 R10: C4 + 1e+06 alpha_ob0_seg0[2] <= 1.000009e+06
 R11: - C1 + 1e+06 alpha_ob0_seg0[3] <= 999970
 R12: - C4 + 1e+06 alpha_ob0_seg0[3] <= 999970
 R13: 1e+06 alpha_ob0_seg0[4] <= 1e+06
 R14: 1e+06 alpha_ob0_seg0[4] <= 1e+06
 R15: 1e+06 alpha_ob0_seg0[5] <= 999900
 R16: 1e+06 alpha_ob0_seg0[5] <= 999900
 R17: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] + alpha_ob0_seg0[4] + alpha_ob0_seg0[5] >= 1
 R18: C0 + 1e+06 alpha_ob1_seg0[0] <= 1.000113e+06
 R19: C3 + 1e+06 alpha_ob1_seg0[0] <= 1.000113e+06
 R20: - C0 + 1e+06 alpha_ob1_seg0[1] <= 999868
 R21: - C3 + 1e+06 alpha_ob1_seg0[1] <= 999868
 R22: C1 + 1e+06 alpha_ob1_seg0[2] <= 999978
 R23: C4 + 1e+06 alpha_ob1_seg0[2] <= 999978
 R24: - C1 + 1e+06 alpha_ob1_seg0[3] <= 1.000001e+06
 R25: - C4 + 1e+06 alpha_ob1_seg0[3] <= 1.000001e+06
 R26: 1e+06 alpha_ob1_seg0[4] <= 1e+06
 R27: 1e+06 alpha_ob1_seg0[4] <= 1e+06
 R28: 1e+06 alpha_ob1_seg0[5] <= 999900
 R29: 1e+06 alpha_ob1_seg0[5] <= 999900
 R30: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] + alpha_ob1_seg0[4] + alpha_ob1_seg0[5] >= 1
 R31: C0 + 1e+06 alpha_ob2_seg0[0] <= 1.000141e+06
 R32: C3 + 1e+06 alpha_ob2_seg0[0] <= 1.000141e+06
 R33: - C0 + 1e+06 alpha_ob2_seg0[1] <= 999840
 R34: - C3 + 1e+06 alpha_ob2_seg0[1] <= 999840
 R35: C1 + 1e+06 alpha_ob2_seg0[2] <= 1.000009e+06
 R36: C4 + 1e+06 alpha_ob2_seg0[2] <= 1.000009e+06
 R37: - C1 + 1e+06 alpha_ob2_seg0[3] <= 999970
 R38: - C4 + 1e+06 alpha_ob2_seg0[3] <= 999970
 R39: 1e+06 alpha_ob2_seg0[4] <= 1e+06
 R40: 1e+06 alpha_ob2_seg0[4] <= 1e+06
 R41: 1e+06 alpha_ob2_seg0[5] <= 999900
 R42: 1e+06 alpha_ob2_seg0[5] <= 999900
 R43: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] + alpha_ob2_seg0[4] + alpha_ob2_seg0[5] >= 1
 R44: C0 + 1e+06 alpha_ob3_seg0[0] <= 1.000141e+06
 R45: C3 + 1e+06 alpha_ob3_seg0[0] <= 1.000141e+06
 R46: - C0 + 1e+06 alpha_ob3_seg0[1] <= 999840
 R47: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999840
 R48: C1 + 1e+06 alpha_ob3_seg0[2] <= 999978
 R49: C4 + 1e+06 alpha_ob3_seg0[2] <= 999978
 R50: - C1 + 1e+06 alpha_ob3_seg0[3] <= 1.000001e+06
 R51: - C4 + 1e+06 alpha_ob3_seg0[3] <= 1.000001e+06
 R52: 1e+06 alpha_ob3_seg0[4] <= 1e+06
 R53: 1e+06 alpha_ob3_seg0[4] <= 1e+06
 R54: 1e+06 alpha_ob3_seg0[5] <= 999900
 R55: 1e+06 alpha_ob3_seg0[5] <= 999900
 R56: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] + alpha_ob3_seg0[4] + alpha_ob3_seg0[5] >= 1
 R57: - C2 + C5 >= 10
 R58: - C0 - C1 + C3 + C4 <= 5
 R59: - C0 - C1 + C3 + C4 >= -5
Bounds
 Constant = 1
Binaries
 alpha_ob0_seg0[0] alpha_ob0_seg0[1] alpha_ob0_seg0[2] alpha_ob0_seg0[3]
 alpha_ob0_seg0[4] alpha_ob0_seg0[5] alpha_ob1_seg0[0] alpha_ob1_seg0[1]
 alpha_ob1_seg0[2] alpha_ob1_seg0[3] alpha_ob1_seg0[4] alpha_ob1_seg0[5]
 alpha_ob2_seg0[0] alpha_ob2_seg0[1] alpha_ob2_seg0[2] alpha_ob2_seg0[3]
 alpha_ob2_seg0[4] alpha_ob2_seg0[5] alpha_ob3_seg0[0] alpha_ob3_seg0[1]
 alpha_ob3_seg0[2] alpha_ob3_seg0[3] alpha_ob3_seg0[4] alpha_ob3_seg0[5]
End
