\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 285 C0 - 7 C1 - 285 C3 - 7 C4 + 40637 Constant + [ 2 C0 ^2 + 2 C1 ^2
   + 2 C3 ^2 + 2 C4 ^2 ] / 2 
Subject To
 R0: C0 = 139.9257360549753
 R1: C1 = 2.488480673075916
 R2: C2 = 0
 R3: C3 = 142.5
 R4: C4 = 3.5
 R5: C0 + 1e+06 alpha_ob0_seg0[0] <= 1.00011e+06
 R6: C3 + 1e+06 alpha_ob0_seg0[0] <= 1.00011e+06
 R7: - C0 + 1e+06 alpha_ob0_seg0[1] <= 999865
 R8: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999865
 R9: C1 + 1e+06 alpha_ob0_seg0[2] <= 1.000006e+06
 R10: C4 + 1e+06 alpha_ob0_seg0[2] <= 1.000006e+06
 R11: - C1 + 1e+06 alpha_ob0_seg0[3] <= 999967
 R12: - C4 + 1e+06 alpha_ob0_seg0[3] <= 999967
 R13: C2 + 1e+06 alpha_ob0_seg0[4] <= 999997
 R14: C5 + 1e+06 alpha_ob0_seg0[4] <= 999997
 R15: - C2 + 1e+06 alpha_ob0_seg0[5] <= 999897
 R16: - C5 + 1e+06 alpha_ob0_seg0[5] <= 999897
 R17: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] + alpha_ob0_seg0[4] + alpha_ob0_seg0[5] >= 1
 R18: C0 + 1e+06 alpha_ob1_seg0[0] <= 1.00011e+06
 R19: C3 + 1e+06 alpha_ob1_seg0[0] <= 1.00011e+06
 R20: - C0 + 1e+06 alpha_ob1_seg0[1] <= 999865
 R21: - C3 + 1e+06 alpha_ob1_seg0[1] <= 999865
 R22: C1 + 1e+06 alpha_ob1_seg0[2] <= 999975
 R23: C4 + 1e+06 alpha_ob1_seg0[2] <= 999975
 R24: - C1 + 1e+06 alpha_ob1_seg0[3] <= 999998
 R25: - C4 + 1e+06 alpha_ob1_seg0[3] <= 999998
 R26: C2 + 1e+06 alpha_ob1_seg0[4] <= 999997
 R27: C5 + 1e+06 alpha_ob1_seg0[4] <= 999997
 R28: - C2 + 1e+06 alpha_ob1_seg0[5] <= 999897
 R29: - C5 + 1e+06 alpha_ob1_seg0[5] <= 999897
 R30: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] + alpha_ob1_seg0[4] + alpha_ob1_seg0[5] >= 1
 R31: C0 + 1e+06 alpha_ob2_seg0[0] <= 1.000138e+06
 R32: C3 + 1e+06 alpha_ob2_seg0[0] <= 1.000138e+06
 R33: - C0 + 1e+06 alpha_ob2_seg0[1] <= 999837
 R34: - C3 + 1e+06 alpha_ob2_seg0[1] <= 999837
 R35: C1 + 1e+06 alpha_ob2_seg0[2] <= 1.000006e+06
 R36: C4 + 1e+06 alpha_ob2_seg0[2] <= 1.000006e+06
 R37: - C1 + 1e+06 alpha_ob2_seg0[3] <= 999967
 R38: - C4 + 1e+06 alpha_ob2_seg0[3] <= 999967
 R39: C2 + 1e+06 alpha_ob2_seg0[4] <= 999997
 R40: C5 + 1e+06 alpha_ob2_seg0[4] <= 999997
 R41: - C2 + 1e+06 alpha_ob2_seg0[5] <= 999897
 R42: - C5 + 1e+06 alpha_ob2_seg0[5] <= 999897
 R43: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] + alpha_ob2_seg0[4] + alpha_ob2_seg0[5] >= 1
 R44: C0 + 1e+06 alpha_ob3_seg0[0] <= 1.000138e+06
 R45: C3 + 1e+06 alpha_ob3_seg0[0] <= 1.000138e+06
 R46: - C0 + 1e+06 alpha_ob3_seg0[1] <= 999837
 R47: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999837
 R48: C1 + 1e+06 alpha_ob3_seg0[2] <= 999975
 R49: C4 + 1e+06 alpha_ob3_seg0[2] <= 999975
 R50: - C1 + 1e+06 alpha_ob3_seg0[3] <= 999998
 R51: - C4 + 1e+06 alpha_ob3_seg0[3] <= 999998
 R52: C2 + 1e+06 alpha_ob3_seg0[4] <= 999997
 R53: C5 + 1e+06 alpha_ob3_seg0[4] <= 999997
 R54: - C2 + 1e+06 alpha_ob3_seg0[5] <= 999897
 R55: - C5 + 1e+06 alpha_ob3_seg0[5] <= 999897
 R56: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] + alpha_ob3_seg0[4] + alpha_ob3_seg0[5] >= 1
 R57: - C2 + C5 >= 15
 R58: - C2 + C5 <= 100
 R59: - C0 - C1 + C3 + C4 <= 5
 R60: - C0 - C1 + C3 + C4 >= -5
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
