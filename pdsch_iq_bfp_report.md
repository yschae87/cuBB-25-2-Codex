## PDSCH IQ Scale & Format (Modulation → BFP9 U‑plane)

### 1) Modulation output (FP16 tensor)
- Constellation points are pre-normalized and then scaled by UE `beta_qam`; symbols are written as `__half2` into the cell output tensor (`cell_output_tensor_addr`).
- QPSK scales ±1/√2 by `beta_qam`; higher QAMs use shared lookup tables multiplied by `beta_qam`.
```
cuPHY/src/cuphy/modulation_mapper/modulation_mapper.cu:201-228
shmem_qam_256[t] = (__half)(rev_qam_256[t] * params[blockIdx.y].beta_qam);
...
modulation_output[output_index] = make_complex<__half2>::create(shmem_qam_256[x_index],
                                                                shmem_qam_256[y_index]);
```
```
cuPHY/src/cuphy/modulation_mapper/modulation_mapper.cu:302-334
float reciprocal_sqrt2 = 0.707106781186547f * params[blockIdx.y].beta_qam;
...
tmp_val.x = ((bit_values & 0x1) == 0) ? reciprocal_sqrt2 : -reciprocal_sqrt2;
tmp_val.y = ((bit_values & 0x2) == 0) ? reciprocal_sqrt2 : -reciprocal_sqrt2;
```
- The DMRS params carry `beta_qam` and the destination tensor pointer used by modulation.
```
cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu:169-181
h_dmrs_params[TB_id].cell_output_tensor_addr = dyn_params->pDataOut->pTDataTx[dyn_cell].pAddr;
h_dmrs_params[TB_id].beta_qam  = ue->beta_qam;
```

### 2) Buffer before compression
- The modulation output remains FP16 (`__half2`) in the grid tensor at `cell_output_tensor_addr`. No additional scaling occurs between modulation and compression.

### 3) BFP compression (DL U‑plane, assume BFP9)
- The compression kernel reads FP16 IQs, applies cell-level `beta_dl` power scaling, finds a PRB exponent (`shift`), and packs mantissas into 9 bits (plus 1-byte exponent) per PRB.
```
cuPHY-CP/cuphydriver/src/common/comp_kernel.cuh:33-56
const float   beta      = params.beta[cell_id];
...
scale_compress_blockFP(iptr, prb_ptrs, beta, prbs_per_symbol, compression_bitwidths, ...);
```
```
cuPHY-CP/compression_decompression/comp_decomp_lib/include/gpu_blockFP.h:101-140
vi[i] = (int)((float)vin.vh[2*i]   * beta);
vq[i] = (int)((float)vin.vh[2*i+1] * beta);
...
shift = max(0, 33 - __clz(maxV) - compbits);   // compbits = 9 for BFP9
...
packOutput<3>(..., shift, compbits, sm, sm_prb_ptr);
```
```
cuPHY-CP/compression_decompression/comp_decomp_lib/include/gpu_packing.h:276-289
// 9-bit mantissa packing (3*9+1 = 28 bytes per PRB)
sm[offset]     = (vi[0] >> 1);
sm[offset + 1] = (vi[0] << 7) | ((vq[0] >> 2) & 0x7f);
...
warpWrite<9,...>(...);
```
- U‑plane builder uses PRB size for BFP9 when forming eCPRI sections.
```
cuPHY-CP/aerial-fh-driver/include/aerial-fh-driver/oran.hpp:1753-1766
compressed_prb_size = (comp_bits_cell == ORAN_BFP_NO_COMPRESSION) ? PRB_SIZE_16F :
                      (comp_bits_cell == ORAN_BFP_COMPRESSION_14_BITS) ? PRB_SIZE_14F :
                                                                      PRB_SIZE_9F;
```

### 4) Scale-determining parameters
- `beta_qam` (per UE/TB): amplitude scaling applied in modulation (directly multiplies constellation points).
- `beta_dl` (per cell): applied during compression; computed from PHY config (`dl_bit_width`, `exponent_dl`, `fs_offset_dl`, `ref_dl`, PRBs) and OAM attenuation. When `fixBetaDl()` is enabled, it forces constants for BFP:  
```
cuPHY-CP/cuphydriver/src/common/cell.cpp:664-688
beta_dl = sqrt(numerator / denominator);
if (fixBetaDl && dl_bit_width == BFP_COMPRESSION_9_BITS)  beta_dl = 65536;
if (fixBetaDl && dl_bit_width == BFP_COMPRESSION_14_BITS) beta_dl = 2097152;
```
- Full expression (before optional forcing):  
  - `sqrt_fs0 = 2^(dl_bit_width - 1) * 2^(2^(exponent_dl) - 1)`  
  - `fs = sqrt_fs0^2 * 2^(-fs_offset_dl)`  
  - `numerator = fs * 10^(ref_dl / 10)`  
  - `denominator = 12 * nPrbDlBwp`  
  - `beta_dl = sqrt(numerator / denominator)`  
- Code implementation of the above:  
```
cuPHY-CP/cuphydriver/src/common/cell.cpp:664-688
sqrt_fs0       = pow(2., dl_bit_width - 1) * pow(2., pow(2., exponent_dl) - 1);
fs             = sqrt_fs0 * sqrt_fs0 * pow(2., -1 * fs_offset_dl);
numerator      = fs * pow(10., ref_dl / 10.);
denominator    = 12 * phy_stat.nPrbDlBwp;
beta_dl        = sqrt(numerator / denominator);
...
if (pdctx->fixBetaDl()) {
    if (dl_bit_width == BFP_COMPRESSION_9_BITS)  { beta_dl = 65536;   }
    else if (dl_bit_width == BFP_COMPRESSION_14_BITS) { beta_dl = 2097152; }
}
```
- `dl_bit_width`: selects BFP bit depth (9 here), which sets mantissa bits and influences `beta_dl` computation.
- PRB exponent `shift`: per-PRB, chosen so `beta_dl * IQ` fits in signed (compbits−1) bits; stored as the compression parameter byte.

### 4a) Example `beta_dl` (BFP9) values
- Assumptions: `dl_bit_width=9`, `exponent_dl=4`, `nPrbDlBwp=273`, `fixBetaDl` disabled.
- Intermediate constant: `sqrt_fs0 = 2^(8) * 2^(2^4 - 1) = 8,388,608`.
- Sensitivity: `beta_dl ∝ sqrt(fs * 10^(ref_dl/10))`. So +3 dB `ref_dl` multiplies `beta_dl` by √2 ≈ 1.414; +1 `fs_offset_dl` multiplies `beta_dl` by 1/√2 ≈ 0.707 (since `fs` halves inside the square root). A +3 dB `ref_dl` bump is roughly canceled by a +1 `fs_offset_dl` bump, which is consistent with the ratios visible in the table (e.g., 58140 → 41111 when `fs_offset_dl` rises by 1; 58140 → 82125 when `ref_dl` rises by 3 dB).
- Resulting `beta_dl` across `fs_offset_dl` ∈ {2,3,4,5,6,7} and `ref_dl` ∈ {0,1,2,3} (rounded; exact values in parentheses):

| fs_offset_dl | ref_dl=0 | ref_dl=1 | ref_dl=2 | ref_dl=3 |
| --- | --- | --- | --- | --- |
| 2 | 73280 (73280.42) | 82222 (82221.99) | 92255 (92254.58) | 103511 (103511.35) |
| 3 | 51817 (51817.08) | 58140 (58139.72) | 65234 (65233.84) | 73194 (73193.58) |
| 4 | 36640 (36640.21) | 41111 (41110.99) | 46127 (46127.29) | 51756 (51755.67) |
| 5 | 25909 (25908.54) | 29070 (29069.86) | 32617 (32616.92) | 36597 (36596.79) |
| 6 | 18320 (18320.11) | 20555 (20555.50) | 23064 (23063.65) | 25878 (25877.84) |
| 7 | 12954 (12954.27) | 14535 (14534.93) | 16308 (16308.46) | 18298 (18298.39) |

### 5) End-to-end numeric path (BFP9)
`IQ_mod` = normalized_QAM * `beta_qam` (FP16)  
`IQ_scaled` = `beta_dl` * `IQ_mod` (FP32 → int)  
`IQ_bfp` = (IQ_scaled >> shift) packed as 9-bit mantissas + 1-byte shift per PRB → ORAN U‑plane payload (`PRB_SIZE_9F` per PRB).

### 6) Example 256QAM symbol → 9-bit BFP (ref_dl=0, exponent_dl=4, `dl_bit_width`=9)
- Assumptions: `beta_qam=1`, symbol is the peak 256QAM point `(I,Q) = (±1.150447, ±1.150447)` from `rev_qam_256`, `nPrbDlBwp=273`, `fixBetaDl` disabled, `fs_offset_dl` ∈ {2,3,4,5,6,7}, `ref_dl=0`, `exponent_dl=4`.
- Computed `beta_dl` for these settings (using the code formula with `sqrt_fs0 = 2^(8) * 2^(2^4 - 1)`):  
  - `fs_offset_dl=2`: 73,280.42  
  - `fs_offset_dl=3`: 51,817.08  
  - `fs_offset_dl=4`: 36,640.21  
  - `fs_offset_dl=5`: 25,908.54  
  - `fs_offset_dl=6`: 18,320.11  
  - `fs_offset_dl=7`: 12,954.27  
- Modulation output (FP16) for the chosen symbol: `IQ_mod ≈ (±1.150447, ±1.150447)`.
- Compression input (FP32 multiply, then int truncation inside `scale_compress_blockFP`):
  - `IQ_scaled = IQ_mod * beta_dl`
  - `IQ_int = int(IQ_scaled)` (truncates toward zero)
- Shift (`shift` exponent byte) for 9-bit BFP:  
  `shift = max(0, 33 - clz(maxV) - 9) = max(0, 1 + bitlen(maxV) - 9)`; values below use the peak magnitude per PRB.
- Resulting 9-bit mantissa pairs (I,Q) per RE for the peak-symbol case:

| fs_offset_dl | beta_dl | IQ_scaled (I,Q) | IQ_int (I,Q) | BFP exponent | Stored mantissa (I,Q) |
| --- | --- | --- | --- | --- | --- |
| 2 | 73,280.42 | (84,305.3, 84,305.3) | (84,305, 84,305) | 9 | (164, 164) |
| 3 | 51,817.08 | (59,612.8, 59,612.8) | (59,612, 59,612) | 8 | (232, 232) |
| 4 | 36,640.21 | (42,152.6, 42,152.6) | (42,152, 42,152) | 8 | (164, 164) |
| 5 | 25,908.54 | (29,806.4, 29,806.4) | (29,806, 29,806) | 7 | (232, 232) |
| 6 | 18,320.11 | (21,076.3, 21,076.3) | (21,076, 21,076) | 7 | (164, 164) |
| 7 | 12,954.27 | (14,903.2, 14,903.2) | (14,903, 14,903) | 6 | (232, 232) |

#### Smallest 256QAM constellation point (±0.076696)
| fs_offset_dl | beta_dl | IQ_scaled (I,Q) | IQ_int (I,Q) | BFP exponent | Stored mantissa (I,Q) |
| --- | --- | --- | --- | --- | --- |
| 2 | 73,280.42 | (5,620.4, 5,620.4) | (5,620, 5,620) | 5 | (175, 175) |
| 3 | 51,817.08 | (3,974.2, 3,974.2) | (3,974, 3,974) | 4 | (248, 248) |
| 4 | 36,640.21 | (2,810.2, 2,810.2) | (2,810, 2,810) | 4 | (175, 175) |
| 5 | 25,908.54 | (1,987.1, 1,987.1) | (1,987, 1,987) | 3 | (248, 248) |
| 6 | 18,320.11 | (1,405.1, 1,405.1) | (1,405, 1,405) | 3 | (175, 175) |
| 7 | 12,954.27 | (993.6, 993.6) | (993, 993) | 2 | (248, 248) |

Relevant code paths:
- Modulation scaling by `beta_qam`: `cuPHY/src/cuphy/modulation_mapper/modulation_mapper.cu:201-228`.
- Compression scaling, int cast, max, shift, and packing:  
  `cuPHY-CP/compression_decompression/comp_decomp_lib/include/gpu_blockFP.h:101-140` (scale and shift) and `gpu_packing.h:276-289` (9-bit packing).

### 7) 4-layer 4T4R precoder example (Table 5.2.2.2.1-8, P_csi-rs = 4, N1=2, N2=1, i_11=i_12=i_13=i_2=0)
- Codebook assumed (Type I single panel, ports=4) as the 4×4 Hadamard/DFT-2 Kronecker with normalization `1/sqrt(4·P_csi-rs) = 1/4` for P_csi-rs=4:  
  `W = (1/4) * [[1,  1,  1,  1], [1, -1,  1, -1], [1,  1, -1, -1], [1, -1, -1,  1]]`  
  (ports as rows, layers as columns).  
- With identical symbols on all 4 layers, the antenna-domain result is:  
  `y0 = (1/4)*(4·s) = s`, `y1 = 0`, `y2 = 0`, `y3 = 0`, where `s` is the per-layer symbol.
- Apply to two symbols (as in prior sections), using the same `beta_dl` values above (ref_dl=0, exponent_dl=4, dl_bit_width=9):
  - Largest 256QAM point `s_max = (±1.150447, ±1.150447)` ⇒ `y0 = s_max`, others 0.
  - Smallest 256QAM point `s_min = (±0.076696, ±0.076696)` ⇒ `y0 = s_min`, others 0.
- BFP9 results for port 0 (ports 1–3 stay zero):

Largest point (y0 = 1.150447 + j·1.150447 after the 1/4 normalized precoder):
| fs_offset_dl | beta_dl | IQ_scaled ≈ (I,Q) | IQ_int (I,Q) | shift | mantissa (I,Q) |
| --- | --- | --- | --- | --- | --- |
| 2 | 73,280.42 | (21,076.3, 21,076.3) | (21,076, 21,076) | 7 | (164, 164) |
| 3 | 51,817.08 | (14,903.2, 14,903.2) | (14,903, 14,903) | 6 | (232, 232) |
| 4 | 36,640.21 | (10,538.2, 10,538.2) | (10,538, 10,538) | 6 | (164, 164) |
| 5 | 25,908.54 | (7,451.6, 7,451.6) | (7,451, 7,451) | 5 | (232, 232) |
| 6 | 18,320.11 | (5,269.1, 5,269.1) | (5,269, 5,269) | 5 | (164, 164) |
| 7 | 12,954.27 | (3,725.8, 3,725.8) | (3,725, 3,725) | 4 | (232, 232) |

Smallest point (y0 = 0.076696 + j·0.076696 after the 1/4 normalized precoder):
| fs_offset_dl | beta_dl | IQ_scaled ≈ (I,Q) | IQ_int (I,Q) | shift | mantissa (I,Q) |
| --- | --- | --- | --- | --- | --- |
| 2 | 73,280.42 | (1,405.1, 1,405.1) | (1,405, 1,405) | 3 | (175, 175) |
| 3 | 51,817.08 | (993.6, 993.6) | (993, 993) | 2 | (248, 248) |
| 4 | 36,640.21 | (702.5, 702.5) | (702, 702) | 2 | (175, 175) |
| 5 | 25,908.54 | (496.8, 496.8) | (496, 496) | 1 | (248, 248) |
| 6 | 18,320.11 | (351.3, 351.3) | (351, 351) | 1 | (175, 175) |
| 7 | 12,954.27 | (248.4, 248.4) | (248, 248) | 0 | (248, 248) |

Notes:
- Shifts use `shift = max(0, 1 + bitlen(max(|I|,|Q|)) - 9)`; mantissas are `IQ_int >> shift`. Values above are rounded for readability.
- The actual precoder matrix is supplied at runtime via `dyn_params->pCellGrpDynPrm->pPmwPrms[pmwPrmIdx].matrix`; the matrix shown here follows the 38.214 Table 5.2.2.2.1-8 base entry for the given indices.

#### Other PMIs from the same table (i_11=2 and i_11=6, i_12=i_13=i_2=0)
- Using the same Type-I single-panel family (Table 5.2.2.2.1-8, P_csi-rs=4) with column phase rotations (diag factors 1, e^{j·i_11·π/2}, e^{j·i_11·π}, e^{j·3·i_11·π/2}) applied to the base columns. Explicitly:
  - `W(i_11=0) = (1/4) * [[1, 1, 1, 1], [1, -1, 1, -1], [1, 1, -1, -1], [1, -1, -1, 1]]`
  - `W(i_11=1) = (1/4) * [[ 1,  j, -1, -j], [ 1, -j, -1,  j], [ 1,  j,  1,  j], [ 1, -j,  1, -j]]`
  - `W(i_11=2) = (1/4) * [[ 1, -1,  1, -1], [ 1,  1,  1,  1], [ 1, -1, -1,  1], [ 1,  1, -1, -1]]`
  - `W(i_11=6) = W(i_11=2)` (same phase pattern modulo 4)
  With equal symbols on all layers, these phase rotations still sum to the same port-0 output and cancel on ports 1–3, so antenna symbols and BFP mantissas match the i_11=0 case:
  - Largest 256QAM point: `y0 = (±1.150447, ±1.150447)`, others 0 → same BFP table as “Largest point”.
  - Smallest 256QAM point: `y0 = (±0.076696, ±0.076696)`, others 0 → same BFP table as “Smallest point”.
