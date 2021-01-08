// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module ibex_pmp #(
    // Granularity of NAPOT access,
    // 0 = No restriction, 1 = 8 byte, 2 = 16 byte, 3 = 32 byte, etc.
    parameter int unsigned PMPGranularity = 0,
    // Number of access channels (e.g. i-side + d-side)
    parameter int unsigned PMPNumChan     = 2,
    // Number of implemented regions
    parameter int unsigned PMPNumRegions  = 4
) (
    // Clock and Reset
    input  logic                    clk_i,
    input  logic                    rst_ni,

    // Interface to CSRs
    input  ibex_pkg::pmp_cfg_t      csr_pmp_cfg_i  [PMPNumRegions],
    input  logic [33:0]             csr_pmp_addr_i [PMPNumRegions],

    input  ibex_pkg::priv_lvl_e     priv_mode_i    [PMPNumChan],
    // Access checking channels
    input  logic [33:0]             pmp_req_addr_i [PMPNumChan],
    input  ibex_pkg::pmp_req_e      pmp_req_type_i [PMPNumChan],
    output logic                    pmp_req_err_o  [PMPNumChan]

);

  import ibex_pkg::*;

  // Access Checking Signals
  logic [33:0]                                region_start_addr [PMPNumRegions];
  logic [33:PMPGranularity+2]                 region_addr_mask  [PMPNumRegions];
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   region_match_high;
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   region_match_low;
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   region_match_both;
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   region_perm_check;
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   machine_access_fault;
  logic [PMPNumChan-1:0][PMPNumRegions-1:0]   user_access_allowed;
  logic [PMPNumChan-1:0]                      access_fault;


  // ---------------
  // Access checking
  // ---------------
  generate
  genvar r;
  genvar b;	
  for (r = 0; r < PMPNumRegions; r++) begin : g_addr_exp
    // Start address for TOR matching
    if (r == 0) begin : g_entry0
      assign region_start_addr[r] = (csr_pmp_cfg_i[r].mode == PMP_MODE_TOR) ? 34'h000000000 :
                                                                              csr_pmp_addr_i[r];
    end else begin : g_oth
      assign region_start_addr[r] = (csr_pmp_cfg_i[r].mode == PMP_MODE_TOR) ? csr_pmp_addr_i[r-1] :
                                                                              csr_pmp_addr_i[r];
    end
    // Address mask for NA matching
    for (b = PMPGranularity+2; b < 34; b++) begin : g_bitmask
      if (b == PMPGranularity+2) begin : g_bit0
        // Always mask bit (G+2) for NAPOT
        assign region_addr_mask[r][b] = (csr_pmp_cfg_i[r].mode != PMP_MODE_NAPOT);
      end else begin : g_others
        // We will mask this bit if it is within the programmed granule
        // i.e. addr = yyyy 0111
        //                  ^
        //                  | This bit pos is the top of the mask, all lower bits set
        // thus mask = 1111 0000
        assign region_addr_mask[r][b] = (csr_pmp_cfg_i[r].mode != PMP_MODE_NAPOT) |
                                        ~&csr_pmp_addr_i[r][b-1:PMPGranularity+2];
      end
    end
  end
  endgenerate	

  generate
  genvar d;
  genvar h;
  for (d = 0; d < PMPNumChan; d++) begin : g_access_check
    for (h = 0; h < PMPNumRegions; h++) begin : g_regions
      // TOR Region high/low matching is reused for all match types
      assign region_match_low[d][h]     = (pmp_req_addr_i[d][33:PMPGranularity+2] >=
                                           // Comparators are sized according to granularity
                                           (region_start_addr[h][33:PMPGranularity+2] &
                                            region_addr_mask[h]));
      assign region_match_high[d][h]    = (pmp_req_addr_i[d][33:PMPGranularity+2] <=
                                           csr_pmp_addr_i[h][33:PMPGranularity+2]);
      assign region_match_both[d][h]    = region_match_low[d][h] & region_match_high[d][h] &
                                          (csr_pmp_cfg_i[h].mode != PMP_MODE_OFF);
      // Check specific required permissions
      assign region_perm_check[d][h] =
          ((pmp_req_type_i[d] == PMP_ACC_EXEC)  & csr_pmp_cfg_i[h].exec) |
          ((pmp_req_type_i[d] == PMP_ACC_WRITE) & csr_pmp_cfg_i[h].write) |
          ((pmp_req_type_i[d] == PMP_ACC_READ)  & csr_pmp_cfg_i[h].read);
      // In machine mode, any match to a locked region without sufficient permissions is a fault
      assign machine_access_fault[d][h] = region_match_both[d][h] & csr_pmp_cfg_i[h].lock &
                                          ~region_perm_check[d][h];
      // In any other mode, any access should fault unless is matches a region
      assign user_access_allowed[d][h]  = region_match_both[d][h] & region_perm_check[d][h];
    end
    assign access_fault[d] = (priv_mode_i[d] == PRIV_LVL_M) ? |machine_access_fault[d] :
                                                              ~|user_access_allowed[d];

    assign pmp_req_err_o[d] = access_fault[d];
  end
  endgenerate

endmodule