
bool Adafruit_ZeroTimer::tc_init()
{
  /* Temporary variable to hold all updates to the CTRLA
	 * register before they are written to it */
  uint16_t ctrla_tmp = 0;
  /* Temporary variable to hold all updates to the CTRLBSET
	 * register before they are written to it */
  uint8_t ctrlbset_tmp = 0;
  /* Temporary variable to hold all updates to the CTRLC
	 * register before they are written to it */
  uint8_t ctrlc_tmp = 0;
  /* Temporary variable to hold TC instance number */
  uint8_t instance = _timernum - TC_INSTANCE_OFFSET;


#if defined(__SAMD51__)
  GCLK->PCHCTRL[inst_gclk_id[instance]].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
                                              (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 0
#else
  /* Enable the user interface clock in the PM */
  PM->APBCMASK.reg |= inst_pm_apbmask[instance];

  /* Enable the slave counter if counter_size is 32-bit */
  if ((_counter_size == TC_COUNTER_SIZE_32BIT))
  {
    /* Enable the user interface clock in the PM */
    PM->APBCMASK.reg |= inst_pm_apbmask[instance + 1];
  }

  /* Setup clock for module */
  GCLK->CLKCTRL.reg = (uint16_t)(   GCLK_CLKCTRL_CLKEN |
                                    GCLK_CLKCTRL_GEN_GCLK0 | 
                                    inst_gclk_id[instance]);
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
#endif