This is the mostly complete DSP pipeline
  ******************************************************************************
  * @file           : main.c
  * @brief          : ECG HRV / stress monitor -- full DSP pipeline
  *                   STM32F429I-Discovery
  *
  *   Per-sample chain at Fs = 500 Hz:
  *
  *     ECG AFE  -> ADC1_IN5 (PA5)
  *              -> FIR HPF   (0.5 Hz, 2001-tap windowed sinc, Hanning)
  *              -> FIR notch ( 50 Hz,  201-tap windowed sinc, Hanning)
  *              -> NLMS      (32-tap, accel reference via I2C3, mu = 0.01)
  *              -> Pan-Tompkins enhancement
  *                    - 5-point derivative
  *                    - squaring
  *                    - 75-sample (150 ms) moving-window integration
  *              -> Adaptive threshold (SPKI / NPKI tracking) + 200 ms refractory
  *
  *     R-peak event:
  *              -> RR interval -> RR ring buffer
  *              -> Time HRV   : SDNN, RMSSD, normalised RMSSD
  *              -> Freq HRV   : Lomb-Scargle periodogram on RR series
  *                                LF (0.04 - 0.15 Hz)
  *                                HF (0.15 - 0.40 Hz)
  *                                LF / HF ratio
  *              -> Rule-based stress score (derivative + level on rmssd_norm)
  *
  *   
  *     FIR_HPF_TAPS, FIR_NOTCH_TAPS    filter quality vs RAM / settling time
  *     NLMS_MU                          step size; lower = stabler, slower
  *     PT_MWI_WIN                       150 ms window; widen for wide QRS
  *     PT_REFRACTORY_SAMPLES            blanking after each detection (200 ms)
  *     LS_FBINS                         Lomb-Scargle frequency resolution
  *     FREQ_HRV_BEATS                   how often LF/HF is recomputed
  *
  ******************************************************************************


  
