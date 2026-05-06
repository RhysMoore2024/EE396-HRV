/* USER CODE BEGIN Header */
/**
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
  *   Notes / tuning knobs (search this file for the symbol):
  *     FIR_HPF_TAPS, FIR_NOTCH_TAPS    filter quality vs RAM / settling time
  *     NLMS_MU                          step size; lower = stabler, slower
  *     PT_MWI_WIN                       150 ms window; widen for wide QRS
  *     PT_REFRACTORY_SAMPLES            blanking after each detection (200 ms)
  *     LS_FBINS                         Lomb-Scargle frequency resolution
  *     FREQ_HRV_BEATS                   how often LF/HF is recomputed
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CRC_HandleTypeDef hcrc;
DMA2D_HandleTypeDef hdma2d;
I2C_HandleTypeDef hi2c3;
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi5;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
SDRAM_HandleTypeDef hsdram1;
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
//static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
//static void MX_LTDC_Init(void);
//static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart1;
extern LTDC_HandleTypeDef hltdc;
extern I2C_HandleTypeDef  hi2c3;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* =========================================================
   VSYNC helper -- wait for the blanking period before drawing
   ========================================================= */
static inline void wait_vsync(void)
{
    while ( LTDC->CDSR & LTDC_CDSR_VSYNCS) { }
    while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS)) { }
}

/* =========================================================
   SYSTEM CONFIG
   ========================================================= */
#define FS                       500       /* ADC sample rate (Hz)               */
#define HR_SMOOTH_LEN              5       /* HR averaging window (beats)        */
#define BTN_DEBOUNCE_MS          300

/* =========================================================
   FIR FILTER CONFIG
   Coefficients computed at boot -- avoids hard-coding 200+ floats.
   ========================================================= */
/* HPF length is large because the cutoff is very low.  At Fs = 500 Hz a
   Hanning-windowed sinc has transition BW ~3.1 Fs / N, so for fc = 0.5 Hz
   we need N ~2001 to land properly at -6 dB at 0.5 Hz with > 35 dB DC
   rejection.  Memory cost: 2001 floats coeff + 2001 floats delay = 16 KB.
   Latency: 1000 samples = 2 s constant group delay (acceptable for HRV;
   RR intervals are differences so absolute delay does not matter).        */
#define FIR_HPF_TAPS            2001       /* 0.5 Hz HPF, Hanning sinc           */
#define FIR_HPF_FC               0.5f      /* cutoff (Hz)                        */

/* Notch at 50 Hz (mains).  201 taps with +/- 5 Hz BW gives ~ -38 dB at 50 Hz
   with sharp skirts.  At Fs = 500 Hz transition BW for Hanning is ~7.7 Hz
   which brackets the notch BW nicely.                                      */
#define FIR_NOTCH_TAPS           201       /* 50 Hz band-stop, Hanning sinc      */
#define FIR_NOTCH_F0            50.0f      /* notch centre (Hz)                  */
#define FIR_NOTCH_BW             5.0f      /* half-bandwidth (Hz) -> 45 to 55    */

/* =========================================================
   NLMS ADAPTIVE FILTER CONFIG
   ========================================================= */
#define NLMS_TAPS                32        /* adaptive filter length             */
#define NLMS_MU                  0.01f     /* step size (lower = stabler)        */
#define NLMS_EPS                 1e-6f     /* power-norm regularisation          */
#define ACCEL_DECIMATE           25        /* read accel every N samples (20 Hz) */

/* =========================================================
   PAN-TOMPKINS QRS ENHANCEMENT CONFIG
   ========================================================= */
#define PT_MWI_WIN               75        /* moving-window integrator (150 ms)  */
#define PT_REFRACTORY_SAMPLES   100        /* 200 ms blanking after each peak    */
#define PT_SPKI_INIT             0.001f    /* signal-peak EMA seed               */
#define PT_NPKI_INIT             0.0001f   /* noise-peak EMA seed                */
#define PT_THRESH_SCALE          0.25f     /* threshold = NPKI + S*(SPKI-NPKI)   */
#define PT_PEAK_ALPHA            0.0625f    /* SPKI / NPKI EMA coefficient        */

/* =========================================================
   LCD LAYOUT  (240 x 320, portrait)
   ========================================================= */
#define WAVE_X                   0
#define WAVE_Y                   100
#define WAVE_W                   240
#define WAVE_H                   215
#define WAVE_MID                 (WAVE_Y + WAVE_H / 2)

#define MODE_ECG                 0
#define MODE_HRV                 1

/* =========================================================
   HRV / STRESS CONFIG
   ========================================================= */
#define RR_BUF_LEN              30         /* ~30 s at 60 BPM -- LS-friendly      */
#define HRV_HIST_LEN             8
#define BASELINE_BEATS          15
#define FILTER_WARMUP_SAMPLES 2500         /* 5 s -- 2 s HPF settle + NLMS converge */
#define STRESS_ALPHA             0.12f
#define DERIV_GAIN               8.0f
#define LEVEL_GAIN               1.2f
#define DERIV_WEIGHT             0.50f
#define LEVEL_WEIGHT             0.50f
#define TRANSITION_BPM_THRESH   15
#define TRANSITION_BEATS         4
#define HRV_DISPLAY_INTERVAL     5

/* Frequency-domain HRV (Lomb-Scargle) */
#define LS_FBINS                64         /* freq bins from F_LO to F_HI         */
#define LS_F_LO                  0.04f     /* sweep starts at LF lower edge       */
#define LS_F_HI                  0.40f     /* sweep ends   at HF upper edge       */
#define LS_LF_LO                 0.04f
#define LS_LF_HI                 0.15f
#define LS_HF_LO                 0.15f
#define LS_HF_HI                 0.40f
#define FREQ_HRV_BEATS          15         /* recompute LF/HF every N beats       */
#define FREQ_HRV_MIN_RR          8         /* need at least N RR points for LS    */

/* =========================================================
   FIR FILTER STATE
   ========================================================= */
static float hpf_coeffs[FIR_HPF_TAPS];
static float hpf_delay [FIR_HPF_TAPS];
static int   hpf_idx = 0;

static float ntc_coeffs[FIR_NOTCH_TAPS];
static float ntc_delay [FIR_NOTCH_TAPS];
static int   ntc_idx = 0;

/* =========================================================
   NLMS STATE
   ========================================================= */
static float nlms_w[NLMS_TAPS];     /* adaptive weights                          */
static float nlms_x[NLMS_TAPS];     /* reference (accel) delay line              */
static int   nlms_idx  = 0;
static float accel_ref = 0.0f;      /* last accel-Z reading, normalised [-1, 1]  */
static int   accel_tick = 0;
static int   accel_present = 0;     /* set to 1 once accel responds; else NLMS off */

/* =========================================================
   PAN-TOMPKINS STATE
   ========================================================= */
static float pt_drv_buf[4]      = {0};   /* 5-pt derivative delay line (x[n-1..n-4]) */
static int   pt_drv_idx          = 0;
static float pt_mwi_buf[PT_MWI_WIN] = {0};
static int   pt_mwi_idx          = 0;
static float pt_mwi_sum          = 0.0f;
static float pt_spki             = PT_SPKI_INIT;
static float pt_npki             = PT_NPKI_INIT;
static float pt_threshold        = 0.0f;

/* =========================================================
   PEAK / SAMPLE COUNTERS
   ========================================================= */
static uint32_t sample_n    = 0;
static uint32_t last_peak_n = 0;
static uint32_t refractory  = 0;

/* =========================================================
   HR SMOOTHING
   ========================================================= */
static float hr_buf[HR_SMOOTH_LEN] = {0};
static int   hr_buf_idx              = 0;
static int   hr_buf_count            = 0;

/* =========================================================
   HRV TIME-DOMAIN STATE
   ========================================================= */
static float rr_buf[RR_BUF_LEN] = {0};
static int   rr_head             = 0;
static int   rr_count            = 0;

static float norm_hist[HRV_HIST_LEN] = {0};
static int   hrv_hist_head            = 0;
static int   hrv_hist_count           = 0;

static float hrv_rmssd      = 0.0f;
static float hrv_sdnn       = 0.0f;
static float hrv_mean_rr    = 0.0f;
static float hrv_rmssd_norm = 0.0f;
static float hrv_d_rmssd    = 0.0f;
static float hrv_d_norm     = 0.0f;
static float hrv_max_delta  = 0.0f;
static float hrv_stress     = 0.0f;
static float hr_display     = 0.0f;

/* HRV frequency-domain state */
static float hrv_lf      = 0.0f;     /* LF band power (rel. units, prop. to ms^2)*/
static float hrv_hf      = 0.0f;     /* HF band power                            */
static float hrv_lf_hf   = 0.0f;     /* LF / HF ratio                            */
static int   freq_hrv_counter = 0;   /* beats since last LS recompute            */

/* Baseline */
static float baseline_norm     = 0.094f;
static float baseline_rmssd    = 40.0f;
static float baseline_mean_rr  = 800.0f;
static int   baseline_count    = 0;
static int   baseline_done     = 0;

/* Transition flag */
static float hr_trans_buf[TRANSITION_BEATS] = {0};
static int   hr_trans_head  = 0;
static int   hr_trans_count = 0;
static int   in_transition  = 0;

/* =========================================================
   DISPLAY STATE
   ========================================================= */
static int      display_mode     = MODE_ECG;
static uint16_t wave_x           = WAVE_X + 1;
static int16_t  prev_y           = WAVE_MID;
static int      draw_skip        = 0;

/* HRV display smoothing */
static float rmssd_disp   = 0.0f;
static float sdnn_disp    = 0.0f;
static float stress_disp  = 0.0f;
static float mean_rr_disp = 0.0f;
static float lf_disp      = 0.0f;
static float hf_disp      = 0.0f;
static float lf_hf_disp   = 0.0f;
static float rmssd_prev = 0.0f;

//Globals


/* =========================================================
   FIR HPF DESIGN  (windowed-sinc, Hanning)
   Cutoff fc, length N, centre M = N/2.
   Normalised cutoff fc_n = fc / Fs (in [0, 0.5]).
   Ideal LPF h_lp[k] = 2 fc_n sinc(2 fc_n k),  sinc(x) = sin(pi x)/(pi x)
   HPF by spectral inversion: h_hp[k] = delta[k] - h_lp[k]
   ========================================================= */
static void fir_hpf_design(void)
{
    const int   N    = FIR_HPF_TAPS;
    const int   M    = N / 2;
    const float fc_n = FIR_HPF_FC / (float)FS;

    for (int n = 0; n < N; n++) {
        int   k = n - M;
        float h_lp;
        if (k == 0) {
            h_lp = 2.0f * fc_n;
        } else {
            float kf = (float)k;
            h_lp = sinf(2.0f * M_PI * fc_n * kf) / (M_PI * kf);
        }
        float h_hp = (k == 0) ? (1.0f - h_lp) : (-h_lp);
        float w    = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)n / (float)(N - 1)));
        hpf_coeffs[n] = h_hp * w;
    }
    memset(hpf_delay, 0, sizeof(hpf_delay));
    hpf_idx = 0;
}

/* =========================================================
   FIR NOTCH DESIGN  (windowed-sinc, Hanning)
   Designed as h_notch = delta - h_bp,
   where h_bp = h_lp(f0+bw) - h_lp(f0-bw).
   ========================================================= */
static void fir_notch_design(void)
{
    const int   N    = FIR_NOTCH_TAPS;
    const int   M    = N / 2;
    const float f1n  = (FIR_NOTCH_F0 - FIR_NOTCH_BW) / (float)FS;
    const float f2n  = (FIR_NOTCH_F0 + FIR_NOTCH_BW) / (float)FS;

    for (int n = 0; n < N; n++) {
        int   k = n - M;
        float h_lp1, h_lp2;
        if (k == 0) {
            h_lp1 = 2.0f * f1n;
            h_lp2 = 2.0f * f2n;
        } else {
            float kf = (float)k;
            h_lp1 = sinf(2.0f * M_PI * f1n * kf) / (M_PI * kf);
            h_lp2 = sinf(2.0f * M_PI * f2n * kf) / (M_PI * kf);
        }
        float h_bp    = h_lp2 - h_lp1;
        float h_notch = (k == 0) ? (1.0f - h_bp) : (-h_bp);
        float w       = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)n / (float)(N - 1)));
        ntc_coeffs[n] = h_notch * w;
    }
    memset(ntc_delay, 0, sizeof(ntc_delay));
    ntc_idx = 0;
}

/* =========================================================
   FIR APPLY  (direct form, circular delay line)
   coeffs[0] multiplies the newest sample, coeffs[N-1] the oldest.
   For symmetric (linear-phase) FIRs, ordering does not affect output.
   ========================================================= */
static float fir_apply(float x, const float *coeffs, float *delay,
                       int *idx, int N)
{
    delay[*idx] = x;

    float y = 0.0f;
    int   k = *idx;
    for (int i = 0; i < N; i++) {
        y += coeffs[i] * delay[k];
        k = (k == 0) ? (N - 1) : (k - 1);
    }
    *idx = (*idx + 1) % N;
    return y;
}

/* =========================================================
   ACCELEROMETER -- LSM303DLHC on I2C3 (default address 0x19)
   The board exposes I2C3 on the extension headers; an external
   accel module is assumed.  If no device responds, accel_present
   stays 0 and the NLMS branch is skipped.
   ========================================================= */
#define ACCEL_I2C_ADDR_W   (0x19 << 1)        /* 7-bit 0x19, HAL expects shifted */
#define ACCEL_REG_CTRL1     0x20
#define ACCEL_REG_OUT_Z_L   0x2C
#define ACCEL_REG_WHO_AM_I  0x0F
#define ACCEL_WHO_AM_I_VAL  0x33               /* LSM303DLHC accel ID            */

static void accel_init(void)
{
    /* Probe WHO_AM_I */
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(&hi2c3, ACCEL_I2C_ADDR_W | 1, ACCEL_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &id, 1, 20) == HAL_OK
        && id == ACCEL_WHO_AM_I_VAL) {
        /* CTRL_REG1: 0x57 -> ODR=100 Hz, normal mode, X/Y/Z enabled */
        uint8_t cfg = 0x57;
        if (HAL_I2C_Mem_Write(&hi2c3, ACCEL_I2C_ADDR_W, ACCEL_REG_CTRL1,
                              I2C_MEMADD_SIZE_8BIT, &cfg, 1, 20) == HAL_OK) {
            accel_present = 1;
        }
    }
    memset(nlms_w, 0, sizeof(nlms_w));
    memset(nlms_x, 0, sizeof(nlms_x));
}

static void accel_read(void)
{
    if (!accel_present) return;

    /* Z axis: 2 bytes auto-increment (set MSB of register address) */
    uint8_t buf[2] = {0, 0};
    if (HAL_I2C_Mem_Read(&hi2c3, ACCEL_I2C_ADDR_W | 1,
                         ACCEL_REG_OUT_Z_L | 0x80,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 5) == HAL_OK) {
        int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
        accel_ref = (float)raw / 32768.0f;
    }
}

/* =========================================================
   NLMS ADAPTIVE FILTER
   Primary  = ECG after FIR (contains residual motion artifact)
   Reference = accel_ref (motion proxy)
   Output   = ECG - estimated_artifact (returned)
   Update   : w += mu * e * x_ref / (||x_ref||^2 + eps)
   ========================================================= */
static float nlms_filter(float ecg_in, float ref)
{
    /* If no accel attached, NLMS is a no-op pass-through */
    if (!accel_present) return ecg_in;

    nlms_x[nlms_idx] = ref;

    /* y = w^T * x */
    float y = 0.0f;
    int   k = nlms_idx;
    for (int i = 0; i < NLMS_TAPS; i++) {
        y += nlms_w[i] * nlms_x[k];
        k  = (k == 0) ? (NLMS_TAPS - 1) : (k - 1);
    }

    float e = ecg_in - y;

    /* ||x||^2 + eps */
    float pwr = NLMS_EPS;
    for (int i = 0; i < NLMS_TAPS; i++) pwr += nlms_x[i] * nlms_x[i];

    float step = NLMS_MU * e / pwr;
    k = nlms_idx;
    for (int i = 0; i < NLMS_TAPS; i++) {
        nlms_w[i] += step * nlms_x[k];
        k = (k == 0) ? (NLMS_TAPS - 1) : (k - 1);
    }

    nlms_idx = (nlms_idx + 1) % NLMS_TAPS;
    return e;
}

/* =========================================================
   PAN-TOMPKINS STAGE 1: 5-POINT DERIVATIVE
   y[n] = (1/8) * (2 x[n] + x[n-1] - x[n-3] - 2 x[n-4]) * Fs
   The Fs scaling cancels into the squaring step's amplitude,
   so we just use the unscaled coefficient pattern.
   Causal form -- introduces 2 samples of group delay.
   ========================================================= */
static float pt_derivative(float x_now)
{
    /* delay buffer holds [x[n-4], x[n-3], x[n-2], x[n-1]] in some rotation;
       pt_drv_idx points at the slot currently holding x[n-4] (oldest).      */
    float xn4 = pt_drv_buf[ pt_drv_idx               ];     /* x[n-4]        */
    float xn3 = pt_drv_buf[(pt_drv_idx + 1) & 3      ];     /* x[n-3]        */
    /* float xn2 = pt_drv_buf[(pt_drv_idx + 2) & 3];        not used         */
    float xn1 = pt_drv_buf[(pt_drv_idx + 3) & 3      ];     /* x[n-1]        */

    /* Overwrite the oldest slot with the current sample */
    pt_drv_buf[pt_drv_idx] = x_now;
    pt_drv_idx = (pt_drv_idx + 1) & 3;

    return 0.125f * (2.0f * x_now + xn1 - xn3 - 2.0f * xn4);
}

/* =========================================================
   PAN-TOMPKINS STAGE 2 + 3: SQUARE + MOVING-WINDOW INTEGRATION
   Returns running mean of the last PT_MWI_WIN squared samples.
   O(1) update via running sum.
   ========================================================= */
static float pt_square_mwi(float drv)
{
    float sq = drv * drv;
    pt_mwi_sum -= pt_mwi_buf[pt_mwi_idx];
    pt_mwi_buf[pt_mwi_idx] = sq;
    pt_mwi_sum += sq;
    pt_mwi_idx = (pt_mwi_idx + 1) % PT_MWI_WIN;
    return pt_mwi_sum / (float)PT_MWI_WIN;
}

/* =========================================================
   PAN-TOMPKINS ADAPTIVE THRESHOLD
   Standard SPKI / NPKI tracking:
     SPKI = a * peak + (1 - a) * SPKI    when peak is QRS
     NPKI = a * peak + (1 - a) * NPKI    when peak is noise
     thr  = NPKI + 0.25 * (SPKI - NPKI)
   ========================================================= */
static void pt_update_signal_peak(float peak)
{
    pt_spki = PT_PEAK_ALPHA * peak + (1.0f - PT_PEAK_ALPHA) * pt_spki;
    pt_threshold = pt_npki + PT_THRESH_SCALE * (pt_spki - pt_npki);
}

static void pt_update_noise_peak(float peak)
{
    pt_npki = PT_PEAK_ALPHA * peak + (1.0f - PT_PEAK_ALPHA) * pt_npki;
    pt_threshold = pt_npki + PT_THRESH_SCALE * (pt_spki - pt_npki);
}

/* =========================================================
   HR SMOOTHING (running mean of last HR_SMOOTH_LEN beats)
   ========================================================= */
static float hr_smoothed(float hr_now)
{
    hr_buf[hr_buf_idx % HR_SMOOTH_LEN] = hr_now;
    hr_buf_idx++;
    if (hr_buf_count < HR_SMOOTH_LEN) hr_buf_count++;

    float sum = 0.0f;
    for (int i = 0; i < hr_buf_count; i++) sum += hr_buf[i];
    return sum / (float)hr_buf_count;
}

/* =========================================================
   RR BUFFER HELPERS
   ========================================================= */
static void rr_push(float rr_s)
{
    rr_buf[rr_head] = rr_s;
    rr_head = (rr_head + 1) % RR_BUF_LEN;
    if (rr_count < RR_BUF_LEN) rr_count++;
}

static int rr_oldest_idx(void)
{
    return ((rr_head - rr_count) % RR_BUF_LEN + RR_BUF_LEN) % RR_BUF_LEN;
}

static float rr_mean_s(void)
{
    if (rr_count == 0) return 0.0f;
    int   start = rr_oldest_idx();
    float sum   = 0.0f;
    for (int i = 0; i < rr_count; i++)
        sum += rr_buf[(start + i) % RR_BUF_LEN];
    return sum / (float)rr_count;
}

static float rr_rmssd_ms(void)
{
    if (rr_count < 2) return 0.0f;
    int   start  = rr_oldest_idx();
    float sum_sq = 0.0f;
    for (int i = 0; i < rr_count - 1; i++) {
        float d = (rr_buf[(start + i + 1) % RR_BUF_LEN]
                 - rr_buf[(start + i)     % RR_BUF_LEN]) * 1000.0f;
        sum_sq += d * d;
    }
    return sqrtf(sum_sq / (float)(rr_count - 1));
}

static float rr_sdnn_ms(void)
{
    if (rr_count < 2) return 0.0f;
    float mean  = rr_mean_s();
    int   start = rr_oldest_idx();
    float var   = 0.0f;
    for (int i = 0; i < rr_count; i++) {
        float d = (rr_buf[(start + i) % RR_BUF_LEN] - mean) * 1000.0f;
        var += d * d;
    }
    return sqrtf(var / (float)rr_count);
}

static float rr_max_delta_ms(void)
{
    if (rr_count < 2) return 0.0f;
    int   start = rr_oldest_idx();
    float max_d = 0.0f;
    for (int i = 0; i < rr_count - 1; i++) {
        float d = fabsf(rr_buf[(start + i + 1) % RR_BUF_LEN]
                      - rr_buf[(start + i)     % RR_BUF_LEN]) * 1000.0f;
        if (d > max_d) max_d = d;
    }
    return max_d;
}

/* =========================================================
   FREQUENCY-DOMAIN HRV: LOMB-SCARGLE PERIODOGRAM
   Operates directly on unevenly-spaced RR samples (no resampling).
   For each angular frequency w:
     tau = (1 / 2w) * atan2(sum sin(2 w t_i), sum cos(2 w t_i))
     P(w) = (1/2) * [ (sum x_i cos(w(t_i - tau)))^2 / sum cos^2(...)
                    + (sum x_i sin(w(t_i - tau)))^2 / sum sin^2(...) ]
   LF = sum P(w_k) * df  for f_k in [LS_LF_LO, LS_LF_HI)
   HF = sum P(w_k) * df  for f_k in [LS_HF_LO, LS_HF_HI)
   Units: relative ms^2 (proportional, not absolute calibrated).
   Cost: 64 freqs * 2 passes * 2 trig calls * 30 RR samples ~ 7.7k transcendentals
         per recompute -- ~10 ms on F4 with FPU, called once per FREQ_HRV_BEATS.
   ========================================================= */
static void compute_freq_hrv(void)
{
    const int N = rr_count;
    if (N < FREQ_HRV_MIN_RR) {
        hrv_lf = 0.0f; hrv_hf = 0.0f; hrv_lf_hf = 0.0f;
        return;
    }

    /* Build evenly-indexed t[] (cumulative beat times) and x[] (RR - mean) in ms */
    float t[RR_BUF_LEN];
    float x[RR_BUF_LEN];
    int   start = rr_oldest_idx();
    float mean  = rr_mean_s() * 1000.0f;     /* ms                                 */

    float t_acc = 0.0f;
    for (int i = 0; i < N; i++) {
        float rr_s  = rr_buf[(start + i) % RR_BUF_LEN];
        t_acc      += rr_s;
        t[i]        = t_acc;                  /* seconds                            */
        x[i]        = rr_s * 1000.0f - mean;  /* ms, mean-centred                   */
    }

    const float df = (LS_F_HI - LS_F_LO) / (float)(LS_FBINS - 1);

    float lf_acc = 0.0f;
    float hf_acc = 0.0f;

    for (int b = 0; b < LS_FBINS; b++) {
        float f = LS_F_LO + (float)b * df;
        float w = 2.0f * M_PI * f;

        /* Pass 1: tau */
        float s2 = 0.0f, c2 = 0.0f;
        for (int i = 0; i < N; i++) {
            float a = 2.0f * w * t[i];
            s2 += sinf(a);
            c2 += cosf(a);
        }
        float tau = atan2f(s2, c2) / (2.0f * w);

        /* Pass 2: periodogram value */
        float xc = 0.0f, xs = 0.0f, cc = 0.0f, ss = 0.0f;
        for (int i = 0; i < N; i++) {
            float a = w * (t[i] - tau);
            float c = cosf(a);
            float s = sinf(a);
            xc += x[i] * c;
            xs += x[i] * s;
            cc += c * c;
            ss += s * s;
        }
        float P = 0.0f;
        if (cc > 1e-9f) P += (xc * xc) / cc;
        if (ss > 1e-9f) P += (xs * xs) / ss;
        P *= 0.5f;

        if      (f >= LS_LF_LO && f < LS_LF_HI) lf_acc += P * df;
        else if (f >= LS_HF_LO && f < LS_HF_HI) hf_acc += P * df;
    }

    hrv_lf    = lf_acc;
    hrv_hf    = hf_acc;
    hrv_lf_hf = (hf_acc > 1e-3f) ? (lf_acc / hf_acc) : 0.0f;
}

/* =========================================================
   HRV UPDATE  -- call once per valid detected beat.
   rr_s   : RR interval (seconds)
   hr_bpm : SMOOTHED BPM (must NOT be instantaneous)
   ========================================================= */
static void hrv_update(float rr_s, float hr_bpm)
{
    /* Gate during filter / NLMS warmup */
    if (sample_n < FILTER_WARMUP_SAMPLES) return;

    /* ---- Rate-transition flag (sliding window of smoothed BPM) ---- */
    hr_trans_buf[hr_trans_head] = hr_bpm;
    hr_trans_head = (hr_trans_head + 1) % TRANSITION_BEATS;
    if (hr_trans_count < TRANSITION_BEATS) hr_trans_count++;

    in_transition = 0;
    if (hr_trans_count == TRANSITION_BEATS) {
        float lo = hr_trans_buf[0], hi = hr_trans_buf[0];
        for (int i = 1; i < TRANSITION_BEATS; i++) {
            float v = hr_trans_buf[i];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if ((hi - lo) > (float)TRANSITION_BPM_THRESH) in_transition = 1;
    }

    /* ---- Push RR, recompute time-domain HRV ---- */
    rr_push(rr_s);

    if (rr_count < 8) return;     /* need a few beats before any metric is stable */

    hrv_mean_rr   = rr_mean_s() * 1000.0f;       /* ms                            */
    hrv_rmssd     = rr_rmssd_ms();
    hrv_sdnn      = rr_sdnn_ms();
    hrv_max_delta = rr_max_delta_ms();

    hrv_rmssd_norm   = (hrv_mean_rr > 1.0f) ? (hrv_rmssd / hrv_mean_rr) : 0.0f;

    /* ---- Per-beat normalised RMSSD history ---- */
    norm_hist[hrv_hist_head] = hrv_rmssd_norm;
    hrv_hist_head = (hrv_hist_head + 1) % HRV_HIST_LEN;
    if (hrv_hist_count < HRV_HIST_LEN) hrv_hist_count++;

    /* Derivatives across history window (norm/beat and ms/beat) */
    if (hrv_hist_count >= HRV_HIST_LEN) {
        int   tail = hrv_hist_head;          /* oldest entry                       */
        float oldest = norm_hist[tail];
        hrv_d_norm  = (hrv_rmssd_norm - oldest) / (float)HRV_HIST_LEN;
        /* Cheap ms/beat estimate: latest-vs-previous, scaled to ms */
        hrv_d_rmssd = hrv_rmssd - rmssd_prev;
        rmssd_prev  = hrv_rmssd;
    }

    /* ---- Calm-snapshot baseline ---- */
    if (!baseline_done && rr_count >= 8) {
    	 if (hrv_rmssd_norm > 0.01f && hrv_rmssd_norm < 0.30f) {
			baseline_count++;
			baseline_norm    = ((baseline_count - 1) * baseline_norm
								+ hrv_rmssd_norm) / (float)baseline_count;
			baseline_rmssd   = ((baseline_count - 1) * baseline_rmssd
								+ hrv_rmssd) / (float)baseline_count;
			baseline_mean_rr = ((baseline_count - 1) * baseline_mean_rr
								+ hrv_mean_rr) / (float)baseline_count;
			if (baseline_count >= BASELINE_BEATS) baseline_done = 1;
    	 }
    }

    /* ---- Frequency-domain HRV (recompute every FREQ_HRV_BEATS) ---- */
    freq_hrv_counter++;
    if (freq_hrv_counter >= FREQ_HRV_BEATS) {
        freq_hrv_counter = 0;
        compute_freq_hrv();
    }

    /* ---- Stress score: derivative + level on rmssd_norm ---- */
    float b = baseline_norm + 0.001f;
    float deriv_signal = 0.0f;
    if (hrv_hist_count >= HRV_HIST_LEN) {
        float rel_d = hrv_d_norm / b;
        deriv_signal = -rel_d * DERIV_GAIN;
    }
    float level_signal = ((baseline_norm - hrv_rmssd_norm) / b) * LEVEL_GAIN;

    float raw = DERIV_WEIGHT * deriv_signal + LEVEL_WEIGHT * level_signal;
    if (raw > 1.0f) raw = 1.0f;
    if (raw < 0.0f) raw = 0.0f;

    hrv_stress += STRESS_ALPHA * (raw * 100.0f - hrv_stress);
}

/* =========================================================
   SCREEN: ECG -- initial draw
   ========================================================= */
static void draw_ecg_screen(void)
{
    wait_vsync();

    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"HEART RATE", CENTER_MODE);

    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(0, 40, (uint8_t *)"-- BPM", CENTER_MODE);

    /* Waveform box */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawRect(WAVE_X, WAVE_Y, WAVE_W, WAVE_H);

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(WAVE_X + 1, WAVE_Y + 1, WAVE_W - 2, WAVE_H - 2);

    wave_x = WAVE_X + 1;
    prev_y = WAVE_MID;
}

static void reset_qrs_detector(void)
{
    /* Reset Pan-Tompkins threshold state */
    pt_spki = PT_SPKI_INIT;
    pt_npki = PT_NPKI_INIT;
    pt_threshold = pt_npki + PT_THRESH_SCALE * (pt_spki - pt_npki);

    /* Reset moving-window integrator */
    pt_mwi_sum = 0.0f;
    pt_mwi_idx = 0;
    memset(pt_mwi_buf, 0, sizeof(pt_mwi_buf));

    /* Reset derivative buffer */
    pt_drv_idx = 0;
    memset(pt_drv_buf, 0, sizeof(pt_drv_buf));

    /* Reset peak tracking */
    last_peak_n = 0;
    refractory = 0;

    /* Reset HR smoothing */
    hr_buf_idx = 0;
    hr_buf_count = 0;

    /* Reset RR/HRV buffer */
    rr_head = 0;
    rr_count = 0;

    /* Reset displayed HR */
    hr_display = 0.0f;
}


/* =========================================================
   SCREEN: HRV STATIC CHROME
   ========================================================= */
static void draw_hrv_static(void)
{
    wait_vsync();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"HRV ANALYSIS", CENTER_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawHLine(0, 30, 240);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, 160, (uint8_t *)"STRESS LEVEL", CENTER_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DrawHLine(0, 267, 240);
    BSP_LCD_DisplayStringAt(0, 273, (uint8_t *)"[BTN] back to ECG", CENTER_MODE);
}

/* =========================================================
   SCREEN: HRV DYNAMIC VALUES
   Repaints only the value rows.  Replaces nRMSSD/dRMSSD debug
   rows with LF / HF / LF/HF.
   ========================================================= */
static void update_hrv_values(void)
{
    char str[64];
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    /* ---- Clear value rows ---- */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(10,  38, 220, 20);
    BSP_LCD_FillRect(10,  60, 220, 20);
    BSP_LCD_FillRect(10,  80, 220, 20);
    BSP_LCD_FillRect(10, 100, 220, 20);
    BSP_LCD_FillRect(10, 120, 220, 16);
    BSP_LCD_FillRect(10, 136, 220, 16);

    /* ---- HR (instant) ---- */
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    int bpm_now = (hr_display > 0.0f) ? (int)(hr_display + 0.5f) : 0;
    snprintf(str, sizeof(str), "HR: %3d BPM  ", bpm_now);
    BSP_LCD_DisplayStringAt(10, 38, (uint8_t *)str, LEFT_MODE);

    /* ---- Time-domain HRV ---- */
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    snprintf(str, sizeof(str), "RMSSD:%6.1f ms ", (double)rmssd_disp);
    BSP_LCD_DisplayStringAt(10, 60, (uint8_t *)str, LEFT_MODE);

    snprintf(str, sizeof(str), "SDNN: %6.1f ms ", (double)sdnn_disp);
    BSP_LCD_DisplayStringAt(10, 80, (uint8_t *)str, LEFT_MODE);

    snprintf(str, sizeof(str), "MnRR: %6.1f ms ", (double)mean_rr_disp);
    BSP_LCD_DisplayStringAt(10, 100, (uint8_t *)str, LEFT_MODE);

    /* ---- Frequency-domain HRV ---- */
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_SetFont(&Font12);
    snprintf(str, sizeof(str), "LF:%6.0f  HF:%6.0f",
             (double)lf_disp, (double)hf_disp);
    BSP_LCD_DisplayStringAt(10, 120, (uint8_t *)str, LEFT_MODE);

    snprintf(str, sizeof(str), "LF/HF: %4.2f   nRMSSD: %.4f",
             (double)lf_hf_disp, (double)hrv_rmssd_norm);
    BSP_LCD_DisplayStringAt(10, 136, (uint8_t *)str, LEFT_MODE);

    /* ---- Stress bar ---- */
    int bar_val = (int)(hrv_stress + 0.5f);
    if (bar_val <   0) bar_val =   0;
    if (bar_val > 100) bar_val = 100;

    uint32_t bar_color;
    if      (bar_val < 30) bar_color = LCD_COLOR_GREEN;
    else if (bar_val < 60) bar_color = LCD_COLOR_YELLOW;
    else                   bar_color = LCD_COLOR_RED;

    int bar_fill = (bar_val * 196) / 100;

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_FillRect(22, 186, 196, 18);

    if (bar_fill > 0) {
        BSP_LCD_SetTextColor(bar_color);
        BSP_LCD_FillRect(22, 186, bar_fill, 18);
    }

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(22, 186, 196, 18);

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawVLine(22 + (30 * 196) / 100, 186, 18);
    BSP_LCD_DrawVLine(22 + (60 * 196) / 100, 186, 18);

    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(bar_color);
    snprintf(str, sizeof(str), " %3d%% ", bar_val);
    BSP_LCD_DisplayStringAt(0, 210, (uint8_t *)str, CENTER_MODE);

    /* ---- Trend label ---- */
    BSP_LCD_SetFont(&Font16);
    const char *trend_str;
    uint32_t    trend_color;

    if (!baseline_done) {
        trend_str  = (sample_n < FILTER_WARMUP_SAMPLES)
                     ? "  FILTER WARMUP  "
                     : "  CALIBRATING... ";
        trend_color = LCD_COLOR_GRAY;
    } else if (in_transition) {
        trend_str  = "   TRANSITION... ";
        trend_color = LCD_COLOR_GRAY;
    } else if (rr_count < 8) {
        trend_str  = "  RECALIBRATING  ";
        trend_color = LCD_COLOR_GRAY;
    } else if (hrv_d_rmssd > 2.0f) {
        trend_str  = " TREND: RELAXING ";
        trend_color = LCD_COLOR_GREEN;
    } else if (hrv_d_rmssd < -2.0f) {
        trend_str  = " TREND: STRESSED ";
        trend_color = LCD_COLOR_RED;
    } else {
        trend_str  = "  TREND: STABLE  ";
        trend_color = LCD_COLOR_YELLOW;
    }
    BSP_LCD_SetTextColor(trend_color);
    BSP_LCD_DisplayStringAt(0, 232, (uint8_t *)trend_str, CENTER_MODE);

    /* ---- Snapshot baseline status ---- */
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    if (!baseline_done) {
        snprintf(str, sizeof(str), " Calibrating: %2d / %2d beats  ",
                 baseline_count, BASELINE_BEATS);
    } else {
        snprintf(str, sizeof(str), "base=%.4f RMSSD=%.0f MnRR=%.0f",
                 (double)baseline_norm,
                 (double)baseline_rmssd,
                 (double)baseline_mean_rr);
    }
    BSP_LCD_DisplayStringAt(0, 252, (uint8_t *)str, CENTER_MODE);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initialises the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
//  MX_FMC_Init();
  MX_I2C3_Init();
//  MX_LTDC_Init();
//  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  */
static void MX_DMA2D_Init(void)
{
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  *         Owns the full DSP pipeline.  See header banner for chain order.
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    /* USER CODE BEGIN 5 */



    /* Boot indicator */
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
        osDelay(150);
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

    const char *banner = "ECG HRV/stress monitor -- DSP pipeline v2\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)banner, strlen(banner), 100);

    for (int i = 0; i < 5; i++)
    {
        const char *test = "UART TEST\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)test, strlen(test), 100);
        osDelay(500);
    }

    /* ---- Compute FIR coefficients (one-shot, ~30 ms) ---- */
    fir_hpf_design();
    fir_notch_design();

    /* ---- Probe accelerometer (NLMS reference) ---- */
    accel_init();
    {
        char abuf[64];
        int  n = snprintf(abuf, sizeof(abuf),
                          "Accel %s -- NLMS %s\r\n",
                          accel_present ? "detected"     : "absent",
                          accel_present ? "active"       : "bypassed");
        HAL_UART_Transmit(&huart1, (uint8_t *)abuf, n, 50);
    }

    /* ---- Initialise SDRAM and LCD ---- */
    BSP_SDRAM_Init();
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
    BSP_LCD_DisplayOn();

    draw_ecg_screen();
    reset_qrs_detector();

    /* ---- Start ADC sampling at FS Hz ---- */
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start(&hadc1);

    uint32_t btn_last_ms    = 0;
    static float last_good_rr_s = 0.0f;
    static int hrv_skip_count = 0;
    static uint32_t last_hrv_ms = 0;
    static uint32_t last_detector_reset_ms = 0;
    uint32_t bpm_last_drawn = 0;


    for (;;)
    {
        /* =============================================================
           BUTTON: PA0 (B1_Pin) -- toggle ECG / HRV display
           ============================================================= */
        if (HAL_GPIO_ReadPin(GPIOA, B1_Pin) == GPIO_PIN_SET) {
            uint32_t now = HAL_GetTick();
            if ((now - btn_last_ms) > BTN_DEBOUNCE_MS) {
                btn_last_ms  = now;
                display_mode ^= 1;
                if (display_mode == MODE_HRV) {
                    draw_hrv_static();
                } else {
                    draw_ecg_screen();

                }
            }
        }

        /* =============================================================
           ADC SAMPLE -- one sample per loop iteration, paced by TIM2
           ============================================================= */
        if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
            HAL_ADC_Start(&hadc1);
            continue;
        }
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Start(&hadc1);
        sample_n++;

        float x = ((float)raw) / 2048.0f;

        /* =============================================================
           STAGE 1: FIXED FIR FILTERING
              1a. 0.5 Hz HPF -- baseline-wander rejection
              1b.  50 Hz notch -- mains rejection
           ============================================================= */
        float x_hpf = fir_apply(x,     hpf_coeffs, hpf_delay, &hpf_idx, FIR_HPF_TAPS);
        float x_lin = fir_apply(x_hpf, ntc_coeffs, ntc_delay, &ntc_idx, FIR_NOTCH_TAPS);

        /* =============================================================
           STAGE 2: NLMS ADAPTIVE FILTER
           Read accel at decimated rate; hold value between reads.
           ============================================================= */
        if (++accel_tick >= ACCEL_DECIMATE) {
            accel_tick = 0;
            accel_read();
        }
        float x_clean = nlms_filter(x_lin, accel_ref);

        /* =============================================================
           STAGE 3: PAN-TOMPKINS QRS ENHANCEMENT
              3a. 5-pt derivative   (slope emphasis)
              3b. squaring          (positive, large-peak emphasis)
              3c. 75-sample MWI     (energy integration)
           ============================================================= */
        float drv     = pt_derivative(x_clean);
        float pt_out  = pt_square_mwi(drv);


        /* =============================================================
           DISPLAY DECIMATION (max-hold of x_clean for the waveform pane)
           ============================================================= */



        draw_skip++;
        if (draw_skip >= 4) {
            draw_skip = 0;


            // Running mean to remove DC bias from display
            static float display_dc = 0.0f;
            display_dc += 0.001f * (x_clean - display_dc);
            float draw_val = x_clean - display_dc;

            if (display_mode == MODE_ECG) {
                wait_vsync();

                int16_t y_now = WAVE_MID + 40 - (int16_t)(draw_val * (WAVE_H / 2) * 2.5f);
                if (y_now < WAVE_Y + 2)        y_now = WAVE_Y + 2;
                if (y_now > WAVE_Y + WAVE_H-2) y_now = WAVE_Y + WAVE_H - 2;

                uint16_t erase_x = wave_x + 1;
                if (erase_x >= WAVE_X + WAVE_W - 1) erase_x = WAVE_X + 1;

                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_DrawVLine(erase_x, WAVE_Y + 1, WAVE_H - 2);

                BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
                int16_t y_top = (y_now < prev_y) ? y_now : prev_y;
                int16_t y_bot = (y_now > prev_y) ? y_now : prev_y;
                BSP_LCD_DrawVLine(wave_x, y_top, y_bot - y_top + 1);

                prev_y = y_now;

                wave_x++;
                if (wave_x >= WAVE_X + WAVE_W - 1) wave_x = WAVE_X + 1;
            }
        }

        /* =============================================================
           STAGE 4: ADAPTIVE THRESHOLD QRS DETECTION
           SPKI / NPKI tracking on the MWI output.
           Two outcomes per non-refractory rising edge above threshold:
             - signal peak  -> update SPKI, register beat
             - noise  peak  -> update NPKI only
           Here we treat any cross above threshold (with refractory) as
           a beat -- the SPKI / NPKI tracking adapts the threshold
           dynamically so this is sufficient.
           ============================================================= */
        if (refractory > 0) refractory--;

        int beat_detected = 0;
        if (refractory == 0 && pt_out > pt_threshold && pt_out > PT_NPKI_INIT)
        {
            beat_detected = 1;
            pt_update_signal_peak(pt_out);
        }
        else if (pt_out > pt_threshold * 0.5f && refractory == 0)
        {
            /* Sub-threshold sustained activity -- treat as noise candidate */
            pt_update_noise_peak(pt_out);
        }

        if (beat_detected)
        {
            /*
             * First detected beat:
             * We cannot calculate BPM yet because BPM needs the time between
             * two beats. So just store this peak position.
             */
            if (last_peak_n == 0)
            {
                last_peak_n = sample_n;
                refractory  = PT_REFRACTORY_SAMPLES;

                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
            }
            else
            {
                uint32_t delta_samples = sample_n - last_peak_n;
                float delta_t = (float)delta_samples / (float)FS;

                if (delta_t < 0.33f)
                {
                    /* Likely T-wave or noise */
                    pt_update_noise_peak(pt_out);

                    /* Ignore timing — do NOT update last_peak_n */
                }
            	/*
            	 * CASE 2:
            	 * Too long = dropout / skipped input / missed beat.
            	 * Reacquire from this beat, but do not feed HRV.
            	 */
                else if (delta_t > 2.0f)
                    {
                        /* Lost lock — reacquire */
                        last_peak_n = sample_n;
                        refractory  = PT_REFRACTORY_SAMPLES;


                        hr_buf_idx = 0;
                        hr_buf_count = 0;

                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

                        char msg[96];
                        int len = snprintf(msg, sizeof(msg),
                                           "REACQUIRE dt=%4dms -- HRV skipped\r\n",
                                           (int)(delta_t * 1000.0f));
                        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 50);
                    }

            	/*
            	 * CASE 3:
            	 * Valid beat.
            	 * Use this for BPM and HRV.
            	 */
                else
                    {
                        /* ---------- BPM ---------- */
                        float hr_inst = 60.0f / delta_t;
                        float hr_avg  = hr_smoothed(hr_inst);
                        hr_display = hr_avg;

                        /* ---------- HRV quality gate ---------- */
                        int hrv_rr_ok = 1;

                        if (last_good_rr_s > 0.0f)
                        {
                            float rr_diff = fabsf(delta_t - last_good_rr_s);
                            float rr_frac = rr_diff / last_good_rr_s;

                            if (rr_diff > 0.250f || rr_frac > 0.35f)
                                hrv_rr_ok = 0;
                        }

                        if (delta_t < 0.40f || delta_t > 1.20f)
                            hrv_rr_ok = 0;

                        if (hrv_rr_ok)
                        {
                            hrv_update(delta_t, hr_avg);
                            last_good_rr_s = delta_t;
                            hrv_skip_count = 0;
                        }
                        else
                        {

                            hrv_skip_count++;

                            if (hrv_skip_count >= 3)
                            {
                                rr_head = 0;
                                rr_count = 0;
                                hrv_skip_count = 0;

                                HAL_UART_Transmit(&huart1,
                                    (uint8_t *)
                                    "HRV RESET -- bad RR sequence\r\n",
                                    31, 50);
                            }
                        }

                        /* ---------- Commit beat ---------- */
                        last_peak_n = sample_n;
                        refractory  = PT_REFRACTORY_SAMPLES;


                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

                        /* ---------- Log ---------- */
                        char msg[192];
                        int len = snprintf(msg, sizeof(msg),
                            "BEAT dt=%4dms HR=%3dbpm RMSSD=%5.1f SDNN=%5.1f "
                            "norm=%.4f base=%.4f stress=%3d mxd=%3d trn=%d\r\n",
                            (int)(delta_t * 1000.0f),
                            (int)(hr_avg + 0.5f),
                            (double)hrv_rmssd,
                            (double)hrv_sdnn,
                            (double)hrv_rmssd_norm,
                            (double)baseline_norm,
                            (int)(hrv_stress + 0.5f),
                            (int)(hrv_max_delta + 0.5f),
                            (int)in_transition);



                        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 50);

                        /* ---- ECG screen: update BPM readout ---- */
                        if (display_mode == MODE_ECG) {
                            uint32_t bpm_int = (uint32_t)(hr_avg + 0.5f);
                            if (bpm_int != bpm_last_drawn) {
                                wait_vsync();
                                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                                BSP_LCD_FillRect(0, 35, 240, 28);
                                char bpm_str[16];
                                snprintf(bpm_str, sizeof(bpm_str), "%3lu BPM", bpm_int);
                                BSP_LCD_SetFont(&Font16);
                                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                                BSP_LCD_DisplayStringAt(0, 40, (uint8_t *)bpm_str, CENTER_MODE);
                                bpm_last_drawn = bpm_int;
                            }

                        }

                    }
            }

					uint32_t now = HAL_GetTick();
					if (display_mode == MODE_HRV && (now - last_hrv_ms) >= 750)
					{
						last_hrv_ms = now;

						rmssd_disp   += 0.15f * (hrv_rmssd   - rmssd_disp);
						sdnn_disp    += 0.10f * (hrv_sdnn    - sdnn_disp);
						stress_disp  += 0.15f * (hrv_stress  - stress_disp);
						mean_rr_disp += 0.15f * (hrv_mean_rr - mean_rr_disp);
						lf_disp      += 0.20f * (hrv_lf      - lf_disp);
						hf_disp      += 0.20f * (hrv_hf      - hf_disp);
						lf_hf_disp   += 0.20f * (hrv_lf_hf   - lf_hf_disp);

						wait_vsync();
						update_hrv_values();

					}
        		}

        	    else
        	    {
        	        /*
        	         * No beat detected.
        	         * Turn LED off after short pulse time.
        	         */
        	    	pt_spki *= 0.9995f;
        	    	pt_threshold = pt_npki + PT_THRESH_SCALE * (pt_spki - pt_npki);

        	        if (refractory < (uint32_t)(PT_REFRACTORY_SAMPLES - 40))
        	        {
        	            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
        	        }
        	    }


        /* =============================================================
           NO-BEAT RECOVERY RESET
           If ECG is still running but no valid beats have been accepted
           for >3 seconds, reset QRS detector AND HRV state.
           ============================================================= */


       if (last_peak_n > 0 &&
    		   (sample_n - last_peak_n) > (uint32_t)(3 * FS))

        {
            uint32_t now_ms = HAL_GetTick();

            /* Prevent repeated resets every loop */
            if ((now_ms - last_detector_reset_ms) > 3000)
            {
                last_detector_reset_ms = now_ms;

                /* ---------- Reset Pan–Tompkins detector ---------- */
                pt_spki = PT_SPKI_INIT;
                pt_npki = PT_NPKI_INIT;
                pt_threshold = pt_npki +
                               PT_THRESH_SCALE * (pt_spki - pt_npki);

                /* Reset derivative buffer */
                pt_drv_idx = 0;
                memset(pt_drv_buf, 0, sizeof(pt_drv_buf));

                /* Reset moving window integrator */
                pt_mwi_sum = 0.0f;
                pt_mwi_idx = 0;
                memset(pt_mwi_buf, 0, sizeof(pt_mwi_buf));

                /* Reset peak timing */
                last_peak_n = 0;
                refractory  = 0;

                /* ---------- Reset HR smoothing ---------- */
                hr_buf_idx   = 0;
                hr_buf_count = 0;
                hr_display   = 0.0f;

                /* ---------- Reset HRV buffers (IMPORTANT) ---------- */
                rr_head  = 0;
                rr_count = 0;

                hrv_rmssd       = 0.0f;
                hrv_sdnn        = 0.0f;
                hrv_mean_rr    = 0.0f;
                hrv_rmssd_norm = 0.0f;
                hrv_d_rmssd    = 0.0f;
                hrv_d_norm     = 0.0f;
                hrv_max_delta  = 0.0f;

                hrv_lf       = 0.0f;
                hrv_hf       = 0.0f;
                hrv_lf_hf    = 0.0f;
                freq_hrv_counter = 0;
                rmssd_prev = 0.0f;

                last_good_rr_s  = 0.0f;
                hrv_skip_count  = 0;

                /* ---------- UI reset ---------- */
                if (display_mode == MODE_ECG)
                {
                    wait_vsync();
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_FillRect(0, 35, 240, 28);
                    BSP_LCD_SetFont(&Font16);
                    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
                    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                    BSP_LCD_DisplayStringAt(0, 40, (uint8_t *)"-- BPM", CENTER_MODE);
                }

                /* ---------- UART log ---------- */
                HAL_UART_Transmit(&huart1,
                    (uint8_t *)
                    "QRS RESET -- no beats detected (detector + HRV reset)\r\n",
                    62,
                    50);
            }
        }
    }
     /* for (;;) */

    /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
