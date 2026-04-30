/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ECG QRS detector + HRV / stress monitor
  *                   STM32F429I-Discovery
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

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
/* USER CODE BEGIN PV */

/* USER CODE END PV */

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

/* =========================================================
   VSYNC helper -- wait for the blanking period before drawing
   ========================================================= */
static inline void wait_vsync(void)
{
    while ( LTDC->CDSR & LTDC_CDSR_VSYNCS) { }
    while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS)) { }
}

/* =========================================================
   SIGNAL PROCESSING CONFIGURATION
   ========================================================= */
#define FS                   500        /* ADC sample rate (Hz)               */
#define THRESHOLD            0.015f     /* fixed threshold for R-wave trigger  */
#define REFRACTORY_SAMPLES   150        /* 300 ms blanking after each peak     */
#define HR_SMOOTH_LEN        5          /* HR averaging window (beats)         */

/* =========================================================
   LCD LAYOUT  (240 x 320, portrait)
   Waveform occupies the lower 180 px, header the upper 80 px
   ========================================================= */
#define WAVE_X               0
#define WAVE_Y               80
#define WAVE_W               240
#define WAVE_H               180
#define WAVE_MID             (WAVE_Y + WAVE_H / 2)

/* =========================================================
   DISPLAY MODE
   ========================================================= */
#define MODE_ECG             0
#define MODE_HRV             1
/* ---- HRV display-smoothing state (UI only) ---- */
    static float rmssd_disp  = 0.0f;
    static float sdnn_disp   = 0.0f;
    static float stress_disp = 0.0f;
    static float level_disp  = 0.0f;
    static float mean_rr_disp = 0.0f;
/* =========================================================
   HRV / STRESS CONFIGURATION
   ========================================================= */
/*
 * HRV-based stress architecture -- v3.
 *
 * Core metric: normalised RMSSD (rmssd_norm = RMSSD / MeanRR, dimensionless).
 *
 * Why normalise:
 *   Raw RMSSD has a mathematical dependence on mean RR -- shorter cycles
 *   constrain the maximum possible successive difference.  At 100 BPM an
 *   RMSSD of 43 ms and at 75 BPM an RMSSD of 43 ms are not physiologically
 *   equivalent.  Dividing by MeanRR removes this HR dependency.  Both
 *   derivative and level components operate on rmssd_norm exclusively.
 *   MeanRR is displayed but does not enter the stress formula.
 *
 * Why this fixes the tachycardia-with-preserved-HRV false positive:
 *   Calm baseline: RMSSD 75 ms, MeanRR 800 ms -> norm = 0.094
 *   True stress  : RMSSD 15 ms, MeanRR 600 ms -> norm = 0.025  lvl = 0.73
 *   Tachycardia, preserved HRV: RMSSD 43 ms, MeanRR 600 ms -> norm = 0.072 lvl = 0.23
 *   The system now correctly reads ~44% for true stress and ~14% for
 *   tachycardia with preserved HRV -- a distinction the raw architecture
 *   could not make.
 *
 * Rate-transition display flag:
 *   When smoothed HR changes by more than TRANSITION_BPM_THRESH across
 *   TRANSITION_BEATS beats, the trend label shows "TRANSITION" for one
 *   update cycle.  No buffer flush, no lockout -- the RR buffer runs
 *   continuously and RMSSD self-corrects as old intervals age out.
 *   The rr_count < 4 guard in hrv_update is the only protection needed.
 *
 * Components:
 *   DERIVATIVE (50%): d(rmssd_norm)/dbeat across HRV_HIST_LEN beats.
 *                     Catches transitions fast.
 *   LEVEL      (50%): (baseline_norm - rmssd_norm) / baseline_norm.
 *                     Holds score up during sustained HRV suppression.
 *
 * Tuning:
 *   DERIV_GAIN            -- raise if bar barely moves during transitions.
 *   LEVEL_GAIN            -- raise if sustained stress underreads.
 *   TRANSITION_BPM_THRESH -- raise if display flickers on normal HRV.
 *   STRESS_ALPHA          -- raise for faster response; lower for smoother.
 */
#define RR_BUF_LEN              30       /* 36 s at 100 BPM -- stable RMSSD window */
#define HRV_HIST_LEN             8       /* derivative window (beats)               */
#define BASELINE_BEATS          15       /* calm snapshot depth (post-warmup beats) */
#define FILTER_WARMUP_SAMPLES  500       /* 1 s at 500 Hz -- skip before baseline   */
#define STRESS_ALPHA           0.12f     /* EMA coeff for stress score              */
#define DERIV_GAIN              8.0f     /* derivative sensitivity (tuned for norm) */
#define LEVEL_GAIN              1.2f     /* level sensitivity (tuned for norm)      */
#define DERIV_WEIGHT            0.50f    /* blend fraction: derivative              */
#define LEVEL_WEIGHT            0.50f    /* blend fraction: level                   */
#define TRANSITION_BPM_THRESH   15       /* smoothed-BPM delta for transition label */
#define TRANSITION_BEATS         4       /* lookback window for transition detection */
#define HRV_DISPLAY_INTERVAL    5        /* refresh HRV screen every N beats        */
#define BTN_DEBOUNCE_MS        300       /* user-button debounce window (ms)        */

/* =========================================================
   FILTER COEFFICIENTS
   Fs = 500 Hz, 2nd-order Butterworth Direct Form I
   HPF fc = 5 Hz  -- removes baseline wander, P/T waves
   LPF fc = 15 Hz -- removes mains hum, muscle noise
   ========================================================= */
static const float hpf_b[3] = {  0.95654368f, -1.91308735f,  0.95654368f };
static const float hpf_a[2] = { -1.91119707f,  0.91497764f };

static const float lpf_b[3] = {  0.00782021f,  0.01564042f,  0.00782021f };
static const float lpf_a[2] = { -1.73472577f,  0.76600660f };

/* =========================================================
   FILTER STATE
   ========================================================= */
static float hpf_x1 = 0.0f, hpf_x2 = 0.0f;
static float hpf_y1 = 0.0f, hpf_y2 = 0.0f;
static float lpf_x1 = 0.0f, lpf_x2 = 0.0f;
static float lpf_y1 = 0.0f, lpf_y2 = 0.0f;

/* =========================================================
   PEAK DETECTION STATE
   ========================================================= */
static uint32_t sample_n    = 0;        /* total samples since boot            */
static uint32_t last_peak_n = 0;        /* sample index of last detected peak  */
static uint32_t refractory  = 0;        /* blanking countdown                  */

/* =========================================================
   HR SMOOTHING  (running average of last HR_SMOOTH_LEN beats)
   ========================================================= */
static float hr_buf[HR_SMOOTH_LEN] = { 0 };
static int   hr_buf_idx             = 0;
static int   hr_buf_count           = 0;

/* =========================================================
   HRV STATE -- all fixed-size, no dynamic allocation
   ========================================================= */

/* RR interval ring buffer (seconds) */
static float rr_buf[RR_BUF_LEN]  = { 0 };
static int   rr_head              = 0;    /* next-write index, bounded [0, RR_BUF_LEN)  */
static int   rr_count             = 0;    /* number of valid entries, [0, RR_BUF_LEN]   */

/* Per-beat normalised RMSSD history for derivative computation.
 * Stores rmssd_norm = RMSSD_ms / MeanRR_ms (dimensionless).
 * MeanRR is excluded from the stress formula -- display only. */
static float norm_hist[HRV_HIST_LEN] = { 0 };
static int   hrv_hist_head            = 0;
static int   hrv_hist_count           = 0;

/* Current HRV metrics (ms) -- updated each valid beat */
static float hrv_rmssd     = 0.0f;
static float hrv_sdnn      = 0.0f;
static float hrv_mean_rr   = 0.0f;
static float hrv_rmssd_norm= 0.0f;   /* dimensionless: RMSSD / MeanRR            */

/* Derivatives -- norm units per beat (used in stress formula and display) */
static float hrv_d_rmssd   = 0.0f;   /* ms/beat -- display only                  */
static float hrv_d_norm    = 0.0f;   /* norm/beat -- stress formula              */
static float hrv_max_delta = 0.0f;   /* max successive RR diff ms -- artefact flag */

/* Stress score [0, 100] */
static float hrv_stress    = 0.0f;

/* Smoothed BPM -- shared between beat loop and update_hrv_values */
static float hr_display    = 0.0f;

/*
 * Calm snapshot baseline -- accumulated over BASELINE_BEATS clean beats
 * (post filter-warmup).  Stores normalised RMSSD only.
 * Raw RMSSD and MeanRR are kept separately for display.
 */
static float baseline_norm     = 0.094f;  /* sane default (~75ms / 800ms)         */
static float baseline_rmssd    = 40.0f;   /* display only                         */
static float baseline_mean_rr  = 800.0f;  /* display only                         */
static int   baseline_count    = 0;
static int   baseline_done     = 0;

/*
 * Rate-transition display flag.
 * hr_trans_buf: sliding window of recent smoothed BPM values.
 * in_transition is set to 1 for one display cycle when the smoothed HR
 * range across TRANSITION_BEATS beats exceeds TRANSITION_BPM_THRESH.
 * No flush, no lockout -- purely a display label.
 */
static float hr_trans_buf[TRANSITION_BEATS] = { 0 };
static int   hr_trans_head  = 0;
static int   hr_trans_count = 0;
static int   in_transition  = 0;   /* 1 = show TRANSITION label this cycle */


/* =========================================================
   DISPLAY STATE  (module-level so draw helpers can access)
   ========================================================= */
static int      display_mode     = MODE_ECG;
static uint16_t wave_x           = WAVE_X + 1;
static int16_t  prev_y           = WAVE_MID;
static int      draw_skip        = 0;
static float    bp_max_in_window = -1.0f;
static float    bp_min_in_window =  1.0f;

/* =========================================================
   STAGE 1A: HIGH-PASS FILTER  (Direct Form I)
   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
   ========================================================= */
static float hpf_filter(float x)
{
    float y = hpf_b[0]*x + hpf_b[1]*hpf_x1 + hpf_b[2]*hpf_x2
                         - hpf_a[0]*hpf_y1  - hpf_a[1]*hpf_y2;
    hpf_x2 = hpf_x1; hpf_x1 = x;
    hpf_y2 = hpf_y1; hpf_y1 = y;
    return y;
}

/* =========================================================
   STAGE 1B: LOW-PASS FILTER  (Direct Form I)
   ========================================================= */
static float lpf_filter(float x)
{
    float y = lpf_b[0]*x + lpf_b[1]*lpf_x1 + lpf_b[2]*lpf_x2
                         - lpf_a[0]*lpf_y1  - lpf_a[1]*lpf_y2;
    lpf_x2 = lpf_x1; lpf_x1 = x;
    lpf_y2 = lpf_y1; lpf_y1 = y;
    return y;
}

/* =========================================================
   HR SMOOTHING -- running mean of last HR_SMOOTH_LEN instants
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
   HRV BUFFER HELPERS
   All buffers are power-of-two sized for safe modulo.
   ========================================================= */

/* Push one RR interval (seconds) into the ring buffer. */
static void rr_push(float rr_s)
{
    rr_buf[rr_head] = rr_s;
    rr_head = (rr_head + 1) % RR_BUF_LEN;
    if (rr_count < RR_BUF_LEN) rr_count++;
}

/*
 * Iterate the rr_count most-recent entries oldest-first.
 * start = oldest index.
 * When rr_count == RR_BUF_LEN, oldest == rr_head (next-overwrite slot).
 * When rr_count <  RR_BUF_LEN, oldest == (rr_head - rr_count).
 * The +RR_BUF_LEN guard makes negative modulo safe in C.
 */
static int rr_oldest_idx(void)
{
    return ((rr_head - rr_count) % RR_BUF_LEN + RR_BUF_LEN) % RR_BUF_LEN;
}

/* Mean RR (returns seconds). */
static float rr_mean_s(void)
{
    if (rr_count == 0) return 0.0f;
    int start = rr_oldest_idx();
    float sum = 0.0f;
    for (int i = 0; i < rr_count; i++)
        sum += rr_buf[(start + i) % RR_BUF_LEN];
    return sum / (float)rr_count;
}

/*
 * RMSSD: root-mean-square of successive RR differences.
 * Returns ms.
 */
static float rr_rmssd_ms(void)
{
    if (rr_count < 2) return 0.0f;
    int start = rr_oldest_idx();
    float sum_sq = 0.0f;
    for (int i = 0; i < rr_count - 1; i++) {
        float d = (rr_buf[(start + i + 1) % RR_BUF_LEN]
                 - rr_buf[(start + i)     % RR_BUF_LEN]) * 1000.0f;
        sum_sq += d * d;
    }
    return sqrtf(sum_sq / (float)(rr_count - 1));
}

/*
 * SDNN: standard deviation of all NN intervals in the buffer.
 * Returns ms.
 */
static float rr_sdnn_ms(void)
{
    if (rr_count < 2) return 0.0f;
    float mean = rr_mean_s();
    int start = rr_oldest_idx();
    float var = 0.0f;
    for (int i = 0; i < rr_count; i++) {
        float d = (rr_buf[(start + i) % RR_BUF_LEN] - mean) * 1000.0f;
        var += d * d;
    }
    return sqrtf(var / (float)rr_count);
}

/*
 * Max absolute successive RR difference in the current buffer (ms).
 * Used as an outlier/artefact diagnostic in the UART log.
 * A value above ~200 ms at stable HR indicates a missed beat or
 * detection glitch rather than genuine HRV.
 */
static float rr_max_delta_ms(void)
{
    if (rr_count < 2) return 0.0f;
    int start = rr_oldest_idx();
    float max_d = 0.0f;
    for (int i = 0; i < rr_count - 1; i++) {
        float d = fabsf(rr_buf[(start + i + 1) % RR_BUF_LEN]
                      - rr_buf[(start + i)     % RR_BUF_LEN]) * 1000.0f;
        if (d > max_d) max_d = d;
    }
    return max_d;
}

/* =========================================================
   HRV UPDATE  -- call once per valid detected beat
   rr_s  : RR interval in seconds
   hr_bpm: SMOOTHED BPM (hr_avg) -- must NOT be instantaneous.
            Instantaneous BPM oscillates ±10 BPM with normal HRV and
            would trigger the transition detector on every beat.
   ========================================================= */
static void hrv_update(float rr_s, float hr_bpm)
{
    /* ---- Gate: IIR filter warmup ---- */
    if (sample_n < FILTER_WARMUP_SAMPLES) return;

    /* ---- Rate-transition display flag ----
     *
     * Slide the smoothed BPM window.  If the range across TRANSITION_BEATS
     * exceeds TRANSITION_BPM_THRESH, set in_transition for one display
     * cycle so the LCD trend label shows "TRANSITION".
     * The RR buffer is NOT flushed -- it self-corrects as intervals age out.
     * The rr_count < 4 guard below is the only artefact protection needed.
     */
    in_transition = 0;
    hr_trans_buf[hr_trans_head] = hr_bpm;
    hr_trans_head = (hr_trans_head + 1) % TRANSITION_BEATS;
    if (hr_trans_count < TRANSITION_BEATS) hr_trans_count++;

    if (hr_trans_count == TRANSITION_BEATS) {
        float hr_min = hr_trans_buf[0], hr_max = hr_trans_buf[0];
        for (int i = 1; i < TRANSITION_BEATS; i++) {
            if (hr_trans_buf[i] < hr_min) hr_min = hr_trans_buf[i];
            if (hr_trans_buf[i] > hr_max) hr_max = hr_trans_buf[i];
        }
        if ((hr_max - hr_min) > (float)TRANSITION_BPM_THRESH) {
            in_transition = 1;
        }
    }

    rr_push(rr_s);
    if (rr_count < 4) return;

    /* ---- Compute raw metrics ---- */
    hrv_mean_rr    = rr_mean_s()   * 1000.0f;
    hrv_rmssd      = rr_rmssd_ms();
    hrv_sdnn       = rr_sdnn_ms();
    hrv_max_delta  = rr_max_delta_ms();
    hrv_rmssd_norm = hrv_rmssd / (hrv_mean_rr + 1.0f);   /* dimensionless */

    /* ---- Phase 1: calm snapshot baseline ----
     *
     * Welford mean of rmssd_norm over BASELINE_BEATS clean post-warmup beats.
     * Raw RMSSD and MeanRR are also tracked for display only.
     * The baseline_norm is the sole reference for both stress components --
     * MeanRR does not enter the stress formula at all.
     */
    if (!baseline_done) {
        baseline_count++;
        baseline_norm    += (hrv_rmssd_norm - baseline_norm)    / (float)baseline_count;
        baseline_rmssd   += (hrv_rmssd      - baseline_rmssd)   / (float)baseline_count;
        baseline_mean_rr += (hrv_mean_rr    - baseline_mean_rr) / (float)baseline_count;
        if (baseline_count >= BASELINE_BEATS) baseline_done = 1;
        return;
    }

    /* ---- Phase 2: push rmssd_norm into history ring ---- */
    norm_hist[hrv_hist_head] = hrv_rmssd_norm;
    hrv_hist_head = (hrv_hist_head + 1) % HRV_HIST_LEN;
    if (hrv_hist_count < HRV_HIST_LEN) hrv_hist_count++;

    if (hrv_hist_count < HRV_HIST_LEN) return;

    /* ---- Phase 3: differentiate rmssd_norm ----
     *
     * slope = (newest - oldest) / (N-1)  [norm / beat]
     * hrv_d_rmssd kept in ms/beat for display only.
     */
    int oldest = hrv_hist_head;
    int newest = (hrv_hist_head - 1 + HRV_HIST_LEN) % HRV_HIST_LEN;

    hrv_d_norm  = (norm_hist[newest] - norm_hist[oldest])
                  / (float)(HRV_HIST_LEN - 1);

    /* Keep a ms/beat derivative for the display trend label */
    hrv_d_rmssd = hrv_d_norm * (hrv_mean_rr + 1.0f);

    /* ---- Phase 4: hybrid stress score -- normalised RMSSD only ----
     *
     * DERIVATIVE component:
     *   rel_d = d(norm)/beat / baseline_norm  (dimensionless rate of change)
     *   Positive when norm is falling (HRV being suppressed).
     *
     * LEVEL component:
     *   Fractional drop of norm below calm baseline.
     *   Zero at rest (norm == baseline_norm).
     *   Rises monotonically as HRV is suppressed.
     *   MeanRR intentionally excluded -- this is a pure HRV metric.
     *
     * Both clamped to [0,1] after blending, scaled to [0,100].
     */
    float b = baseline_norm + 0.001f;   /* guard div/0 */

    float rel_d      = hrv_d_norm / b;
    float deriv_signal = -rel_d * DERIV_GAIN;   /* +ve = norm falling = stressed */

    float level_signal = ((baseline_norm - hrv_rmssd_norm) / b) * LEVEL_GAIN;

    float raw = DERIV_WEIGHT * deriv_signal + LEVEL_WEIGHT * level_signal;
    if (raw > 1.0f) raw = 1.0f;
    if (raw < 0.0f) raw = 0.0f;

    hrv_stress += STRESS_ALPHA * (raw * 100.0f - hrv_stress);
}

/* =========================================================
   SCREEN: ECG  -- initial draw (call on boot or mode switch)
   ========================================================= */
static void draw_ecg_screen(void)
{
    wait_vsync();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    /* Title */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"HEART RATE", CENTER_MODE);

    /* Waveform border */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawRect(WAVE_X, WAVE_Y, WAVE_W, WAVE_H);

    /* Clear waveform area */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(WAVE_X + 1, WAVE_Y + 1, WAVE_W - 2, WAVE_H - 2);

    /* Reset waveform cursor so it starts cleanly on return */
    wave_x = WAVE_X + 1;
    prev_y = WAVE_MID;
}

/* =========================================================
   SCREEN: HRV STATIC CHROME
   Call ONCE when entering HRV mode.  Draws only the elements
   that never change: title, dividers, field labels, footer.
   Dynamic values are painted by update_hrv_values() below.
   ========================================================= */
static void draw_hrv_static(void)
{
    wait_vsync();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    /* Title */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"HRV ANALYSIS", CENTER_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawHLine(0, 30, 240);

    /* Fixed "STRESS LEVEL" heading above the bar */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, 160, (uint8_t *)"STRESS LEVEL", CENTER_MODE);

    /* Footer divider + hint */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DrawHLine(0, 267, 240);
    BSP_LCD_DisplayStringAt(0, 273, (uint8_t *)"[BTN] back to ECG", CENTER_MODE);
}

/* =========================================================
   SCREEN: HRV DYNAMIC VALUES
   Call every HRV_DISPLAY_INTERVAL beats (or immediately on mode
   switch after draw_hrv_static).  Repaints ONLY the value fields
   in-place -- no full-screen clear, no visible flash.
   Fixed-width format strings ensure old characters are fully
   overwritten by the new string's black-filled character cells.
   ========================================================= */
static void update_hrv_values(void)
{

	 char str[48];

	    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	    /* ---- Clear value rows ONLY ---- */
	    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	    BSP_LCD_FillRect(10, 38, 220, 20);
	    BSP_LCD_FillRect(10, 60, 220, 20);
	    BSP_LCD_FillRect(10, 80, 220, 20);
	    BSP_LCD_FillRect(10,100, 220, 20);

	    /* ---- Row: instant HR ---- */
	    BSP_LCD_SetFont(&Font16);
	    BSP_LCD_SetTextColor(LCD_COLOR_RED);
	    int bpm_now = (hr_display > 0.0f) ? (int)(hr_display + 0.5f) : 0;
	    snprintf(str, sizeof(str), "HR: %3d BPM  ", bpm_now);
	    BSP_LCD_DisplayStringAt(10, 38, (uint8_t *)str, LEFT_MODE);

	    /* ---- Rows: HRV metrics ---- */
	    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);

	    snprintf(str, sizeof(str), "RMSSD:%6.1f ms ", rmssd_disp);
	    BSP_LCD_DisplayStringAt(10, 60, (uint8_t *)str, LEFT_MODE);

	    snprintf(str, sizeof(str), "SDNN: %6.1f ms ", sdnn_disp);
	    BSP_LCD_DisplayStringAt(10, 80, (uint8_t *)str, LEFT_MODE);

	    snprintf(str, sizeof(str), "MnRR: %6.1f ms ", mean_rr_disp);
	    BSP_LCD_DisplayStringAt(10,100, (uint8_t *)str, LEFT_MODE);


    /* ---- Rows: normalised RMSSD + derivative ----
     * nRMSSD replaces dMnRR -- MeanRR is display-only, not in stress formula.
     * Trend threshold uses ms/beat (hrv_d_rmssd) for intuitive reading.
     */
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_SetFont(&Font12);
    snprintf(str, sizeof(str), "nRMSSD: %.4f (base %.4f)",
             (double)hrv_rmssd_norm, (double)baseline_norm);
    BSP_LCD_DisplayStringAt(10, 122, (uint8_t *)str, LEFT_MODE);

    snprintf(str, sizeof(str), "dRMSSD: %+6.2f ms/beat  ", (double)hrv_d_rmssd);
    BSP_LCD_DisplayStringAt(10, 136, (uint8_t *)str, LEFT_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawHLine(0, 153, 240);

    /* ---- Stress bar ---- */
    int bar_val = (int)(hrv_stress + 0.5f);
    if (bar_val <   0) bar_val =   0;
    if (bar_val > 100) bar_val = 100;

    /* Colour bands for a 0-100 unipolar stress scale.
     * 0-30  = calm (green)
     * 30-60 = elevated (yellow)
     * 60+   = stressed (red)
     */
    uint32_t bar_color;
    if      (bar_val < 30) bar_color = LCD_COLOR_GREEN;
    else if (bar_val < 60) bar_color = LCD_COLOR_YELLOW;
    else                   bar_color = LCD_COLOR_RED;

    /* Total usable bar width = 196 px (x=22 to x=218) */
    int bar_fill = (bar_val * 196) / 100;

    /* 1) Dark-gray empty track */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_FillRect(22, 186, 196, 18);

    /* 2) Coloured fill on top */
    if (bar_fill > 0) {
        BSP_LCD_SetTextColor(bar_color);
        BSP_LCD_FillRect(22, 186, bar_fill, 18);
    }

    /* 3) White border drawn last so it's never obscured */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(22, 186, 196, 18);

    /* 4) Segmentation ticks at 40% and 70% thresholds */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawVLine(22 + (30 * 196) / 100, 186, 18);  /* green/yellow boundary */
    BSP_LCD_DrawVLine(22 + (60 * 196) / 100, 186, 18);  /* yellow/red boundary   */

    /* ---- Percentage label ---- */
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(bar_color);
    snprintf(str, sizeof(str), " %3d%% ", bar_val);   /* padding erases old digit count */
    BSP_LCD_DisplayStringAt(0, 210, (uint8_t *)str, CENTER_MODE);

    /* ---- Trend label ----
     * Reports the DERIVATIVE direction (what is changing right now).
     * The bar height captures the level; this label captures direction.
     *
     * 2.0 ms/beat boundary: at 70 BPM this is ~2 ms/s, a physiologically
     * meaningful rate of change.
     */
    BSP_LCD_SetFont(&Font16);
    const char *trend_str;
    uint32_t    trend_color;

    if (!baseline_done) {
        if (sample_n < FILTER_WARMUP_SAMPLES) {
            trend_str  = "  FILTER WARMUP  ";
        } else {
            trend_str  = "  CALIBRATING... ";
        }
        trend_color = LCD_COLOR_GRAY;
    } else if (in_transition) {
        /* Smoothed HR changing rapidly -- label only, output continues */
        trend_str  = "   TRANSITION... ";
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

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initialises the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
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
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_LTDC_Init(void)
//{
//
//  /* USER CODE BEGIN LTDC_Init 0 */
//
//  /* USER CODE END LTDC_Init 0 */
//
//  LTDC_LayerCfgTypeDef pLayerCfg = {0};
//
//  /* USER CODE BEGIN LTDC_Init 1 */
//
//  /* USER CODE END LTDC_Init 1 */
//  hltdc.Instance = LTDC;
//  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
//  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
//  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
//  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
//  hltdc.Init.HorizontalSync = 9;
//  hltdc.Init.VerticalSync = 1;
//  hltdc.Init.AccumulatedHBP = 29;
//  hltdc.Init.AccumulatedVBP = 3;
//  hltdc.Init.AccumulatedActiveW = 269;
//  hltdc.Init.AccumulatedActiveH = 323;
//  hltdc.Init.TotalWidth = 279;
//  hltdc.Init.TotalHeigh = 327;
//  hltdc.Init.Backcolor.Blue = 0;
//  hltdc.Init.Backcolor.Green = 0;
//  hltdc.Init.Backcolor.Red = 0;
//  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pLayerCfg.WindowX0 = 0;
//  pLayerCfg.WindowX1 = 240;
//  pLayerCfg.WindowY0 = 0;
//  pLayerCfg.WindowY1 = 320;
//  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
//  pLayerCfg.Alpha = 255;
//  pLayerCfg.Alpha0 = 0;
//  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
//  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
//  pLayerCfg.FBStartAdress = 0xD0000000;
//  pLayerCfg.ImageWidth = 240;
//  pLayerCfg.ImageHeight = 320;
//  pLayerCfg.Backcolor.Blue = 0;
//  pLayerCfg.Backcolor.Green = 0;
//  pLayerCfg.Backcolor.Red = 0;
//  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN LTDC_Init 2 */
//
//  /* USER CODE END LTDC_Init 2 */
//
//}

//static void MX_SPI5_Init(void)
//{
//
//  /* USER CODE BEGIN SPI5_Init 0 */
//
//  /* USER CODE END SPI5_Init 0 */
//
//  /* USER CODE BEGIN SPI5_Init 1 */
//
//  /* USER CODE END SPI5_Init 1 */
//  hspi5.Instance = SPI5;
//  hspi5.Init.Mode = SPI_MODE_MASTER;
//  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi5.Init.NSS = SPI_NSS_SOFT;
//  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi5.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI5_Init 2 */
//
//  /* USER CODE END SPI5_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
//static void MX_FMC_Init(void)
//{
//
//  /* USER CODE BEGIN FMC_Init 0 */
//
//  /* USER CODE END FMC_Init 0 */
//
//  FMC_SDRAM_TimingTypeDef SdramTiming = {0};
//
//  /* USER CODE BEGIN FMC_Init 1 */
//
//  /* USER CODE END FMC_Init 1 */
//
//  /** Perform the SDRAM1 memory initialization sequence
//  */
//  hsdram1.Instance = FMC_SDRAM_DEVICE;
//  /* hsdram1.Init */
//  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
//  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
//  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
//  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
//  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
//  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
//  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
//  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
//  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
//  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
//  /* SdramTiming */
//  SdramTiming.LoadToActiveDelay = 2;
//  SdramTiming.ExitSelfRefreshDelay = 7;
//  SdramTiming.SelfRefreshTime = 4;
//  SdramTiming.RowCycleDelay = 7;
//  SdramTiming.WriteRecoveryTime = 3;
//  SdramTiming.RPDelay = 2;
//  SdramTiming.RCDDelay = 2;
//
//  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
//  {
//    Error_Handler( );
//  }
//
//  /* USER CODE BEGIN FMC_Init 2 */
//
//  /* USER CODE END FMC_Init 2 */
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
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

    const char *banner = "ECG QRS detector + HRV/Stress monitor\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)banner, strlen(banner), 100);

    /* ===== Initialise SDRAM and LCD via BSP ===== */
    BSP_SDRAM_Init();
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
    BSP_LCD_DisplayOn();

    draw_ecg_screen();

    /* ===== Start ADC sampling ===== */
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start(&hadc1);

    uint32_t bpm_last_drawn = 0;
    uint32_t btn_last_ms    = 0;



    for (;;)
    {
        /* =============================================================
           BUTTON: PA0 (B1_Pin, blue user button, active-high)
           GPIO_MODE_EVT_RISING still allows IDR reads -- polling is safe.
           300 ms debounce prevents bounce from registering as two presses.
           ============================================================= */
        if (HAL_GPIO_ReadPin(GPIOA, B1_Pin) == GPIO_PIN_SET) {
            uint32_t now = HAL_GetTick();
            if ((now - btn_last_ms) > BTN_DEBOUNCE_MS) {
                btn_last_ms = now;

                display_mode ^= 1;   /* toggle between MODE_ECG and MODE_HRV */

                if (display_mode == MODE_HRV) {
                	draw_hrv_static();
                } else {
                    draw_ecg_screen();
                    bpm_last_drawn = 0;   /* force BPM redraw on first beat back */
                }
            }
        }

        /* =============================================================
           ADC SAMPLE
           ============================================================= */
        if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
            HAL_ADC_Start(&hadc1);
            continue;
        }
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Start(&hadc1);

        sample_n++;

        float x    = (float)raw / 4095.0f;
        float x_hp = hpf_filter(x);
        float x_bp = lpf_filter(x_hp);

        /* =============================================================
           MAX-HOLD DECIMATION
           Always accumulate, regardless of display mode, so the window
           resets cleanly and there are no stale values on mode return.
           ============================================================= */
        if (x_bp > bp_max_in_window) bp_max_in_window = x_bp;
        if (x_bp < bp_min_in_window) bp_min_in_window = x_bp;

        draw_skip++;
        if (draw_skip >= 8) {
            draw_skip = 0;

            float draw_val = (fabsf(bp_max_in_window) > fabsf(bp_min_in_window))
                             ? bp_max_in_window : bp_min_in_window;
            bp_max_in_window = -1.0f;
            bp_min_in_window =  1.0f;

            /* Draw waveform only when ECG screen is active */
            if (display_mode == MODE_ECG) {

                /* Wait for vertical sync ONCE per draw */
                wait_vsync();

                /* Convert signal to screen Y */
                int16_t y_now = WAVE_MID
                              - (int16_t)(draw_val * (WAVE_H / 2) * 20.0f);  // reduced gain

                if (y_now < WAVE_Y + 2)        y_now = WAVE_Y + 2;
                if (y_now > WAVE_Y + WAVE_H-2) y_now = WAVE_Y + WAVE_H - 2;

                /* -------------------------------------------------
                 * Erase ONLY the column ahead (write-head gap)
                 * ------------------------------------------------- */
                uint16_t erase_x = wave_x + 1;
                if (erase_x >= WAVE_X + WAVE_W - 1)
                    erase_x = WAVE_X + 1;

                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_DrawVLine(erase_x, WAVE_Y + 1, WAVE_H - 2);

                /* -------------------------------------------------
                 * Draw continuous line from previous Y to current Y
                 * (NO pixel-only special case)
                 * ------------------------------------------------- */
                BSP_LCD_SetTextColor(LCD_COLOR_GREEN);

                int16_t y_top = (y_now < prev_y) ? y_now : prev_y;
                int16_t y_bot = (y_now > prev_y) ? y_now : prev_y;

                BSP_LCD_DrawVLine(wave_x, y_top, y_bot - y_top + 1);

                prev_y = y_now;

                /* -------------------------------------------------
                 * Advance write position
                 * ------------------------------------------------- */
                wave_x++;
                if (wave_x >= WAVE_X + WAVE_W - 1)
                    wave_x = WAVE_X + 1;
            }
        }

        /* =============================================================
           QRS DETECTION
           ============================================================= */
        if (refractory > 0) refractory--;

        if (refractory == 0 && x_bp > THRESHOLD)
        {
            if (last_peak_n > 0)
            {
                uint32_t delta_samples = sample_n - last_peak_n;
                float    delta_t       = (float)delta_samples / (float)FS;

                /* Sanity-check: 0.3 s (200 BPM) to 1.2 s (50 BPM) */
                if (delta_t >= 0.3f && delta_t <= 1.2f)
                {
                    float hr_inst = 60.0f / delta_t;
                    float hr_avg  = hr_smoothed(hr_inst);
                    hr_display    = hr_avg;   /* share with update_hrv_values */

                    /* Update HRV metrics and stress score */
                    hrv_update(delta_t, hr_avg);   /* smoothed -- see hrv_update comment */

                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

                    /* UART log
                     * norm : rmssd_norm = RMSSD / MeanRR (dimensionless)
                     * lvl  : level component pre-blend [0-100]
                     * trn  : 1 if smoothed HR is transitioning rapidly
                     */
                    char msg[128];
                    float _lvl = ((baseline_norm - hrv_rmssd_norm)
                                  / (baseline_norm + 0.001f)) * LEVEL_GAIN;
                    if (_lvl < 0.0f) _lvl = 0.0f;
                    if (_lvl > 1.0f) _lvl = 1.0f;
                    int len = snprintf(msg, sizeof(msg),
                        "BEAT dt=%4dms HR=%3dbpm RMSSD=%5.1f SDNN=%5.1f"
                        " norm=%.4f base=%.4f stress=%3d lvl=%2d mxd=%3d trn=%d\r\n",
                        (int)(delta_t * 1000.0f),
                        (int)(hr_avg + 0.5f),
                        (double)hrv_rmssd,
                        (double)hrv_sdnn,
                        (double)hrv_rmssd_norm,
                        (double)baseline_norm,
                        (int)(hrv_stress + 0.5f),
                        (int)(_lvl * 100.0f + 0.5f),
                        (int)(hrv_max_delta + 0.5f),
                        in_transition);
                    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 50);

                    /* ---- ECG screen: update BPM readout ---- */
                    if (display_mode == MODE_ECG) {
                        uint32_t bpm_int = (uint32_t)(hr_avg + 0.5f);
                        if (bpm_int != bpm_last_drawn) {
                            wait_vsync();
                            char bpm_str[16];
                            snprintf(bpm_str, sizeof(bpm_str), "%3lu BPM", bpm_int);
                            BSP_LCD_SetTextColor(LCD_COLOR_RED);
                            BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                            BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)bpm_str, CENTER_MODE);
                            bpm_last_drawn = bpm_int;
                        }
                    }
                    /* ---- HRV screen: throttled partial update ---- */
                    static uint32_t last_hrv_ms = 0;
                    uint32_t now = HAL_GetTick();

                    if (display_mode == MODE_HRV && (now - last_hrv_ms) >= 750) {
                        last_hrv_ms = now;

                        /* ---- display-only smoothing ---- */
                        rmssd_disp  += 0.15f * (hrv_rmssd  - rmssd_disp);
                        sdnn_disp   += 0.10f * (hrv_sdnn   - sdnn_disp);
                        stress_disp += 0.15f * (hrv_stress - stress_disp);
                        level_disp  += 0.20f * ((float)hrv_stress - level_disp);
                        mean_rr_disp += 0.15f * (hrv_mean_rr - mean_rr_disp);

                        wait_vsync();
                        update_hrv_values();
                    }

                }
            }

            last_peak_n = sample_n;
            refractory  = REFRACTORY_SAMPLES;
        }
        else if (refractory < REFRACTORY_SAMPLES - 40)
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
        }

        /* =============================================================
           WATCHDOG: clear BPM/HRV if no beats for 3 seconds
           ============================================================= */
        if (last_peak_n > 0 && (sample_n - last_peak_n) > (uint32_t)(3 * FS))
        {
            if (bpm_last_drawn != 0 && display_mode == MODE_ECG) {
                wait_vsync();
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_FillRect(50, 35, 140, 30);
                BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"  -- BPM", CENTER_MODE);
                bpm_last_drawn = 0;

                /* Flush HR averaging buffer so stale values don't persist */
                hr_buf_idx   = 0;
                hr_buf_count = 0;
            }
        }

    } /* for (;;) */

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

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
