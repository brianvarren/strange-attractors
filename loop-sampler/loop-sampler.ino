#include <Arduino.h>
#include <SPI.h>
#include <cmath>
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/interp.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <U8g2lib.h>
//#include "VideoLight9pt7b.h"
#include "samples.h"
#include "sampledata.h"

#define TWO32 4294967296.0 // 2^32

// Display stuff (SH1122 256x64 SPI OLED)
#define SCREEN_WIDTH            256
#define SCREEN_HEIGHT           64
#define OLED_PIN_MOSI           7   // SDA
#define OLED_PIN_SCK            6   // SCL
#define OLED_PIN_CS             5
#define OLED_PIN_RST            12
#define OLED_PIN_DC             13

// Peaks data: Make sure you have 256 samples per waveform!
// If not, see the note at the end for an interpolation function.

#include "peaks.h"

// Initialize U8g2 for SH1122 SPI
U8G2_SH1122_256X64_F_4W_HW_SPI u8g2(
    U8G2_R0,
    OLED_PIN_CS,
    OLED_PIN_DC,
    OLED_PIN_RST
);

// Front panel stuff
#define PREV_NEXT_SCROLL_TIMEOUT 1000
#define SWITCH_DELAY_MILLIS     100
#define PIN_SW_PREV             15
#define PIN_SW_NEXT             14
#define PIN_TRIG_BTN            0
#define PIN_TRIG_JACK           1
#define ADC_CHAN_KNOB_PITCH     0
#define ADC_CHAN_CV_PITCH       1
#define ADC_CHAN_CV_SEL         2

bool next_switch_held = false;
bool prev_switch_held = false;
unsigned long next_switch_held_start_time = 0;
unsigned long prev_switch_held_start_time = 0;
unsigned long switch_last_checked_time = 0;

const unsigned long debounce_millis = 150;
unsigned long sw_prev_next_millis = to_ms_since_boot(get_absolute_time());
unsigned long trig_btn_millis = to_ms_since_boot(get_absolute_time());

// PWM output stuff
#define PIN_PWM_OUT             6
#define PWM_RESOLUTION          4096
#define PWM_CENTER              (PWM_RESOLUTION / 2)
#define AUDIO_BLOCK_SIZE        256

int slice_num = 0;
int dma_chan_a, dma_chan_b;

// Ping-pong buffers
volatile uint16_t buf_a[AUDIO_BLOCK_SIZE] __attribute__((aligned(AUDIO_BLOCK_SIZE * sizeof(uint16_t))));
volatile uint16_t buf_b[AUDIO_BLOCK_SIZE] __attribute__((aligned(AUDIO_BLOCK_SIZE * sizeof(uint16_t))));
static const uint size_bits = 9;
static_assert((1<<size_bits) == AUDIO_BLOCK_SIZE * sizeof(uint16_t));
static volatile uint16_t* out_buf_ptr;

volatile int dma_transfer_complete_flag = 0;

volatile int out = 0;

// Sample playback stuff
#define BASE_FREQ_OFFSET 0.02
#define BASE_FREQ_MULTIPLIER 6

volatile double freq = 0.1;
volatile double inc = 0;
volatile double phase_0 = 0.0;
volatile double phase_x = 0.0;

volatile double index_0 = 0;
volatile double index_1 = 0;
volatile double index_x = 0;
volatile double index_y = 0;

int audio_rate = 0;
volatile int table_size = 0;
double one_div_SR = 0;

int base_sample = 0;
volatile int cv_offset = 0;
volatile int adjusted_sample = 0;
int trigger_counter = 0;
bool trigger_received = false;

volatile int retrigger_flag = 0;
volatile int fade_out_flag = 0;
volatile bool dmaEnabled = true;

void onTriggerButton() {
    if ((to_ms_since_boot(get_absolute_time()) - trig_btn_millis) > debounce_millis) {
        retrigger_flag = 1;
        trig_btn_millis = to_ms_since_boot(get_absolute_time());
    }
}

uint16_t fastAnalogRead(uint adc_chan) {
    adc_select_input(adc_chan);
    return adc_read();
}

void DMAChanATransCpltCallback() {
    dma_hw->ints0 = 1u << dma_chan_a;
    out_buf_ptr = &buf_a[0];
    dma_transfer_complete_flag = 1;
}

void DMAChanBTransCpltCallback() {
    dma_hw->ints1 = 1u << dma_chan_b;
    out_buf_ptr = &buf_b[0];
    dma_transfer_complete_flag = 1;
}

double makeFreq(){
    double pitch_knob_result = 1 - (fastAnalogRead(ADC_CHAN_KNOB_PITCH) / 4096.0);
    double pitch_cv_result = (fastAnalogRead(ADC_CHAN_CV_PITCH) / 4096.0);
    return BASE_FREQ_OFFSET + (BASE_FREQ_MULTIPLIER * pow(pitch_knob_result, 3)) + (pow(pitch_cv_result, 2));
}

void setupInterpolator() {
    interp_config cfg = interp_default_config();
    interp_config_set_blend(&cfg, true);
    interp_set_config(interp0, 0, &cfg);
    cfg = interp_default_config();
    interp_set_config(interp0, 1, &cfg);
}

uint16_t interpolate(uint16_t x, uint16_t y, uint16_t mu_scaled) {
    interp0->base[0] = x;
    interp0->base[1] = y;
    uint16_t acc = mu_scaled;
    interp0->accum[1] = acc;
    uint16_t result = interp0->peek[1];
    return result;
}

void process() {
    static int fadeCounter = 0;
    double fadeAmount = 0.0;
    static int fade_duration;
    const double fade_ratio = 0.005;

    for (int process_index = 0; process_index < AUDIO_BLOCK_SIZE; process_index++) {
        if (retrigger_flag == 1) {
            phase_x = phase_0;
            phase_0 = 0;
            fade_duration = (int)(table_size * fade_ratio);
            fadeCounter = fade_duration;
            uint16_t cv_value = fastAnalogRead(ADC_CHAN_CV_SEL);
            cv_offset = map(cv_value, 0, 4095, 0, NUM_SAMPLES - 1);
            adjusted_sample = (base_sample + cv_offset) % NUM_SAMPLES;
            retrigger_flag = 0;
        }
        if (phase_0 < 0.99){
            phase_0 += inc;
        }
        double index_0 = fmod(phase_0 * table_size, table_size);
        double index_1 = fmod(index_0 + 1, table_size);
        uint16_t mu_scaled = (uint16_t)((index_0 - (int)index_0) * 256);

        uint16_t sample_0 = sample[adjusted_sample].data[(int)index_0];
        uint16_t sample_1 = sample[adjusted_sample].data[(int)index_1];
        uint16_t interpolatedSample = interpolate(sample_0, sample_1, mu_scaled);

        if (fadeCounter > 0) {
            fadeAmount = 1.0 - ((double)fadeCounter / (double)fade_duration);
            double index_x = fmod(phase_x * table_size, table_size);
            uint16_t sample_x = sample[adjusted_sample].data[(int)index_x];
            out = (int)((sample_x * (1 - fadeAmount)) + (interpolatedSample * fadeAmount));
            fadeCounter--;
        } else {
            out = interpolatedSample;
        }

        out_buf_ptr[process_index] = (phase_0 < 0.99) ? constrain(out, 0, PWM_RESOLUTION - 1) : PWM_CENTER;
    }
}

// ----------- Display logic for SH1122 OLED ------------

// If your peaks[] array is 128-wide and you want 256 pixels on the OLED, use this helper to interpolate for the display:
uint8_t interpPeak(const uint8_t* src, int srcLen, int i, int outLen) {
    // Linear interp from src[0..srcLen-1] to outLen points, return i'th value
    float srcIdx = (float)i * (srcLen-1) / (outLen-1);
    int idx = floor(srcIdx);
    float frac = srcIdx - idx;
    if (idx >= srcLen-1) return src[srcLen-1];
    return src[idx] * (1-frac) + src[idx+1] * frac;
}

void updateDisplay(int display_sample){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_9x15_tr);

    // Draw sample name at top left
    u8g2.drawStr(0, 14, sample[display_sample].name);

    // Vertically center waveform
    int y_offset = 32; // 64px/2
    int peaksLen = sizeof(peaks[display_sample]) / sizeof(peaks[display_sample][0]);
    for (int i = 0; i < 255; i++) {
        int y1, y2;
        if (peaksLen == 256) {
            y1 = y_offset + (peaks[display_sample][i]   / 2);
            y2 = y_offset + (peaks[display_sample][i+1] / 2);
        } else if (peaksLen == 128) {
            y1 = y_offset + (interpPeak(peaks[display_sample], 128, i, 256)   / 2);
            y2 = y_offset + (interpPeak(peaks[display_sample], 128, i+1, 256) / 2);
        } else {
            // Fallback, treat as 256
            y1 = y_offset + (peaks[display_sample][i]   / 2);
            y2 = y_offset + (peaks[display_sample][i+1] / 2);
        }
        u8g2.drawLine(i, y1, i+1, y2);
    }

    u8g2.sendBuffer();
}

void handleSwitch(int &base_sample, int pin, bool &switch_held, unsigned long &switch_held_start_time, unsigned long &switch_last_checked_time) {
    unsigned long current_time = millis();
    if (digitalRead(pin) == LOW) {
        if (!switch_held) {
            base_sample = (base_sample + 1) % NUM_SAMPLES;
            table_size = sample[base_sample].size;
            updateDisplay(base_sample);
            switch_held = true;
            switch_held_start_time = current_time;
            switch_last_checked_time = current_time;
        } else if (current_time - switch_held_start_time >= PREV_NEXT_SCROLL_TIMEOUT) {
            if (current_time - switch_last_checked_time >= SWITCH_DELAY_MILLIS) {
                base_sample = (base_sample + 1) % NUM_SAMPLES;
                table_size = sample[base_sample].size;
                updateDisplay(base_sample);
                switch_last_checked_time = current_time;
            }
        }
    } else {
        switch_held = false;
    }
}

void setup() {
    adc_init();

    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    pinMode(PIN_SW_PREV, INPUT_PULLUP);
    pinMode(PIN_SW_NEXT, INPUT_PULLUP);
    pinMode(PIN_TRIG_BTN, INPUT_PULLUP);
    pinMode(PIN_TRIG_JACK, INPUT_PULLUP);

    attachInterrupt(PIN_TRIG_BTN, onTriggerButton, FALLING);

    // Setup SPI0 for non-default pins
    SPI.setSCK(OLED_PIN_SCK);
    SPI.setTX(OLED_PIN_MOSI);
    SPI.begin();

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_9x15_tr);
    u8g2.setFontPosTop();
    u8g2.drawStr(90, 25, "play");
    u8g2.sendBuffer();

    delay(200);

    updateDisplay(base_sample);

    configurePWM();
    audio_rate = clock_get_hz(clk_sys) / (PWM_RESOLUTION - 1);
    table_size = sample[base_sample].size;
    one_div_SR = 1.0 / audio_rate;

    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
        buf_a[i] = 0;
        buf_b[i] = 0;
    }

    setupInterpolator();
}

void loop() {
    if (readTrigJack()) {
        retrigger_flag = 1;
    }

    handleSwitch(base_sample, PIN_SW_NEXT, next_switch_held, next_switch_held_start_time, switch_last_checked_time);
    handleSwitch(base_sample, PIN_SW_PREV, prev_switch_held, prev_switch_held_start_time, switch_last_checked_time);

    inc = makeFreq() * one_div_SR;

    if (dma_transfer_complete_flag > 0) {
        process();
        dma_transfer_complete_flag = 0;
    }
}

bool readTrigJack(){
    static int current_read = 0;
    static int last_read = 0;
    static bool result = false;
    last_read = current_read;
    current_read = gpio_get(PIN_TRIG_JACK);
    if (!current_read && last_read){
        result = true;
    } else {
        result = false;
    }
    return result;
}

void configurePWM(){
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PIN_PWM_OUT);
    pwm_set_clkdiv(slice_num, 1);
    pwm_set_wrap(slice_num, PWM_RESOLUTION);
    pwm_set_enabled(slice_num, true);
    pwm_set_irq_enabled(slice_num, true);

    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);

    dma_channel_config cfg_a = dma_channel_get_default_config(dma_chan_a);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg_a, true);
    channel_config_set_dreq(&cfg_a, DREQ_PWM_WRAP0 + slice_num);
    channel_config_set_ring(&cfg_a, false, size_bits);
    channel_config_set_chain_to(&cfg_a, dma_chan_b);

    dma_channel_configure(
        dma_chan_a,
        &cfg_a,
        &pwm_hw->slice[slice_num].cc,
        buf_a,
        AUDIO_BLOCK_SIZE,
        false
    );

    dma_channel_set_irq0_enabled(dma_chan_a, true);
    irq_set_exclusive_handler(DMA_IRQ_0, DMAChanATransCpltCallback);
    irq_set_enabled(DMA_IRQ_0,true);

    dma_channel_config cfg_b = dma_channel_get_default_config(dma_chan_b);
    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg_b, true);
    channel_config_set_dreq(&cfg_b, DREQ_PWM_WRAP0 + slice_num);
    channel_config_set_ring(&cfg_b, false, size_bits);
    channel_config_set_chain_to(&cfg_b, dma_chan_a);

    dma_channel_configure(
        dma_chan_b,
        &cfg_b,
        &pwm_hw->slice[slice_num].cc,
        buf_b,
        AUDIO_BLOCK_SIZE,
        false
    );

    dma_channel_set_irq1_enabled(dma_chan_b, true);
    irq_set_exclusive_handler(DMA_IRQ_1, DMAChanBTransCpltCallback);
    irq_set_enabled(DMA_IRQ_1,true);

    dma_channel_start(dma_chan_a);
}
