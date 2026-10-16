#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"

static const char *TAG = "AFINADOR_ESP32S3";

// Taxa de amostragem e Decimação:
// Caso o hardware exija fs_hw mínima de 20 kHz, amostramos a 20 kHz e 
// decimamos por um fator de 5, obtendo fs_efetiva = 4000 Hz.
#define FS_HARDWARE_HZ      20000
#define DECIMATION_FACTOR   5
#define FS_EFETIVA_HZ       ((float)FS_HARDWARE_HZ / DECIMATION_FACTOR) // 4000.0 Hz

// Tamanho da FFT e Segmentação (Resolução = 4000 / 1024 = 3.906 Hz/bin)
#define N_FFT               1024
#define HOP_SIZE            256     // Sobreposição de 75% (atualiza a cada 64 ms)

// Configurações do DMA (Formato TYPE2 = 4 bytes por amostra)
#define BYTES_PER_SAMPLE    sizeof(adc_digi_output_data_t)
#define DMA_FRAME_SAMPLES   256
#define CONV_FRAME_SIZE     (DMA_FRAME_SAMPLES * BYTES_PER_SAMPLE) // 1024 bytes
#define MAX_STORE_BUF_SIZE  (CONV_FRAME_SIZE * 4)                  // 4096 bytes

// Limite de 2 Oitavas: Violão padrão (E2 = 82.41 Hz até E4 = 329.63 Hz)
#define FREQ_MIN_HZ         300.0f
#define FREQ_MAX_HZ         1200.0f

// Limiar de energia (Gate de Silêncio em unidades do ADC de 12 bits: 0-4095)
#define LIMIAR_RMS_SILENCIO 0.1f

static QueueHandle_t fila_notificacao_dma;

// Buffer circular para armazenar as amostras decimadas
static float buffer_circular[N_FFT];
static size_t head_escrita = 0;
static size_t amostras_novas = 0;

// Vetores alinhados em 16 bytes exigidos para otimização SIMD do esp-dsp
__attribute__((aligned(16))) static float quadro_linear[N_FFT];
__attribute__((aligned(16))) static float janela_hann[N_FFT];
__attribute__((aligned(16))) static float fft_complex_buf[N_FFT * 2];
__attribute__((aligned(16))) static float espectro_mag[N_FFT / 2 + 1];

// Nomes das 12 notas da escala cromática (Temperamento Igual)
static const char *NOMES_NOTAS[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

static bool IRAM_ATTR on_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t hp_task_woken = pdFALSE;
    uint32_t tamanho_pronto = edata->size;
    // Sinaliza a task consumidora que um bloco de conversão terminou
    xQueueSendFromISR(fila_notificacao_dma, &tamanho_pronto, &hp_task_woken);
    return (hp_task_woken == pdTRUE);
}

static adc_continuous_handle_t inicializar_adc_dma(void)
{
    adc_continuous_handle_t adc_handle = NULL;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = MAX_STORE_BUF_SIZE,
        .conv_frame_size    = CONV_FRAME_SIZE,
        .flags = {
            .flush_pool = 1,
        },
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));
    // Configura Canal 2 da Unidade 1 (ADC1_CH2 -> GPIO3 no ESP32-S3) com atenuação de 12 dB
    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_DB_0,
        .channel   = ADC_CHANNEL_2,
        .unit      = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    adc_continuous_config_t cfg = {
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
        .sample_freq_hz = FS_HARDWARE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    return adc_handle;
}

static void processar_quadro_audio(void)
{
    // 1. Desenrolar o buffer circular para um vetor linear contíguo e calcular a média (DC)
    float soma_dc = 0.0f;
    for (int i = 0; i < N_FFT; i++) {
        size_t idx = (head_escrita + i) % N_FFT;
        quadro_linear[i] = buffer_circular[idx];
        soma_dc += quadro_linear[i];
    }
    float media_dc = soma_dc / N_FFT;

    // 2. Remover nível DC de polarização do microfone eletreto
    for (int i = 0; i < N_FFT; i++) {
        quadro_linear[i] -= media_dc;
    }

    // 3. Calcular RMS usando produto interno acelerado por hardware (esp-dsp)
    float soma_quadrados = 0.0f;
    dsps_dotprod_f32(quadro_linear, quadro_linear, &soma_quadrados, N_FFT);
    float valor_rms = sqrtf(soma_quadrados / N_FFT);

    // Segmentação orientada a evento (Gate de silêncio por limiar)
    if (valor_rms < LIMIAR_RMS_SILENCIO) {
        return; // Sinal abaixo do limiar de atividade sonora
    }

    // 4. Aplicar Janela de Hann e montar vetor complexo (Im[i] = 0)
    for (int i = 0; i < N_FFT; i++) {
        fft_complex_buf[i * 2 + 0] = quadro_linear[i] * janela_hann[i];
        fft_complex_buf[i * 2 + 1] = 0.0f;
    }

    // 5. Executar FFT Radix-2 e Reordenação Bit-Reversal
    dsps_fft2r_fc32(fft_complex_buf, N_FFT);
    dsps_bit_rev_fc32(fft_complex_buf, N_FFT);

    // 6. Calcular magnitude nos bins referentes às 2 oitavas de interesse
    int k_min = (int)floorf((FREQ_MIN_HZ * N_FFT) / FS_EFETIVA_HZ);
    int k_max = (int)ceilf((FREQ_MAX_HZ * N_FFT) / FS_EFETIVA_HZ);
    if (k_min < 1) k_min = 1;
    if (k_max > (N_FFT / 2)) k_max = N_FFT / 2;

    float mag_max_global = 0.0f;
    int k_pico_global = k_min;

    for (int k = k_min; k <= k_max; k++) {
        float re = fft_complex_buf[k * 2 + 0];
        float im = fft_complex_buf[k * 2 + 1];
        espectro_mag[k] = sqrtf((re * re + im * im) / N_FFT);

        if (espectro_mag[k] > mag_max_global) {
            mag_max_global = espectro_mag[k];
            k_pico_global = k;
        }
    }

    // 7. Mitigação de Harmónicos: procurar o primeiro pico local significativo (>= 35% do máximo)
    // Isso garante que pegamos a fundamental f0 mesmo que o 2º harmónico (2*f0) seja mais forte.
    int k_fundamental = k_pico_global;
    float limiar_harmonico = mag_max_global * 0.35f;

    for (int k = k_min + 1; k < k_max; k++) {
        bool eh_pico_local = (espectro_mag[k] > espectro_mag[k - 1]) &&
                             (espectro_mag[k] >= espectro_mag[k + 1]);
        if (eh_pico_local && (espectro_mag[k] >= limiar_harmonico)) {
            k_fundamental = k;
            break; // Primeiro pico relevante encontrado
        }
    }

    // 8. Interpolação parabólica nos 3 bins vizinhos para refinar a resolução em frequência
    float delta_k = 0.0f;
    if (k_fundamental > k_min && k_fundamental < k_max) {
        float alpha = espectro_mag[k_fundamental - 1];
        float beta  = espectro_mag[k_fundamental];
        float gamma = espectro_mag[k_fundamental + 1];
        float denom = (alpha - 2.0f * beta + gamma);
        if (fabsf(denom) > 1e-6f) {
            delta_k = 0.5f * (alpha - gamma) / denom;
        }
    }

    float f0_estimada = ((float)k_fundamental + delta_k) * (FS_EFETIVA_HZ / N_FFT);

    // 9. Equação do Temperamento Igual: f(n) = 440 * 2^((n-49)/12)
    // Inversa: n = round(12 * log2(f0 / 440.0) + 49)
    float n_exato = 12.0f * log2f(f0_estimada / 440.0f) + 49.0f;
    int n_tecla = (int)roundf(n_exato);

    // Frequência ideal da nota mais próxima
    float freq_ideal = 440.0f * powf(2.0f, (n_tecla - 49.0f) / 12.0f);
    float erro_hz = f0_estimada - freq_ideal;

    // Converter índice da tecla do piano (n) para Nota + Oitava (C4 = tecla 40)
    int indice_midi = n_tecla + 20; // A4 (49) -> MIDI 69; C4 (40) -> MIDI 60
    int indice_nota = ((indice_midi % 12) + 12) % 12;
    int oitava      = (indice_midi / 12) - 1;

    printf("Nota: %2s%d | f0: %6.2f Hz",
           NOMES_NOTAS[indice_nota], oitava, f0_estimada);
}

void app_main(void)
{
    ESP_LOGI(TAG, "A inicializar biblioteca esp-dsp e tabelas de FFT...");
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_FFT));
    dsps_wind_hann_f32(janela_hann, N_FFT);

    fila_notificacao_dma = xQueueCreate(8, sizeof(uint32_t));
    adc_continuous_handle_t adc_handle = inicializar_adc_dma();
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    ESP_LOGI(TAG, "Aquisição iniciada! Fs_efetiva = %.1f Hz, Janela = %d amostras", FS_EFETIVA_HZ, N_FFT);

    uint8_t buffer_leitura_dma[CONV_FRAME_SIZE];
    uint32_t bytes_lidos = 0;
    uint32_t evento_dma = 0;

    // Variáveis para filtro média móvel de decimação anti-aliasing digital
    float acumulador_decimacao = 0.0f;
    int contador_decimacao = 0;

    while (1) {
        // Bloqueia a task até a ISR sinalizar que um frame DMA está pronto
        if (xQueueReceive(fila_notificacao_dma, &evento_dma, portMAX_DELAY) == pdTRUE) {
            
            while (adc_continuous_read(adc_handle, buffer_leitura_dma, CONV_FRAME_SIZE, &bytes_lidos, 0) == ESP_OK) {
                
                for (uint32_t i = 0; i < bytes_lidos; i += BYTES_PER_SAMPLE) {
                    adc_digi_output_data_t *dado = (adc_digi_output_data_t *)&buffer_leitura_dma[i];

                    // Verifica se a amostra pertence à unidade e canal configurados (Formato TYPE2)
                    if (dado->type2.unit == ADC_UNIT_1 && dado->type2.channel == ADC_CHANNEL_2) {
                        acumulador_decimacao += (float)dado->type2.data;
                        contador_decimacao++;

                        // Completa 1 amostra efetiva a cada DECIMATION_FACTOR leituras (Oversampling + Decimação)
                        if (contador_decimacao >= DECIMATION_FACTOR) {
                            float amostra_filtrada = acumulador_decimacao / DECIMATION_FACTOR;
                            acumulador_decimacao = 0.0f;
                            contador_decimacao = 0;

                            // Insere no Buffer Circular (O(1), sem memmove)
                            buffer_circular[head_escrita] = amostra_filtrada;
                            head_escrita = (head_escrita + 1) % N_FFT;
                            amostras_novas++;

                            // Aciona o cálculo a cada HOP_SIZE amostras novas (Janela Deslizante)
                            if (amostras_novas >= HOP_SIZE) {
                                amostras_novas = 0;
                                processar_quadro_audio();
                            }
                        }
                    }
                }
            }
        }
    }
}