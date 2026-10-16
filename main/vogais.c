#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"

static const char *TAG = "VOGAIS_ESP32S3";

/* ==========================================================================
 * PARÂMETROS DE PROJETO E AMOSTRAGEM
 * ========================================================================== */
#define FS_HARDWARE_HZ      20000
#define DECIMATION_FACTOR   2
#define FS_EFETIVA_HZ       ((float)FS_HARDWARE_HZ / DECIMATION_FACTOR) // 10000.0 Hz

// Quadro de 512 amostras a 10 kHz = 51.2 ms (Resolução = 19.53 Hz/bin)
#define N_FFT               512
#define HOP_SIZE            256     // Sobreposição de 50% (atualiza a cada 25.6 ms)
#define NUM_BINS_UTEIS      (N_FFT / 2 + 1)

// Configurações do DMA (Formato TYPE2 = 4 bytes por amostra)
#define BYTES_PER_SAMPLE    sizeof(adc_digi_output_data_t)
#define DMA_FRAME_SAMPLES   256
#define CONV_FRAME_SIZE     (DMA_FRAME_SAMPLES * BYTES_PER_SAMPLE)
#define MAX_STORE_BUF_SIZE  (CONV_FRAME_SIZE * 4)

// Limiar de atividade de voz (Gate de Silêncio por RMS)
#define LIMIAR_RMS_VOZ      20.0f

// Largura da janela de suavização espectral para extrair a envoltória (formantes)
// 9 bins * 19.53 Hz ~= 175 Hz (suaviza os harmónicos da fonte glotal)
#define LARGURA_SUAVIZACAO  9

/* ==========================================================================
 * MAPA DE REFERÊNCIA DAS VOGAIS (PLANO F1 x F2)
 * ========================================================================== */
#define NUM_VOGAIS          5

typedef struct {
    const char *simbolo;
    float f1_ref;
    float f2_ref;
} perfil_vogal_t;

// Valores médios de referência baseados no gráfico F1 x F2 (Português Brasileiro)
static const perfil_vogal_t TABELA_VOGAIS[NUM_VOGAIS] = {
    { "[a]", 850.0f, 1200.0f },
    { "[e]", 450.0f, 1800.0f },
    { "[i]", 300.0f, 2200.0f },
    { "[o]", 450.0f,  950.0f },
    { "[u]", 300.0f,  800.0f }
};

/* ==========================================================================
 * BUFFERS ALINHADOS E VARIÁVEIS DE ESTADO
 * ========================================================================== */
static QueueHandle_t fila_notificacao_dma;

static float buffer_circular[N_FFT];
static size_t head_escrita = 0;
static size_t amostras_novas = 0;

__attribute__((aligned(16))) static float quadro_linear[N_FFT];
__attribute__((aligned(16))) static float janela_hann[N_FFT];
__attribute__((aligned(16))) static float fft_complex_buf[N_FFT * 2];

static float espectro_db[NUM_BINS_UTEIS];
static float envoltoria_db[NUM_BINS_UTEIS];

/* ==========================================================================
 * CALLBACK DE INTERRUPÇÃO DO DMA E CONFIGURAÇÃO DO ADC
 * ========================================================================== */
static bool IRAM_ATTR on_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t hp_task_woken = pdFALSE;
    uint32_t tamanho_pronto = edata->size;
    xQueueSendFromISR(fila_notificacao_dma, &tamanho_pronto, &hp_task_woken);
    return (hp_task_woken == pdTRUE);
}

static adc_continuous_handle_t inicializar_adc_dma(void)
{
    adc_continuous_handle_t adc_handle = NULL;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = MAX_STORE_BUF_SIZE,
        .conv_frame_size    = CONV_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_DB_12,
        .channel   = ADC_CHANNEL_2, // GPIO3 no ESP32-S3
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

/* ==========================================================================
 * PROCESSAMENTO DO QUADRO: ENVOLTÓRIA, FORMANTES E CLASSIFICAÇÃO
 * ========================================================================== */
static void processar_quadro_vogais(void)
{
    // 1. Desenrolar o buffer circular e calcular a média DC
    float soma_dc = 0.0f;
    for (int i = 0; i < N_FFT; i++) {
        size_t idx = (head_escrita + i) % N_FFT;
        quadro_linear[i] = buffer_circular[idx];
        soma_dc += quadro_linear[i];
    }
    float media_dc = soma_dc / N_FFT;

    // 2. Remover offset DC de polarização do microfone
    for (int i = 0; i < N_FFT; i++) {
        quadro_linear[i] -= media_dc;
    }

    // 3. Calcular RMS usando esp-dsp (Gate de atividade vocal)
    float soma_quadrados = 0.0f;
    dsps_dotprod_f32(quadro_linear, quadro_linear, &soma_quadrados, N_FFT);
    float valor_rms = sqrtf(soma_quadrados / N_FFT);

    if (valor_rms < LIMIAR_RMS_VOZ) {
        return; // Silêncio
    }

    // 4. Calcular Taxa de Cruzamento por Zero (ZCR) para rejeitar sons fricativos/ruído
    int cruzamentos = 0;
    for (int i = 0; i < N_FFT - 1; i++) {
        if ((quadro_linear[i] * quadro_linear[i + 1]) < 0.0f) {
            cruzamentos++;
        }
    }
    float zcr = (float)cruzamentos / (float)(N_FFT - 1);
    if (zcr > 0.35f) {
        return; // ZCR muito alto indica consoante surda (ex: 's', 'f') ou ruído, não vogal
    }

    // 5. Pré-ênfase leve H(z) = 1 - 0.95*z^-1 (realça o 2º formante F2 que cai naturalmente em dB)
    // e aplicação da Janela de Hann para a FFT
    float amostra_anterior = quadro_linear[0];
    for (int i = 0; i < N_FFT; i++) {
        float pre_enfase = quadro_linear[i] - 0.95f * amostra_anterior;
        amostra_anterior = quadro_linear[i];

        fft_complex_buf[i * 2 + 0] = pre_enfase * janela_hann[i];
        fft_complex_buf[i * 2 + 1] = 0.0f;
    }

    // 6. Executar FFT Radix-2 e Bit-Reversal
    dsps_fft2r_fc32(fft_complex_buf, N_FFT);
    dsps_bit_rev_fc32(fft_complex_buf, N_FFT);

    // 7. Calcular espectro de potência em dB: 10 * log10(|X[k]|^2 / N)
    for (int k = 0; k < NUM_BINS_UTEIS; k++) {
        float re = fft_complex_buf[k * 2 + 0];
        float im = fft_complex_buf[k * 2 + 1];
        float potencia = (re * re + im * im) / (float)N_FFT;
        espectro_db[k] = 10.0f * log10f(potencia + 1e-6f);
    }

    // 8. Extrair a Envoltória Espectral por média móvel nos bins de frequência
    int meia_janela = LARGURA_SUAVIZACAO / 2;
    for (int k = meia_janela; k < NUM_BINS_UTEIS - meia_janela; k++) {
        float soma_env = 0.0f;
        for (int j = -meia_janela; j <= meia_janela; j++) {
            soma_env += espectro_db[k + j];
        }
        envoltoria_db[k] = soma_env / (float)LARGURA_SUAVIZACAO;
    }

    // 9. Busca de Formantes:
    // F1 situa-se na banda [200 Hz, 950 Hz]
    int k_f1_min = (int)floorf((200.0f * N_FFT) / FS_EFETIVA_HZ);
    int k_f1_max = (int)ceilf((950.0f * N_FFT) / FS_EFETIVA_HZ);
    int k_f1 = k_f1_min;
    float max_f1_db = -FLT_MAX;

    for (int k = k_f1_min; k <= k_f1_max; k++) {
        if (envoltoria_db[k] > max_f1_db) {
            max_f1_db = envoltoria_db[k];
            k_f1 = k;
        }
    }
    float f1_hz = (float)k_f1 * (FS_EFETIVA_HZ / N_FFT);

    // F2 situa-se na banda [750 Hz, 2600 Hz], exigindo separação mínima de 250 Hz acima de F1
    int k_f2_min = (int)floorf(((f1_hz + 250.0f) * N_FFT) / FS_EFETIVA_HZ);
    int k_bin_750 = (int)floorf((750.0f * N_FFT) / FS_EFETIVA_HZ);
    if (k_f2_min < k_bin_750) {
        k_f2_min = k_bin_750;
    }
    int k_f2_max = (int)ceilf((2600.0f * N_FFT) / FS_EFETIVA_HZ);

    int k_f2 = k_f2_min;
    float max_f2_db = -FLT_MAX;

    for (int k = k_f2_min; k <= k_f2_max; k++) {
        if (envoltoria_db[k] > max_f2_db) {
            max_f2_db = envoltoria_db[k];
            k_f2 = k;
        }
    }
    float f2_hz = (float)k_f2 * (FS_EFETIVA_HZ / N_FFT);

    // 10. Classificação via Distância Euclidiana no plano (F1, F2)
    // Como F2 tem faixa dinâmica ~3x maior que F1 (800-2200 Hz vs 300-850 Hz),
    // ponderamos as diferenças pelas escalas médias para equilibrar os dois eixos.
    int indice_vogal_vencedora = 0;
    float menor_distancia = FLT_MAX;

    for (int v = 0; v < NUM_VOGAIS; v++) {
        float diff_f1 = (f1_hz - TABELA_VOGAIS[v].f1_ref) / 550.0f;  // amplitude de faixa F1
        float diff_f2 = (f2_hz - TABELA_VOGAIS[v].f2_ref) / 1400.0f; // amplitude de faixa F2
        float dist_euclidiana = sqrtf((diff_f1 * diff_f1) + (diff_f2 * diff_f2));

        if (dist_euclidiana < menor_distancia) {
            menor_distancia = dist_euclidiana;
            indice_vogal_vencedora = v;
        }
    }

    printf("Vogal: %s | F1: %5.0f Hz | F2: %5.0f Hz | Dist: %.2f | RMS: %4.1f | ZCR: %.2f\n",
           TABELA_VOGAIS[indice_vogal_vencedora].simbolo,
           f1_hz, f2_hz, menor_distancia, valor_rms, zcr);
}

/* ==========================================================================
 * APLICAÇÃO PRINCIPAL
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "A inicializar Detetor de Vogais (FFT + Envoltoria Espectral)...");
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_FFT));
    dsps_wind_hann_f32(janela_hann, N_FFT);

    fila_notificacao_dma = xQueueCreate(8, sizeof(uint32_t));
    adc_continuous_handle_t adc_handle = inicializar_adc_dma();
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    ESP_LOGI(TAG, "Sistema pronto! Fs_efetiva = %.1f Hz, Quadro = %d amostras (%.1f ms)",
             FS_EFETIVA_HZ, N_FFT, (N_FFT * 1000.0f) / FS_EFETIVA_HZ);

    uint8_t buffer_leitura_dma[CONV_FRAME_SIZE];
    uint32_t bytes_lidos = 0;
    uint32_t evento_dma = 0;

    float acumulador_decimacao = 0.0f;
    int contador_decimacao = 0;

    while (1) {
        if (xQueueReceive(fila_notificacao_dma, &evento_dma, portMAX_DELAY) == pdTRUE) {
            
            while (adc_continuous_read(adc_handle, buffer_leitura_dma, CONV_FRAME_SIZE, &bytes_lidos, 0) == ESP_OK) {
                
                for (uint32_t i = 0; i < bytes_lidos; i += BYTES_PER_SAMPLE) {
                    adc_digi_output_data_t *dado = (adc_digi_output_data_t *)&buffer_leitura_dma[i];

                    if (dado->type2.unit == ADC_UNIT_1 && dado->type2.channel == ADC_CHANNEL_2) {
                        acumulador_decimacao += (float)dado->type2.data;
                        contador_decimacao++;

                        if (contador_decimacao >= DECIMATION_FACTOR) {
                            float amostra_filtrada = acumulador_decimacao / DECIMATION_FACTOR;
                            acumulador_decimacao = 0.0f;
                            contador_decimacao = 0;

                            buffer_circular[head_escrita] = amostra_filtrada;
                            head_escrita = (head_escrita + 1) % N_FFT;
                            amostras_novas++;

                            if (amostras_novas >= HOP_SIZE) {
                                amostras_novas = 0;
                                processar_quadro_vogais();
                            }
                        }
                    }
                }
            }
        }
    }
}