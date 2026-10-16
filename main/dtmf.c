#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"

static const char *TAG = "DTMF_ESP32S3";

#define FS_HARDWARE_HZ      20000
#define DECIMATION_FACTOR   2
#define FS_EFETIVA_HZ       ((float)FS_HARDWARE_HZ / DECIMATION_FACTOR) // 10000.0 Hz

// Bloco de 256 amostras
#define N_BLOCO             256
#define HOP_SIZE            128     // Sobreposição de 50% 

// Configurações de DMA (Formato TYPE2, 4 bytes por amostra)
#define BYTES_PER_SAMPLE    sizeof(adc_digi_output_data_t)
#define DMA_FRAME_SAMPLES   256
#define CONV_FRAME_SIZE     (DMA_FRAME_SAMPLES * BYTES_PER_SAMPLE)
#define MAX_STORE_BUF_SIZE  (CONV_FRAME_SIZE * 4)

// Limiares de decisão
#define LIMIAR_RMS_SILENCIO 5.0f   // Gate de ruído de fundo no domínio do tempo
#define RAZAO_DOMINANCIA    2.5f    // O pico do grupo deve ser 2.5x maior que a 2ª maior energia

#define NUM_FREQS_GRUPO     4

static const float FREQS_BAIXAS[NUM_FREQS_GRUPO] = { 697.0f, 770.0f, 852.0f, 941.0f };
static const float FREQS_ALTAS[NUM_FREQS_GRUPO]  = { 1209.0f, 1336.0f, 1477.0f, 1633.0f };

static const char MAPA_TECLAS[NUM_FREQS_GRUPO][NUM_FREQS_GRUPO] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '*', '0', '#', 'D' }
};

static float coef_baixas[NUM_FREQS_GRUPO];
static float coef_altas[NUM_FREQS_GRUPO];

static QueueHandle_t fila_notificacao_dma;

static float buffer_circular[N_BLOCO];
static size_t head_escrita = 0;
static size_t amostras_novas = 0;

__attribute__((aligned(16))) static float quadro_linear[N_BLOCO];
__attribute__((aligned(16))) static float janela_hamming[N_BLOCO];

static char ultima_tecla_validada = '\0';

static void inicializar_tabelas_dtmf(void)
{
    // Janela de Hamming
    for (int n = 0; n < N_BLOCO; n++) {
        janela_hamming[n] = 0.54f - 0.46f * cosf((2.0f * M_PI * n) / (N_BLOCO - 1));
    }

    // Coeficientes de Goertzel
    for (int i = 0; i < NUM_FREQS_GRUPO; i++) {
        float omega_b = (2.0f * M_PI * FREQS_BAIXAS[i]) / FS_EFETIVA_HZ;
        float omega_a = (2.0f * M_PI * FREQS_ALTAS[i])  / FS_EFETIVA_HZ;

        coef_baixas[i] = 2.0f * cosf(omega_b);
        coef_altas[i]  = 2.0f * cosf(omega_a);
    }
}

static float calcular_energia_goertzel(const float *sinal, int n_amostras, float coef_2cos)
{
    float v_n1 = 0.0f; // v[n-1]
    float v_n2 = 0.0f; // v[n-2]

    for (int n = 0; n < n_amostras; n++) {
        float v_n = sinal[n] + (coef_2cos * v_n1) - v_n2;
        v_n2 = v_n1;
        v_n1 = v_n;
    }

    float energia = (v_n1 * v_n1) + (v_n2 * v_n2) - (coef_2cos * v_n1 * v_n2);
    return energia / (float)n_amostras;
}

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
        .flags = {
            .flush_pool = 1,
        },
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_DB_0,
        .channel   = ADC_CHANNEL_2, // GPIO3
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

static void processar_quadro_dtmf(void)
{
    // buffer circular e calcular média DC
    float soma_dc = 0.0f;
    for (int i = 0; i < N_BLOCO; i++) {
        size_t idx = (head_escrita + i) % N_BLOCO;
        quadro_linear[i] = buffer_circular[idx];
        soma_dc += quadro_linear[i];
    }
    float media_dc = soma_dc / N_BLOCO;

    // remover offset DC do eletreto
    for (int i = 0; i < N_BLOCO; i++) {
        quadro_linear[i] -= media_dc;
    }

    float soma_quadrados = 0.0f;
    dsps_dotprod_f32(quadro_linear, quadro_linear, &soma_quadrados, N_BLOCO);
    float valor_rms = sqrtf(soma_quadrados / N_BLOCO);

    if (valor_rms < LIMIAR_RMS_SILENCIO) {
        ultima_tecla_validada = '\0'; // Reseta estado ao soltar a tecla
        return;
    }

    // 4. Aplicar Janela de Hamming 
    for (int i = 0; i < N_BLOCO; i++) {
        quadro_linear[i] *= janela_hamming[i];
    }

    int idx_max_baixa = 0, idx_max_alta = 0;
    float max1_baixa = 0.0f, max2_baixa = 0.0f;
    float max1_alta  = 0.0f, max2_alta  = 0.0f;

    for (int i = 0; i < NUM_FREQS_GRUPO; i++) {
        float e_baixa = calcular_energia_goertzel(quadro_linear, N_BLOCO, coef_baixas[i]);
        if (e_baixa > max1_baixa) {
            max2_baixa = max1_baixa;
            max1_baixa = e_baixa;
            idx_max_baixa = i;
        } else if (e_baixa > max2_baixa) {
            max2_baixa = e_baixa;
        }

        float e_alta = calcular_energia_goertzel(quadro_linear, N_BLOCO, coef_altas[i]);
        if (e_alta > max1_alta) {
            max2_alta = max1_alta;
            max1_alta = e_alta;
            idx_max_alta = i;
        } else if (e_alta > max2_alta) {
            max2_alta = e_alta;
        }
    }

    bool baixa_exclusiva = (max1_baixa > (max2_baixa * RAZAO_DOMINANCIA));
    bool alta_exclusiva  = (max1_alta  > (max2_alta  * RAZAO_DOMINANCIA));

    if (baixa_exclusiva && alta_exclusiva) {
        char tecla_detectada = MAPA_TECLAS[idx_max_baixa][idx_max_alta];

        if (tecla_detectada != ultima_tecla_validada) {
            ultima_tecla_validada = tecla_detectada;
            printf("Tecla DTMF: [ %c ] | f_baixa: %.0f Hz | f_alta: %.0f Hz | RMS: %.1f\n",
                   tecla_detectada,
                   FREQS_BAIXAS[idx_max_baixa],
                   FREQS_ALTAS[idx_max_alta],
                   valor_rms);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Decodificador DTMF");
    inicializar_tabelas_dtmf();

    fila_notificacao_dma = xQueueCreate(8, sizeof(uint32_t));
    adc_continuous_handle_t adc_handle = inicializar_adc_dma();
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    ESP_LOGI(TAG, "Fs_efetiva = %.1f Hz, Janela = %d amostras (%.1f ms)",
             FS_EFETIVA_HZ, N_BLOCO, (N_BLOCO * 1000.0f) / FS_EFETIVA_HZ);

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
                            head_escrita = (head_escrita + 1) % N_BLOCO;
                            amostras_novas++;

                            if (amostras_novas >= HOP_SIZE) {
                                amostras_novas = 0;
                                processar_quadro_dtmf();
                            }
                        }
                    }
                }
            }
        }
    }
}