#include "regen.h"
#include <math.h>
#include <stdlib.h>
// funões de inicialização


namespace {

float limites(float a, float min, float max)
{
  if (a > max) return max;
  if (a < min) return min;
  return a;
};

bool numeroValido(float a) {
    return isfinite(a);
}

void adicionaMotivo(uint16_t* motivos, uint16_t motivo) {
    *motivos |= motivo;
}

} 












// Inicialização dos parâmetros padrão

void regenParamsPadrao(RegenParams* params) {

     // proteção contra ponteiros nulos
    if (params == nullptr) {
        return;
    }

    params->torque_freio = -1.0f;
    params->torque_lift = -0.5f;
    params->torque_min_cmd = -2.0f;

    params->i_carga_max = 4.0f;
    params->v_reduz = 51.5f;
    params->v_corte = 53.0f;
    params->v_religa = 51.0f;

    params->rpm_sinal = 1.0f;
    params->rpm_zero = 500.0f;
    params->rpm_pleno = 1000.0f;

    params->v_plausivel_min = 20.0f;

    params->term_ini = 75.0f;
    params->term_fim = 100.0f;

    params->usar_flt_id = true;
    params->flt_id_ok = 0;

    params->timeout_a3_ms = 200;
    params->timeout_a4_ms = 500;

    params->lift_entra_nm = 0.5f;
    params->lift_sai_nm = 1.0f;

    params->taxa_regen_sobe = 50.0f;
    params->taxa_regen_solta = 100.0f;
    params->taxa_tracao_sobe = 0.0f;
    params->taxa_tracao_desce = 0.0f;

    params->pedal_min = 470;
    params->pedal_max = 1150;
    params->pedal_tolerancia = 100;
    params->pedal_margem_baixa = 150;
    params->pedal_margem_alta = 150;
}


// Reset dos estados 

void regenReset(RegenState* state) {

    // proteção contra ponteiros nulos
    if (state == nullptr) {
        return;
    }

    state->ultimo_torque = 0.0f;
    state->trava_tensao = false;
    state->lift_ativo = false;
    state->iniciado = false;
    state->t_anterior = 0;
}

// verifica se os parâmetros são válidos 

void regenStep(
    const RegenParams* params,
    RegenState* state,
    const RegenIn* input,
    uint32_t agora_ms,
    RegenOut* output
){
   















    

    }