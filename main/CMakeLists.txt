# ==============================================================================
# ESCOLHA MANUAL DO CÓDIGO A SER COMPILADO:
# Deixe descomentada APENAS a linha do arquivo que deseja rodar agora.
# ==============================================================================

# set(SRC_ATIVO "afinador.c")
 set(SRC_ATIVO "dtmf.c")
# set(SRC_ATIVO "vogais.c")

# ==============================================================================

idf_component_register(
    SRCS "${SRC_ATIVO}"
    INCLUDE_DIRS "."
)