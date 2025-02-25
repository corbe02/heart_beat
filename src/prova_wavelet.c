#include "wavelib.h"
#include <stdio.h>
#include <stdlib.h>

int main() {
    // Inizializza la wavelet Haar
    wave_object wave = wave_init("haar");

    // Inizializza la DWT con 2 livelli di decomposizione
    wt_object wt = wt_init(wave, "dwt", 2, 0);

    // Segnale di input
    double input_signal[] = {1, 2, 3, 4, 5, 6, 7, 8};
    int signal_length = 8;

    // Esegui la DWT
    dwt(wt, input_signal);

    // Stampa i coefficienti della decomposizione
    printf("Approssimazione livello 1:\n");
    for (int i = 0; i < wt->outlength[0]; i++) {
        printf("%lf ", wt->output[i]);
    }
    printf("\n");

    // Pulizia della memoria
    wave_free(wave);
    wt_free(wt);

    return 0;
}
