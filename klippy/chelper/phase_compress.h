#ifndef PHASE_COMPRESS_H
#define PHASE_COMPRESS_H

#include <stdint.h>

struct phase_compressed_move {
    double start_position;  // In microstep units (will be converted to 16.16)
    double velocity;        // Per-interval velocity
    double acceleration;    // Per-interval acceleration
    int count;              // Number of intervals
};

struct phase_compressor {
    double max_error; // Maximum fit error in microstep units
};

struct phase_compressor *phase_compressor_alloc(void);
void phase_compressor_set_max_error(struct phase_compressor *pc
                                    , double max_error);
int phase_compressor_compress(struct phase_compressor *pc
                              , double *positions, int num_samples
                              , struct phase_compressed_move *out, int max_out);

struct phase_mcu_move {
    int32_t start_position;
    int32_t velocity;
    int32_t acceleration;
    uint16_t count;
};

int phase_compressor_to_fixed(struct phase_compressed_move *segments
                              , int num_segments
                              , struct phase_mcu_move *out_mcu, int max_out);

#endif // phase_compress.h
