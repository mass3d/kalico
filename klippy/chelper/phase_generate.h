#ifndef PHASE_GENERATE_H
#define PHASE_GENERATE_H

#include "itersolve.h" // stepper_kinematics

struct phase_sample {
    double time;
    double position; // In microstep units
};

struct phase_generator {
    struct stepper_kinematics *sk;
    double step_dist;
    double update_interval;
    double last_flush_time;
    double last_position;
    double phase_offset;  // Microstep offset between klipper pos and TMC phase
    double direction;     // +1.0 or -1.0 (accounts for dir_pin inversion)
    int have_last_position;
};

struct phase_generator *phase_generator_alloc(void);
void phase_generator_set_time(struct phase_generator *pg, double time);
void phase_generator_set_last_position(struct phase_generator *pg, double pos);
void phase_generator_set_offset(struct phase_generator *pg, double offset);
void phase_generator_set_direction(struct phase_generator *pg, double dir);
void phase_generator_config(struct phase_generator *pg
                            , struct stepper_kinematics *sk
                            , double step_dist, double update_interval);
int phase_generator_generate(struct phase_generator *pg
                             , double flush_time
                             , struct phase_sample *samples, int max_samples);
int phase_generator_extract_positions(struct phase_sample *samples
                                      , int num_samples, double *positions);
int phase_generator_find_move_start(double *positions, int num_samples
                                     , double threshold);

#endif // phase_generate.h
