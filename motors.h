#pragma once

// TODO proper includes

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    MOTOR_Type_e    type;               //!< Defines the motor type
    uint_least16_t  numPolePairs;       //!< Defines the number of pole pairs
    float_t         Ls_d_H;             //!< Defines the direct stator inductance, H
    float_t         Ls_q_H;             //!< Defines the quadrature stator inductance, H
    float_t         Rr_Ohm;             //!< Defines the rotor resistance, Ohm
    float_t         Rs_Ohm;             //!< Defines the stator resistance, Ohm
    float_t         ratedFlux_VpHz;     //!< Defines the rated flux, V/Hz
    float_t magnetizingCurrent;
    float_t resEstCurrent_A;
    float_t indEstCurrent_A;
    float_t maxCurrent_A;
    float_t fluxEstFreq_Hz;
    float_t maxSpeed_krpm;
    uint_least32_t encoderLines;
    float_t inertia;
    float_t friction;
} MOTORCONFIG_MotorData_t;


// === Motor Configurations ===

static const MOTORCONFIG_MotorData_t MOTORCONFIG_TMotor_Antigravity_4004_300kv = {
    .type = MOTOR_Type_Pm,
    .numPolePairs = 12,
    .Rr_Ohm = 0.0,
    .Rs_Ohm = 0.2324751,
    .Ls_d_H = 0.00014, // A3
    .Ls_q_H = 0.00014,
    .ratedFlux_VpHz = 0.009390805,
    .magnetizingCurrent = 0,
    .resEstCurrent_A = 3.0,
    .indEstCurrent_A = -1.0,
    .maxCurrent_A = 9.0,
    .fluxEstFreq_Hz = 120.0, // Given by excel spreadsheet. Default 20.
    .maxSpeed_krpm = 6.0,
    .encoderLines = 5000,
    .inertia = 0.12937843799591064,  // determined with lab12a
    .friction = 0.10034477710723877,  // determined with lab12a
};


static const MOTORCONFIG_MotorData_t MOTORCONFIG_TMotor_Antigravity_4006_380kv = {
    .type = MOTOR_Type_Pm,
    .numPolePairs = 12,
    .Rr_Ohm = 0.0,
    .Rs_Ohm = 0.09872509,
    .Ls_d_H = 0.0000512, // A3 0.00005391928e-05
    .Ls_q_H = 0.0000512,
    .ratedFlux_VpHz = 0.007828592,
    .magnetizingCurrent = 0,
    .resEstCurrent_A = 5.0,
    .indEstCurrent_A = -1.0,
    .maxCurrent_A = 9.0,
    .fluxEstFreq_Hz = 120.0, // Given by excel spreadsheet. Default 20.
    .maxSpeed_krpm = 6.0,

    .encoderLines = 2048.0,
    .inertia = 0.2467952967,  // determined with lab12a
    .friction = 0.3493707776,  // determined with lab12a  // determined with lab12a
};


static const MOTORCONFIG_MotorData_t MOTORCONFIG_TMotor_Antigravity_MN7005_115kv = {
    .type = MOTOR_Type_Pm,
    .numPolePairs = 14,
    .Rr_Ohm = 0,  // Use 0 instead of NULL.  Is this good?
    .Rs_Ohm = 0.159925,
    .Ls_d_H = 0.00005119999,  // A3 0.00005391928e-05
    .Ls_q_H = 0.00005119999,
    .ratedFlux_VpHz = 0.01285466,
    .magnetizingCurrent = 0,
    .resEstCurrent_A = 4.0,
    .indEstCurrent_A = 4.0,
    .maxCurrent_A = 15.0,
    .fluxEstFreq_Hz = 120.0,  // Given by excel spreadsheet. Default 20.
    .maxSpeed_krpm = 6.0,

    .encoderLines = 5000,
    .inertia = 0.1023780107,  // determined with lab12a
    .friction = 0.2193766236,  // determined with lab12a
};


#ifdef __cplusplus
}
#endif // extern "C"

