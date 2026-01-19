#ifndef _ENCODER_H
#define _ENCODER_H           

#include "zf_common_typedef.h"

#define ENCODER_QUAD_lf                    (TC_CH07_ENCODER)                      // 编码器接口
#define ENCODER_QUAD_lf_PHASE_A            (TC_CH07_ENCODER_CH1_P02_0)            // PHASE_A 对应的引脚
#define ENCODER_QUAD_lf_PHASE_B            (TC_CH07_ENCODER_CH2_P02_1)            // PHASE_B 对应的引脚

#define ENCODER_QUAD_rf                    (TC_CH09_ENCODER)                      // 编码器接口  
#define ENCODER_QUAD_rf_PHASE_A            (TC_CH09_ENCODER_CH1_P05_0)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD_rf_PHASE_B            (TC_CH09_ENCODER_CH2_P05_1)            // PHASE_B 对应的引脚      

#define ENCODER_QUAD_lb                    (TC_CH19_ENCODER)                      // 编码器接口  
#define ENCODER_QUAD_lb_PHASE_A            (TC_CH19_ENCODER_CH1_P08_0)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD_lb_PHASE_B            (TC_CH19_ENCODER_CH2_P08_1)            // PHASE_B 对应的引脚                   

#define ENCODER_QUAD_rb                    (TC_CH36_ENCODER)                      // 编码器接口
#define ENCODER_QUAD_rb_PHASE_A            (TC_CH36_ENCODER_CH1_P12_0)            // PHASE_A 对应的引脚
#define ENCODER_QUAD_rb_PHASE_B            (TC_CH36_ENCODER_CH2_P12_1)            // PHASE_B 对应的引脚

typedef struct {
    float lf,rf,lb,rb;
} Encoder;

extern Encoder encoder_data;
void Encoder_Init();
void Encoder_GetCount();


#endif