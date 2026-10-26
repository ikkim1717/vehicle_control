#ifndef ESP_CORE_H
#define ESP_CORE_H
/*
 * 차량 ESP 시스템 설계 - Bicycle model + saturation tire model + ESP logic
 * (bicycle4.m / esplogic.m / fy_sat.m / steer1.m 을 C로 옮긴 코어)
 */
#ifdef __cplusplus
extern "C" {
#endif

#define ESP_N_INPUTS   1
#define ESP_N_PARAMS   12
#define ESP_N_OUTPUTS  17
#define ESP_N_STATES   6
#define ESP_DT_MAX     0.001   /* 내부 RK4 적분 최대 스텝 [s] */

/* 파라미터 인덱스 */
enum {
    ESP_P_M = 0,        /* 차량 질량 [kg] */
    ESP_P_IZZ,          /* 요 관성모멘트 [kg m^2] */
    ESP_P_LF,           /* 무게중심~전륜 [m] */
    ESP_P_LR,           /* 무게중심~후륜 [m] */
    ESP_P_CALPHA,       /* 타이어 1개 cornering stiffness [N/rad] */
    ESP_P_MU,           /* 비선형 타이어 계수 mu */
    ESP_P_K,            /* 비선형 타이어 계수 K */
    ESP_P_KE,           /* ESP 제어 게인 (0 = without ESP) */
    ESP_P_VX0,          /* 초기 속도 [m/s] */
    ESP_P_STEER_MODE,   /* 1: 내부 step steer(steer1.m), 0: 입력 delf_deg 사용 */
    ESP_P_STEER_TIME,   /* step steer 시작 시간 [s] */
    ESP_P_STEER_AMP_DEG /* step steer 크기 [deg] */
};

/* 출력 인덱스 */
enum {
    ESP_Y_VX = 0, ESP_Y_VY, ESP_Y_GAMMA, ESP_Y_X, ESP_Y_Y, ESP_Y_PSI,
    ESP_Y_GAMMA_DEG, ESP_Y_GAMMAD, ESP_Y_MZ, ESP_Y_ALPHA_F, ESP_Y_ALPHA_R,
    ESP_Y_FYF, ESP_Y_FYR, ESP_Y_BETA_DEG, ESP_Y_AY, ESP_Y_R, ESP_Y_DELF
};

typedef struct {
    double p[ESP_N_PARAMS];
    double x[ESP_N_STATES];   /* vx, vy, r, X, Y, psi */
} EspModel;

void   esp_default_params(double p[ESP_N_PARAMS]);
void   esp_reset(EspModel *mdl);                 /* x = [vx0 0 0 0 0 0] */
double esp_fy_sat(double Calpha, double mu, double K, double alpha);
double esp_logic(const EspModel *mdl, const double x[ESP_N_STATES], double delf);
double esp_steer(const EspModel *mdl, double t, double delf_in_rad);
void   esp_deriv(const EspModel *mdl, double t, const double x[ESP_N_STATES],
                 double delf_in_rad, double dx[ESP_N_STATES]);
/* t 에서 t+h 까지 적분 (RK4, 내부 스텝 <= ESP_DT_MAX). 성공 1, 실패 0 */
int    esp_advance(EspModel *mdl, double t, double h, double delf_in_rad);
void   esp_outputs(const EspModel *mdl, double t, double delf_in_rad,
                   double y[ESP_N_OUTPUTS]);

#ifdef __cplusplus
}
#endif
#endif
