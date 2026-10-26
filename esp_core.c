#include "esp_core.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double sgn(double v) { return (v > 0.0) - (v < 0.0); }

void esp_default_params(double p[ESP_N_PARAMS])
{
    p[ESP_P_M]             = 2045.0;
    p[ESP_P_IZZ]           = 5428.0;
    p[ESP_P_LF]            = 1.488;
    p[ESP_P_LR]            = 1.712;
    p[ESP_P_CALPHA]        = 39000.0;
    p[ESP_P_MU]            = 0.9;
    p[ESP_P_K]             = 19.0;
    p[ESP_P_KE]            = 8000.0;
    p[ESP_P_VX0]           = 80.0 / 3.6;
    p[ESP_P_STEER_MODE]    = 1.0;
    p[ESP_P_STEER_TIME]    = 2.0;
    p[ESP_P_STEER_AMP_DEG] = 3.0;
}

void esp_reset(EspModel *mdl)
{
    int i;
    for (i = 0; i < ESP_N_STATES; i++)
        mdl->x[i] = 0.0;
    mdl->x[0] = mdl->p[ESP_P_VX0];
}

/* fy_sat.m : fy = Calpha*mu/K*atan(K*alpha/mu) */
double esp_fy_sat(double Calpha, double mu, double K, double alpha)
{
    return Calpha * mu / K * atan(K * alpha / mu);
}

/* esplogic.m : Mz = Ke*(gammad - gamma) */
static double esp_gammad(const EspModel *mdl, double vx, double delf)
{
    const double *p = mdl->p;
    double Cf = p[ESP_P_CALPHA], Cr = p[ESP_P_CALPHA];
    double l   = p[ESP_P_LF] + p[ESP_P_LR];
    double num = vx * delf;
    double tmp = (p[ESP_P_LF] * Cf - p[ESP_P_LR] * Cr) / (Cf * Cr);
    double den = l - (p[ESP_P_M] * tmp * vx * vx) / (2.0 * l);
    return num / den;
}

double esp_logic(const EspModel *mdl, const double x[ESP_N_STATES], double delf)
{
    return mdl->p[ESP_P_KE] * (esp_gammad(mdl, x[0], delf) - x[2]);
}

/* steer1.m : t >= 2 s 에서 delf = 3 deg (steer_mode = 1), 아니면 외부 입력 */
double esp_steer(const EspModel *mdl, double t, double delf_in_rad)
{
    if (mdl->p[ESP_P_STEER_MODE] >= 0.5) {
        /* 1e-9: 통신 스텝 누적 오차로 step 시점이 한 스텝 밀리는 것 방지 */
        if (t >= mdl->p[ESP_P_STEER_TIME] - 1e-9)
            return mdl->p[ESP_P_STEER_AMP_DEG] * M_PI / 180.0;
        return 0.0;
    }
    return delf_in_rad;
}

static void tire_forces(const EspModel *mdl, const double x[ESP_N_STATES], double del_f,
                        double *alpha_f, double *alpha_r, double *Fyf, double *Fyr)
{
    const double *p = mdl->p;
    double af = atan2(x[1] + p[ESP_P_LF] * x[2], x[0]) - del_f;
    double ar = atan2(x[1] - p[ESP_P_LR] * x[2], x[0]);
    double fyf = esp_fy_sat(p[ESP_P_CALPHA], p[ESP_P_MU], p[ESP_P_K], af);
    double fyr = esp_fy_sat(p[ESP_P_CALPHA], p[ESP_P_MU], p[ESP_P_K], ar);
    fyf = -sgn(af) * fabs(fyf);
    fyr = -sgn(ar) * fabs(fyr);
    *alpha_f = af;
    *alpha_r = ar;
    *Fyf = 2.0 * fyf;
    *Fyr = 2.0 * fyr;
}

/* bicycle4.m (Ke = 0 이면 bicycle3.m 과 동일) */
void esp_deriv(const EspModel *mdl, double t, const double x[ESP_N_STATES],
               double delf_in_rad, double dx[ESP_N_STATES])
{
    const double *p = mdl->p;
    double del_f = esp_steer(mdl, t, delf_in_rad);
    double af, ar, Fyf, Fyr, Mz;

    tire_forces(mdl, x, del_f, &af, &ar, &Fyf, &Fyr);
    Mz = esp_logic(mdl, x, del_f);

    dx[0] = 0.0;
    dx[1] = (Fyf + Fyr) / p[ESP_P_M] - x[0] * x[2];
    dx[2] = (p[ESP_P_LF] * Fyf - p[ESP_P_LR] * Fyr + Mz) / p[ESP_P_IZZ];
    dx[3] = x[0] * cos(x[5]) - x[1] * sin(x[5]);
    dx[4] = x[0] * sin(x[5]) + x[1] * cos(x[5]);
    dx[5] = x[2];
}

int esp_advance(EspModel *mdl, double t, double h, double delf_in_rad)
{
    int n, k, i;
    double dt;
    double k1[ESP_N_STATES], k2[ESP_N_STATES], k3[ESP_N_STATES], k4[ESP_N_STATES];
    double xt[ESP_N_STATES];

    if (!(h > 0.0) || !isfinite(h))
        return 0;

    n = (int)ceil(h / ESP_DT_MAX - 1e-9);
    if (n < 1) n = 1;
    dt = h / (double)n;

    for (k = 0; k < n; k++) {
        double tk = t + (double)k * dt;
        double *x = mdl->x;

        esp_deriv(mdl, tk, x, delf_in_rad, k1);
        for (i = 0; i < ESP_N_STATES; i++) xt[i] = x[i] + 0.5 * dt * k1[i];
        esp_deriv(mdl, tk + 0.5 * dt, xt, delf_in_rad, k2);
        for (i = 0; i < ESP_N_STATES; i++) xt[i] = x[i] + 0.5 * dt * k2[i];
        esp_deriv(mdl, tk + 0.5 * dt, xt, delf_in_rad, k3);
        for (i = 0; i < ESP_N_STATES; i++) xt[i] = x[i] + dt * k3[i];
        esp_deriv(mdl, tk + dt, xt, delf_in_rad, k4);

        for (i = 0; i < ESP_N_STATES; i++) {
            x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            if (!isfinite(x[i]))
                return 0;
        }
    }
    return 1;
}

void esp_outputs(const EspModel *mdl, double t, double delf_in_rad, double y[ESP_N_OUTPUTS])
{
    const double *x = mdl->x;
    double del_f = esp_steer(mdl, t, delf_in_rad);
    double af, ar, Fyf, Fyr;

    tire_forces(mdl, x, del_f, &af, &ar, &Fyf, &Fyr);

    y[ESP_Y_VX]        = x[0];
    y[ESP_Y_VY]        = x[1];
    y[ESP_Y_GAMMA]     = x[2];
    y[ESP_Y_X]         = x[3];
    y[ESP_Y_Y]         = x[4];
    y[ESP_Y_PSI]       = x[5];
    y[ESP_Y_GAMMA_DEG] = x[2] * 180.0 / M_PI;
    y[ESP_Y_GAMMAD]    = esp_gammad(mdl, x[0], del_f);
    y[ESP_Y_MZ]        = esp_logic(mdl, x, del_f);
    y[ESP_Y_ALPHA_F]   = af;
    y[ESP_Y_ALPHA_R]   = ar;
    y[ESP_Y_FYF]       = Fyf;
    y[ESP_Y_FYR]       = Fyr;
    y[ESP_Y_BETA_DEG]  = atan2(x[1], x[0]) * 180.0 / M_PI;
    y[ESP_Y_AY]        = (Fyf + Fyr) / mdl->p[ESP_P_M];
    y[ESP_Y_R]         = (fabs(x[2]) > 1e-6) ? x[0] / x[2] : 0.0;
    y[ESP_Y_DELF]      = del_f;
}
