/*
 * FMI 2.0 Co-Simulation wrapper : 차량 ESP 시스템 (bicycle model + ESP logic)
 *
 * Value reference
 *   0          : 입력  delf_deg (전륜 조향각 [deg], steer_mode = 0 일 때 사용)
 *   100 ~ 111  : 파라미터 (esp_core.h 의 ESP_P_* 순서)
 *   200 ~ 216  : 출력 (esp_core.h 의 ESP_Y_* 순서)
 */
#include "fmi2Functions.h"
#include "esp_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ESP_FMU_GUID "{3f6a1d52-7c0e-4b8a-9d41-e5b0c2a7f104}"
#define ESP_MODEL_IDENTIFIER "esp_bicycle_fmu"

#define VR_IN_BASE    0u
#define VR_PARAM_BASE 100u
#define VR_OUT_BASE   200u

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    char *instanceName;
    fmi2CallbackFunctions cb;
    fmi2Boolean loggingOn;
    fmi2Boolean initialized;
    fmi2Real time;
    fmi2Real inputs[ESP_N_INPUTS];
    fmi2Real outputs[ESP_N_OUTPUTS];
    EspModel model;
} FmuInstance;

static void* fmu_alloc(const fmi2CallbackFunctions *cb, size_t nobj, size_t size)
{
    if (cb && cb->allocateMemory)
        return cb->allocateMemory(nobj, size);
    return calloc(nobj, size);
}

static void fmu_free(const fmi2CallbackFunctions *cb, void *p)
{
    if (!p)
        return;
    if (cb && cb->freeMemory)
        cb->freeMemory(p);
    else
        free(p);
}

static char* fmu_strdup(const fmi2CallbackFunctions *cb, const char *s)
{
    size_t n;
    char *p;
    if (!s) s = "";
    n = strlen(s) + 1u;
    p = (char*)fmu_alloc(cb, n, sizeof(char));
    if (p)
        memcpy(p, s, n);
    return p;
}

static FmuInstance* asInstance(fmi2Component c)
{
    return (FmuInstance*)c;
}

static void logMessage(FmuInstance *inst, fmi2Status status, const char *category, const char *message)
{
    if (inst && inst->loggingOn && inst->cb.logger) {
        inst->cb.logger(inst->cb.componentEnvironment,
            inst->instanceName ? inst->instanceName : ESP_MODEL_IDENTIFIER,
            status,
            category ? category : "log",
            "%s", message ? message : "");
    }
}

static fmi2Status unsupported(FmuInstance *inst, const char *what)
{
    logMessage(inst, fmi2Error, "unsupported", what);
    return fmi2Error;
}

static double delfInputRad(const FmuInstance *inst)
{
    return inst->inputs[0] * M_PI / 180.0;
}

static void updateOutputs(FmuInstance *inst)
{
    esp_outputs(&inst->model, inst->time, delfInputRad(inst), inst->outputs);
}

const char* fmi2GetTypesPlatform(void)
{
    return fmi2TypesPlatform;
}

const char* fmi2GetVersion(void)
{
    return fmi2Version;
}

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[])
{
    FmuInstance *inst = asInstance(c);
    (void)nCategories;
    (void)categories;
    if (!inst)
        return fmi2Error;
    inst->loggingOn = loggingOn;
    return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
    fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
    FmuInstance *inst;

    (void)fmuResourceLocation;
    (void)visible;

    if (fmuType != fmi2CoSimulation)
        return NULL;
    if (fmuGUID && strcmp(fmuGUID, ESP_FMU_GUID) != 0)
        return NULL;

    inst = (FmuInstance*)fmu_alloc(functions, 1u, sizeof(FmuInstance));
    if (!inst)
        return NULL;
    memset(inst, 0, sizeof(*inst));

    if (functions)
        inst->cb = *functions;
    inst->loggingOn = loggingOn;

    inst->instanceName = fmu_strdup(functions, instanceName ? instanceName : ESP_MODEL_IDENTIFIER);
    if (!inst->instanceName) {
        fmu_free(functions, inst);
        return NULL;
    }

    esp_default_params(inst->model.p);
    esp_reset(&inst->model);
    inst->initialized = fmi2False;
    inst->time = 0.0;
    updateOutputs(inst);

    return inst;
}

void fmi2FreeInstance(fmi2Component c)
{
    FmuInstance *inst = asInstance(c);
    fmi2CallbackFunctions cb;
    if (!inst)
        return;
    cb = inst->cb;
    fmu_free(&cb, inst->instanceName);
    fmu_free(&cb, inst);
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
    fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    FmuInstance *inst = asInstance(c);
    (void)toleranceDefined;
    (void)tolerance;
    (void)stopTimeDefined;
    (void)stopTime;
    if (!inst)
        return fmi2Error;
    inst->time = startTime;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c)
{
    FmuInstance *inst = asInstance(c);
    if (!inst)
        return fmi2Error;
    esp_reset(&inst->model);
    updateOutputs(inst);
    return fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c)
{
    FmuInstance *inst = asInstance(c);
    if (!inst)
        return fmi2Error;
    /* 초기화 모드에서 설정된 파라미터(vx0 등)로 초기 상태를 확정 */
    esp_reset(&inst->model);
    updateOutputs(inst);
    inst->initialized = fmi2True;
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c)
{
    FmuInstance *inst = asInstance(c);
    if (!inst)
        return fmi2Error;
    inst->initialized = fmi2False;
    return fmi2OK;
}

fmi2Status fmi2Reset(fmi2Component c)
{
    FmuInstance *inst = asInstance(c);
    if (!inst)
        return fmi2Error;
    esp_default_params(inst->model.p);
    esp_reset(&inst->model);
    memset(inst->inputs, 0, sizeof(inst->inputs));
    inst->time = 0.0;
    inst->initialized = fmi2False;
    updateOutputs(inst);
    return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    FmuInstance *inst = asInstance(c);
    size_t i;
    if (!inst || !vr || !value)
        return fmi2Error;
    updateOutputs(inst);
    for (i = 0; i < nvr; i++) {
        if (vr[i] < VR_IN_BASE + ESP_N_INPUTS) {
            value[i] = inst->inputs[vr[i] - VR_IN_BASE];
        } else if (vr[i] >= VR_PARAM_BASE && vr[i] < VR_PARAM_BASE + ESP_N_PARAMS) {
            value[i] = inst->model.p[vr[i] - VR_PARAM_BASE];
        } else if (vr[i] >= VR_OUT_BASE && vr[i] < VR_OUT_BASE + ESP_N_OUTPUTS) {
            value[i] = inst->outputs[vr[i] - VR_OUT_BASE];
        } else {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    FmuInstance *inst = asInstance(c);
    size_t i;
    if (!inst || !vr || !value)
        return fmi2Error;
    for (i = 0; i < nvr; i++) {
        if (!isfinite(value[i]))
            return fmi2Error;
        if (vr[i] < VR_IN_BASE + ESP_N_INPUTS) {
            inst->inputs[vr[i] - VR_IN_BASE] = value[i];
        } else if (vr[i] >= VR_PARAM_BASE && vr[i] < VR_PARAM_BASE + ESP_N_PARAMS) {
            inst->model.p[vr[i] - VR_PARAM_BASE] = value[i];
        } else {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    FmuInstance *inst = asInstance(c);

    (void)noSetFMUStatePriorToCurrentPoint;

    if (!inst)
        return fmi2Error;

    if (communicationStepSize <= 0.0 || !isfinite(communicationStepSize)) {
        logMessage(inst, fmi2Error, "error", "communicationStepSize must be positive and finite.");
        return fmi2Error;
    }

    if (!esp_advance(&inst->model, currentCommunicationPoint, communicationStepSize, delfInputRad(inst))) {
        logMessage(inst, fmi2Error, "error", "esp_advance failed (non-finite state).");
        return fmi2Error;
    }

    inst->time = currentCommunicationPoint + communicationStepSize;
    updateOutputs(inst);
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c)
{
    return unsupported(asInstance(c), "fmi2CancelStep is not supported.");
}

fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
    const fmi2Integer order[], const fmi2Real value[])
{
    (void)vr; (void)nvr; (void)order; (void)value;
    return unsupported(asInstance(c), "Input derivatives are not supported.");
}

fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
    const fmi2Integer order[], fmi2Real value[])
{
    (void)vr; (void)nvr; (void)order; (void)value;
    return unsupported(asInstance(c), "Output derivatives are not supported.");
}

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
{
    (void)s;
    if (!c || !value)
        return fmi2Error;
    *value = fmi2OK;
    return fmi2OK;
}

fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
{
    FmuInstance *inst = asInstance(c);
    if (!inst || !value)
        return fmi2Error;
    if (s == fmi2LastSuccessfulTime) {
        *value = inst->time;
        return fmi2OK;
    }
    return fmi2Discard;
}

fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
{
    (void)c; (void)s; (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
{
    (void)c; (void)s; (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
{
    (void)c; (void)s; (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "Integer variables are not used.");
}

fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "Integer variables are not used.");
}

fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "Boolean variables are not used.");
}

fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "Boolean variables are not used.");
}

fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "String variables are not used.");
}

fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    (void)vr; (void)value;
    if (nvr == 0) return fmi2OK;
    return unsupported(asInstance(c), "String variables are not used.");
}

fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
{
    (void)FMUstate;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
{
    (void)FMUstate;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
{
    (void)FMUstate;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t* size)
{
    (void)FMUstate; (void)size;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
{
    (void)FMUstate; (void)serializedState; (void)size;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
{
    (void)serializedState; (void)size; (void)FMUstate;
    return unsupported(asInstance(c), "FMU state serialization is not supported.");
}

fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
    const fmi2ValueReference vKnown_ref[], size_t nKnown, const fmi2Real dvKnown[], fmi2Real dvUnknown[])
{
    (void)vUnknown_ref; (void)nUnknown; (void)vKnown_ref; (void)nKnown; (void)dvKnown; (void)dvUnknown;
    return unsupported(asInstance(c), "Directional derivatives are not supported.");
}
