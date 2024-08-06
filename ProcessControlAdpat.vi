#include <stdio.h>
#include "NIDAQmx.h"  // Inclui a biblioteca NI-DAQmx para aquisição de dados

// Estrutura para o controlador PID adaptativo
typedef struct {
    double kp;
    double ki;
    double kd;
    double prev_error;
    double integral;
    double adaptive_factor;
} PIDController;

// Função para criar e inicializar um controlador PID
PIDController CreatePIDController(double kp, double ki, double kd, double adaptive_factor) {
    PIDController pid = {kp, ki, kd, 0.0, 0.0, adaptive_factor};
    return pid;
}

// Função para calcular a saída do controlador PID
double CalcPIDOutput(PIDController *pid, double setpoint, double measured_value) {
    double error = setpoint - measured_value;
    pid->integral += error;
    double derivative = error - pid->prev_error;
    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Ajuste adaptativo
    pid->kp += pid->adaptive_factor * error;
    pid->ki += pid->adaptive_factor * error * 0.01;
    pid->kd += pid->adaptive_factor * derivative * 0.01;

    pid->prev_error = error;
    return output;
}

// Função para aquisição de dados robusta
int ReadSensorData(TaskHandle *taskHandle, const char* channel, double *data) {
    int32 error;
    float64 dataBuffer[1];

    // Configurações do canal
    DAQmxCreateAIVoltageChan(*taskHandle, channel, "", DAQmx_Val_Cfg_Default, 0.0, 10.0, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(*taskHandle, "", 1000, DAQmx_Val_RisingEdge, DAQmx_Val_FiniteSamps, 1);

    // Inicia a tarefa de aquisição de dados
    DAQmxStartTask(*taskHandle);

    // Lê os dados do sensor
    error = DAQmxReadAnalogF64(*taskHandle, 1, 10.0, DAQmx_Val_GroupByScanNumber, dataBuffer, 1, NULL, NULL);
    if (error) {
        printf("Erro na leitura do sensor: %d\n", error);
        return -1;
    }
    *data = dataBuffer[0];

    // Finaliza a tarefa
    DAQmxStopTask(*taskHandle);
    return 0;
}

// Função para registrar dados em um arquivo
void LogData(double level, double temperature, double output) {
    FILE *file = fopen("tank_data_log.txt", "a");
    fprintf(file, "Level: %.2f, Temperature: %.2f, Output: %.2f\n", level, temperature, output);
    fclose(file);
}

int main() {
    TaskHandle myDAQ, myDAQ2;
    PIDController myPID;
    double level, temperature, output;
    double setpoint = 100;

    // Inicializa o controlador PID adaptativo
    myPID = CreatePIDController(1.0, 0.01, 0.001, 0.1);

    // Loop de controle principal
    while (1) {
        // Aquisição de dados para o nível do tanque
        DAQmxCreateTask("Aquisição de nível", &myDAQ);
        if (ReadSensorData(&myDAQ, "Dev1/ai0", &level) < 0) {
            // Se falhar na leitura, continue o loop
            continue;
        }

        // Aquisição de dados para a temperatura
        DAQmxCreateTask("Aquisição de temperatura", &myDAQ2);
        if (ReadSensorData(&myDAQ2, "Dev1/ai1", &temperature) < 0) {
            // Se falhar na leitura, continue o loop
            continue;
        }

        // Atualiza o setpoint com base na temperatura
        setpoint = 100 + 0.1 * temperature;

        // Calcula o sinal de controle
        output = CalcPIDOutput(&myPID, setpoint, level);

        // Verificação de segurança para o nível do tanque
        if (level < 10 || level > 190) {
            printf("Aviso: Nível do tanque crítico!\n");
            // Implementar lógica de segurança, como desligamento do sistema
            continue;
        }

        // Registro de dados
        LogData(level, temperature, output);

        // Atualização de saída do controlador (atuador hipotético)
        printf("Controle: Level: %.2f, Temperature: %.2f, Output: %.2f\n", level, temperature, output);

        // Espera antes do próximo ciclo de controle
        Sleep(1000);
    }

    return 0;
}
