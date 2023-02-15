#include <SoftwareSerial.h>
#define rxPin 10
#define txPin 9
    // Pinos de controle do Motor
    const int motorDirPin = 5; // Input 1
const int motorPWMPin = 6;     // Input 2
const int EnablePin = 8;       // Enable
const int LED = 13;
// Pino de encoder
const int encoderPinA = 2;
const int encoderPinB = 3;
int encoderPos = 0;
// encoder value change motor turn angles
const float ratio = 360. / 188.611 / 48.;
// 360. -> 1 turn
// 188.611 -> Gear Ratio
// 48. -> Encoder : Countable Events Per Revolution ( Motor Shaft )
// PID
float Kp = 20;      // Proporcional
float Ki = 0.00001; // Integrativo
float Kd = 0.0000;  // Derivativo
float target;       // Posicao desejada
float last_target = 0;
float last_error = 0; // Armazenamento do erro anterior de PID
float Ierror;         // Erro integral
float Derror;         // Erro derivativo
// Tempo
unsigned long current_time; // Momento atual de calculo do PID
unsigned long last_time;    // Ultimo momento calculado do PID
// Comunicacao modbus
char c;
String recData;
int i = 0;
String resposta = ":";
int setpoint_char[4];
int dec_lrc[4];
int lrc;
int lrc_bin[16];
String sendData = "";
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
int setPoint = 0;
int newSetPoint = 1;
int nova = 0;
// MODBUS
bool condicoes_ok(String recData)
{
    if (!check_slave(recData))
    {
        return false;
    }
    return true;
}
bool check_slave(String recData)
{
    char c = recData[2];
    if (c == '1')
    {
        return true;
    }
    return false;
}
int BintoDec(int val1, int val2, int val3, int val4)
{
    int result = 0;
    result += val4;
    result += val3 * 2;
    result += val2 * 4;
    result += val1 * 8;
    return result;
}
String formataResposta(String resposta, int setpoint)
{
    int s = 0;
    // 1 e 2 - Slave Adress (01)
    resposta += "0";
    resposta += "1";
    // 3 e 4 - Function Code (06)
    resposta += "0";
    resposta += "6";
    // 5 ,6 ,7 ,8 - Controlador (0001)
    resposta += "0";
    resposta += "0";
    resposta += "0";
    resposta += "1";
    // 9 ,10 ,11 ,12 - Setpoint ( ABCD )
    resposta += "00";
    resposta += setpoint;
    // 13 ,14 ,15 ,16 - LRC
    lrc = resposta[1] ^ resposta[2] ^ resposta[3] ^ resposta[4] ^ resposta[5] ^
          resposta[6] ^ resposta[7] ^
          resposta[8] ^ resposta[9] ^ resposta[10] ^ resposta[11] ^ resposta[12];
    for (int k = 0; lrc > 0; k++)
    {
        lrc_bin[k] = lrc % 2;
        lrc = lrc / 2;
    }
    int dec_lrc[4];
    dec_lrc[3] = BintoDec(lrc_bin[3], lrc_bin[2], lrc_bin[1], lrc_bin[0]); // lsb
    dec_lrc[2] = BintoDec(lrc_bin[7], lrc_bin[6], lrc_bin[5], lrc_bin[4]);
    dec_lrc[1] = BintoDec(lrc_bin[11], lrc_bin[10], lrc_bin[9], lrc_bin[8]);
    dec_lrc[0] = BintoDec(lrc_bin[12], lrc_bin[11], lrc_bin[10], lrc_bin[9]); // msb
    int m = 0;
    for (int j = 13; j <= 16; j++)
    {
        if (dec_lrc[m] > 10)
        {
            resposta += dec_lrc[m];
        }
        else
        {
            resposta += dec_lrc[m];
        }
        m++;
    }
    // 17 e 18 - CR ,LF
    resposta += '\r';
    resposta += '\n';
    return resposta;
}
// TASK1
#define INTERVALO1 50 // Tarefa1 a cada 0.005 s
void task1()
{
    // Print na tela a posicao do motor
    Serial.println(target);
    Serial.println(float(encoderPos) * ratio);
} // task1
// TASK2
#define INTERVALO2 50 // Tarefa2 a cada 0.005 s
void task2()
{
    // Verifica posicao atual e ajusta com PID
    current_time = millis();                    // Atualiza instante atual
    float motorDeg = float(encoderPos) * ratio; // Posicao atual do motor
    int dt = current_time - last_time;          // Diferenca de tempo entre os ajustes
    float error = target - motorDeg;            // Erro de controle
    Ierror += error;                            // Incrementa erro integral
    Derror = error - last_error;                // Atualiza erro derivativo
    float control = (Kp * error) +              // Calcula output decontrole
                    (Ki * Ierror * dt) +
                    (Kp * Kd * Derror / dt);
    digitalWrite(EnablePin, 255); // Seta comando pro motor
    doMotor((control >= 0) ? HIGH : LOW, min(abs(control), 255));
    last_target = target;
    last_error = error;       // Atualiza o erro de controle
    last_time = current_time; // Atualiza o instante de controle
} // task2
#define INTERVALO3 200
void task3()
{
    if (mySerial.available() > 0)
    {
        recData = "";
        while (mySerial.available())
        {
            c = mySerial.read();
            recData += c;
            nova = 1;
        }
        if (nova == 1 && condicoes_ok(recData))
        {
            Serial.println("SS Data received = " + recData);
            setpoint_char[3] = recData[9] - 48;
            setpoint_char[2] = recData[10] - 48;
            setpoint_char[1] = recData[11] - 48;
            setpoint_char[0] = recData[12] - 48;
            for (int m = 3; m >= 0; m--)
            {
                setPoint = 10 * setPoint + setpoint_char[m];
            }
            sendData = formataResposta(resposta, setPoint);
            resposta = ":";
            newSetPoint = setPoint;
            Serial.println(" Transmiting : " + sendData);
            mySerial.print(sendData);
            sendData = "";
            setPoint = 0;
        }
        nova = 0;
    }
}
// TASK SWITCHER
typedef struct
{
    void (*task)();
    long interval;
    long current_time;
    int status;
} TaskControl;
#define MAX_TASKS 3
#define READY 1
#define WAIT 0
TaskControl taskList[MAX_TASKS];
void createTask(int taskNum, void (*t)(), long interval)
{
    // Cria uma task
    taskList[taskNum].task = t;
    taskList[taskNum].interval = interval;
    taskList[taskNum].current_time = 0;
    taskList[taskNum].status = WAIT;
} // createTask
void runCurrentTask()
{
    // Executa a task atual
    int i;
    void (*task)();
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (taskList[i].status == READY)
        {
            task = taskList[i].task;
            (*task)();
            noInterrupts();
            taskList[i].status = WAIT;
            taskList[i].current_time = 0;
            interrupts();
        } // if task is READY
    }     // for each task
} // runCurrentTask
void updateTickCounter()
{
    // Atualiza o contador para os intervalos
    // das tasks
    int i;
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (taskList[i].status == WAIT)
        {
            taskList[i].current_time++;
            if (taskList[i].current_time >= taskList[i].interval)
            {
                taskList[i].status = READY;
            }
        } // if task is WAITing
    }     // for each task
} // updateTickCounter
// ===========================
// SETUP
void setup()
{
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    // set the data rate for the SoftwareSerial port
    mySerial.begin(2400);
    Serial.begin(2400);
    // Configura os pinos
    pinMode(encoderPinA, INPUT_PULLUP);
    attachInterrupt(0, doEncoderA, CHANGE);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(1, doEncoderB, CHANGE);
    pinMode(LED, OUTPUT);
    pinMode(motorDirPin, OUTPUT);
    pinMode(EnablePin, OUTPUT);
    // Cria as tarefas
    createTask(0, &task1, INTERVALO1);
    createTask(1, &task2, INTERVALO2);
    createTask(2, &task3, INTERVALO3);
    // Inicia interrupcao do timer
    // para 1ms
    setTimerInterrupt(1000); // int @1ms (1000 us)
}
void loop()
{
    // target = 10; // Setando posicao desejada para o motor
    if (target != newSetPoint)
    {
        target = newSetPoint;
    }
    runCurrentTask(); // executa tarefa atual
}
// Interrupcao do Timer
ISR(TIMER1_COMPA_vect)
{
    updateTickCounter();
} // ISR
// set up timer1 -
// compare interrupt @ uSecs microseconds
void setTimerInterrupt(long uSecs)
{
    noInterrupts(); // Desabilita interrupcoes
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // compare match register 16 MHz /256 * t(s) - 1
    OCR1A = (16e6 / 256L * uSecs) / 1e6 - 1;
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    interrupts();            // enable all interrupts
}
void doEncoderA()
{
    encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}
void doEncoderB()
{
    encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}
void doMotor(bool dir, int vel)
{
    digitalWrite(motorDirPin, dir);
    digitalWrite(LED, dir);
    analogWrite(motorPWMPin, dir ? (255 - vel) : vel);
}
