void pid_seguelinha();//controlador PID
void lerSens();//determina a posicao do robo em relacao a linha

void motor(char lado, char dir , int pwm);//funcao para facilitar o controle dos motores - fica da forma motor(lado, direcao, velocidade)
void motorD_PWM(int vel);
void motorE_PWM(int vel);

void setColor(char sensor, int h, int s, int v);

void execute(char* cmd);