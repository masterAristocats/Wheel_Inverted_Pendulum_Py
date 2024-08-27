# -*- coding: micropython -*-

from src import Robot, Accelerometer, Gyro
import machine
import utime
import math

TIME_STEP = 16

# Limita o acionamento do motor
MAX_SPEED = 10

# Constante do filtro da leitura do ângulo
FILTER_CONSTANT = 0.999

# Constantes do controlador
GANHO_P = 0.5
GANHO_D = 0.8
GANHO_I = 0.005

# Inicializa os pinos do driver A4988
step_pin = machine.Pin(18, machine.Pin.OUT)
direction_pin = machine.Pin(19, machine.Pin.OUT)

def step_motor(direction, steps):
    # Set direction pin high or low depending on the direction
    if direction == 1:
        direction_pin.value(1)
    else:
        direction_pin.value(0)
    
    # Send step signal
    for i in range(steps):
        step_pin.value(1)
        utime.sleep_ms(1)  # adjust the delay according to your motor's requirements
        step_pin.value(0)
        utime.sleep_ms(1)

def get_angle(angulo_antigo: float, first_read: bool = False) -> float:
    '''Lê o acelerômetro e o giroscópio e descobre a inclinação

        :param angulo_antigo: valor do último ângulo filtrado lido
        :param first_read: se é a primeira leitura ou não (usado para ignorar o filtro)
        :returns: angulo filtrado no formato
    '''
    # TO DO: implementar a leitura do acelerômetro e giroscópio
    # x, z, y = acel.getValues()
    # gx, gy, gz = gyro.getValues()
    # ...
    
    # Usar uma das duas fórmulas
    # Para o meu caso foi a segunda
    #angulo_bruto = atan(x/(sqrt(y**2 + z**2))) * 180/math.pi
    angulo_bruto  = atan(y/(sqrt(x**2 + z**2))) * 180.0/math.pi
    
    # A primeira medição apenas retorna o ângulo medido direto
    if first_read:
        return angulo_bruto
            
    # gx *= -180.0/math.pi
    # gx *= TIME_STEP/1000.0
        
    # Formula usada para filrar leitura de ângulo
    angulo_filtrado = FILTER_CONSTANT*(angulo_antigo + gx) + (1-FILTER_CONSTANT)*angulo_bruto
            
    return angulo_filtrado


# Força a inicialização pro sensor retornar um numero valido
# Obs: O acelerômetro só inicializa depois de 32ms
utime.sleep_ms(32)

angulo = get_angle(0, first_read = True)
print("Ângulo inicial: ", angulo)

erro = -angulo
erro_acumulado = 0


while True:
    angulo = get_angle(angulo)
                
    # Controle PID
    erro_diferenca = angulo - erro
    erro = angulo
    erro_acumulado += erro
    
    controle_motor = GANHO_P * erro + GANHO_D * erro_diferenca + GANHO_I * erro_acumulado
    
    # Calculate the number of steps based on the controle_motor value
    steps = int(controle_motor * 10)  # adjust the scaling factor according to your motor's requirements
    
    # Send step and direction signals to the driver
    direction = 1 if controle_motor > 0 else -1
    step_motor(direction, abs(steps))
    
    print(angulo, controle_motor, erro, erro_diferenca, erro_acumulado)