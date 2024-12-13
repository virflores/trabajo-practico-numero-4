from controller import Robot

AVANZAR = "Avanzar"
RETROCEDER = "Retroceder"
GIRAR = "Girar"

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Sensores de proximidad
sensor_adelante = robot.getDevice('ps0')  
sensor_izquierdo = robot.getDevice('ps5')  
sensor_derecho = robot.getDevice('ps2')  

sensor_adelante.enable(timestep)
sensor_izquierdo.enable(timestep)
sensor_derecho.enable(timestep)

# Sensor de color
sensor_color = robot.getDevice('color_sensor')
sensor_color.enable(timestep)

# LiDAR
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

# Giroscopio
giroscopio = robot.getDevice('gyro')
giroscopio.enable(timestep)

# Motores
motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')

motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))

estado_actual = AVANZAR
velocidad_base = 3.0
velocidad_maxima = 6.28
umbral_proximidad = 150

baldosas_azules = 0

# Funciones
def leer_sensores():
    return sensor_adelante.getValue(), sensor_izquierdo.getValue(), sensor_derecho.getValue()

def imprimir_telemetria(frontal, izquierdo, derecho):
    print(f"Telemetría: Frontal={frontal:.2f}, Izquierdo={izquierdo:.2f}, Derecho={derecho:.2f}")

def establecer_velocidades(vel_izquierda, vel_derecha):
    motor_izquierdo.setVelocity(vel_izquierda)
    motor_derecho.setVelocity(vel_derecha)

def girar(direccion, angulo_deseado):
    velocidad_giro = 2.0 if direccion == "izquierda" else -2.0
    establecer_velocidades(velocidad_giro, -velocidad_giro)

    angulo_acumulado = 0.0
    while robot.step(timestep) != -1:
        delta_angulo = giroscopio.getValues()[2] * (timestep / 1000.0)  
        angulo_acumulado += abs(delta_angulo)

        if angulo_acumulado >= angulo_deseado:
            break

    establecer_velocidades(0.0, 0.0)

def leer_color():
    rgb = sensor_color.getImage()  # datos RGB
    red = sensor_color.imageGetRed(rgb, 1, 0, 0)
    green = sensor_color.imageGetGreen(rgb, 1, 0, 0)
    blue = sensor_color.imageGetBlue(rgb, 1, 0, 0)
    return (red, green, blue)

def es_baldosa_azul(rgb):
    red, green, blue = rgb
    return 50 < red < 80 and 50 < green < 100 and 150 < blue < 255

def detectar_cartel():
    point_cloud = lidar.getPointCloud()
    for point in point_cloud:
        if 0.1 < point[0] < 0.5 and -0.2 < point[1] < 0.2:  
            print("Cartel detectado en posición cercana.")
            return True
    return False

def evitar_obstaculos():
    establecer_velocidades(-velocidad_base, -velocidad_base)
    robot.step(500) 
    girar("derecha", 1.57)  

# Cuerpo principal del código
while robot.step(timestep) != -1:
    frontal, izquierdo, derecho = leer_sensores()
    imprimir_telemetria(frontal, izquierdo, derecho)

    # Detección de baldosas
    rgb_actual = leer_color()
    if es_baldosa_azul(rgb_actual):
        baldosas_azules += 1
        print(f"Baldosa azul detectada. Total: {baldosas_azules}")

    # Detección de obstáculos
    if frontal > umbral_proximidad or izquierdo > umbral_proximidad or derecho > umbral_proximidad:
        evitar_obstaculos()
        continue

    # Detección de cartel
    if detectar_cartel():
        print("Cartel reportado.")

    # Movimiento principal
    if estado_actual == AVANZAR:
        print("Estado: Avanzar")
        establecer_velocidades(velocidad_base, velocidad_base)

print(f"Total de baldosas azules detectadas: {baldosas_azules}")
