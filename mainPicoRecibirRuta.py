# =============================================================================
# Jose_main_picoW.py  —  LADO PICO W (reemplaza main.py en el carrito)
# Tarea de José: Carrito recibe ruta y la ejecuta punto por punto con G2G
#
# ¿Qué hace?
#   1. Se conecta al broker MQTT
#   2. Escucha el topic R00/teleop/ruta esperando la ruta de la laptop
#   3. Al recibirla, ejecuta Go-to-Goal para cada punto de la ruta en orden
#   4. Al terminar, publica "done" de vuelta a la laptop
#
# ¿Cómo usar?
#   - Subir este archivo al Pico W como main.py (reemplaza el anterior)
#   - El ROBOT_ID debe coincidir con el que usa la laptop
#   - Asegúrate de que student_config.py tenga la IP del broker y WiFi correctos
# =============================================================================

from machine import Pin
from config.config import CONFIG
from robot import get_robot
from utils.communication import CommunicationManager
from config.student_config import COMMUNICATION_CFG
from utils.pose_estimator import Pose
from utils.goto_controller import (
    control_velocity_to_position,
    control_velocity_to_orientation,
    calculate_direction_goal
)
from utils.transforms import global_to_robot_frame
import uasyncio
import utime
import ujson

# ─── CONFIGURACIÓN ─────────────────────────────────────────────
ROBOT_ID = "R00"          # ← CAMBIAR al número de su robot el día de la presentación

TOPIC_RUTA   = f"{ROBOT_ID}/teleop/ruta"     # Laptop → Pico W
TOPIC_STATUS = f"{ROBOT_ID}/sensors/status"  # Pico W → Laptop
TOPIC_THETA  = f"{ROBOT_ID}/sensors/imu/theta"

TOLERANCIA_POS = 0.06   # metros — qué tan cerca debe llegar a cada punto
GAIN_POS       = 0.5    # ganancia Go-to-Goal posición
SAMPLE_US      = 10000  # periodo control en microsegundos (10 ms)

# PID de los motores (los mismos que tenían calibrados)
PID = [
    (0, 17.718, 0),  # motor 0
    (0, 17.1,   0),  # motor 1
    (0, 16.64,  0),  # motor 2
    (0, 17.606, 0),  # motor 3
]
# ───────────────────────────────────────────────────────────────

# ─── ESTADO GLOBAL ─────────────────────────────────────────────
led             = Pin("LED", Pin.OUT)
ROBOT_TYPE      = 'mecanum'
robot           = get_robot(ROBOT_TYPE, CONFIG[ROBOT_TYPE])
comm            = CommunicationManager(COMMUNICATION_CFG)

ruta_pendiente  = None   # Lista de [x, y] recibida desde la laptop
ejecutando_ruta = False  # True mientras el robot sigue la ruta
# ───────────────────────────────────────────────────────────────


# ─── SETUP ROBOT ───────────────────────────────────────────────
def setup_robot():
    robot.set_control_mode(closed_loop=True)
    for i, (kp, ki, kd) in enumerate(PID):
        robot.set_pid_constants(motor_index=i, kp=kp, ki=ki, kd=kd)
    robot.enable_auto_update()
    robot.enable_auto_update_odometry(interval=10)
    led.on()
    print("[ROBOT] Setup completado.")


# ─── CALLBACK MQTT: recibe la ruta desde la laptop ─────────────
def callback_ruta(msg):
    global ruta_pendiente, ejecutando_ruta

    if ejecutando_ruta:
        print("[WARN] Ruta recibida pero ya hay una en curso. Ignorando.")
        return

    if isinstance(msg, bytes):
        msg = msg.decode('utf-8')

    try:
        datos = ujson.loads(msg)
        ruta  = datos["ruta"]          # lista de [x, y] en metros
        print(f"[RUTA] Recibida: {len(ruta)} puntos")
        print(f"       Primer punto: {ruta[0]}")
        print(f"       Último punto: {ruta[-1]}")
        ruta_pendiente = ruta
    except Exception as e:
        print(f"[ERROR] al parsear ruta: {e}")


# ─── TAREA: seguir la ruta punto por punto ─────────────────────
async def ejecutar_ruta(ruta):
    global ejecutando_ruta
    ejecutando_ruta = True

    print(f"\n[G2G] Iniciando recorrido de {len(ruta)} puntos...")
    prevT = utime.ticks_us()

    for idx, punto in enumerate(ruta):
        gx, gy = punto[0], punto[1]
        goal   = Pose(x=gx, y=gy, theta=0)
        print(f"  → Punto {idx+1}/{len(ruta)}: ({gx}, {gy})")

        # Moverse hacia este punto con Go-to-Goal
        while True:
            currT = utime.ticks_us()
            dt    = utime.ticks_diff(currT, prevT)

            if dt >= SAMPLE_US:
                prevT = currT
                robot.update_odometry()

                # Calcula velocidades hacia el punto objetivo
                velocities = control_velocity_to_position(
                    robot.pose, gx, gy, gain=GAIN_POS
                )
                vx, vy, vw = global_to_robot_frame(velocities, robot.pose.theta)
                robot.move(vx, vy, vw)

                # Verificar si llegó al punto (tolerancia)
                ex = abs(gx - robot.pose.x)
                ey = abs(gy - robot.pose.y)
                if ex <= TOLERANCIA_POS and ey <= TOLERANCIA_POS:
                    break

            await uasyncio.sleep_ms(1)  # ceder control al event loop

        print(f"    ✓ Punto {idx+1} alcanzado.")

    # Terminó toda la ruta
    robot.stop()
    print("\n[G2G] Recorrido completo. Publicando 'done'...")
    comm.publish(TOPIC_STATUS, "done")
    led.toggle()
    ejecutando_ruta = False


# ─── TAREA: publicar theta periódicamente ─────────────────────
async def publisher_task():
    while True:
        theta = robot.sensors.read_imu_theta()
        comm.publish(TOPIC_THETA, str(theta))
        await uasyncio.sleep_ms(50)


# ─── TAREA: escuchar mensajes MQTT ─────────────────────────────
async def subscription_task():
    comm.add_callback(TOPIC_RUTA, callback_ruta)
    comm.subscribe(TOPIC_RUTA)
    print(f"[MQTT] Suscrito a: {TOPIC_RUTA}")
    print("[MQTT] Esperando ruta desde la laptop...")

    while True:
        comm.check_incoming()
        await uasyncio.sleep_ms(10)


# ─── TAREA: vigilar si llegó una ruta nueva y ejecutarla ───────
async def watcher_task():
    global ruta_pendiente
    while True:
        if ruta_pendiente is not None and not ejecutando_ruta:
            ruta = ruta_pendiente
            ruta_pendiente = None
            await ejecutar_ruta(ruta)
        await uasyncio.sleep_ms(100)


# ─── MAIN ──────────────────────────────────────────────────────
async def main_async():
    setup_robot()
    tasks = [
        uasyncio.create_task(publisher_task()),
        uasyncio.create_task(subscription_task()),
        uasyncio.create_task(watcher_task()),
    ]
    await uasyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        uasyncio.run(main_async())
    except KeyboardInterrupt:
        print("Programa terminado")
    finally:
        robot.stop()
