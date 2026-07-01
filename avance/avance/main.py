from machine import Pin
from config.config import CONFIG
from robot import get_robot
from utils.communication import CommunicationManager
from config.student_config import COMMUNICATION_CFG
import uasyncio
import utime
import ujson

# ====================================================
# ================= ROBOT SETUP ======================
# ====================================================
ROBOT_TYPE = 'mecanum'
led = Pin("LED", Pin.OUT)
robot = get_robot(ROBOT_TYPE, CONFIG[ROBOT_TYPE])
robot.set_control_mode(closed_loop = False)
comm = CommunicationManager(COMMUNICATION_CFG)
led.on()
# ====================================================
# =================  COMM FUNCTIONS ==================
# ====================================================
async def publisher_task():
    """Lectura de sensores y publicación de datos """
    
    # Creamos dos canales diferentes (cambia '00')
    topic_simple = "R04/sensors/simple"
    topic_json = "R04/sensors/json"
    
    while True:
        theta = robot.sensors.read_imu_theta() [cite: 246]
        dist_center = robot.sensors.read_ultrasonic_value('center') [cite: 298]
        dist_left = robot.sensors.read_ultrasonic_value('left') [cite: 299]
        dist_right = robot.sensors.read_ultrasonic_value('right') [cite: 299]
        
        # 1. El dato de orientación va por el canal simple
        comm.publish(topic_simple, str(theta)) [cite: 247]
        
        # 2. El paquete completo va por el canal JSON
        payload = ujson.dumps({ [cite: 302]
            "timestamp": uasyncio.ticks_ms(),
            "theta": theta, [cite: 304]
            "dist_center": dist_center, [cite: 305]
            "dist_left": dist_left, [cite: 306]
            "dist_right": dist_right [cite: 307]
        })
        comm.publish(topic_json, payload) [cite: 308]
        
        await uasyncio.sleep_ms(50) [cite: 311]

async def subscription_task():
    """Suscripción a mensajes"""
    topic_subs = "R04/teleop/cmd" 
    
    comm.add_callback(topic_subs, msgCallbackDict) [cite: 488]
    comm.subscribe(topic_subs) [cite: 489]
    
    while True:
        comm.check_incoming() [cite: 491]
        await uasyncio.sleep_ms(10) [cite: 492]

def msgCallback(msg):
    """Función para controlar movimiento de robot"""
    
    print(f"Comando recibido: {msg}", end="\r")
    robot.simple_teleop(msg, 1.0,1.5)
    led.toggle()
    
def msgCallbackDict(msg):
    """Función para controlar movimiento y velocidad de robot"""
    
    try:
        data = ujson.loads(msg)
        cmd = data["cmd"]
        speed = data["speed"]
        print(f"Comando recibido: {cmd}, velocidad: {speed}", end="\r")
        robot.simple_teleop(cmd, int(speed))
    except Exception as e:
        print("Error en callback:", e)

async def main_async():
    """Función principal asíncrona"""
    # Crear tareas concurrentes
    tasks = [
        uasyncio.create_task(publisher_task()),
        uasyncio.create_task(subscription_task())
    ]
    
    # Ejecutar todas las tareas indefinidamente
    await uasyncio.gather(*tasks)

# ====================================================
# ================= ENTRY POINT ======================
# ====================================================
if __name__ == "__main__":

    try:
        uasyncio.run(main_async())
    except KeyboardInterrupt:
        print("Programa terminado")
    finally:
        robot.stop()