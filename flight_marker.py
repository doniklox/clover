import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Инициализация ROS-узла
rospy.init_node('flight')

# Инициализация сервисов Clover
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Инициализация CvBridge для работы с изображениями
bridge = CvBridge()

# Целевой цвет для посадки
TARGET_COLOR = 'red'  # Цвет, на который дрон должен сесть

# Определение диапазонов цветов в HSV
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'green': ([35, 100, 100], [85, 255, 255]),
    'blue': ([100, 100, 100], [130, 255, 255]),
}

# Текущий целевой маркер
current_marker_id = 60  # Начинаем с маркера 0
max_marker_id = 99      # Максимальный ID маркера (первые 3 маркера: 0, 1, 2)
target_color_detected = False  # Флаг обнаружения целевого цвета

# Функция для обработки изображения с камеры
def image_callback(msg):
    global target_color_detected
    try:
        # Преобразование ROS-сообщения в изображение OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Преобразование изображения в цветовое пространство HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Поиск цветов на изображении
        for color, (lower, upper) in color_ranges.items():
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            
            # Создание маски для цвета
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # Поиск контуров на маске
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Если цвет обнаружен
            if contours:
                print(f"Detected {color} color.")
                
                # Если обнаружен целевой цвет
                if color == TARGET_COLOR:
                    print(f"Target color {TARGET_COLOR} detected. Landing immediately...")
                    target_color_detected = True
                    land()  # Немедленная посадка
                    rospy.signal_shutdown("Target color detected. Landing completed.")  # Завершение программы
        
    except Exception as e:
        print(f"Error in image processing: {e}")

# Подписка на топик камеры
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

# Взлет и зависание на высоте 1.5 метра
print('Take off and hover 1.5 m above the ground')
navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
rospy.sleep(5)

# Облет первых трех маркеров по порядку
for marker_id in range(current_marker_id, max_marker_id + 1):
    if target_color_detected:
        break  # Прерываем облет, если целевой цвет обнаружен
    
    print(f"Moving to ArUco marker {marker_id}")
    navigate(x=0, y=0, z=1.5, frame_id=f'aruco_{marker_id}')
    rospy.sleep(5)  # Ожидание завершения навигации

    # Поиск цвета после каждого перемещения
    if target_color_detected:
        break  # Прерываем облет, если целевой цвет обнаружен

# Посадка, если целевой цвет не был обнаружен
if not target_color_detected:
    print("Target color not detected. Landing at the last waypoint.")
    land()

# Основной цикл
rospy.spin()
