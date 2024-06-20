#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class CollisionAvoidance:
    def __init__(self):
        rospy.init_node('homework_node')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.5
        self.safe_distance = 0.3

        self.twist = Twist()
        self.collision_detected = False
        self.last_direction = 0 # 1 для прямого; -1 для обратного; 0 для исходного состояния

        # Добавлен атрибут для хранения направления препятствия
        self.obstacle_direction = 0

    def scan_callback(self, data):
        # Берем 10 ближайших точек лидара
        closest_points = sorted(data.ranges)[:10]
        
        # Находим направление препятствия
        min_index = data.ranges.index(min(closest_points))
        angle = data.angle_min + min_index * data.angle_increment
        self.obstacle_direction = math.degrees(angle)

        # Проверяем, нет ли препятствий на безопасном расстоянии
        if min(closest_points) < self.safe_distance and not self.collision_detected:
            # Немедленно останавливаем робота
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.logwarn("Обнаружено препятствие! Остановка.")

            self.collision_detected = True

        elif min(closest_points) >= self.safe_distance and self.collision_detected:
            # Останавливаем робота и сообщаем о нахождении в безопасной зоне
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("Нахожусь в безопасной зоне! Ожидаю новых команд.")
            self.collision_detected = False

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            key = input("Введите команду (w: вперед, s: назад, a: влево, d: вправо, q: выход/остановка ноды): ")

            if key == 'w' and self.collision_detected:
                rospy.logwarn("Невозможно двигаться вперед, так как обнаружено препятствие.")
            elif key == 'w':
                self.twist.linear.x = self.max_linear_speed
                self.twist.angular.z = 0
                self.last_direction = 1  # Двигаемся вперед
            elif key == 's':
                # Добавлена проверка на направление препятствия при движении назад
                if abs(self.obstacle_direction) < 45 or abs(self.obstacle_direction - 180) < 45:
                    rospy.logwarn("Невозможно двигаться назад, так как обнаружено препятствие.")
                else:
                    self.twist.linear.x = -self.max_linear_speed
                    self.twist.angular.z = 0
                    self.last_direction = -1  # Двигаемся назад
            elif key == 'a':
                self.twist.linear.x = 0
                self.twist.angular.z = self.max_angular_speed
            elif key == 'd':
                self.twist.linear.x = 0
                self.twist.angular.z = -self.max_angular_speed
            elif key == 'q':
                break
            else:
                rospy.logwarn("Недопустимая комманда!")

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        ca = CollisionAvoidance()
        ca.run()
    except rospy.ROSInterruptException:
        pass