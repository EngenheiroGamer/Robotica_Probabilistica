import numpy as np
import math

def algorithm(current_position, goal, lidar_data, angle_min, angle_increment, current_orientation):

    # Parâmetros de modelagem
    k_atrativo = 1.0    # Atração para o objetivo
    goal_threshold = 0.5  # Distância mínima para considerar o objetivo alcançado
    max_linear_speed = 0.5  # Velocidade linear máxima
    max_angular_speed = 0.4 # Velocidade angular máxima

    # Se já estiver próximo do objetivo, para
    if abs(current_position.x - goal[0]) < goal_threshold and abs(current_position.y - goal[1]) < goal_threshold:
        return 0.0, 0.0

    # Vetor para o objetivo (campo atrativo)
    dx = goal[0] - current_position.x
    dy = goal[1] - current_position.y
    distance_to_goal = math.sqrt(dx**2 + dy**2)
    print(dx, dy)
    total_field = np.array([k_atrativo * dx / distance_to_goal, k_atrativo * dy / distance_to_goal])

    # Cálculo do ângulo desejado baseado no campo resultante
    desired_angle = math.atan2(total_field[1], total_field[0])
    current_angle = current_orientation

    # Cálculo da diferença de ângulo ajustada para o intervalo [-pi, pi]
    angle_diff = ((desired_angle - current_angle + np.pi) % (2 * np.pi)) - np.pi

    # Velocidade linear proporcional à magnitude do vetor resultante
    linear_velocity = np.clip(np.linalg.norm(total_field), 0, max_linear_speed)

    # Velocidade angular proporcional à diferença de ângulo, sem forçar realinhamento
    angular_velocity = np.clip(angle_diff, -max_angular_speed, max_angular_speed)

    # Debug: Verificar os vetores de força e as velocidades
    # print(f"Total field: {total_field}")
    print(f"Linear velocity: {linear_velocity}, Angular velocity: {angular_velocity}")

    return linear_velocity, angular_velocity