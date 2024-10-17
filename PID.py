import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.signal import find_peaks

def analyze_transfer_function(num, den):
    # Создаем передаточную функцию
    system = ctrl.TransferFunction(num, den)
    
    # Вычисляем коэффициент усиления
    gain = num[0]  # Коэффициент усиления
    
    # Инициализация переменных
    time_constant = None
    delay_time = 0  # Предположим, что задержка равна 0 для простоты

    # Извлекаем параметры из знаменателя
    if len(den) == 3:  # Для второй степени
        a = den[0]  # 4
        b = den[1]  # 2
        c = den[2]  # 1
        
        # Собственная частота и коэффициент затухания
        omega_n = np.sqrt(c / a)  # Собственная частота
        zeta = b / (2 * np.sqrt(a * c))  # Коэффициент затухания
        
        # Постоянная времени (приближенно)
        time_constant = 1 / omega_n if omega_n != 0 else None
        
    # Генерируем временной вектор
    t = np.linspace(0, 20, 1000)
    
    # Получаем ответ системы на единичный ступенчатый сигнал
    t_out, y_out = ctrl.step_response(system, T=t)

    # Находим пики для определения периода колебаний
    peaks, _ = find_peaks(y_out)
    
    if len(peaks) >= 2:
        periods = np.diff(t_out[peaks])  # Разности между последовательными пиками
        average_period = np.mean(periods)  # Средний период колебаний
    else:
        average_period = None
    
    # Строим график
    plt.figure()
    plt.plot(t_out, y_out)
    plt.title('Ответ системы на единичный ступенчатый сигнал')
    plt.xlabel('Время (с)')
    plt.ylabel('Амплитуда')
    plt.grid()
    plt.show()
    
    return gain, time_constant, delay_time, average_period

def calculate_pid_parameters(gain, time_constant, zeta):
    if time_constant is None or zeta is None:
        raise ValueError("Необходимы корректные значения для постоянной времени и коэффициента затухания.")

    Kp = gain * (1 / zeta)  # Пропорциональный коэффициент
    Ti = time_constant / zeta  # Время интегрирования
    Ki = Kp / Ti              # Интегральный коэффициент
    Td = time_constant         # Время дифференцирования
    Kd = Kp * Td              # Дифференциальный коэффициент
    
    return Kp, Ki, Kd

# Пример использования функции для передаточной функции 1/(4s^2 + 2s + 1)
numerator = [1]  # Числитель передаточной функции (например, для H(s) = 1/(4s^2 + 2s + 1))
denominator = [4, 2, 1]  # Знаменатель передаточной функции

gain, time_constant, delay_time, average_period = analyze_transfer_function(numerator, denominator)

# Параметры PID регулятора
Kp, Ki, Kd = calculate_pid_parameters(gain, time_constant, (denominator[1] / (2 * np.sqrt(denominator[0] * denominator[2]))))

print(f'Коэффициент усиления: {gain}')
print(f'Постоянная времени: {time_constant}')
print(f'Время запаздывания: {delay_time}')
print(f'Период колебаний: {average_period}')
print(f'Параметры PID контроллера:')
print(f'Kp: {Kp}')
print(f'Ki: {Ki}')
print(f'Kd: {Kd}')

# Построение графика отклика системы с PID контроллером (добавьте эту часть позже)