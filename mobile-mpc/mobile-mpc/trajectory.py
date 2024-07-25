import numpy as np
import matplotlib.pyplot as plt


def select_path_points(path, current_state, n=5, lookahead_distance=1.0):
    """
    현재 위치에서 추종해야 할 path points를 n개 선택하는 함수.
    현재 위치에서 일정 거리 앞에 있는 점을 기준으로 선택.

    Args:
    path: array_like, shape (m, 3)
        경로 상의 포인트 리스트, 각 포인트는 (x, y, theta) 형식.
    current_state: tuple, shape (3,)
        현재 위치 상태, (x, y, theta) 형식.
    n: int, optional (default=5)
        선택할 path points의 개수.
    lookahead_distance: float, optional (default=1.0)
        현재 위치에서 앞으로의 거리.

    Returns:
    selected_points: array_like, shape (n, 3)
        선택된 path points 리스트.
    is_end_of_path: bool
        경로의 끝에 도달했는지 여부.
    """
    path = np.array(path)
    current_x, current_y, current_theta = current_state

    # Lookahead 지점 계산
    lookahead_x = current_x + lookahead_distance * np.cos(current_theta)
    lookahead_y = current_y + lookahead_distance * np.sin(current_theta)

    # lookahead 지점과 path 상의 각 점 사이의 거리 계산
    distances = np.linalg.norm(path[:, :2] - np.array([lookahead_x, lookahead_y]), axis=1)

    # 가장 가까운 점의 인덱스 찾기
    closest_index = np.argmin(distances)

    # 선택할 점들의 인덱스 계산
    start_index = closest_index
    end_index = min(len(path), start_index + n)

    # 선택된 점들의 리스트 반환
    selected_points = path[start_index:end_index]

    # 경로의 끝에 도달했는지 여부
    is_end_of_path = (end_index == len(path))

    return selected_points, is_end_of_path


def generate_sin_path(amplitude=1, frequency=1, length=10, num_points=100):
    """
    Sin 형태의 경로를 생성하는 함수.

    Args:
    amplitude: float, optional (default=1)
        Sin 파형의 진폭.
    frequency: float, optional (default=1)
        Sin 파형의 주파수.
    length: float, optional (default=10)
        경로의 길이.
    num_points: int, optional (default=100)
        경로 상의 포인트 개수.

    Returns:
    path: array_like, shape (num_points, 3)
        생성된 경로의 포인트 리스트.
    """
    x = np.linspace(0, length, num_points)
    y = amplitude * np.sin(frequency * x)
    theta = np.arctan2(np.gradient(y), np.gradient(x))
    path = np.stack((x, y, theta), axis=-1)
    return path


def generate_random_curve_path(length=10, num_points=100):
    """
    임의의 곡선 경로를 생성하는 함수.

    Args:
    length: float, optional (default=10)
        경로의 길이.
    num_points: int, optional (default=100)
        경로 상의 포인트 개수.

    Returns:
    path: array_like, shape (num_points, 3)
        생성된 경로의 포인트 리스트.
    """
    x = np.linspace(0, length, num_points)
    y = np.cumsum(np.random.randn(num_points))
    theta = np.arctan2(np.gradient(y), np.gradient(x))
    path = np.stack((x, y, theta), axis=-1)
    return path

def plot_path(path, title="Path"):
    """
    경로를 시각화하는 함수.

    Args:
    path: array_like, shape (n, 3)
        경로 상의 포인트 리스트.
    title: str, optional (default="Path")
        플롯의 제목.
    """
    x = path[:, 0]
    y = path[:, 1]
    
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, marker='o')
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # 예제 경로 생성 및 시각화
    sin_path = generate_sin_path(amplitude=1, frequency=2, length=10, num_points=100)
    random_curve_path = generate_random_curve_path(length=10, num_points=100)
    print(sin_path)
    # 경로 시각화
    plot_path(sin_path, title="Sin Path")
    plot_path(random_curve_path, title="Random Curve Path")