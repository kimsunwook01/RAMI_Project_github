import numpy as np
import math

class OccupancyGridMap:
    def __init__(self, width_m=30.0, height_m=30.0, resolution=0.05):
        """
        :param width_m: 맵의 가로 크기 (미터)
        :param height_m: 맵의 세로 크기 (미터)
        :param resolution: 격자 하나당 실제 크기 (미터)
        """
        self.resolution = resolution
        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)
        
        # 맵의 중심을 (0,0)으로 가정하기 위한 오프셋
        self.origin_x = self.width_cells // 2
        self.origin_y = self.height_cells // 2
        
        # 로그 오즈(Log-odds) 기반 그리드 맵. 0은 50% 확률을 의미
        self.grid = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        
        # 확률 업데이트 설정값 (Log-odds)
        self.l_occ = 0.9    # 장애물이 있을 때 더할 값
        self.l_free = -0.7  # 빈 공간일 때 뺄 값
        self.l_max = 5.0    # 최대 Log-odds (약 99% 확률)
        self.l_min = -5.0   # 최소 Log-odds (약 1% 확률)

    def world_to_grid(self, x, y):
        """실제 좌표를 그리드 인덱스로 변환"""
        gx = int(x / self.resolution) + self.origin_x
        gy = int(y / self.resolution) + self.origin_y
        return gx, gy

    def grid_to_world(self, gx, gy):
        """그리드 인덱스를 실제 좌표로 변환"""
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def is_valid(self, gx, gy):
        return 0 <= gx < self.width_cells and 0 <= gy < self.height_cells

    def update_ray(self, start_x, start_y, end_x, end_y, is_hit):
        """
        하나의 센서 Ray에 대해 Bresenham 알고리즘을 사용하여 빈 공간(Free)과 장애물(Occupied)을 업데이트합니다.
        :param start_x, start_y: 센서의 글로벌 좌표
        :param end_x, end_y: Ray의 끝점 (장애물이면 히트 위치, 아니면 최대 측정 거리)
        :param is_hit: 끝점이 실제 장애물에 맞았는지 여부
        """
        x0, y0 = self.world_to_grid(start_x, start_y)
        x1, y1 = self.world_to_grid(end_x, end_y)

        # Bresenham's Line Algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if not self.is_valid(x, y):
                break
                
            # 끝점 처리
            if x == x1 and y == y1:
                if is_hit:
                    self.grid[y, x] += self.l_occ
                else:
                    self.grid[y, x] += self.l_free
                break
                
            # 지나가는 경로는 모두 빈 공간(Free)
            self.grid[y, x] += self.l_free
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        # Limit the log-odds values
        np.clip(self.grid, self.l_min, self.l_max, out=self.grid)

    def get_probability_map(self):
        """Log-odds 맵을 확률 맵(0~1)으로 변환"""
        return 1.0 - (1.0 / (1.0 + np.exp(self.grid)))
