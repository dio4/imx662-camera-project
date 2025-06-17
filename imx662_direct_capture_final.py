#!/usr/bin/env python3
"""
IMX662 Direct Capture Module - ФИНАЛЬНАЯ ВЕРСИЯ
Правильный расчет размера кадра для RG10 формата
"""

import subprocess
import time
import numpy as np
import cv2
import os

class IMX662DirectCapture:
    """
    Класс для прямого захвата RAW данных с IMX662 через /dev/video0
    Обходит проблемы с OpenCV и получает настоящие 10-bit RAW данные
    """
    
    def __init__(self, device='/dev/video0', width=1936, height=1100, verbose=True):
        self.device = device
        self.width = width
        self.height = height
        self.verbose = verbose
        # RG10 - это 10-bit данные в 16-bit контейнере, поэтому 2 байта на пиксель
        self.frame_size = width * height * 2  # 16-bit container for 10-bit data
        self._is_opened = False
        
    def _configure_v4l2_format(self):
        """Настройка V4L2 формата для RAW Bayer захвата"""
        try:
            cmd = [
                'v4l2-ctl',
                f'--device={self.device}',
                f'--set-fmt-video=width={self.width},height={self.height},pixelformat=RG10'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            
            if self.verbose:
                print(f"V4L2 format configured: {self.width}x{self.height} RG10")
                print(f"Frame size: {self.frame_size} bytes")
                
        except subprocess.CalledProcessError as e:
            if self.verbose:
                print(f"ERROR: V4L2 configuration failed: {e}")
            raise RuntimeError(f"Failed to configure V4L2 format: {e}")
    
    def open(self):
        """Открытие камеры для захвата"""
        if self._is_opened:
            if self.verbose:
                print("WARNING: Capture already opened")
            return True
            
        if self.verbose:
            print("Opening IMX662 direct capture...")
        
        # Настройка V4L2 формата
        self._configure_v4l2_format()
        
        # Проверка доступности устройства
        if not os.path.exists(self.device):
            if self.verbose:
                print(f"ERROR: Device {self.device} not found")
            return False
        
        self._is_opened = True
        
        if self.verbose:
            print("IMX662 direct capture opened successfully")
            
        return True
    
    def close(self):
        """Закрытие захвата"""
        self._is_opened = False
        
        if self.verbose:
            print("IMX662 direct capture closed")
    
    def capture_raw_frame(self):
        """
        Захват RAW кадра прямым доступом к /dev/video0
        
        Returns:
            numpy.ndarray: RAW 10-bit данные или None при ошибке
        """
        if not self._is_opened:
            if self.verbose:
                print("ERROR: Capture not opened")
            return None
        
        try:
            # Прямое чтение RAW данных
            with open(self.device, 'rb') as video_dev:
                raw_data = video_dev.read(self.frame_size)
            
            if len(raw_data) != self.frame_size:
                if self.verbose:
                    print(f"WARNING: Expected {self.frame_size} bytes, got {len(raw_data)}")
                # Попробуем с фактическим размером
                if len(raw_data) > 0:
                    # Вычисляем новые размеры на основе полученных данных
                    pixel_count = len(raw_data) // 2
                    new_height = pixel_count // self.width
                    if self.verbose:
                        print(f"Adjusting to: {self.width}x{new_height}")
            
            # Конвертируем в numpy array
            # RG10 - это 10-bit данные в 16-bit контейнере
            raw_array = np.frombuffer(raw_data, dtype=np.uint16)
            
            # Преобразуем в 2D массив
            try:
                frame_2d = raw_array.reshape((self.height, self.width))
            except ValueError:
                # Если не получается с заданными размерами, попробуем автоматический расчет
                pixel_count = len(raw_array)
                new_height = pixel_count // self.width
                frame_2d = raw_array.reshape((new_height, self.width))
                if self.verbose:
                    print(f"Auto-adjusted to: {self.width}x{new_height}")
            
            if self.verbose:
                print(f"RAW frame captured: {frame_2d.shape}, range: {frame_2d.min()}-{frame_2d.max()}")
            
            return frame_2d
            
        except Exception as e:
            if self.verbose:
                print(f"ERROR: Frame capture failed: {e}")
            return None
    
    def capture_frame(self):
        """
        Захват кадра с автоматической конвертацией в BGR
        
        Returns:
            numpy.ndarray: BGR изображение или None при ошибке
        """
        raw_frame = self.capture_raw_frame()
        
        if raw_frame is None:
            return None
        
        try:
            # Конвертируем 10-bit в 8-bit для debayering
            bayer_8bit = (raw_frame >> 2).astype(np.uint8)
            
            # Применяем debayering для RGGB pattern (IMX662)
            bgr_frame = cv2.cvtColor(bayer_8bit, cv2.COLOR_BayerRG2BGR)
            
            if self.verbose:
                print(f"Converted to BGR: {bgr_frame.shape}")
            
            return bgr_frame
            
        except Exception as e:
            if self.verbose:
                print(f"ERROR: Frame conversion failed: {e}")
            return None
    
    @property
    def is_opened(self):
        """Проверка, открыт ли захват"""
        return self._is_opened
    
    # Контекстный менеджер
    def __enter__(self):
        if not self.open():
            raise RuntimeError("Failed to open IMX662 camera")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def test_direct_capture():
    """Тест прямого захвата RAW данных"""
    print("Testing IMX662 Direct Capture - FINAL")
    print("=" * 40)
    
    try:
        with IMX662DirectCapture() as capture:
            time.sleep(0.5)
            
            # Захват RAW кадра
            raw_frame = capture.capture_raw_frame()
            if raw_frame is not None:
                timestamp = int(time.time())
                
                # Сохранение RAW данных
                raw_filename = f"raw_direct_final_{timestamp}.raw"
                raw_frame.tofile(raw_filename)
                print(f"RAW data saved: {raw_filename}")
                
                # Захват BGR кадра
                bgr_frame = capture.capture_frame()
                if bgr_frame is not None:
                    bgr_filename = f"bgr_direct_final_{timestamp}.jpg"
                    cv2.imwrite(bgr_filename, bgr_frame)
                    print(f"SUCCESS: BGR image saved: {bgr_filename}")
                    print(f"Image shape: {bgr_frame.shape}")
                    print(f"Pixel value range: {bgr_frame.min()}-{bgr_frame.max()}")
                    
                    return True
                else:
                    print("ERROR: Failed to convert to BGR")
                    return False
            else:
                print("ERROR: Failed to capture RAW frame")
                return False
                
    except Exception as e:
        print(f"ERROR: {e}")
        return False


if __name__ == "__main__":
    test_direct_capture() 