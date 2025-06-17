#!/usr/bin/env python3
"""
Конвертер RAW Bayer данных IMX662 в RGB изображение
Формат: RG10 (10-bit Bayer RGRG/GBGB)
Размер: 1936x1100 пикселей
"""

import numpy as np
import cv2
import struct
import argparse
import os

def read_rg10_raw(filename, width, height):
    """
    Читает RAW файл в формате RG10 (10-bit Bayer)
    RG10: каждый пиксель занимает 10 бит, упакованы в байты
    """
    expected_size = width * height * 10 // 8  # 10 бит на пиксель
    
    with open(filename, 'rb') as f:
        raw_data = f.read()
    
    print(f"Файл размер: {len(raw_data)} байт")
    print(f"Ожидаемый размер: {expected_size} байт")
    
    if len(raw_data) != expected_size:
        print(f"Предупреждение: размер файла не соответствует ожидаемому")
        print(f"Пробуем читать как 16-bit данные...")
        # Возможно данные сохранены как 16-bit
        if len(raw_data) == width * height * 2:
            # Читаем как 16-bit little-endian
            raw_16bit = np.frombuffer(raw_data, dtype=np.uint16)
            # Берем только младшие 10 бит
            raw_10bit = (raw_16bit & 0x3FF).astype(np.uint16)
            return raw_10bit.reshape((height, width))
    
    # Распаковываем 10-bit данные
    # 4 пикселя занимают 5 байт (4*10 = 40 бит = 5 байт)
    pixels_per_group = 4
    bytes_per_group = 5
    total_pixels = width * height
    total_groups = total_pixels // pixels_per_group
    
    raw_10bit = np.zeros(total_pixels, dtype=np.uint16)
    
    for i in range(total_groups):
        offset = i * bytes_per_group
        if offset + 4 < len(raw_data):
            # Читаем 5 байт
            b0, b1, b2, b3, b4 = struct.unpack('5B', raw_data[offset:offset+5])
            
            # Распаковываем 4 пикселя из 5 байт
            pix0 = b0 | ((b4 & 0x03) << 8)
            pix1 = b1 | ((b4 & 0x0c) << 6)  
            pix2 = b2 | ((b4 & 0x30) << 4)
            pix3 = b3 | ((b4 & 0xc0) << 2)
            
            base_idx = i * pixels_per_group
            raw_10bit[base_idx:base_idx+4] = [pix0, pix1, pix2, pix3]
    
    return raw_10bit.reshape((height, width))

def debayer_rggb(bayer_img):
    """
    Применяет debayering к Bayer изображению (RGGB pattern)
    IMX662 использует RGGB паттерн
    """
    # Конвертируем в 8-bit для OpenCV
    bayer_8bit = (bayer_img >> 2).astype(np.uint8)  # 10-bit -> 8-bit
    
    # Применяем debayering с помощью OpenCV
    rgb_img = cv2.cvtColor(bayer_8bit, cv2.COLOR_BAYER_RG2BGR)
    
    return rgb_img

def enhance_image(img):
    """
    Базовая обработка изображения для улучшения качества
    """
    # Автоматическая коррекция яркости и контраста
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Применяем CLAHE к L каналу
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    l = clahe.apply(l)
    
    enhanced = cv2.merge([l, a, b])
    enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
    
    return enhanced

def main():
    parser = argparse.ArgumentParser(description='Конвертер RAW Bayer в RGB')
    parser.add_argument('input', help='Входной RAW файл')
    parser.add_argument('-o', '--output', help='Выходной файл (по умолчанию: output.jpg)')
    parser.add_argument('-w', '--width', type=int, default=1936, help='Ширина изображения')
    parser.add_argument('--height', type=int, default=1100, help='Высота изображения')
    parser.add_argument('--no-enhance', action='store_true', help='Не применять улучшения')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Ошибка: файл {args.input} не найден")
        return 1
    
    output_file = args.output or 'output.jpg'
    
    print(f"Читаем RAW файл: {args.input}")
    print(f"Размер изображения: {args.width}x{args.height}")
    
    try:
        # Читаем RAW данные
        bayer_img = read_rg10_raw(args.input, args.width, args.height)
        print(f"RAW данные прочитаны: {bayer_img.shape}")
        print(f"Диапазон значений: {bayer_img.min()} - {bayer_img.max()}")
        
        # Применяем debayering
        rgb_img = debayer_rggb(bayer_img)
        print(f"Debayering выполнен: {rgb_img.shape}")
        
        # Улучшаем изображение
        if not args.no_enhance:
            rgb_img = enhance_image(rgb_img)
            print("Применены улучшения изображения")
        
        # Сохраняем результат
        cv2.imwrite(output_file, rgb_img)
        print(f"Изображение сохранено: {output_file}")
        
        return 0
        
    except Exception as e:
        print(f"Ошибка: {e}")
        return 1

if __name__ == '__main__':
    import sys
    sys.exit(main()) 