# Результаты демо-тестов IMX662

**Дата тестирования:** 16 июня 2024  
**Статус:** Все тесты успешно выполнены  
**Производительность:** 18.91 FPS при разрешении 1936x1100

## Выполненные демо-тесты

### DEMO 1: Single Frame Capture
**Цель:** Захват одиночного кадра с базовыми настройками  
**Настройки:** Gain=15, Exposure=1000  
**Результат:** Успешно  

**Файл:** demo1_single_frame_capture.jpg
- Размер: 33KB
- Разрешение: 1936x1100 
- Формат: BGR (конвертировано из RG10)

### DEMO 2: Multiple Captures with Different Settings
**Цель:** Серия захватов с разными параметрами усиления  
**Результат:** Все 3 режима успешно  

**Файлы:**
- demo2_multiple_captures_low_gain.jpg - Gain=5, Exposure=500
- demo2_multiple_captures_medium_gain.jpg - Gain=15, Exposure=1000  
- demo2_multiple_captures_high_gain.jpg - Gain=30, Exposure=1500

**Анализ:** Видна разница в яркости и шуме между режимами

### DEMO 3: Performance Test
**Цель:** Измерение производительности захвата  
**Длительность:** 10 секунд  
**Результат:** Отличная производительность  

**Показатели:**
- Кадров захвачено: 190
- Время: 10.05 секунд
- Средний FPS: 18.91
- Формат: RG10 → BGR конвертация
- Разрешение: 1936x1100

### DEMO 4: Enhanced Image Processing
**Цель:** Демонстрация различных алгоритмов обработки  
**Результат:** Все 5 режимов обработки  

**Файлы:**
- demo4_enhanced_processing_original.jpg - Исходное изображение
- demo4_enhanced_processing_grayscale.jpg - Черно-белое
- demo4_enhanced_processing_blur.jpg - Размытие Гаусса
- demo4_enhanced_processing_edges.jpg - Детекция границ (Canny)
- demo4_enhanced_processing_enhanced.jpg - CLAHE улучшение контраста

**Алгоритмы:**
- Grayscale: cv2.COLOR_BGR2GRAY
- Blur: cv2.GaussianBlur(15x15)
- Edges: cv2.Canny(100, 200)
- Enhanced: CLAHE с clipLimit=3.0

### DEMO 5: Stream Capture (Headless)
**Цель:** Захват серии кадров из потока  
**Количество:** 5 кадров с интервалом 0.5 сек  
**Результат:** Все кадры захвачены  

**Файлы:**
- demo5_stream_capture_frame_001.jpg
- demo5_stream_capture_frame_002.jpg
- demo5_stream_capture_frame_003.jpg
- demo5_stream_capture_frame_004.jpg
- demo5_stream_capture_frame_005.jpg

**Настройки:** Gain=20, Exposure=1200

## Финальное решение

### Direct Capture Solution
**Файл:** final_solution_direct_capture.jpg
- Размер: 124KB (высокое качество)
- Источник: Прямой доступ к /dev/video0
- Обработка: 10-bit RAW → 8-bit BGR
- Качество: Отличное, без зеленых артефактов

## Технические детали

### Успешные конфигурации
# V4L2 формат
v4l2-ctl --set-fmt-video=width=1936,height=1100,pixelformat=RG10

# Системные настройки
camera_auto_detect=0
dtoverlay=imx662

### Производительность по тестам
| Тест | Кадров | Время | FPS | Качество |
|------|--------|-------|-----|----------|
| Single Frame | 1 | ~1s | - | Отличное |
| Multiple Captures | 3 | ~2s | - | Отличное |
| Performance Test | 190 | 10.05s | 18.91 | Стабильное |
| Enhanced Processing | 5 | ~3s | - | Различное |
| Stream Capture | 5 | 2.5s | ~2 | Отличное |

### Форматы данных
- Входной: RG10 (10-bit Bayer в 16-bit контейнере)
- Промежуточный: 16-bit numpy array
- Выходной: 8-bit BGR JPEG
- Размер RAW кадра: 4.2MB (1936×1100×2 байта)
- Размер JPEG: 25-125KB (зависит от содержимого)

## Решенные проблемы

### 1. Зеленые кадры
- Причина: ISP автоконвертация + OpenCV конфликт
- Решение: Отключение camera_auto_detect + прямой доступ

### 2. GUI через SSH
- Причина: Qt/X11 недоступны через SSH
- Решение: Создание headless версии демо-скрипта

### 3. Производительность
- Достигнуто: 18.91 FPS при полном разрешении
- Стабильность: 190 кадров без потерь за 10 секунд

## Структура результатов

images/
├── demo1_single_frame_capture.jpg           # Одиночный кадр
├── demo2_multiple_captures_low_gain.jpg     # Низкое усиление
├── demo2_multiple_captures_medium_gain.jpg  # Среднее усиление  
├── demo2_multiple_captures_high_gain.jpg    # Высокое усиление
├── demo4_enhanced_processing_original.jpg   # Исходное
├── demo4_enhanced_processing_grayscale.jpg  # Ч/Б
├── demo4_enhanced_processing_blur.jpg       # Размытие
├── demo4_enhanced_processing_edges.jpg      # Границы
├── demo4_enhanced_processing_enhanced.jpg   # Улучшенное
├── demo5_stream_capture_frame_001.jpg       # Поток кадр 1
├── demo5_stream_capture_frame_002.jpg       # Поток кадр 2
├── demo5_stream_capture_frame_003.jpg       # Поток кадр 3
├── demo5_stream_capture_frame_004.jpg       # Поток кадр 4
├── demo5_stream_capture_frame_005.jpg       # Поток кадр 5
├── final_solution_direct_capture.jpg        # ФИНАЛЬНОЕ РЕШЕНИЕ
└── old_tests/                               # Старые тестовые файлы

## Команды для воспроизведения

### Запуск всех демо
# На RPI4
python3 demo_imx662_opencv_headless.py --all

### Отдельные тесты
# Одиночный кадр
python3 demo_imx662_opencv_headless.py
# Выбрать: 1

# Множественные захваты
python3 demo_imx662_opencv_headless.py  
# Выбрать: 2

# Тест производительности
python3 demo_imx662_opencv_headless.py
# Выбрать: 3

### Финальное решение
python3 imx662_direct_capture_final.py

## Выводы

Все демо-тесты успешно выполнены  
Производительность 18.91 FPS достигнута  
Различные режимы обработки работают  
Проблема зеленых кадров полностью решена  
Headless режим для SSH реализован  
