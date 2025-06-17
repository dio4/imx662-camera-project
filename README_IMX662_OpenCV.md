# 🎥 IMX662 OpenCV Universal Integration

**Универсальная интеграция камеры Sony IMX662 с OpenCV без RPi-зависимостей**

## 🎯 Основные преимущества

- ✅ **Универсальность** - работает на любой Linux системе с V4L2
- ✅ **Независимость** - не требует RPi-специфичных библиотек
- ✅ **Стандартность** - использует только OpenCV + V4L2
- ✅ **Простота** - готовый к использованию класс
- ✅ **Производительность** - оптимизированная обработка RAW данных

## 📋 Требования

### Системные требования:
- Linux с поддержкой V4L2
- Установленный драйвер IMX662 (imx662.ko)
- Устройство `/dev/video0` (или другое)

### Python зависимости:
```bash
pip install opencv-python numpy
```

### Дополнительные утилиты:
```bash
sudo apt install v4l-utils
```

## 🚀 Быстрый старт

### 1. Простой захват кадра
```python
from imx662_opencv_capture import IMX662OpenCVCapture
import cv2

# Создаем захват
with IMX662OpenCVCapture() as cap:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('IMX662', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
```

### 2. Захват с настройками
```python
cap = IMX662OpenCVCapture(device_id=0, width=1936, height=1100)
cap.open()

# Настройка параметров
cap.set_gain(20)
cap.set_exposure(1000)

# Захват
ret, frame = cap.read()
if ret:
    cv2.imwrite('imx662_photo.jpg', frame)

cap.close()
```

### 3. Живой поток
```python
from imx662_opencv_capture import IMX662OpenCVCapture, IMX662StreamProcessor

cap = IMX662OpenCVCapture()
cap.open()

processor = IMX662StreamProcessor(cap)
processor.process_stream(mode='enhanced', display=True)

cap.close()
```

## 📚 API Документация

### IMX662OpenCVCapture

#### Инициализация
```python
cap = IMX662OpenCVCapture(device_id=0, width=1936, height=1100)
```

**Параметры:**
- `device_id` - ID видеоустройства (0 для /dev/video0)
- `width` - ширина кадра (1936 для IMX662)
- `height` - высота кадра (1100 для IMX662)

#### Основные методы

##### `open()` → bool
Открывает захват с камеры
```python
success = cap.open()
```

##### `read()` → (bool, np.ndarray)
Захватывает и обрабатывает кадр
```python
ret, frame = cap.read()
```

##### `close()`
Закрывает захват
```python
cap.close()
```

##### `set_gain(value)` → bool
Устанавливает усиление
```python
success = cap.set_gain(20)  # 0-100
```

##### `set_exposure(value)` → bool
Устанавливает экспозицию
```python
success = cap.set_exposure(1000)  # микросекунды
```

##### `get_frame_info()` → dict
Получает информацию о кадре
```python
info = cap.get_frame_info()
print(info)
```

### IMX662StreamProcessor

#### Инициализация
```python
processor = IMX662StreamProcessor(capture)
```

#### Режимы обработки
- `'raw'` - минимальная обработка
- `'enhanced'` - улучшение качества (CLAHE)
- `'analyzed'` - анализ с наложением информации

#### Обработка потока
```python
processor.process_stream(mode='enhanced', display=True, save_frames=False)
```

## 🧪 Демонстрационные примеры

Запуск демо-скрипта:
```bash
python3 demo_imx662_opencv.py
```

### Доступные демо:
1. **Single Frame Capture** - захват одиночного кадра
2. **Multiple Captures** - серия кадров с разными настройками
3. **Live Stream** - живой поток с обработкой
4. **Performance Test** - тест производительности FPS
5. **Enhanced Processing** - расширенная обработка изображений

## 🔧 Техническая информация

### Формат данных
- **Входной формат:** RG10 (10-bit RAW Bayer RGRG/GBGB)
- **Выходной формат:** BGR 8-bit (стандартный OpenCV)
- **Разрешение:** 1936×1100 (нативное для IMX662)

### Обработка изображений
1. **V4L2 захват** - установка RG10 формата через v4l2-ctl
2. **OpenCV VideoCapture** - захват через CAP_V4L2 backend
3. **Debayering** - преобразование Bayer → BGR
4. **Bit conversion** - 10-bit → 8-bit нормализация

### Архитектура
```
IMX662 Sensor → V4L2 Driver → OpenCV VideoCapture → Debayering → BGR Frame
```

## ⚙️ Настройка и оптимизация

### Рекомендуемые параметры
```python
# Для обычных условий освещения
cap.set_gain(15)
cap.set_exposure(1000)

# Для слабого освещения
cap.set_gain(30)
cap.set_exposure(2000)

# Для яркого освещения
cap.set_gain(5)
cap.set_exposure(500)
```

### Проверка производительности
```python
# Тест FPS
frame_count = 0
start_time = time.time()

while time.time() - start_time < 10:  # 10 секунд
    ret, frame = cap.read()
    if ret:
        frame_count += 1

fps = frame_count / 10
print(f"Average FPS: {fps:.2f}")
```

## 🛠️ Troubleshooting

### Проблема: Камера не открывается
```bash
# Проверить наличие устройства
ls -l /dev/video*

# Проверить драйвер
lsmod | grep imx662

# Проверить I2C
sudo i2cdetect -y 10
```

### Проблема: Низкая производительность
```python
# Оптимизация буферов OpenCV
cap.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Отключение автонастроек
cap.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
```

### Проблема: Плохое качество изображения
```python
# Ручная настройка параметров
cap.set_gain(20)
cap.set_exposure(1200)

# Использование enhanced режима
processor.process_stream(mode='enhanced')
```

## 🔗 Интеграция в проекты

### Computer Vision проекты
```python
import cv2
from imx662_opencv_capture import IMX662OpenCVCapture

cap = IMX662OpenCVCapture()
cap.open()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Ваша обработка изображения
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)
    
    cv2.imshow('Edges', edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.close()
cv2.destroyAllWindows()
```

### Machine Learning pipeline
```python
import numpy as np
from imx662_opencv_capture import IMX662OpenCVCapture

cap = IMX662OpenCVCapture()
cap.open()

def preprocess_frame(frame):
    """Предобработка для ML модели"""
    resized = cv2.resize(frame, (224, 224))
    normalized = resized.astype(np.float32) / 255.0
    return normalized

# Захват данных для ML
frames = []
for i in range(100):
    ret, frame = cap.read()
    if ret:
        processed = preprocess_frame(frame)
        frames.append(processed)

# Конвертация в numpy array для ML
data = np.array(frames)
print(f"Dataset shape: {data.shape}")

cap.close()
```

## 📊 Сравнение с альтернативами

| Решение | Зависимости | Универсальность | Производительность |
|---------|-------------|-----------------|-------------------|
| **IMX662OpenCV** | OpenCV + V4L2 | ✅ Высокая | ✅ Высокая |
| libcamera | libcamera + RPi | ❌ Только RPi | ⚠️ Средняя |
| rpicam-apps | RPi-специфичные | ❌ Только RPi | ⚠️ Средняя |
| V4L2 прямой | Только V4L2 | ✅ Высокая | ⚠️ Требует ручной обработки |

## 📝 Лицензия

Этот код распространяется по лицензии MIT. См. файл LICENSE для подробностей.

## 🤝 Вклад в проект

Приветствуются pull requests и issue reports в репозитории проекта.

## 📞 Поддержка

При возникновении проблем создайте issue с подробным описанием:
- Версия операционной системы
- Версия OpenCV
- Версия Python  
- Логи ошибок
- Шаги воспроизведения

---

**🎯 IMX662 OpenCV Integration - универсальное решение для работы с камерой Sony IMX662 через OpenCV!** 