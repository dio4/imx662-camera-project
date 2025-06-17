#!/usr/bin/env python3
"""
Демонстрация работы IMX662 - РАБОЧАЯ ВЕРСИЯ
Использует прямой доступ вместо сломанного OpenCV модуля
"""

import time
import sys
import cv2
from imx662_direct_capture_final import IMX662DirectCapture

def demo_single_capture():
    """DEMO 1: Захват одиночного кадра"""
    print("DEMO 1: Single Frame Capture (DIRECT ACCESS)")
    print("-" * 50)
    
    with IMX662DirectCapture() as capture:
        time.sleep(0.5)
        
        # Захват кадра
        frame = capture.capture_frame()
        if frame is not None:
            filename = f"demo1_single_frame_WORKING.jpg"
            cv2.imwrite(filename, frame)
            print(f"✅ Frame saved: {filename}")
            print(f"📏 Frame size: {frame.shape}")
        else:
            print("❌ Failed to capture frame")

def demo_multiple_captures():
    """DEMO 2: Серия захватов (прямой доступ не поддерживает gain/exposure через V4L2)"""
    print("\nDEMO 2: Multiple Captures (DIRECT ACCESS)")
    print("-" * 50)
    
    # Прямой доступ не может менять настройки на лету, но можем делать серию кадров
    capture_names = ["first", "second", "third"]
    
    with IMX662DirectCapture() as capture:
        for i, name in enumerate(capture_names):
            print(f"\n📸 Capture {i+1}: {name}")
            time.sleep(0.5)  # Пауза между кадрами
            
            frame = capture.capture_frame()
            if frame is not None:
                filename = f"demo2_multiple_{name}_WORKING.jpg"
                cv2.imwrite(filename, frame)
                print(f"✅ {filename}")
            else:
                print(f"❌ Failed to capture {name}")

def demo_performance_test():
    """DEMO 3: Тест производительности прямого доступа"""
    print("\nDEMO 3: Performance Test (DIRECT ACCESS)")
    print("-" * 40)
    
    with IMX662DirectCapture() as capture:
        frame_count = 0
        start_time = time.time()
        test_duration = 5  # 5 секунд для теста
        
        print(f"⏱️ Testing direct capture performance for {test_duration} seconds...")
        
        while time.time() - start_time < test_duration:
            frame = capture.capture_frame()
            if frame is not None:
                frame_count += 1
            else:
                print("❌ Frame capture failed")
                break
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed
        
        print(f"📊 Performance Results:")
        print(f"   • Frames captured: {frame_count}")
        print(f"   • Time elapsed: {elapsed:.2f}s")
        print(f"   • Average FPS: {fps:.2f}")
        print(f"   • Method: Direct /dev/video0 access")

def demo_enhanced_processing():
    """DEMO 4: Расширенная обработка изображений"""
    print("\nDEMO 4: Enhanced Image Processing (DIRECT ACCESS)")
    print("-" * 50)
    
    import numpy as np
    
    with IMX662DirectCapture() as capture:
        time.sleep(0.5)
        
        # Захватываем базовый кадр
        frame = capture.capture_frame()
        if frame is None:
            print("❌ Failed to capture frame")
            return
        
        # Различные обработки
        processing_modes = {
            'original': frame,
            'grayscale': cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
            'blur': cv2.GaussianBlur(frame, (15, 15), 0),
            'edges': cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 100, 200),
            'enhanced': None  # Будет обработан ниже
        }
        
        # Enhanced processing (CLAHE)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        processing_modes['enhanced'] = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        
        # Сохранение обработанных изображений
        for mode, processed_frame in processing_modes.items():
            if processed_frame is not None:
                filename = f"demo4_processing_{mode}_WORKING.jpg"
                
                # Для grayscale и edges нужно специальное сохранение
                if mode in ['grayscale', 'edges']:
                    cv2.imwrite(filename, processed_frame)
                else:
                    cv2.imwrite(filename, processed_frame)
                
                print(f"✅ Saved: {filename}")

def demo_stream_capture():
    """DEMO 5: Захват серии кадров из потока"""
    print("\nDEMO 5: Stream Capture (DIRECT ACCESS)")
    print("-" * 40)
    
    with IMX662DirectCapture() as capture:
        frame_count = 0
        max_frames = 5
        
        print(f"📹 Capturing {max_frames} frames from direct stream...")
        
        while frame_count < max_frames:
            frame = capture.capture_frame()
            if frame is not None:
                filename = f"demo5_stream_frame_{frame_count+1:03d}_WORKING.jpg"
                cv2.imwrite(filename, frame)
                print(f"✅ Saved: {filename}")
                frame_count += 1
                time.sleep(0.3)  # Пауза между кадрами
            else:
                print("❌ Failed to capture frame")
                break

def run_all_demos():
    """Запуск всех демо подряд"""
    print("🎥 IMX662 Direct Capture Demo Suite - WORKING VERSION")
    print("=" * 60)
    print("⚠️  Using DIRECT ACCESS instead of broken OpenCV module")
    print("=" * 60)
    
    demo_single_capture()
    demo_multiple_captures()
    demo_performance_test()
    demo_enhanced_processing()
    demo_stream_capture()
    
    print("\n" + "=" * 60)
    print("✅ All demos completed with WORKING direct access!")
    print("=" * 60)

def main():
    """Главная функция"""
    if len(sys.argv) > 1 and sys.argv[1] == '--all':
        run_all_demos()
    else:
        print("🎥 IMX662 Direct Capture Demo Suite - WORKING VERSION")
        print("=" * 60)
        print("⚠️  This version uses DIRECT ACCESS to fix green/black frames")
        print("=" * 60)
        print("Available demos:")
        print("  1. Single Frame Capture")
        print("  2. Multiple Captures")
        print("  3. Performance Test")
        print("  4. Enhanced Processing")
        print("  5. Stream Capture")
        print("  a. Run All Demos")
        print()
        print("Usage:")
        print("  python3 demo_imx662_direct_fixed.py --all")
        print("  python3 demo_imx662_direct_fixed.py")
        print()
        
        choice = input("Select demo (1-5, a for all, q to quit): ").strip().lower()
        
        if choice == '1':
            demo_single_capture()
        elif choice == '2':
            demo_multiple_captures()
        elif choice == '3':
            demo_performance_test()
        elif choice == '4':
            demo_enhanced_processing()
        elif choice == '5':
            demo_stream_capture()
        elif choice == 'a':
            run_all_demos()
        elif choice == 'q':
            print("Goodbye!")
        else:
            print("Invalid choice!")

if __name__ == "__main__":
    main() 