#!/usr/bin/env python3
"""
Receive base64-encoded JPEG images from ESP32 serial port, save them, and visualize
ROI (Region of Interest) and density area on the images.

The density area is computed as the union of horizontal intervals from detections
within the ROI, matching the ESP32's computation.

Usage:
    python3 receive_images_and_infer.py [--port PORT] [--baud BAUD] [--output-dir DIR]
                                        [--model MODEL] [--roi-x1 PCT] [--roi-x2 PCT]
                                        [--roi-y1 PCT] [--roi-y2 PCT] [--score-threshold THR]
"""

import argparse
import base64
import serial
import numpy as np
from datetime import datetime
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow as tf
        tflite = tf.lite
    except ImportError:
        print("Error: Need tflite_runtime or tensorflow. Install with:")
        print("  pip3 install tflite-runtime")
        print("  or")
        print("  pip3 install tensorflow")
        exit(1)

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_OUTPUT_DIR = "captured_images"
DEFAULT_MODEL = "epoch_100_int8.tflite"
DEFAULT_SCORE_THRESHOLD = 0.65  # 65% confidence

# Default ROI percentages (matching ESP32 defaults)
DEFAULT_ROI_X1_PCT = 5
DEFAULT_ROI_X2_PCT = 95
DEFAULT_ROI_Y1_PCT = 45
DEFAULT_ROI_Y2_PCT = 95


def load_tflite_model(model_path):
    """Load TFLite model and create interpreter."""
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter


def preprocess_image(image, model_w=192, model_h=192):
    """
    Preprocess image to match ESP32's preprocessing:
    1. Image is 320x240 (QVGA)
    2. Center crop to 240x240
    3. Resize to model input size (192x192)
    4. Convert to RGB and normalize/quantize
    """
    # Original frame size
    frame_w, frame_h = 320, 240
    crop_w, crop_h = 240, 240
    
    # Center crop coordinates
    crop_x0 = (frame_w - crop_w) // 2  # 40
    crop_y0 = (frame_h - crop_h) // 2  # 0
    
    # Resize image to frame size if needed
    if image.size != (frame_w, frame_h):
        image = image.resize((frame_w, frame_h), Image.Resampling.LANCZOS)
    
    # Center crop to 240x240
    image_crop = image.crop((crop_x0, crop_y0, crop_x0 + crop_w, crop_y0 + crop_h))
    
    # Resize to model input size
    image_resized = image_crop.resize((model_w, model_h), Image.Resampling.LANCZOS)
    
    # Convert to RGB array
    img_array = np.array(image_resized.convert('RGB'), dtype=np.float32)
    
    # Get input tensor quantization parameters
    return img_array, (crop_x0, crop_y0, crop_w, crop_h, frame_w, frame_h)


def run_inference(interpreter, img_array):
    """Run inference and return output tensors."""
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    # Get expected input shape (should be [1, H, W, C] for NHWC)
    expected_shape = input_details[0]['shape']
    
    # Add batch dimension if needed (img_array is [H, W, C], need [1, H, W, C])
    if len(img_array.shape) == 3:
        img_array = np.expand_dims(img_array, axis=0)
    
    # Ensure shape matches expected
    if img_array.shape != tuple(expected_shape):
        # Reshape if dimensions match but shape is different
        if img_array.size == np.prod(expected_shape):
            img_array = img_array.reshape(expected_shape)
        else:
            raise ValueError(f"Input shape mismatch: got {img_array.shape}, expected {expected_shape}")
    
    # Get quantization parameters
    input_scale = input_details[0]['quantization_parameters']['scales'][0]
    input_zero_point = input_details[0]['quantization_parameters']['zero_points'][0]
    
    # Normalize and quantize input
    # Check if we should normalize to 0..1 or use 0..255
    if input_scale < 0.1:  # Heuristic: small scale means normalized input
        img_array = img_array / 255.0
    else:
        img_array = img_array  # Use 0..255
    
    # Quantize
    img_quantized = np.round(img_array / input_scale + input_zero_point).astype(np.int8)
    img_quantized = np.clip(img_quantized, -128, 127)
    
    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], img_quantized)
    
    # Run inference
    interpreter.invoke()
    
    # Get output
    output_tensor = interpreter.get_tensor(output_details[0]['index'])
    output_scale = output_details[0]['quantization_parameters']['scales'][0]
    output_zero_point = output_details[0]['quantization_parameters']['zero_points'][0]
    
    return output_tensor, output_scale, output_zero_point, input_details[0]['shape']


def parse_detections(output_tensor, output_scale, output_zero_point, score_threshold, 
                     crop_info, model_w=192, model_h=192):
    """
    Parse YOLO-like detections from model output.
    Returns list of (x1, y1, x2, y2, score, class_id) in full frame coordinates.
    """
    crop_x0, crop_y0, crop_w, crop_h, frame_w, frame_h = crop_info
    
    detections = []
    
    # Remove batch dimension if present (e.g., [1, N, 6] -> [N, 6])
    if len(output_tensor.shape) == 3 and output_tensor.shape[0] == 1:
        output_tensor = output_tensor[0]
    
    # Output shape is typically [N, 6] or [N, 5+classes]
    if len(output_tensor.shape) < 2:
        return detections
    
    rows = output_tensor.shape[0]
    cols = output_tensor.shape[1]
    
    if cols < 5:
        return detections
    
    for r in range(rows):
        # Dequantize and convert to float scalars
        x = float((output_tensor[r, 0] - output_zero_point) * output_scale)
        y = float((output_tensor[r, 1] - output_zero_point) * output_scale)
        w = float((output_tensor[r, 2] - output_zero_point) * output_scale)
        h = float((output_tensor[r, 3] - output_zero_point) * output_scale)
        obj = float((output_tensor[r, 4] - output_zero_point) * output_scale)
        
        # Get class score
        score = obj
        class_id = 0
        if cols > 5:
            best = float((output_tensor[r, 5] - output_zero_point) * output_scale)
            best_id = 0
            for c in range(6, cols):
                v = float((output_tensor[r, c] - output_zero_point) * output_scale)
                if v > best:
                    best = v
                    best_id = c - 5
            class_id = best_id
            score = obj * best
        
        if score < score_threshold:
            continue
        
        # Check if normalized or pixel coordinates
        m = max(abs(x), abs(y), abs(w), abs(h))
        if m <= 1.5:
            x_norm, y_norm, w_norm, h_norm = x, y, w, h
        else:
            x_norm = x / model_w
            y_norm = y / model_h
            w_norm = w / model_w
            h_norm = h / model_h
        
        # Convert xywh (center) to x1y1x2y2 in crop space
        x1c = (x_norm - w_norm * 0.5) * crop_w
        y1c = (y_norm - h_norm * 0.5) * crop_h
        x2c = (x_norm + w_norm * 0.5) * crop_w
        y2c = (y_norm + h_norm * 0.5) * crop_h
        
        # Map crop -> full frame
        x1f = x1c + crop_x0
        y1f = y1c + crop_y0
        x2f = x2c + crop_x0
        y2f = y2c + crop_y0
        
        detections.append((x1f, y1f, x2f, y2f, score, class_id))
    
    return detections


def compute_density_intervals(detections, roi, topk=10):
    """
    Compute horizontal density intervals from detections within ROI.
    Returns list of (x1, x2) intervals and density value (0..1).
    """
    # Filter and clip detections to ROI
    intervals = []
    for det in sorted(detections, key=lambda d: d[4], reverse=True):  # Sort by score
        if len(intervals) >= topk:
            break
        
        x1, y1, x2, y2, score, class_id = det
        
        # Clip to ROI
        x1 = max(x1, roi[0])
        x2 = min(x2, roi[2])
        y1 = max(y1, roi[1])
        y2 = min(y2, roi[3])
        
        if x2 <= x1 or y2 <= y1:
            continue
        
        intervals.append((x1, x2))
    
    if not intervals:
        return [], 0.0
    
    # Sort by x1
    intervals.sort(key=lambda i: i[0])
    
    # Merge overlapping intervals
    merged = []
    cur = intervals[0]
    for i in range(1, len(intervals)):
        nxt = intervals[i]
        if nxt[0] <= cur[1]:
            cur = (cur[0], max(cur[1], nxt[1]))
        else:
            merged.append(cur)
            cur = nxt
    merged.append(cur)
    
    # Compute total length
    total_len = sum(x2 - x1 for x1, x2 in merged)
    roi_width = roi[2] - roi[0]
    density = min(1.0, total_len / roi_width) if roi_width > 0 else 0.0
    
    return merged, density


def draw_visualization(image, roi, density_intervals, density_value, detections, score_threshold):
    """Draw ROI and density area on the image."""
    draw = ImageDraw.Draw(image)
    
    # Try to load a font, fallback to default if not available
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    except:
        try:
            font = ImageFont.truetype("arial.ttf", 14)
        except:
            font = ImageFont.load_default()
    
    # Draw ROI rectangle (green outline, not filled)
    roi_rect = [roi[0], roi[1], roi[2], roi[3]]
    draw.rectangle(roi_rect, outline="green", width=3)
    draw.text((roi[0] + 5, roi[1] + 5), "ROI", fill="green", font=font)
    
    # Draw single density area box (red outline, not filled)
    # Combine all density intervals into one bounding box
    if density_intervals:
        # Get the leftmost and rightmost x coordinates
        min_x = min(x1 for x1, x2 in density_intervals)
        max_x = max(x2 for x1, x2 in density_intervals)
        
        # Draw single red box spanning the entire density area
        draw.rectangle([min_x, roi[1], max_x, roi[3]], outline="red", width=3)
        
        # Draw density value text
        density_text = f"Density: {density_value:.1%}"
        draw.text((min_x + 5, roi[3] - 25), density_text, fill="red", font=font)
    
    return image


def main():
    parser = argparse.ArgumentParser(
        description='Receive images from ESP32, visualize ROI and density area'
    )
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--output-dir', type=str, default=DEFAULT_OUTPUT_DIR,
                        help=f'Output directory (default: {DEFAULT_OUTPUT_DIR})')
    parser.add_argument('--model', type=str, default=DEFAULT_MODEL,
                        help=f'TFLite model path (default: {DEFAULT_MODEL})')
    parser.add_argument('--score-threshold', type=float, default=DEFAULT_SCORE_THRESHOLD,
                        help=f'Detection score threshold (default: {DEFAULT_SCORE_THRESHOLD})')
    parser.add_argument('--roi-x1', type=int, default=DEFAULT_ROI_X1_PCT,
                        help=f'ROI x1 percentage (default: {DEFAULT_ROI_X1_PCT})')
    parser.add_argument('--roi-x2', type=int, default=DEFAULT_ROI_X2_PCT,
                        help=f'ROI x2 percentage (default: {DEFAULT_ROI_X2_PCT})')
    parser.add_argument('--roi-y1', type=int, default=DEFAULT_ROI_Y1_PCT,
                        help=f'ROI y1 percentage (default: {DEFAULT_ROI_Y1_PCT})')
    parser.add_argument('--roi-y2', type=int, default=DEFAULT_ROI_Y2_PCT,
                        help=f'ROI y2 percentage (default: {DEFAULT_ROI_Y2_PCT})')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_dir.absolute()}")
    
    # Load model
    try:
        model_path = Path(args.model)
        if not model_path.exists():
            print(f"Error: Model file not found: {model_path}")
            return 1
        print(f"Loading model: {model_path}")
        interpreter = load_tflite_model(str(model_path))
        print("Model loaded successfully")
    except Exception as e:
        print(f"Error loading model: {e}")
        return 1
    
    # Calculate ROI in pixels (frame is 320x240)
    frame_w, frame_h = 320, 240
    roi_x1 = int(frame_w * args.roi_x1 / 100)
    roi_x2 = int(frame_w * args.roi_x2 / 100)
    roi_y1 = int(frame_h * args.roi_y1 / 100)
    roi_y2 = int(frame_h * args.roi_y2 / 100)
    roi = (roi_x1, roi_y1, roi_x2, roi_y2)
    
    print(f"\nROI configuration:")
    print(f"  X: {args.roi_x1}% - {args.roi_x2}% ({roi_x1} - {roi_x2} pixels)")
    print(f"  Y: {args.roi_y1}% - {args.roi_y2}% ({roi_y1} - {roi_y2} pixels)")
    print(f"  Score threshold: {args.score_threshold}")
    print()
    
    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        print(f"Opened serial port: {args.port} at {args.baud} baud")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return 1
    
    # State machine for parsing base64 images
    state = "WAITING"
    b64_data = []
    image_count = 0
    
    print("\nListening for base64-encoded images...")
    print("Press Ctrl+C to stop.\n")
    
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                continue
            
            if state == "WAITING":
                if line == "IMG_B64_START":
                    state = "READING"
                    b64_data = []
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Receiving image...")
            
            elif state == "READING":
                if line == "IMG_B64_END":
                    try:
                        b64_str = ''.join(b64_data)
                        image_bytes = base64.b64decode(b64_str)
                        
                        # Load image
                        from io import BytesIO
                        image = Image.open(BytesIO(image_bytes))
                        
                        # Preprocess and run inference
                        img_array, crop_info = preprocess_image(image)
                        output_tensor, output_scale, output_zero_point, input_shape = run_inference(
                            interpreter, img_array
                        )
                        
                        # Parse detections
                        detections = parse_detections(
                            output_tensor, output_scale, output_zero_point,
                            args.score_threshold, crop_info
                        )
                        
                        # Debug: show detection info
                        print(f"  Total detections above threshold: {len(detections)}")
                        if len(detections) > 0 and len(detections) <= 5:
                            for i, (x1, y1, x2, y2, score, cls) in enumerate(detections[:5]):
                                print(f"    Det {i+1}: score={score:.3f}, box=[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}]")
                        
                        # Compute density intervals
                        density_intervals, density_value = compute_density_intervals(
                            detections, roi, topk=10
                        )
                        
                        print(f"  Density intervals: {len(density_intervals)}, Density: {density_value:.1%}")
                        
                        # Draw visualization
                        image_annotated = draw_visualization(
                            image.copy(), roi, density_intervals, density_value,
                            detections, args.score_threshold
                        )
                        
                        # Save images
                        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                        image_path = output_dir / f"image_{timestamp}.jpg"
                        annotated_path = output_dir / f"image_{timestamp}_annotated.jpg"
                        
                        image.save(image_path)
                        image_annotated.save(annotated_path)
                        
                        print(f"  Saved: {image_path.name}")
                        print(f"  Annotated: {annotated_path.name}")
                        image_count += 1
                        
                    except Exception as e:
                        import traceback
                        print(f"  Error processing image: {e}")
                        traceback.print_exc()
                    
                    state = "WAITING"
                    b64_data = []
                
                else:
                    b64_data.append(line)
            
            # Also print other log lines for debugging
            elif "MQTT" in line or "MBDET" in line or "ERROR" in line or "WARN" in line:
                print(f"[LOG] {line}")
    
    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {image_count} images total.")
        print(f"Images saved in: {output_dir.absolute()}")
    
    finally:
        ser.close()
        print("Serial port closed.")
    
    return 0


if __name__ == "__main__":
    exit(main())
