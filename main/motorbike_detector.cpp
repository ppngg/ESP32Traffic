// Motorbike detector + density metric (plan.md).
// Uses OV2640 (esp32-camera) + TFLite Micro INT8 model embedded into the ELF.

#include "motorbike_detector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "mbedtls/base64.h"
}

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/schema/schema_utils.h"

static const char * TAG = "MBDET";

// Embedded model (epoch_100_int8.tflite).
extern const uint8_t epoch_100_int8_tflite_start[] asm( "_binary_epoch_100_int8_tflite_start" );
extern const uint8_t epoch_100_int8_tflite_end[] asm( "_binary_epoch_100_int8_tflite_end" );

namespace {

struct RoiPx
{
    int x1, y1, x2, y2; // inclusive-exclusive [x1,x2), [y1,y2)
};

struct BoxF
{
    float x1, y1, x2, y2; // normalized (0..1) in model space before mapping
    float score;
    int class_id;
};

struct IntervalF
{
    float x1, x2;
};

static RoiPx roi_for_frame( int w, int h )
{
    const int x1 = ( w * CONFIG_MBDET_ROI_X1_PCT ) / 100;
    const int x2 = ( w * CONFIG_MBDET_ROI_X2_PCT ) / 100;
    const int y1 = ( h * CONFIG_MBDET_ROI_Y1_PCT ) / 100;
    const int y2 = ( h * CONFIG_MBDET_ROI_Y2_PCT ) / 100;

    RoiPx r{};
    r.x1 = std::clamp( x1, 0, w );
    r.x2 = std::clamp( x2, 0, w );
    r.y1 = std::clamp( y1, 0, h );
    r.y2 = std::clamp( y2, 0, h );

    if( r.x2 < r.x1 ) std::swap( r.x1, r.x2 );
    if( r.y2 < r.y1 ) std::swap( r.y1, r.y2 );
    return r;
}

static float clamp01( float v )
{
    if( v < 0.f ) return 0.f;
    if( v > 1.f ) return 1.f;
    return v;
}

static float score_thr()
{
    return ( ( float ) CONFIG_MBDET_SCORE_THR_PCT ) / 100.0f;
}

static float compute_hfill( std::vector<BoxF> & dets, const RoiPx & roi, int frame_w, int frame_h, int * out_box_count )
{
    if( out_box_count != nullptr )
    {
        *out_box_count = 0;
    }

    // Convert detections to x-intervals after ROI clip, then merge.
    std::sort( dets.begin(), dets.end(), []( const BoxF & a, const BoxF & b ) { return a.score > b.score; } );

    std::vector<IntervalF> intervals;
    intervals.reserve( CONFIG_MBDET_TOPK );

    const float fx1 = static_cast<float>( roi.x1 );
    const float fx2 = static_cast<float>( roi.x2 );
    const float fy1 = static_cast<float>( roi.y1 );
    const float fy2 = static_cast<float>( roi.y2 );

    for( const auto & d : dets )
    {
        if( d.score < score_thr() )
        {
            break;
        }
        if( static_cast<int>( intervals.size() ) >= CONFIG_MBDET_TOPK )
        {
            break;
        }

        // d is in frame pixels already (we will store as pixels below).
        float x1 = d.x1;
        float y1 = d.y1;
        float x2 = d.x2;
        float y2 = d.y2;

        // Clip to ROI.
        x1 = std::max( x1, fx1 );
        x2 = std::min( x2, fx2 );
        y1 = std::max( y1, fy1 );
        y2 = std::min( y2, fy2 );

        if( x2 <= x1 || y2 <= y1 )
        {
            continue;
        }

        intervals.push_back( { x1, x2 } );
    }

    if( out_box_count != nullptr )
    {
        *out_box_count = ( int ) intervals.size();
    }

    if( intervals.empty() )
    {
        return 0.f;
    }

    std::sort( intervals.begin(), intervals.end(), []( const IntervalF & a, const IntervalF & b ) { return a.x1 < b.x1; } );

    float merged_len = 0.f;
    IntervalF cur = intervals[ 0 ];
    for( size_t i = 1; i < intervals.size(); ++i )
    {
        const IntervalF nxt = intervals[ i ];
        if( nxt.x1 <= cur.x2 )
        {
            cur.x2 = std::max( cur.x2, nxt.x2 );
        }
        else
        {
            merged_len += ( cur.x2 - cur.x1 );
            cur = nxt;
        }
    }
    merged_len += ( cur.x2 - cur.x1 );

    const float roi_w = static_cast<float>( roi.x2 - roi.x1 );
    if( roi_w <= 1.f )
    {
        return 0.f;
    }

    return clamp01( merged_len / roi_w );
}

static float median3( float a, float b, float c )
{
    if( a > b ) std::swap( a, b );
    if( b > c ) std::swap( b, c );
    if( a > b ) std::swap( a, b );
    return b;
}

static camera_config_t make_camera_config()
{
    camera_config_t config{};

#if CONFIG_MBDET_CAM_PRESET_ESP32S3_EYE
    // ESP32-S3-EYE (OV2640). This matches Espressif's typical reference wiring.
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.pin_xclk = 15;
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;

    config.pin_d7 = 16; // Y9
    config.pin_d6 = 17; // Y8
    config.pin_d5 = 18; // Y7
    config.pin_d4 = 12; // Y6
    config.pin_d3 = 10; // Y5
    config.pin_d2 = 8;  // Y4
    config.pin_d1 = 9;  // Y3
    config.pin_d0 = 11; // Y2

    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_pclk = 13;

#elif CONFIG_MBDET_CAM_PRESET_XIAO_ESP32S3
    // XIAO ESP32S3 Sense (OV2640) mapping (from your person-detection reference).
    config.pin_pwdn = -1;
    config.pin_reset = -1;

    config.pin_vsync = 38;
    config.pin_href = 47;
    config.pin_pclk = 13;
    config.pin_xclk = 10;

    config.pin_sccb_sda = 40;
    config.pin_sccb_scl = 39;

    config.pin_d0 = 15; // Y2
    config.pin_d1 = 17; // Y3
    config.pin_d2 = 18; // Y4
    config.pin_d3 = 16; // Y5
    config.pin_d4 = 14; // Y6
    config.pin_d5 = 12; // Y7
    config.pin_d6 = 11; // Y8
    config.pin_d7 = 48; // Y9
#else
    config.pin_pwdn = CONFIG_MBDET_CAM_PIN_PWDN;
    config.pin_reset = CONFIG_MBDET_CAM_PIN_RESET;
    config.pin_xclk = CONFIG_MBDET_CAM_PIN_XCLK;
    config.pin_sccb_sda = CONFIG_MBDET_CAM_PIN_SIOD;
    config.pin_sccb_scl = CONFIG_MBDET_CAM_PIN_SIOC;

    config.pin_d7 = CONFIG_MBDET_CAM_PIN_Y9;
    config.pin_d6 = CONFIG_MBDET_CAM_PIN_Y8;
    config.pin_d5 = CONFIG_MBDET_CAM_PIN_Y7;
    config.pin_d4 = CONFIG_MBDET_CAM_PIN_Y6;
    config.pin_d3 = CONFIG_MBDET_CAM_PIN_Y5;
    config.pin_d2 = CONFIG_MBDET_CAM_PIN_Y4;
    config.pin_d1 = CONFIG_MBDET_CAM_PIN_Y3;
    config.pin_d0 = CONFIG_MBDET_CAM_PIN_Y2;

    config.pin_vsync = CONFIG_MBDET_CAM_PIN_VSYNC;
    config.pin_href = CONFIG_MBDET_CAM_PIN_HREF;
    config.pin_pclk = CONFIG_MBDET_CAM_PIN_PCLK;
#endif

    // 15MHz is a commonly working value for OV2640 on small boards (and matches your reference project).
    config.xclk_freq_hz = 15000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    // Plan wants RGB frames (so we avoid JPEG decode cost).
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 12;
    // QVGA RGB565 is relatively large; start with 1 frame buffer to reduce DMA-cap heap pressure.
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    return config;
}

static esp_err_t ensure_camera_inited()
{
    static bool inited = false;
    if( inited ) return ESP_OK;

    const camera_config_t cfg = make_camera_config();
    const esp_err_t err = esp_camera_init( &cfg );
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG, "esp_camera_init failed: %s", esp_err_to_name( err ) );
        return err;
    }

    sensor_t * s = esp_camera_sensor_get();
    if( s != nullptr )
    {
        // Basic defaults that often help stability outdoors.
        s->set_framesize( s, FRAMESIZE_QVGA );
        s->set_pixformat( s, PIXFORMAT_RGB565 );
    }

    inited = true;
    ESP_LOGI( TAG, "Camera initialized (RGB565 320x240)" );
    return ESP_OK;
}

// Convert one RGB565 pixel to 8-bit RGB.
static inline void rgb565_to_rgb888( uint16_t p, uint8_t & r, uint8_t & g, uint8_t & b )
{
    r = static_cast<uint8_t>( ( ( p >> 11 ) & 0x1F ) * 255 / 31 );
    g = static_cast<uint8_t>( ( ( p >> 5 ) & 0x3F ) * 255 / 63 );
    b = static_cast<uint8_t>( ( p & 0x1F ) * 255 / 31 );
}

// Resize + quantize into TFLM input tensor (NHWC).
static void crop_resize_quantize_rgb565_center_to_tensor(
    const uint16_t * src_rgb565,
    int src_w,
    int src_h,
    int crop_w,
    int crop_h,
    int dst_w,
    int dst_h,
    int8_t * dst_int8,
    float q_scale,
    int q_zero_point )
{
    const int crop_x0 = ( src_w - crop_w ) / 2;
    const int crop_y0 = ( src_h - crop_h ) / 2;

    const bool norm01 =
#if CONFIG_MBDET_INPUT_NORM_0_1
        true;
#elif CONFIG_MBDET_INPUT_NORM_0_255
        false;
#else
        ( q_scale < 0.1f ); // heuristic
#endif

    const float inv255 = 1.0f / 255.0f;

    // Nearest-neighbor resize for speed.
    for( int y = 0; y < dst_h; ++y )
    {
        const int src_y = crop_y0 + ( y * crop_h ) / dst_h;
        for( int x = 0; x < dst_w; ++x )
        {
            const int src_x = crop_x0 + ( x * crop_w ) / dst_w;
            const uint16_t p = src_rgb565[ src_y * src_w + src_x ];
            uint8_t r8, g8, b8;
            rgb565_to_rgb888( p, r8, g8, b8 );

            const float rf = norm01 ? ( r8 * inv255 ) : static_cast<float>( r8 );
            const float gf = norm01 ? ( g8 * inv255 ) : static_cast<float>( g8 );
            const float bf = norm01 ? ( b8 * inv255 ) : static_cast<float>( b8 );

            const int ri = static_cast<int>( std::lround( rf / q_scale ) + q_zero_point );
            const int gi = static_cast<int>( std::lround( gf / q_scale ) + q_zero_point );
            const int bi = static_cast<int>( std::lround( bf / q_scale ) + q_zero_point );

            const int idx = ( y * dst_w + x ) * 3;
            dst_int8[ idx + 0 ] = static_cast<int8_t>( std::clamp( ri, -128, 127 ) );
            dst_int8[ idx + 1 ] = static_cast<int8_t>( std::clamp( gi, -128, 127 ) );
            dst_int8[ idx + 2 ] = static_cast<int8_t>( std::clamp( bi, -128, 127 ) );
        }
    }
}

class TflmEngine
{
  public:
    bool init()
    {
        if( inited_ ) return true;

        const uint8_t * model_data = epoch_100_int8_tflite_start;
        const size_t model_len = static_cast<size_t>( epoch_100_int8_tflite_end - epoch_100_int8_tflite_start );
        if( model_len < 16 )
        {
            ESP_LOGE( TAG, "Model data too small (%u bytes)", static_cast<unsigned>( model_len ) );
            return false;
        }

        model_ = tflite::GetModel( model_data );
        if( model_->version() != TFLITE_SCHEMA_VERSION )
        {
            ESP_LOGE( TAG, "Model schema %d != supported %d", model_->version(), TFLITE_SCHEMA_VERSION );
            return false;
        }

        // Log operator codes in the model to make missing-op debugging easy.
        if( model_->operator_codes() != nullptr )
        {
            const auto * opcodes = model_->operator_codes();
            ESP_LOGI( TAG, "Model opcodes: %u", (unsigned) opcodes->size() );
            for( unsigned i = 0; i < opcodes->size(); ++i )
            {
                const auto * oc = opcodes->Get( i );
                const tflite::BuiltinOperator bop = tflite::GetBuiltinCode( oc );
                if( bop == tflite::BuiltinOperator_CUSTOM )
                {
                    const char * custom = ( oc->custom_code() != nullptr ) ? oc->custom_code()->c_str() : "(null)";
                    ESP_LOGI( TAG, "  [%u] CUSTOM: %s", i, custom );
                }
                else
                {
                    ESP_LOGI( TAG, "  [%u] %s", i, tflite::EnumNameBuiltinOperator( bop ) );
                }
            }
        }

        // esp-tflite-micro does not ship AllOpsResolver; use a mutable resolver and
        // register a broad set of common ops used by CNN/YOLO-style graphs.
        // If AllocateTensors fails due to a missing op, extend this list.
        // NOLINTNEXTLINE(runtime-global-variables)
        static tflite::MicroMutableOpResolver<96> resolver;
        static bool resolver_inited = false;
        if( !resolver_inited )
        {
            ( void ) resolver.AddAdd();
            ( void ) resolver.AddMul();
            ( void ) resolver.AddSub();
            ( void ) resolver.AddDiv();
            ( void ) resolver.AddMaximum();
            ( void ) resolver.AddMinimum();
            ( void ) resolver.AddSquaredDifference();

            ( void ) resolver.AddConv2D();
            ( void ) resolver.AddDepthwiseConv2D();
            ( void ) resolver.AddFullyConnected();

            ( void ) resolver.AddAveragePool2D();
            ( void ) resolver.AddMaxPool2D();

            ( void ) resolver.AddReshape();
            ( void ) resolver.AddConcatenation();
            ( void ) resolver.AddTranspose();
            ( void ) resolver.AddPack();
            ( void ) resolver.AddUnpack();

            ( void ) resolver.AddPad();
            ( void ) resolver.AddPadV2();
            ( void ) resolver.AddSlice();
            ( void ) resolver.AddStridedSlice();
            ( void ) resolver.AddSplit();
            ( void ) resolver.AddSplitV();
            ( void ) resolver.AddGather();
            ( void ) resolver.AddShape();

            ( void ) resolver.AddLogistic();
            ( void ) resolver.AddLeakyRelu();
            ( void ) resolver.AddRelu();
            ( void ) resolver.AddRelu6();
            ( void ) resolver.AddTanh();

            ( void ) resolver.AddSoftmax();

            ( void ) resolver.AddQuantize();
            ( void ) resolver.AddDequantize();

            ( void ) resolver.AddMean();
            ( void ) resolver.AddSum();

            ( void ) resolver.AddExp();
            ( void ) resolver.AddLog();
            ( void ) resolver.AddSqrt();

            ( void ) resolver.AddResizeNearestNeighbor();
            ( void ) resolver.AddResizeBilinear();

            // Common TF custom op used by some detection export pipelines.
            ( void ) resolver.AddDetectionPostprocess();

            resolver_inited = true;
        }

        arena_ = static_cast<uint8_t *>( heap_caps_malloc( CONFIG_MBDET_TENSOR_ARENA_BYTES, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM ) );
        if( arena_ == nullptr )
        {
            ESP_LOGW( TAG, "PSRAM arena alloc failed, trying internal RAM" );
            arena_ = static_cast<uint8_t *>( heap_caps_malloc( CONFIG_MBDET_TENSOR_ARENA_BYTES, MALLOC_CAP_8BIT ) );
        }
        if( arena_ == nullptr )
        {
            ESP_LOGE( TAG, "Tensor arena alloc failed (%d bytes)", CONFIG_MBDET_TENSOR_ARENA_BYTES );
            return false;
        }

        interpreter_ = std::make_unique<tflite::MicroInterpreter>(
            model_, resolver, arena_, CONFIG_MBDET_TENSOR_ARENA_BYTES );

        if( interpreter_->AllocateTensors() != kTfLiteOk )
        {
            ESP_LOGE( TAG, "AllocateTensors failed (likely missing op or tensor arena too small). "
                           "Try increasing MBDET_TENSOR_ARENA_BYTES and check logs above for missing op name." );
            return false;
        }

        input_ = interpreter_->input( 0 );
        if( input_ == nullptr || input_->type != kTfLiteInt8 )
        {
            ESP_LOGE( TAG, "Expected int8 input tensor" );
            return false;
        }

        if( input_->dims == nullptr || input_->dims->size != 4 )
        {
            ESP_LOGE( TAG, "Unexpected input dims" );
            return false;
        }

        // NHWC
        in_h_ = input_->dims->data[ 1 ];
        in_w_ = input_->dims->data[ 2 ];
        in_c_ = input_->dims->data[ 3 ];

        if( in_c_ != 3 )
        {
            ESP_LOGW( TAG, "Input channels=%d (expected 3)", in_c_ );
        }

        ESP_LOGI( TAG, "TFLM ready. Input: %dx%dx%d int8 (scale=%g zp=%d). Outputs=%d",
                  in_w_, in_h_, in_c_, input_->params.scale, input_->params.zero_point, interpreter_->outputs_size() );
        inited_ = true;
        return true;
    }

    bool run_from_rgb565_qvga( const uint16_t * rgb565, int w, int h )
    {
        if( !init() ) return false;
        if( w != 320 || h != 240 )
        {
            // Still allow, but crop assumes center.
            ESP_LOGW( TAG, "Unexpected frame size %dx%d (expected 320x240)", w, h );
        }

        crop_resize_quantize_rgb565_center_to_tensor(
            rgb565,
            w,
            h,
            240,
            240,
            in_w_,
            in_h_,
            input_->data.int8,
            input_->params.scale,
            input_->params.zero_point );

        // Measure inference time
        const int64_t t_start = esp_timer_get_time();
        if( interpreter_->Invoke() != kTfLiteOk )
        {
            ESP_LOGE( TAG, "Invoke failed" );
            return false;
        }
        const int64_t t_end = esp_timer_get_time();
        const int64_t inference_time_us = t_end - t_start;
        ESP_LOGI( TAG, "Inference time: %lld ms (%.1f fps)", 
                  inference_time_us / 1000, 
                  1000000.0 / inference_time_us );
        
        return true;
    }

    // Extract detections into frame pixel coordinates (320x240) using the known crop mapping:
    // full frame -> center crop 240x240 at x0=40,y0=0 -> resized to in_w_/in_h_.
    std::vector<BoxF> extract_detections_as_frame_px( int frame_w, int frame_h ) const
    {
        std::vector<BoxF> out;

        const int crop_w = 240;
        const int crop_h = 240;
        const int crop_x0 = ( frame_w - crop_w ) / 2;
        const int crop_y0 = ( frame_h - crop_h ) / 2;

        const int out_count = interpreter_->outputs_size();
        for( int oi = 0; oi < out_count; ++oi )
        {
            const TfLiteTensor * t = interpreter_->output( oi );
            if( t == nullptr || t->dims == nullptr || t->dims->size < 2 )
            {
                continue;
            }

            const int last = t->dims->data[ t->dims->size - 1 ];
            int rows = 1;
            for( int di = 0; di < t->dims->size - 1; ++di )
            {
                rows *= t->dims->data[ di ];
            }
            if( last < 5 || rows <= 0 )
            {
                continue;
            }

            const float os = t->params.scale;
            const int ozp = t->params.zero_point;

            auto get_f = [&]( int row, int col ) -> float {
                const int idx = row * last + col;
                switch( t->type )
                {
                    case kTfLiteInt8:
                        return ( static_cast<int>( t->data.int8[ idx ] ) - ozp ) * os;
                    case kTfLiteUInt8:
                        return ( static_cast<int>( t->data.uint8[ idx ] ) - ozp ) * os;
                    case kTfLiteFloat32:
                        return t->data.f[ idx ];
                    default:
                        return 0.f;
                }
            };

            for( int r = 0; r < rows; ++r )
            {
                // Interpret as YOLO-like: [x,y,w,h,obj,(cls...)]
                const float x = get_f( r, 0 );
                const float y = get_f( r, 1 );
                const float w = get_f( r, 2 );
                const float h = get_f( r, 3 );

                float objectness = get_f( r, 4 );
                float score = objectness;
                int class_id = 0;

                if( last > 5 )
                {
                    float best = get_f( r, 5 );
                    int best_id = 0;
                    for( int c = 6; c < last; ++c )
                    {
                        const float v = get_f( r, c );
                        if( v > best )
                        {
                            best = v;
                            best_id = c - 5;
                        }
                    }
                    class_id = best_id;
                    score = objectness * best;
                }

                if( score < score_thr() )
                {
                    continue;
                }

                // Heuristic: if numbers are <= ~1.5, treat as normalized; else treat as pixels in input tensor space.
                const float m = std::max( std::max( std::fabs( x ), std::fabs( y ) ), std::max( std::fabs( w ), std::fabs( h ) ) );
                float x_norm, y_norm, w_norm, h_norm;
                if( m <= 1.5f )
                {
                    x_norm = x;
                    y_norm = y;
                    w_norm = w;
                    h_norm = h;
                }
                else
                {
                    x_norm = x / static_cast<float>( in_w_ );
                    y_norm = y / static_cast<float>( in_h_ );
                    w_norm = w / static_cast<float>( in_w_ );
                    h_norm = h / static_cast<float>( in_h_ );
                }

                // Convert xywh (center) to x1y1x2y2 in crop pixels.
                float x1c = ( x_norm - w_norm * 0.5f ) * crop_w;
                float y1c = ( y_norm - h_norm * 0.5f ) * crop_h;
                float x2c = ( x_norm + w_norm * 0.5f ) * crop_w;
                float y2c = ( y_norm + h_norm * 0.5f ) * crop_h;

                // Map crop -> full frame.
                float x1f = x1c + crop_x0;
                float y1f = y1c + crop_y0;
                float x2f = x2c + crop_x0;
                float y2f = y2c + crop_y0;

                // Clip to frame bounds.
                x1f = std::clamp( x1f, 0.f, static_cast<float>( frame_w ) );
                x2f = std::clamp( x2f, 0.f, static_cast<float>( frame_w ) );
                y1f = std::clamp( y1f, 0.f, static_cast<float>( frame_h ) );
                y2f = std::clamp( y2f, 0.f, static_cast<float>( frame_h ) );

                if( x2f <= x1f || y2f <= y1f )
                {
                    continue;
                }

                out.push_back( { x1f, y1f, x2f, y2f, score, class_id } );
            }
        }

        return out;
    }

  private:
    bool inited_ = false;

    const tflite::Model * model_ = nullptr;
    std::unique_ptr<tflite::MicroInterpreter> interpreter_;

    TfLiteTensor * input_ = nullptr;

    uint8_t * arena_ = nullptr;
    int in_w_ = 0, in_h_ = 0, in_c_ = 0;
};

static TflmEngine & engine()
{
    static TflmEngine e;
    return e;
}

static int capture_and_hfill_once( float * out_hfill, int * out_box_count )
{
    if( out_hfill == nullptr ) return -1;
    *out_hfill = 0.f;
    if( out_box_count != nullptr ) *out_box_count = 0;

    const esp_err_t cam_err = ensure_camera_inited();
    if( cam_err != ESP_OK ) return -2;

    camera_fb_t * fb = esp_camera_fb_get();
    if( fb == nullptr )
    {
        ESP_LOGE( TAG, "esp_camera_fb_get failed" );
        return -3;
    }
    if( fb->format != PIXFORMAT_RGB565 )
    {
        ESP_LOGE( TAG, "Unexpected pixel format %d (expected RGB565)", fb->format );
        esp_camera_fb_return( fb );
        return -4;
    }

    const int w = fb->width;
    const int h = fb->height;
    const uint16_t * rgb565 = reinterpret_cast<const uint16_t *>( fb->buf );

    if( !engine().run_from_rgb565_qvga( rgb565, w, h ) )
    {
        esp_camera_fb_return( fb );
        return -5;
    }

    // Extract detections and compute hfill.
    std::vector<BoxF> dets = engine().extract_detections_as_frame_px( w, h );
    const RoiPx roi = roi_for_frame( w, h );
    int box_count = 0;
    const float hfill = compute_hfill( dets, roi, w, h, &box_count );

    esp_camera_fb_return( fb );

    *out_hfill = hfill;
    if( out_box_count != nullptr ) *out_box_count = box_count;
    ESP_LOGI( TAG, "frame: boxes=%d hfill=%.3f", box_count, ( double ) hfill );
    return 0;
}

} // namespace

int motorbike_density_compute_now( float * out_density_now, int * out_count_now )
{
    if( out_density_now == nullptr ) return -1;

#if !CONFIG_MBDET_ENABLE
    *out_density_now = 0.f;
    if( out_count_now != nullptr ) *out_count_now = 0;
    return 0;
#else
    const int burst_n = CONFIG_MBDET_BURST_N;
    float vals[ 10 ] = { 0 };
    int counts[ 10 ] = { 0 };
    const int n = std::clamp( burst_n, 1, 10 );

    for( int i = 0; i < n; ++i )
    {
        float hfill = 0.f;
        int box_count = 0;
        const int rc = capture_and_hfill_once( &hfill, &box_count );
        if( rc != 0 )
        {
            ESP_LOGE( TAG, "Burst frame %d failed rc=%d", i, rc );
            // Keep going; treat as 0 to avoid blocking publishes.
            hfill = 0.f;
            counts[ i ] = 0;
        }
        else
        {
            counts[ i ] = box_count;
        }
        vals[ i ] = hfill;

        // Optional spacing 200–500ms as per plan.
        if( i + 1 < n )
        {
            const uint32_t delay_ms = 200 + ( esp_random() % 301 ); // 200..500
            vTaskDelay( pdMS_TO_TICKS( delay_ms ) );
        }
    }

    float density = 0.f;
    if( n == 1 )
    {
        density = vals[ 0 ];
    }
    else if( n == 2 )
    {
        density = 0.5f * ( vals[ 0 ] + vals[ 1 ] );
    }
    else
    {
        density = median3( vals[ 0 ], vals[ 1 ], vals[ 2 ] );
        if( n > 3 )
        {
            // General median for >3: sort copy (still small).
            std::vector<float> v( vals, vals + n );
            std::sort( v.begin(), v.end() );
            density = v[ n / 2 ];
        }
    }

    *out_density_now = clamp01( density );
    // Motorbike "count" (topK+ROI+score filtered)
    int count_now = counts[ 0 ];
    if( n > 1 )
    {
        std::vector<int> c( counts, counts + n );
        std::sort( c.begin(), c.end() );
        count_now = c[ n / 2 ];
    }
    if( out_count_now != nullptr )
    {
        *out_count_now = count_now;
    }
    ESP_LOGI( TAG, "motorbike_count_now=%d (filtered)", count_now );
    // Summary log (count is per-frame already; we keep this line stable for MQTT demo logs).
    ESP_LOGI( TAG, "density_now=%.3f (%.1f%%)", ( double ) *out_density_now, ( double ) ( ( *out_density_now ) * 100.f ) );
    return 0;
#endif
}

int motorbike_send_image_base64_serial( void )
{
#if !CONFIG_MBDET_ENABLE
    return -1;
#else
    const esp_err_t cam_err = ensure_camera_inited();
    if( cam_err != ESP_OK ) return -2;

    camera_fb_t * fb = esp_camera_fb_get();
    if( fb == nullptr )
    {
        ESP_LOGE( TAG, "esp_camera_fb_get failed" );
        return -3;
    }

    // Convert frame to JPEG
    uint8_t * jpg_buf = nullptr;
    size_t jpg_len = 0;
    bool jpeg_ok = frame2jpg( fb, 80, &jpg_buf, &jpg_len );
    esp_camera_fb_return( fb );

    if( !jpeg_ok || jpg_buf == nullptr || jpg_len == 0 )
    {
        ESP_LOGE( TAG, "frame2jpg failed" );
        if( jpg_buf != nullptr ) free( jpg_buf );
        return -4;
    }

    // Base64 encode: first pass to get required buffer size
    size_t b64_len = 0;
    int b64_rc = mbedtls_base64_encode( nullptr, 0, &b64_len, jpg_buf, jpg_len );
    if( b64_rc != 0 && b64_rc != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL )
    {
        ESP_LOGE( TAG, "mbedtls_base64_encode size query failed: %d", b64_rc );
        free( jpg_buf );
        return -5;
    }

    // Allocate buffer with extra byte for null terminator
    uint8_t * b64_buf = static_cast<uint8_t *>( malloc( b64_len + 1 ) );
    if( b64_buf == nullptr )
    {
        ESP_LOGE( TAG, "malloc for base64 buffer failed (size=%zu)", b64_len );
        free( jpg_buf );
        return -6;
    }

    // Second pass: actual encoding
    size_t out_len = 0;
    b64_rc = mbedtls_base64_encode( b64_buf, b64_len, &out_len, jpg_buf, jpg_len );
    free( jpg_buf );

    if( b64_rc != 0 || out_len == 0 || out_len > b64_len )
    {
        ESP_LOGE( TAG, "mbedtls_base64_encode failed: %d (out_len=%zu, b64_len=%zu)", b64_rc, out_len, b64_len );
        free( b64_buf );
        return -7;
    }

    b64_buf[ out_len ] = '\0';

    // Send over serial with markers
    printf( "IMG_B64_START\n" );
    printf( "%s\n", ( const char * ) b64_buf );
    printf( "IMG_B64_END\n" );
    fflush( stdout );

    free( b64_buf );
    ESP_LOGI( TAG, "Sent JPEG image (%zu bytes) as base64 over serial", jpg_len );
    return 0;
#endif
}


