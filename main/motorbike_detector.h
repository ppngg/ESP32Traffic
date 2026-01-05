#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Computes the "traffic density now" metric described in plan.md.
 *
 * @param[out] out_density_now 0..1 horizontal occupancy (median of burst).
 * @param[out] out_count_now Number of motorbikes detected (median of burst). Can be NULL.
 * @return 0 on success, negative on error.
 */
int motorbike_density_compute_now( float * out_density_now, int * out_count_now );

/**
 * Captures a JPEG frame, encodes it as base64, and sends it over serial.
 * Format: "IMG_B64_START\n<base64_data>\nIMG_B64_END\n"
 *
 * @return 0 on success, negative on error.
 */
int motorbike_send_image_base64_serial( void );

#ifdef __cplusplus
}
#endif


