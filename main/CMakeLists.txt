idf_component_register(
    SRCS
	"app_main.c"
	"mqtt_demo_mutual_auth.c"
        "motorbike_detector.cpp"
    INCLUDE_DIRS
	"."
    PRIV_REQUIRES
        backoffAlgorithm
        coreMQTT
        posix_compat
        protocol_examples_common
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        esp32-camera
        esp-tflite-micro
)

# If client cert/key are present (and embedded by the top-level CMakeLists),
# enable the code path that uses the embedded symbols.
if(EXISTS "${CMAKE_SOURCE_DIR}/main/certs/client.crt" AND
   EXISTS "${CMAKE_SOURCE_DIR}/main/certs/client.key")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CLIENT_CERT_EMBEDDED=1)
endif()