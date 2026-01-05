# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
    "$ENV{IDF_PATH}/examples/common_components/protocol_examples_common"
    "${CMAKE_CURRENT_LIST_DIR}/libraries/backoffAlgorithm"
    "${CMAKE_CURRENT_LIST_DIR}/libraries/coreMQTT"
    "${CMAKE_CURRENT_LIST_DIR}/libraries/common/posix_compat"
	)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tls_mutual_auth)

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/epoch_100_int8.tflite")
    message(FATAL_ERROR
        "Missing model file: ${CMAKE_CURRENT_LIST_DIR}/epoch_100_int8.tflite\n"
        "Place your INT8 TFLite model at the project root as described in plan.md.")
endif()
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "epoch_100_int8.tflite" BINARY)

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/main/certs/root_cert_auth.crt")
    message(FATAL_ERROR
        "Missing root CA certificate file: ${CMAKE_CURRENT_LIST_DIR}/main/certs/root_cert_auth.crt\n"
        "Create it (PEM text) so it can be embedded via target_add_binary_data().")
endif()
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/root_cert_auth.crt" TEXT)

# Client cert/key are optional when using esp_secure_cert_mgr or a secure element.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/main/certs/client.crt" AND
   EXISTS "${CMAKE_CURRENT_LIST_DIR}/main/certs/client.key")
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/client.key" TEXT)
else()
    message(STATUS
        "Client cert/key not found under main/certs/. "
        "If you are not using esp_secure_cert_mgr or a secure element, add:\n"
        "  - main/certs/client.crt\n"
        "  - main/certs/client.key")
endif()
