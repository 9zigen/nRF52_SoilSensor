project(SoilSensor C ASM)
cmake_minimum_required(VERSION 3.7)

# specify the cross compiler
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-c++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")

set(AS arm-none-eabi-as)
set(AR arm-none-eabi-ar)
set(OBJCOPY arm-none-eabi-objcopy)
set(OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

SET(SDK_ROOT ../)

set(CMAKE_OSX_SYSROOT "/")
set(CMAKE_OSX_DEPLOYMENT_TARGET "")

# language standard/version settings
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 98)

list(APPEND USER_SOURCES
    "${SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52.S"
    "${SDK_ROOT}/components/libraries/button/app_button.c"
    "${SDK_ROOT}/components/libraries/util/app_error.c"
    "${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c"
    "${SDK_ROOT}/components/libraries/util/app_error_weak.c"
    "${SDK_ROOT}/components/libraries/scheduler/app_scheduler.c"
    "${SDK_ROOT}/components/libraries/timer/app_timer.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/blockwise.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/cifra_cmac.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/cifra_eax_aes.c"
    "${SDK_ROOT}/components/libraries/crc16/crc16.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/eax.c"
    "${SDK_ROOT}/components/libraries/fds/fds.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/gf128.c"
    "${SDK_ROOT}/components/libraries/hardfault/hardfault_implementation.c"
    "${SDK_ROOT}/components/libraries/mem_manager/mem_manager.c"
    "${SDK_ROOT}/external/cifra_AES128-EAX/modes.c"
    "${SDK_ROOT}/components/libraries/util/nrf_assert.c"
    "${SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c"
    "${SDK_ROOT}/components/libraries/atomic_flags/nrf_atflags.c"
    "${SDK_ROOT}/components/libraries/atomic/nrf_atomic.c"
    "${SDK_ROOT}/components/libraries/balloc/nrf_balloc.c"
    "${SDK_ROOT}/external/fprintf/nrf_fprintf.c"
    "${SDK_ROOT}/external/fprintf/nrf_fprintf_format.c"
    "${SDK_ROOT}/components/libraries/fstorage/nrf_fstorage.c"
    "${SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_sd.c"
    "${SDK_ROOT}/components/libraries/memobj/nrf_memobj.c"
    "${SDK_ROOT}/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c"
    "${SDK_ROOT}/components/libraries/queue/nrf_queue.c"
    "${SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c"
    "${SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c"
    "${SDK_ROOT}/components/libraries/strerror/nrf_strerror.c"
    "${SDK_ROOT}/components/libraries/sensorsim/sensorsim.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecc.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdh.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdsa.c"
    "${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_clock.c"
    "${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_rng.c"
    "${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_uart.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_clock.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_power_clock.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rng.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uart.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_ppi.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_timer.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cifra/cifra_backend_aes_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_init.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng_mbedtls.c"
    "${SDK_ROOT}/components/libraries/bsp/bsp.c"
    "${SDK_ROOT}/components/libraries/bsp/bsp_btn_ble.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT_printf.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_aes.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_aes_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_chacha_poly_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_ecc.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_ecdh.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_ecdsa.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_eddsa.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_hash.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_hmac.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_init.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_mutex.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_rng.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310/cc310_backend_shared.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_chacha_poly_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecc.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdh.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdsa.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_eddsa.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hash.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hmac.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_aes.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_aes_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_ecc.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_ecdh.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_ecdsa.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_hash.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_hmac.c"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls/mbedtls_backend_init.c"
    "${SDK_ROOT}/components/ble/peer_manager/auth_status_tracker.c"
    "${SDK_ROOT}/components/ble/common/ble_advdata.c"
    "${SDK_ROOT}/components/ble/ble_advertising/ble_advertising.c"
    "${SDK_ROOT}/components/ble/common/ble_conn_params.c"
    "${SDK_ROOT}/components/ble/common/ble_conn_state.c"
    "${SDK_ROOT}/components/ble/common/ble_srv_common.c"
    "${SDK_ROOT}/components/ble/peer_manager/gatt_cache_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/gatts_cache_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/id_manager.c"
    "${SDK_ROOT}/components/ble/nrf_ble_gatt/nrf_ble_gatt.c"
    "${SDK_ROOT}/components/ble/peer_manager/nrf_ble_lesc.c"
    "${SDK_ROOT}/components/ble/nrf_ble_qwr/nrf_ble_qwr.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_data_storage.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_database.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_id.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_manager_handler.c"
    "${SDK_ROOT}/components/ble/peer_manager/pm_buffer.c"
    "${SDK_ROOT}/components/ble/peer_manager/security_dispatcher.c"
    "${SDK_ROOT}/components/ble/peer_manager/security_manager.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_bas/ble_bas.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_dis/ble_dis.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_hrs/ble_hrs.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_backend_rtt.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_backend_serial.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_backend_uart.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_default_backends.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c"
    "${SDK_ROOT}/components/boards/boards.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_aead.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes_shared.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecc.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdh.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdsa.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_eddsa.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_error.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_hash.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_hkdf.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_hmac.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_init.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_rng.c"
    "${SDK_ROOT}/components/libraries/crypto/nrf_crypto_shared.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_ias/ble_ias.c"
    "${SDK_ROOT}/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c"
    "${SDK_ROOT}/external/mbedtls/library/aes.c"
    "${SDK_ROOT}/external/mbedtls/library/aesni.c"
    "${SDK_ROOT}/external/mbedtls/library/arc4.c"
    "${SDK_ROOT}/external/mbedtls/library/asn1parse.c"
    "${SDK_ROOT}/external/nrf_tls/mbedtls/replacements/asn1write.c"
    "${SDK_ROOT}/external/mbedtls/library/base64.c"
    "${SDK_ROOT}/external/mbedtls/library/bignum.c"
    "${SDK_ROOT}/external/mbedtls/library/blowfish.c"
    "${SDK_ROOT}/external/mbedtls/library/camellia.c"
    "${SDK_ROOT}/external/mbedtls/library/ccm.c"
    "${SDK_ROOT}/external/mbedtls/library/certs.c"
    "${SDK_ROOT}/external/mbedtls/library/cipher.c"
    "${SDK_ROOT}/external/mbedtls/library/cipher_wrap.c"
    "${SDK_ROOT}/external/mbedtls/library/cmac.c"
    "${SDK_ROOT}/external/mbedtls/library/ctr_drbg.c"
    "${SDK_ROOT}/external/mbedtls/library/debug.c"
    "${SDK_ROOT}/external/mbedtls/library/des.c"
    "${SDK_ROOT}/external/mbedtls/library/dhm.c"
    "${SDK_ROOT}/external/mbedtls/library/ecdh.c"
    "${SDK_ROOT}/external/mbedtls/library/ecdsa.c"
    "${SDK_ROOT}/external/mbedtls/library/ecp.c"
    "${SDK_ROOT}/external/mbedtls/library/ecp_curves.c"
    "${SDK_ROOT}/external/mbedtls/library/entropy.c"
    "${SDK_ROOT}/external/mbedtls/library/entropy_poll.c"
    "${SDK_ROOT}/external/mbedtls/library/error.c"
    "${SDK_ROOT}/external/mbedtls/library/gcm.c"
    "${SDK_ROOT}/external/mbedtls/library/havege.c"
    "${SDK_ROOT}/external/mbedtls/library/hmac_drbg.c"
    "${SDK_ROOT}/external/mbedtls/library/md.c"
    "${SDK_ROOT}/external/mbedtls/library/md2.c"
    "${SDK_ROOT}/external/mbedtls/library/md4.c"
    "${SDK_ROOT}/external/mbedtls/library/md5.c"
    "${SDK_ROOT}/external/mbedtls/library/md_wrap.c"
    "${SDK_ROOT}/external/mbedtls/library/memory_buffer_alloc.c"
    "${SDK_ROOT}/external/mbedtls/library/oid.c"
    "${SDK_ROOT}/external/mbedtls/library/padlock.c"
    "${SDK_ROOT}/external/mbedtls/library/pem.c"
    "${SDK_ROOT}/external/mbedtls/library/pk.c"
    "${SDK_ROOT}/external/mbedtls/library/pk_wrap.c"
    "${SDK_ROOT}/external/mbedtls/library/pkcs11.c"
    "${SDK_ROOT}/external/mbedtls/library/pkcs12.c"
    "${SDK_ROOT}/external/mbedtls/library/pkcs5.c"
    "${SDK_ROOT}/external/mbedtls/library/pkparse.c"
    "${SDK_ROOT}/external/mbedtls/library/pkwrite.c"
    "${SDK_ROOT}/external/mbedtls/library/platform.c"
    "${SDK_ROOT}/external/mbedtls/library/ripemd160.c"
    "${SDK_ROOT}/external/mbedtls/library/rsa.c"
    "${SDK_ROOT}/external/mbedtls/library/sha1.c"
    "${SDK_ROOT}/external/mbedtls/library/sha256.c"
    "${SDK_ROOT}/external/mbedtls/library/sha512.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_cache.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_ciphersuites.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_cli.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_cookie.c"
    "${SDK_ROOT}/external/nrf_tls/mbedtls/replacements/ssl_srv.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_ticket.c"
    "${SDK_ROOT}/external/mbedtls/library/ssl_tls.c"
    "${SDK_ROOT}/external/mbedtls/library/threading.c"
    "${SDK_ROOT}/external/mbedtls/library/version.c"
    "${SDK_ROOT}/external/mbedtls/library/version_features.c"
    "${SDK_ROOT}/external/mbedtls/library/x509.c"
    "${SDK_ROOT}/external/mbedtls/library/x509_create.c"
    "${SDK_ROOT}/external/mbedtls/library/x509_crl.c"
    "${SDK_ROOT}/external/mbedtls/library/x509_crt.c"
    "${SDK_ROOT}/external/mbedtls/library/x509_csr.c"
    "${SDK_ROOT}/external/mbedtls/library/xtea.c"
    "${SDK_ROOT}/modules/nrfx/mdk/system_nrf52.c"
    "${SDK_ROOT}/external/utf_converter/utf.c"
    "${SDK_ROOT}/components/ble/ble_db_discovery/ble_db_discovery.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_saadc.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_pwm.c"
    "main.c"
    "bsp/custom_board.h"
    "pwm.c"
    "ble_cus.c"
    "ble_services.c"
    "timers.c"
    "sensors/sensors.c"
    "sensors/ec_sensor.c"
    "sensors/soil_sensor.c"
    "sensors/bat_sensor.c"
    "sensors/isl29035.c"
    "sensors/shtc3.c"
    "i2c.c")

# SoftDevice
include_directories(
    "${SDK_ROOT}/components/softdevice/s132/headers"
    "${SDK_ROOT}/components/softdevice/s132/headers/nrf52")

include_directories(
    "${SDK_ROOT}/components/nfc/ndef/generic/message"
    "${SDK_ROOT}/components/nfc/t2t_lib"
    "${SDK_ROOT}/components/nfc/t4t_parser/hl_detection_procedure"
    "${SDK_ROOT}/components/ble/ble_services/ble_ancs_c"
    "${SDK_ROOT}/components/ble/ble_services/ble_ias_c"
    "${SDK_ROOT}/components/libraries/pwm"
    "${SDK_ROOT}/external/nrf_oberon/include"
    "${SDK_ROOT}/components/libraries/usbd/class/cdc/acm"
    "${SDK_ROOT}/components/libraries/usbd/class/hid/generic"
    "${SDK_ROOT}/components/libraries/usbd/class/msc"
    "${SDK_ROOT}/components/libraries/usbd/class/hid"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser"
    "${SDK_ROOT}/components/libraries/log"
    "${SDK_ROOT}/components/ble/ble_services/ble_gls"
    "${SDK_ROOT}/components/libraries/fstorage"
    "${SDK_ROOT}/components/nfc/ndef/text"
    "${SDK_ROOT}/components/libraries/mutex"
    "${SDK_ROOT}/components/libraries/gpiote"
    "${SDK_ROOT}/components/libraries/bootloader/ble_dfu"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/common"
    "${SDK_ROOT}/components/boards"
    "${SDK_ROOT}/components/nfc/ndef/generic/record"
    "${SDK_ROOT}/components/nfc/t4t_parser/cc_file"
    "${SDK_ROOT}/components/ble/ble_advertising"
    "${SDK_ROOT}/external/utf_converter"
    "${SDK_ROOT}/components/ble/ble_services/ble_bas_c"
    "${SDK_ROOT}/components/libraries/crypto/backend/micro_ecc"
    "${SDK_ROOT}/modules/nrfx/drivers/include"
    "${SDK_ROOT}/components/libraries/experimental_task_manager"
    "${SDK_ROOT}/external/cifra_AES128-EAX"
    "${SDK_ROOT}/components/ble/ble_services/ble_hrs_c"
    "${SDK_ROOT}/components/softdevice/s132/headers/nrf52"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/le_oob_rec"
    "${SDK_ROOT}/components/libraries/crypto/backend/oberon"
    "${SDK_ROOT}/components/libraries/queue"
    "${SDK_ROOT}/components/libraries/pwr_mgmt"
    "${SDK_ROOT}/components/ble/ble_dtm"
    "${SDK_ROOT}/components/toolchain/cmsis/include"
    "${SDK_ROOT}/components/ble/ble_services/ble_rscs_c"
    "${SDK_ROOT}/components/ble/common"
    "${SDK_ROOT}/components/ble/ble_services/ble_lls"
    "${SDK_ROOT}/components/libraries/bsp"
    "${SDK_ROOT}/components/libraries/crypto/backend/nrf_hw"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/ac_rec"
    "${SDK_ROOT}/components/ble/ble_services/ble_bas"
    "${SDK_ROOT}/components/libraries/mpu"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/ble/ble_services/ble_ans_c"
    "${SDK_ROOT}/components/libraries/slip"
    "${SDK_ROOT}/components/libraries/delay"
    "${SDK_ROOT}/components/libraries/mem_manager"
    "${SDK_ROOT}/components/libraries/usbd/class/cdc"
    "${SDK_ROOT}/components/libraries/memobj"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310_bl"
    "${SDK_ROOT}/components/ble/ble_services/ble_nus_c"
    "${SDK_ROOT}/components/softdevice/common"
    "${SDK_ROOT}/components/ble/ble_services/ble_ias"
    "${SDK_ROOT}/components/libraries/usbd/class/hid/mouse"
    "${SDK_ROOT}/components/libraries/low_power_pwm"
    "${SDK_ROOT}/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser"
    "${SDK_ROOT}/components/ble/ble_services/ble_dfu"
    "${SDK_ROOT}/external/fprintf"
    "${SDK_ROOT}/components/libraries/svc"
    "${SDK_ROOT}/components/libraries/atomic"
    "${SDK_ROOT}/components"
    "${SDK_ROOT}/external/nrf_cc310/include"
    "${SDK_ROOT}/components/libraries/scheduler"
    "${SDK_ROOT}/components/libraries/cli"
    "${SDK_ROOT}/components/ble/ble_services/ble_lbs"
    "${SDK_ROOT}/components/ble/ble_services/ble_hts"
    "${SDK_ROOT}/components/libraries/crc16"
    "${SDK_ROOT}/components/nfc/t4t_parser/apdu"
    "${SDK_ROOT}/components/libraries/util"
    "${SDK_ROOT}/components/libraries/csense_drv"
    "${SDK_ROOT}/components/libraries/csense"
    "${SDK_ROOT}/components/libraries/balloc"
    "${SDK_ROOT}/components/libraries/ecc"
    "${SDK_ROOT}/components/libraries/hardfault"
    "${SDK_ROOT}/components/ble/ble_services/ble_cscs"
    "${SDK_ROOT}/components/libraries/hci"
    "${SDK_ROOT}/components/libraries/usbd/class/hid/kbd"
    "${SDK_ROOT}/components/libraries/timer"
    "${SDK_ROOT}/components/softdevice/s132/headers"
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/components/libraries/stack_info"
    "${SDK_ROOT}/components/nfc/t4t_parser/tlv"
    "${SDK_ROOT}/components/libraries/sortlist"
    "${SDK_ROOT}/components/libraries/spi_mngr"
    "${SDK_ROOT}/components/libraries/led_softblink"
    "${SDK_ROOT}/components/nfc/ndef/conn_hand_parser"
    "${SDK_ROOT}/components/libraries/sdcard"
    "${SDK_ROOT}/components/nfc/t4t_lib"
    "${SDK_ROOT}/components/nfc/ndef/parser/record"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/components/ble/ble_services/ble_cts_c"
    "${SDK_ROOT}/external/nrf_tls/mbedtls/nrf_crypto/config"
    "${SDK_ROOT}/components/ble/ble_services/ble_nus"
    "${SDK_ROOT}/components/libraries/twi_mngr"
    "${SDK_ROOT}/components/ble/ble_services/ble_hids"
    "${SDK_ROOT}/components/libraries/strerror"
    "${SDK_ROOT}/components/libraries/crc32"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_oob_advdata"
    "${SDK_ROOT}/components/nfc/t2t_parser"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_msg"
    "${SDK_ROOT}/components/libraries/usbd/class/audio"
    "${SDK_ROOT}/components/nfc/t4t_lib/hal_t4t"
    "${SDK_ROOT}/components/libraries/sensorsim"
    "${SDK_ROOT}/components/libraries/crypto/backend/nrf_sw"
    "${SDK_ROOT}/components/ble/peer_manager"
    "${SDK_ROOT}/components/drivers_nrf/usbd"
    "${SDK_ROOT}/components/libraries/ringbuf"
    "${SDK_ROOT}/components/ble/ble_services/ble_tps"
    "${SDK_ROOT}/components/ble/ble_services/ble_rscs"
    "${SDK_ROOT}/components/nfc/ndef/parser/message"
    "${SDK_ROOT}/components/libraries/crypto/backend/mbedtls"
    "${SDK_ROOT}/components/libraries/crypto/backend/cc310"
    "${SDK_ROOT}/external/nrf_oberon"
    "${SDK_ROOT}/external/micro-ecc/micro-ecc"
    "${SDK_ROOT}/components/ble/ble_services/ble_dis"
    "${SDK_ROOT}/components/nfc/ndef/uri"
    "${SDK_ROOT}/components/ble/nrf_ble_gatt"
    "${SDK_ROOT}/components/ble/nrf_ble_qwr"
    "${SDK_ROOT}/components/libraries/gfx"
    "${SDK_ROOT}/components/libraries/button"
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/components/libraries/twi_sensor"
    "${SDK_ROOT}/integration/nrfx/legacy"
    "${SDK_ROOT}/components/libraries/usbd"
    "${SDK_ROOT}/external/mbedtls/include"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/ep_oob_rec"
    "${SDK_ROOT}/external/segger_rtt"
    "${SDK_ROOT}/components/libraries/atomic_fifo"
    "${SDK_ROOT}/components/ble/ble_services/ble_lbs_c"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_lib"
    "${SDK_ROOT}/components/libraries/crypto"
    "${SDK_ROOT}/components/ble/ble_racp"
    "${SDK_ROOT}/components/libraries/fds"
    "${SDK_ROOT}/components/nfc/ndef/launchapp"
    "${SDK_ROOT}/components/libraries/atomic_flags"
    "${SDK_ROOT}/components/ble/ble_services/ble_hrs"
    "${SDK_ROOT}/components/libraries/crypto/backend/cifra"
    "${SDK_ROOT}/components/nfc/ndef/connection_handover/hs_rec"
    "${SDK_ROOT}/components/nfc/t2t_lib/hal_t2t"
    "${SDK_ROOT}/components/nfc/ndef/conn_hand_parser/ac_rec_parser"
    "${SDK_ROOT}/components/libraries/stack_guard"
    "${SDK_ROOT}/components/libraries/log/src"
    "${SDK_ROOT}/components/ble/ble_db_discovery"
    "${SDK_ROOT}/components/ble/ble_link_ctx_manager/"
    "${SDK_ROOT}/modules/nrfx/drivers/include"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/bsp"
    "${CMAKE_CURRENT_SOURCE_DIR}/sensors/include"
)

# Legacy
include_directories("${SDK_ROOT}/integration/nrfx/legacy")

# NRFX
include_directories(${SDK_ROOT}/modules/nrfx)
include_directories(${SDK_ROOT}/modules/nrfx/mdk)
include_directories(${SDK_ROOT}/modules/nrfx/hal)
include_directories(${SDK_ROOT}/integration/nrfx)

# CPU config
include_directories("pca10040/s132/config")
include_directories("${SDK_ROOT}/modules/nrfx/mdk")

add_definitions(-DBLE_STACK_SUPPORT_REQD
    -DBOARD_CUSTOM
    -DCONFIG_GPIO_AS_PINRESET
    -DFLOAT_ABI_HARD
    -DMBEDTLS_CONFIG_FILE=\"nrf_crypto_mbedtls_config.h\"
    -DNRF52832_XXAA
    -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
    -DNRF_SD_BLE_API_VERSION=6
    -DS132
    -DSOFTDEVICE_PRESENT
    -DSWI_DISABLE0
    -DuECC_ENABLE_VLI_API=0
    -DuECC_OPTIMIZATION_LEVEL=3
    -DuECC_SQUARE_FUNC=0
    -DuECC_SUPPORT_COMPRESSED_POINT=0
    -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1
    -DCLION)

# nRF52
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/gcc_nrf52.ld")
set(CPU_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# Compiler/assambler/linker flags
set(COMMON_FLAGS "${CPU_FLAGS} -mthumb -mabi=aapcs -Wall -Werror -O3 -g3 -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums")

# compiler/assambler/linker flags
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_CXX_FLAGS "-O3 -g3")
set(CMAKE_ASM_FLAGS "-g3 -mthumb -mabi=aapcs -T${LINKER_SCRIPT} ${CPU_FLAGS} -Wl,--gc-sections --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "-mthumb -mabi=aapcs -std=gnu++98 -std=c99 -L ${NRF5_SDK_PATH}/components/toolchain/gcc -T${LINKER_SCRIPT} ${CPU_FLAGS} -Wl,--gc-sections --specs=nano.specs -lc -lnosys -lm")

# note: we must override the default cmake linker flags so that CMAKE_C_FLAGS are not added implicitly
#set(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_COMPILER} <LINK_FLAGS> <OBJECTS> -o <TARGET>")
#set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_C_COMPILER} <LINK_FLAGS> <OBJECTS> -lstdc++ -o <TARGET>")

#SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
#SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
#SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections -flto -T ${LINKER_SCRIPT}")

#set(CMAKE_CXX_STANDARD 11)
#
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

# executable
add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${LINKER_SCRIPT})

#set_target_properties(${PROJECT_NAME} PROPERTIES ${CMAKE_EXE_LINKER_FLAGS} "-Wl,-Map=${EXECUTABLE_NAME}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(SOFT_DEVICE ../../components/softdevice/s132/hex/s132_nrf52_6.1.0_softdevice.hex)
set(FIRMWARE ${PROJECT_NAME}_softdevice.hex)
message( ${SOFT_DEVICE} )

add_custom_command(TARGET ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
    COMMAND ${OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${HEX_FILE}
    COMMAND ${OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${BIN_FILE}
    COMMAND mergehex --merge ${SOFT_DEVICE} ${HEX_FILE} --output ${PROJECT_BINARY_DIR}/${FIRMWARE}
    COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")


add_custom_target(UPLOAD
    arm-none-eabi-gdb -iex "target remote tcp:127.0.0.1:2331"
    -iex "monitor reset"
    -iex "file ${PROJECT_NAME}.elf"
    -iex "load"
    -iex "continue"
    -iex "disconnect" -iex "quit")

#add_custom_target(NRF52 COMMAND make
#    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})