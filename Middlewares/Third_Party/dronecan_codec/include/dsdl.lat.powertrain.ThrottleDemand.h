
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_MAX_SIZE 4
#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_SIGNATURE (0x99C717F39FD9011ULL)

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_ID 20101



#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_IDLE 0

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_TAKEOFF 1

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_CLIMB 2

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_CRUISE 3

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_LAND 4

#define DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_FAULT 5





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dsdl_lat_powertrain_ThrottleDemand_cxx_iface;
#endif


struct dsdl_lat_powertrain_ThrottleDemand {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dsdl_lat_powertrain_ThrottleDemand_cxx_iface;
#endif




    uint8_t flight_phase;





    float throttle_pct;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _dsdl_lat_powertrain_ThrottleDemand_encode(struct dsdl_lat_powertrain_ThrottleDemand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _dsdl_lat_powertrain_ThrottleDemand_decode(const CanardRxTransfer* transfer, struct dsdl_lat_powertrain_ThrottleDemand* msg);

static inline uint32_t dsdl_lat_powertrain_ThrottleDemand_encode(struct dsdl_lat_powertrain_ThrottleDemand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _dsdl_lat_powertrain_ThrottleDemand_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool dsdl_lat_powertrain_ThrottleDemand_decode(const CanardRxTransfer* transfer, struct dsdl_lat_powertrain_ThrottleDemand* msg) {

    return _dsdl_lat_powertrain_ThrottleDemand_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)

static inline void __dsdl_lat_powertrain_ThrottleDemand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_ThrottleDemand* msg, bool tao);
static inline bool __dsdl_lat_powertrain_ThrottleDemand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_ThrottleDemand* msg, bool tao);
void __dsdl_lat_powertrain_ThrottleDemand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_ThrottleDemand* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->flight_phase);

    *bit_ofs += 4;





    *bit_ofs += 12;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->throttle_pct);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;





}

/*
 decode dsdl_lat_powertrain_ThrottleDemand, return true on failure, false on success
*/
bool __dsdl_lat_powertrain_ThrottleDemand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_ThrottleDemand* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->flight_phase);

    *bit_ofs += 4;






    *bit_ofs += 12;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->throttle_pct = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dsdl_lat_powertrain_ThrottleDemand sample_dsdl_lat_powertrain_ThrottleDemand_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>


BROADCAST_MESSAGE_CXX_IFACE(dsdl_lat_powertrain_ThrottleDemand, DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_ID, DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_SIGNATURE, DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_MAX_SIZE);


#endif
#endif
