
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DSDL_LAT_POWERTRAIN_PTCONCISE_MAX_SIZE 13
#define DSDL_LAT_POWERTRAIN_PTCONCISE_SIGNATURE (0x9AEC32B516738954ULL)

#define DSDL_LAT_POWERTRAIN_PTCONCISE_ID 20100





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dsdl_lat_powertrain_PTConcise_cxx_iface;
#endif


struct dsdl_lat_powertrain_PTConcise {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dsdl_lat_powertrain_PTConcise_cxx_iface;
#endif




    bool egu_ok;



    bool batt_ok;





    float v_bus;



    float i_load;



    float i_bat;



    float i_rect;



    float batt_soc;



    float fuel_consumption;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _dsdl_lat_powertrain_PTConcise_encode(struct dsdl_lat_powertrain_PTConcise* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _dsdl_lat_powertrain_PTConcise_decode(const CanardRxTransfer* transfer, struct dsdl_lat_powertrain_PTConcise* msg);

static inline uint32_t dsdl_lat_powertrain_PTConcise_encode(struct dsdl_lat_powertrain_PTConcise* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _dsdl_lat_powertrain_PTConcise_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool dsdl_lat_powertrain_PTConcise_decode(const CanardRxTransfer* transfer, struct dsdl_lat_powertrain_PTConcise* msg) {

    return _dsdl_lat_powertrain_PTConcise_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)

static inline void __dsdl_lat_powertrain_PTConcise_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_PTConcise* msg, bool tao);
static inline bool __dsdl_lat_powertrain_PTConcise_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_PTConcise* msg, bool tao);
void __dsdl_lat_powertrain_PTConcise_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_PTConcise* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->egu_ok);

    *bit_ofs += 1;






    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->batt_ok);

    *bit_ofs += 1;





    *bit_ofs += 6;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->v_bus);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->i_load);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->i_bat);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->i_rect);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->batt_soc);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;






    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->fuel_consumption);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }

    *bit_ofs += 16;





}

/*
 decode dsdl_lat_powertrain_PTConcise, return true on failure, false on success
*/
bool __dsdl_lat_powertrain_PTConcise_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_lat_powertrain_PTConcise* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->egu_ok);

    *bit_ofs += 1;







    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->batt_ok);

    *bit_ofs += 1;






    *bit_ofs += 6;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->v_bus = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->i_load = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->i_bat = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->i_rect = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->batt_soc = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;







    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->fuel_consumption = canardConvertFloat16ToNativeFloat(float16_val);
    }

    *bit_ofs += 16;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dsdl_lat_powertrain_PTConcise sample_dsdl_lat_powertrain_PTConcise_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>


BROADCAST_MESSAGE_CXX_IFACE(dsdl_lat_powertrain_PTConcise, DSDL_LAT_POWERTRAIN_PTCONCISE_ID, DSDL_LAT_POWERTRAIN_PTCONCISE_SIGNATURE, DSDL_LAT_POWERTRAIN_PTCONCISE_MAX_SIZE);


#endif
#endif
