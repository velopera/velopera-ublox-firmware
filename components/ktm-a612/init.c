#include <stdlib.h>
#include <string.h>

#include "ktm-a612.h"
#include "parser_library.h"

can_obj_ktm_a612_h_t *alloc_state()
{

    return calloc(1, sizeof(can_obj_ktm_a612_h_t));
}

static int handle_message(struct _library_funcs *funcs, void *can_state, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp)
{
    int ret = 0;
    can_obj_ktm_a612_h_t *state = (can_obj_ktm_a612_h_t *)can_state;

    unpack_message(can_state, id, data, dlc, time_stamp);

    switch (id)
    {
    case 0x12d: // can_0x12b_ABS_Wheel_Speed_info_930_t
    {
        double speed;
        ret = decode_can_0x12d_ABS_Front_Wheel_Speed(state, &speed);
        printf("speedy! %g\n", speed);
    }
    break;

    case 0x120: // can_0x120_ECU_Engine_RPM_and_others_930_t:
    {
        uint16_t rpm;
        ret = decode_can_0x120_ECU_Engine_Rpm(state, &rpm);
        printf("rpm! %d\n", rpm);
    }
    break;

    case 0x129: // can_0x129_ECU_Gear_Pos_Clutch_Switch_619_t
    {
        uint8_t gear;
        ret = decode_can_0x129_ECU_Gear_Position(state, &gear);
        printf("gear! %d\n", gear);
    }
    break;

    default:
        //
        ;
    }
    return 0;
}

void lib_init_pja612(library_funcs *funcs)
{
    bzero(funcs, sizeof(*funcs));

    printf("LIB INIT PJA612 superduke\n");

    funcs->unpack = unpack_message;
    funcs->print = print_message;
    funcs->alloc_state = alloc_state;
    funcs->handle_message = handle_message;
}