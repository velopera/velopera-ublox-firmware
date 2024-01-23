#include <stdlib.h>
#include <string.h>

#include "velopera.h"
#include "parser_library.h"

can_obj_velopera_h_t *alloc_state()
{

    return calloc(1, sizeof(can_obj_velopera_h_t));
}

static int handle_message(struct _library_funcs *funcs, void *can_state, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp)
{
    int ret = 0;
    can_obj_velopera_h_t *state = (can_obj_velopera_h_t *)can_state;

    unpack_message(can_state, id, data, dlc, time_stamp);

    switch (id)
    {
    case 0x321: // can_0x321_Speed
    {
        double speed;
        ret = decode_can_0x321_Speed(state, &speed);
        // printf("speedy! %g\n", speed);
        if (funcs->callbacks.updateSpeed)
        {
            funcs->callbacks.updateSpeed(state, speed, 1);
        }
    }
    break;

    default:
        //
        ;
    }
    return 0;
}

void lib_init_velopera(library_funcs *funcs)
{
    bzero(funcs, sizeof(*funcs));

    printf("LIB INIT VelOPERA\n");

    funcs->unpack = unpack_message;
    funcs->print = print_message;
    funcs->alloc_state = alloc_state;
    funcs->handle_message = handle_message;
}