#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#ifndef DBCC_TIME_STAMP
#define DBCC_TIME_STAMP
    typedef uint32_t
        dbcc_time_stamp_t; /* Time stamp for message; you decide on units */
#endif

    typedef struct _library_funcs
    {
        // unpack a message into the (opaque) state struct for further processing
        int (*unpack)(void *can_state, const unsigned long id, uint64_t data,
                      uint8_t dlc, dbcc_time_stamp_t time_stamp);

        /// print a message into a json-encoded form, decoding the data fields to
        /// meaningful units
        int (*print)(const void *can_state, const unsigned long id, char *buf,
                     int bufSize);

        /// unpack, decode and call callback functions
        int (*handle_message)(struct _library_funcs *funcs, void *can_state,
                              const unsigned long id, uint64_t data, uint8_t dlc,
                              dbcc_time_stamp_t time_stamp);

        /// allocate memory for a state structure
        void *(*alloc_state)();

    } library_funcs;

    typedef void (*lib_init_func)(library_funcs *);

    extern void lib_init_pja612(library_funcs *);
    extern void lib_init_velopera(library_funcs *);

    /** the (at least) one exported function by the library, passing out the
     * callback functions */
    void lib_init(library_funcs *funcs);

#ifdef __cplusplus
}
#endif